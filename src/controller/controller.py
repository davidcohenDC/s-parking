import logging
import carla
from typing import Optional, Dict, Any, List

import numpy as np

from src.controller.parking_state import ParkingState
from src.sensors.obstacle_detector import ObstacleDetector
from src.utils.distance import euclid_dist
from src.controller.vehicle_controller import VehicleController
from src.utils.sensor_buffer import SensorBuffer
from src.utils.normalize_angle import normalize_angle


class ParkingController:
    """
    Manages the parking state machine by processing sensor data,
    applying vehicle controls, and transitioning through various parking phases.

    This implementation follows SOLID and DRY principles by delegating responsibilities,
    centralizing configuration, and keeping methods focused and concise.
    """

    def __init__(self, ego_vehicle: carla.Vehicle, lidar_sensor, obstacle_detectors: List[ObstacleDetector], config: Dict[str, Any]):
        self.ego_vehicle = ego_vehicle
        # Convert list of lidar sensors to dictionary keyed by sensor name.
        self.lidar = lidar_sensor
        self.obstacle_detectors = obstacle_detectors
        self.config = config
        self.vehicle_control = VehicleController(ego_vehicle)

        # Initialize state and frame tracking
        self.state: ParkingState = ParkingState.IDLE
        self.frame_count: int = 0
        self.fps: int = config.get('FPS')

        # Phase distance thresholds (in meters)
        self.dist_begin_steer: float = config.get('PARKING_ALIGN_BEGIN_STEERING_DIST')
        self.arc1_length: float = config.get('VEHICLE_ARC1_LENGTH')
        self.reverse_straight_length: float = config.get('VEHICLE_REVERSE_STRAIGHT_LENGTH')
        self.arc2_length: float = config.get('VEHICLE_ARC2_LENGTH')
        self.min_back_distance: float = config.get('min_back_distance')
        self.forward_min_distance: float = config.get('PARKING_FORWARD_LENGTH')

        # Smoothing & hysteresis
        self.required_frames: int = config.get('LIDAR_REQUIRED_FRAMES')
        self.sensor_buffer = SensorBuffer(max_size=config.get('LIDAR_BUFFER_SIZE'))
        self.consec_count: int = 0

        # Realignment parameters
        self.yaw_threshold: float = config.get('REALIGNMENT_YAW_THRESHOLD')
        self.conversion_factor: float = config.get('REALIGNMENT_CONVERSION_FACTOR')
        self.max_correction: float = config.get('REALIGNMENT_MAX_CORRECTION')
        self.desired_yaw: float = config.get('REALIGNMENT_DESIRED_YAW')
        self.forward_throttle: float = config.get('VEHICLE_FORWARD_THROTTLE')

        # Tracking variables for distance and corrections
        self.distance_on_arc: float = 0.0
        self.prev_location: Optional[carla.Location] = None
        self.realign_count: int = 0
        self.parking_forward_min_dist: float = config.get('PARKING_FORWARD_MIN_DIST')

        self.min_obstacle_distance: float = config.get('OBSTACLE_MIN_DISTANCE')

    def update(self) -> None:
        # Determine sensor indices based on the current state.
        if self.state in [ParkingState.REVERSE_STRAIGHT_1,
                          ParkingState.STEER_INTO_SLOT,
                          ParkingState.REVERSE_STRAIGHT_2,
                          ParkingState.COUNTERSTEER]:
            sensor_indices = [3, 4, 5]  # Rear sensors
        elif self.state == ParkingState.FORWARD_STRAIGHT:
            sensor_indices = [1]  # Front center sensor
        else:
            sensor_indices = []  # No sensor check needed

        # Global safety check performed only once every second (or every fps frames).
        if sensor_indices and self.frame_count % self.fps == 0:
            closest_obs = self._check_relevant_obstacles(sensor_indices)
            # Use a forward-specific threshold when in FORWARD_STRAIGHT
            threshold = self.parking_forward_min_dist if self.state == ParkingState.FORWARD_STRAIGHT else self.min_obstacle_distance
            if closest_obs is not None and 0.0 < closest_obs['distance'] < threshold:
                self._transition_to_state(
                    ParkingState.STOPPED,
                    f"[ParkingController] Obstacle {closest_obs['other_actor']} detected at {closest_obs['distance']:.2f} m -> STOPPED"
                )
                self.vehicle_control.stop()
                return

        # Update the parking state machine for the current frame.
        self.frame_count += 1

        # Update vehicle travel distance since last frame.
        current_loc = self.ego_vehicle.get_transform().location
        if self.prev_location is None:
            self.prev_location = current_loc
        dist_step = euclid_dist(current_loc, self.prev_location)
        self.prev_location = current_loc

        # Retrieve LiDAR sensor distances (default to 0.0 if no value)
        dist_right = self.lidar.get_min_distance_right() or 0.0
        dist_back = self.lidar.get_min_distance_back() or 0.0

        # Update the sensor buffer based on current state.
        self._update_sensor_buffer(dist_right, dist_back)
        dist_smoothed = self.sensor_buffer.median() or 0.0

        # Determine which sensor is being used and its position index based on the current state.
        if self.state == ParkingState.REVERSE_STRAIGHT_1:
            sensor_used = "rear_right (right)"
            sensor_pos_index = 3  # right rear sensor
        elif self.state == ParkingState.REVERSE_STRAIGHT_2:
            sensor_used = "rear_right (back)"
            sensor_pos_index = 4  # center rear sensor
        elif self.state == ParkingState.FORWARD_STRAIGHT:
            sensor_used = "front (center)"
            sensor_pos_index = 1  # center front sensor (from obstacle detectors)
        else:
            sensor_used = "N/A"
            sensor_pos_index = "N/A"

        # Log sensor and state information periodically.
        if self.frame_count % self.fps == 0:
            logging.info(
                f"[ParkingController] | State: {self.state.name} | Step: {dist_step:.2f} m | "
                f"Right: {dist_right:.2f} m | Back: {dist_back:.2f} m | "
                f"Smoothed ({sensor_used}, pos {sensor_pos_index}): {dist_smoothed:.2f} m"
            )

        # State machine transitions.
        if self.state == ParkingState.IDLE:
            self._transition_to_state(ParkingState.REVERSE_STRAIGHT_1, "IDLE -> REVERSE_STRAIGHT_1")
        elif self.state == ParkingState.REVERSE_STRAIGHT_1:
            self._reverse_straight_1(dist_step, dist_smoothed)
        elif self.state == ParkingState.STEER_INTO_SLOT:
            self._steer_in_phase(dist_step)
        elif self.state == ParkingState.REVERSE_STRAIGHT_2:
            self._reverse_straight_2(dist_step)
        elif self.state == ParkingState.COUNTERSTEER:
            self._countersteer_phase(dist_step)
        elif self.state == ParkingState.FORWARD_STRAIGHT:
            self._forward_straight_phase(dist_step)
        elif self.state == ParkingState.STOPPED:
            self._stop()

    def _check_relevant_obstacles(self, sensor_indices: List[int]) -> Optional[Dict[str, Any]]:
        """
        Check the specified obstacle detectors and return the closest obstacle detected.
        """
        closest_obs = None
        for idx in sensor_indices:
            detector = self.obstacle_detectors[idx]
            obs = detector.get_closest_obstacle()
            if obs is not None:
                logging.info(f"detector {idx} distance {obs['distance']}")
                if closest_obs is None or obs['distance'] < closest_obs['distance']:
                    closest_obs = obs
        return closest_obs

    def _update_sensor_buffer(self, dist_right: float, dist_back: float) -> None:
        """
        Update the sensor buffer with the appropriate reading based on current state.
        """
        if self.state == ParkingState.REVERSE_STRAIGHT_1 and dist_right > 0.0:
            self.sensor_buffer.add(dist_right)
        elif self.state == ParkingState.REVERSE_STRAIGHT_2 and dist_back > 0.0:
            self.sensor_buffer.add(dist_back)

    def _reset_phase(self, reset_consec: bool = True) -> None:
        """Reset phase-specific tracking variables."""
        self.distance_on_arc = 0.0
        self.prev_location = None
        self.sensor_buffer.clear()
        if reset_consec:
            self.consec_count = 0
        self.realign_count = 0

    def _transition_to_state(self, new_state: ParkingState, message: str, reset_phase: bool = True) -> None:
        """Handle state transitions with logging and resetting phase variables."""
        logging.info(f"[ParkingController] Transition: {message}")
        self.state = new_state
        if reset_phase:
            self._reset_phase()

    # ------------------ State Machine Methods ------------------

    def _reverse_straight_1(self, dist_step: float, dist_smoothed: float) -> None:
        """
        Reverse straight until the smoothed sensor distance exceeds the threshold
        for the required number of consecutive frames.
        """
        self.vehicle_control.reverse_straight()
        if dist_smoothed > self.dist_begin_steer:
            self.consec_count += 1
            if self.consec_count >= self.required_frames:
                self._transition_to_state(
                    ParkingState.STEER_INTO_SLOT,
                    "REVERSE_STRAIGHT_1 -> STEER_INTO_SLOT (Arc1 threshold met)"
                )
                return
        else:
            self.consec_count = 0
        self.distance_on_arc += dist_step

    def _steer_in_phase(self, dist_step: float) -> None:
        """Execute the first steering arc in reverse (Arc1)."""
        self.vehicle_control.reverse_steer_right()
        self.distance_on_arc += dist_step
        if self.distance_on_arc >= self.arc1_length:
            self._transition_to_state(
                ParkingState.REVERSE_STRAIGHT_2,
                "STEER_INTO_SLOT -> REVERSE_STRAIGHT_2 (Arc1 completed)"
            )

    def _reverse_straight_2(self, dist_step: float) -> None:
        """
        Reverse straight until a fixed distance is reached or an obstacle is detected.
        """
        self.vehicle_control.reverse_straight()
        self.distance_on_arc += dist_step

        if self.distance_on_arc >= self.reverse_straight_length:
            self._transition_to_state(
                ParkingState.COUNTERSTEER,
                "REVERSE_STRAIGHT_2 -> COUNTERSTEER (Distance reached)"
            )

    def _countersteer_phase(self, dist_step: float) -> None:
        """Execute the countersteering phase (Arc2) in reverse."""
        self.vehicle_control.counter_steer()
        self.distance_on_arc += dist_step
        if self.distance_on_arc >= self.arc2_length:
            self._transition_to_state(
                ParkingState.FORWARD_STRAIGHT,
                "COUNTERSTEER -> FORWARD_STRAIGHT (Arc2 completed)"
            )

    def _forward_straight_phase(self, dist_step: float) -> None:
        """
        Move forward to finalize parking. Applies a proportional correction
        based on the yaw error if it exceeds the configured threshold.
        """
        current_yaw = self.ego_vehicle.get_transform().rotation.yaw
        yaw_error = normalize_angle(self.desired_yaw - current_yaw)

        if abs(yaw_error) > self.yaw_threshold:
            raw_correction = yaw_error * self.conversion_factor
            correction = max(-self.max_correction, min(self.max_correction, raw_correction))
            logging.info(f"[ParkingController] Realigning: yaw_error={yaw_error:.2f}, correction={correction:.2f}")
            self.vehicle_control.drive(
                throttle=self.forward_throttle,
                steer=correction,
                reverse=False
            )
        else:
            self.vehicle_control.forward_straight()

        # Update distance traveled during forward phase.
        self.distance_on_arc += dist_step
        if self.distance_on_arc >= self.forward_min_distance:
            self._transition_to_state(
                ParkingState.STOPPED,
                "FORWARD_STRAIGHT -> STOPPED (Distance reached)"
            )

    def _stop(self) -> None:
        """Stop the vehicle immediately."""
        self.vehicle_control.stop()
