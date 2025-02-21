import logging
import carla
from typing import Optional, Dict, Any

from src.controller.parking_state import ParkingState
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

    def __init__(self, ego_vehicle: carla.Vehicle, lidar_sensors, config: Dict[str, Any]) -> None:
        self.ego_vehicle = ego_vehicle
        # Convert list of lidar sensors to dictionary keyed by sensor name.
        self.lidars = {sensor.name: sensor for sensor in lidar_sensors}
        self.config = config
        self.vehicle_control = VehicleController(ego_vehicle)

        # Initialize state and frame tracking
        self.state: ParkingState = ParkingState.IDLE
        self.frame_count: int = 0
        self.fps: int = config.get('fps')

        # Phase distance thresholds (in meters)
        self.dist_begin_steer: float = config.get('distance_to_begin_steering')
        self.arc1_length: float = config.get('arc1_length')
        self.reverse_straight_length: float = config.get('reverse_straight_length')
        self.arc2_length: float = config.get('arc2_length')
        self.min_back_distance: float = config.get('min_back_distance')
        self.forward_min_distance: float = config.get('forward_min_distance')

        # Smoothing & hysteresis
        self.required_frames: int = config.get('required_consecutive_frames')
        self.sensor_buffer = SensorBuffer(max_size=config.get('lidar_buffer_size'))
        self.consec_count: int = 0

        # Realignment parameters
        self.yaw_threshold: float = config.get('yaw_threshold')
        self.conversion_factor: float = config.get('conversion_factor')
        self.max_correction: float = config.get('max_correction')
        self.desired_yaw: float = config.get('desired_yaw')
        self.forward_throttle: float = config.get('forward_throttle')

        # Tracking variables for distance and corrections
        self.distance_on_arc: float = 0.0
        self.prev_location: Optional[carla.Location] = None
        self.realign_count: int = 0

    def update(self) -> None:
        """Update the parking state machine for the current frame."""
        self.frame_count += 1

        # Update vehicle travel distance since last frame.
        current_loc = self.ego_vehicle.get_transform().location
        if self.prev_location is None:
            self.prev_location = current_loc
        dist_step = euclid_dist(current_loc, self.prev_location)
        self.prev_location = current_loc

        # Retrieve LiDAR sensor distances (default to 0.0 if no value)
        dist_right = self.lidars.get("rear_right").get_min_distance_right() or 0.0
        dist_back = self.lidars.get("rear_right").get_min_distance_back() or 0.0

        # Update the sensor buffer based on current state.
        self._update_sensor_buffer(dist_right, dist_back)
        dist_smoothed = self.sensor_buffer.median() or 0.0

        # Log sensor and state information periodically.
        if self.frame_count % self.fps == 0:
            sensor_used = "rear_right (right)" if self.state == ParkingState.REVERSE_STRAIGHT_1 else (
                "rear_right (back)" if self.state == ParkingState.REVERSE_STRAIGHT_2 else "N/A"
            )
            logging.info(
                f"[ParkingController] | State: {self.state.name} | Step: {dist_step:.2f} m | "
                f"Right: {dist_right:.2f} m | Back: {dist_back:.2f} m | "
                f"Smoothed ({sensor_used}): {dist_smoothed:.2f} m"
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
        back_dist = self.lidars.get("rear_right").get_min_distance_right() or 0.0
        if 0.0 < back_dist < self.min_back_distance:
            self._transition_to_state(
                ParkingState.COUNTERSTEER,
                f"Obstacle detected behind ({back_dist:.2f} m) -> COUNTERSTEER"
            )
            return
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

        front_dist = self.lidars.get("front_center").get_min_distance_front()
        if front_dist is not None and front_dist < self.forward_min_distance:
            self._transition_to_state(
                ParkingState.STOPPED,
                f"Obstacle detected ahead ({front_dist:.2f} m) -> STOPPED"
            )
            return

        self.distance_on_arc += dist_step
        if self.distance_on_arc >= self.forward_min_distance:
            self._transition_to_state(
                ParkingState.STOPPED,
                "FORWARD_STRAIGHT -> STOPPED (Distance reached)"
            )

    def _stop(self) -> None:
        """Stop the vehicle immediately."""
        self.vehicle_control.stop()
