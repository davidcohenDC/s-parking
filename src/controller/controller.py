import logging
import carla
from typing import Optional

from src.controller.parking_state import ParkingState
from src.utils.distance import euclid_dist
from src.controller.vehicle_controller import VehicleController
from src.utils.sensor_buffer import SensorBuffer


class ParkingController:
    """
    Manages the parking state machine by processing sensor data,
    applying vehicle controls, and transitioning through various parking phases.
    """
    def __init__(self, ego_vehicle: carla.Vehicle, lidar_sensors, config: dict) -> None:
        self.ego_vehicle = ego_vehicle
        # Convert list of lidar sensors to dictionary keyed by sensor name.
        self.lidars = {s.name: s for s in lidar_sensors}
        self.config = config
        self.vehicle_control = VehicleController(ego_vehicle)

        # Initialize state
        self.state: ParkingState = ParkingState.IDLE
        self.frame_count: int = 0

        # Distances for each phase (in meters)
        self.dist_begin_steer: float = config['distance_to_begin_steering']
        self.arc1_length: float = config['arc1_length']
        self.reverse_straight_length: float = config['reverse_straight_length']
        self.arc2_length: float = config['arc2_length']
        self.min_back_distance: float = config['min_back_distance']
        self.forward_min_distance: float = config['forward_min_distance']

        # Smoothing & hysteresis
        self.required_frames: int = config['required_consecutive_frames']
        self.sensor_buffer = SensorBuffer(max_size=config['lidar_buffer_size'])
        self.consec_count: int = 0

        # Track distance traveled in current phase
        self.distance_on_arc: float = 0.0
        self.prev_location: Optional[carla.Location] = None

    def update(self) -> None:
        """Update the parking state machine for the current frame."""
        self.frame_count += 1

        # Calculate distance traveled since last update
        current_loc = self.ego_vehicle.get_transform().location
        if self.prev_location is None:
            self.prev_location = current_loc
        dist_step = euclid_dist(current_loc, self.prev_location)
        self.prev_location = current_loc

        # Retrieve current LiDAR distances (defaulting to 0.0 if None)
        dist_right = self.lidars["rear_right"].get_min_distance_right() or 0.0
        dist_back = self.lidars["rear_right"].get_min_distance_back() or 0.0

        # Update sensor buffer based on the current state
        self._update_sensor_buffer(dist_right, dist_back)
        dist_smoothed = self.sensor_buffer.median() or 0.0

        # Log state and sensor readings every 10 frames
        if self.frame_count % 20 == 0:
            # Determine which sensor reading is used for smoothing based on state
            sensor_used = "rear_right (right)" if self.state == ParkingState.REVERSE_STRAIGHT_1 else (
                "rear_right (back)" if self.state == ParkingState.REVERSE_STRAIGHT_2 else "N/A"
            )
            # Log current state and sensor readings
            logging.info(
                f"[ParkingController] | State: {self.state.name} | "
                f"Step: {dist_step:.2f} m | Right: {dist_right:.2f} m | "
                f"Back: {dist_back:.2f} m | Smoothed ({sensor_used}): {dist_smoothed:.2f} m"
            )

        # State machine transitions
        if self.state == ParkingState.IDLE:
            self._start_reverse_straight_1()
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
        Update the sensor buffer with the appropriate sensor reading
        based on the current state.
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

    # ------------------ State Machine Methods ------------------

    def _start_reverse_straight_1(self) -> None:
        logging.info("[ParkingController] Transition: IDLE -> REVERSE_STRAIGHT_1")
        self.state = ParkingState.REVERSE_STRAIGHT_1
        self._reset_phase()

    def _reverse_straight_1(self, dist_step: float, dist_smoothed: float) -> None:
        """
        Reverse straight until the smoothed sensor distance exceeds the threshold
        for the required number of consecutive frames.
        """
        self.vehicle_control.reverse_straight()
        if dist_smoothed > self.dist_begin_steer:
            self.consec_count += 1
            if self.consec_count >= self.required_frames:
                logging.info("[ParkingController] Transition: REVERSE_STRAIGHT_1 -> STEER_INTO_SLOT (Arc1)")
                self.state = ParkingState.STEER_INTO_SLOT
                self._reset_phase()
                return
        else:
            self.consec_count = 0
        self.distance_on_arc += dist_step

    def _steer_in_phase(self, dist_step: float) -> None:
        """Execute the first steering arc in reverse (Arc1)."""
        self.vehicle_control.reverse_steer_right()
        self.distance_on_arc += dist_step
        if self.distance_on_arc >= self.arc1_length:
            logging.info("[ParkingController] Transition: STEER_INTO_SLOT -> REVERSE_STRAIGHT_2 (Arc1 completed)")
            self.state = ParkingState.REVERSE_STRAIGHT_2
            self._reset_phase()

    def _reverse_straight_2(self, dist_step: float) -> None:
        """
        Reverse straight until a fixed distance is reached or an obstacle is detected.
        """
        self.vehicle_control.reverse_straight()
        self.distance_on_arc += dist_step

        # Using right sensor measurement to detect obstacles behind
        back_dist = self.lidars["rear_right"].get_min_distance_right() or 0.0
        if 0.0 < back_dist < self.min_back_distance:
            logging.warning(f"[ParkingController] Transition: Obstacle detected behind ({back_dist:.2f} m) -> COUNTERSTEER")
            self.state = ParkingState.COUNTERSTEER
            self._reset_phase()
            return

        if self.distance_on_arc >= self.reverse_straight_length:
            logging.info("[ParkingController] Transition: REVERSE_STRAIGHT_2 -> COUNTERSTEER (Distance reached)")
            self.state = ParkingState.COUNTERSTEER
            self._reset_phase()

    def _countersteer_phase(self, dist_step: float) -> None:
        """Execute the countersteering phase (Arc2) in reverse."""
        self.vehicle_control.counter_steer()
        self.distance_on_arc += dist_step
        if self.distance_on_arc >= self.arc2_length:
            logging.info("[ParkingController] Transition: COUNTERSTEER -> FORWARD_STRAIGHT (Arc2 completed)")
            self.state = ParkingState.FORWARD_STRAIGHT
            self._reset_phase()

    def _forward_straight_phase(self, dist_step: float) -> None:
        """
        Move forward to finalize parking.
        """

        # Determine yaw error (desired yaw = 0°)
        current_yaw = self.ego_vehicle.get_transform().rotation.yaw
        yaw_error = current_yaw  # error with respect to 0°
        yaw_threshold = 5.0  # degrees tolerance

        if abs(yaw_error) > yaw_threshold:
            # Proportional correction (tuning gain as necessary)
            correction = yaw_error / 60.0
            logging.info(f"[ParkingController] Realigning: yaw_error={yaw_error:.2f}, correction={correction:.2f}")
            self.vehicle_control.drive(
                throttle=self.config['forward_throttle'],
                steer=correction,
                reverse=False
            )
        else:
            self.vehicle_control.forward_straight()

        front_dist = self.lidars["front_center"].get_min_distance_front()
        if front_dist is not None and front_dist < self.forward_min_distance:
            logging.info(f"[ParkingController] Transition: Obstacle detected ahead ({front_dist:.2f} m) -> STOPPED")
            self.state = ParkingState.STOPPED
            self._reset_phase()
            return

        self.distance_on_arc += dist_step
        if self.distance_on_arc >= 1.0:
            logging.info("[ParkingController] Transition: FORWARD_STRAIGHT -> STOPPED (Distance reached)")
            self.state = ParkingState.STOPPED
            self._reset_phase()

    def _stop(self) -> None:
        """Stop the vehicle immediately."""
        self.vehicle_control.stop()
