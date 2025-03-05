# src/controller/parking_controller.py

import carla
import logging
from typing import Optional, Dict, Any, List

from src.sensors.lidar_sensor import LidarSensorManager
from src.sensors.obstacle_detector import ObstacleDetector
from src.utils.distance import euclid_dist
from src.controller.vehicle_controller import VehicleController
from src.controller.parking_controller_interface import ParkingControllerInterface
from src.controller.behaviour_states import ParkingState, ReverseAligningState, SteerIntoSlotState, \
    ReverseStraightState, CountersteerState, ForwardStraightState, StoppedState, ParkingStateBase
from src.parking_config import ParkingConfig


class ParkingController(ParkingControllerInterface):
    def __init__(self, ego_vehicle: carla.Vehicle, lidar: LidarSensorManager, obstacle_detectors: List[ObstacleDetector],
                 config: Dict[str, Any]):
        self._ego_vehicle = ego_vehicle
        self._lidar = lidar
        self._obstacle_detectors = obstacle_detectors
        self._config = ParkingConfig(config)  # Use your ParkingConfig class
        self._vehicle_control = VehicleController(ego_vehicle)
        self.frame_count = 0
        self.obs_fps_check = config.get('OBSTACLE_NUM_FPS_CHECK')

        # State management
        self.state_map: Dict[ParkingState, ParkingStateBase] = {
            ParkingState.REVERSE_ALIGNING: ReverseAligningState(self),
            ParkingState.STEER_INTO_SLOT: SteerIntoSlotState(self),
            ParkingState.REVERSE_STRAIGHT: ReverseStraightState(self),
            ParkingState.COUNTERSTEER: CountersteerState(self),
            ParkingState.FORWARD_STRAIGHT: ForwardStraightState(self),
            ParkingState.STOPPED: StoppedState(self),
        }
        self.current_state: ParkingStateBase = self.state_map[ParkingState.REVERSE_ALIGNING]
        self.current_state.enter_state()

        self.min_obstacle_distance = config.get('OBSTACLE_MIN_DISTANCE')
        self.parking_forward_min_dist = config.get('PARKING_FORWARD_MIN_DIST')

    @property
    def ego_vehicle(self) -> carla.Vehicle:
        return self._ego_vehicle

    @property
    def vehicle_control(self) -> VehicleController:
        return self._vehicle_control

    @property
    def config(self) -> ParkingConfig:
        return self._config

    @property
    def lidar(self):
        return self._lidar

    @property
    def obstacle_detectors(self) -> List[ObstacleDetector]:
        return self._obstacle_detectors

    @property
    def sensor_buffer(self): # Now unused
        return None

    def update(self) -> None:
        """Updates the parking controller, delegating to the current state."""

        self.frame_count += 1

        # Obstacle check *first* (using guard clause)
        if self.frame_count % self.obs_fps_check == 0:  # Check at the defined FPS
            if self._check_for_obstacles():
                return  # Exit immediately if obstacle detected

        # Calculate distance step
        current_location = self._ego_vehicle.get_transform().location
        dist_step = 0.0
        if self.current_state.prev_location is not None:
            dist_step = euclid_dist(current_location, self.current_state.prev_location)
        self.current_state.prev_location = current_location

        # Delegate to the current state
        new_state = self.current_state.update(dist_step)

        # Handle state transition (if needed)
        if new_state is not None:
            self._transition_to_state(new_state)

    def _transition_to_state(self, new_state: ParkingState) -> None:
        """Handles state transitions."""
        logging.info(f"[ParkingController] Transition: {type(self.current_state).__name__} -> {new_state.name}")
        self.current_state.exit_state()
        self.current_state = self.state_map[new_state]
        self.current_state.enter_state()


    def _check_for_obstacles(self) -> bool:
        """Checks for obstacles using relevant sensors based on the current state."""
        detectors = self.get_relevant_obstacle_detectors()
        if not detectors:
            return False

        closest_obs = self._check_relevant_obstacles(detectors)
        threshold = self.parking_forward_min_dist if isinstance(self.current_state,
                                                                ForwardStraightState) else self.min_obstacle_distance

        if closest_obs is not None and 0.0 < closest_obs['distance'] < threshold:
            logging.warning(
                f"[ParkingController] Obstacle {closest_obs['other_actor']} detected at {closest_obs['distance']:.2f} m -> STOPPED")
            self._transition_to_state(ParkingState.STOPPED)
            self.vehicle_control.stop()
            return True  # Indicate obstacle detected
        return False

    @staticmethod
    def _check_relevant_obstacles(detectors: List[ObstacleDetector]) -> Optional[Dict[str, Any]]:
        """Check the specified obstacle detectors and return the closest obstacle."""
        closest_obs = None
        for detector in detectors:
            obs = detector.get_closest_obstacle()
            if obs is not None:
                logging.debug(f"Detector {id(detector)}: Distance = {obs['distance']:.2f}m")
                if closest_obs is None or obs['distance'] < closest_obs['distance']:
                    closest_obs = obs
        return closest_obs
    def get_relevant_obstacle_detectors(self) -> List[ObstacleDetector]:
        """Returns the relevant obstacle detectors based on the current state."""
        if isinstance(self.current_state,
                      (ReverseAligningState, SteerIntoSlotState, ReverseStraightState, CountersteerState)):
            return [self._obstacle_detectors[i] for i in [3, 4, 5]]  # Rear sensors
        elif isinstance(self.current_state, ForwardStraightState):
            return [self._obstacle_detectors[1]]  # Front center sensor
        else:
            return []

    def get_relevant_distance(self) -> float:
        """Gets the relevant distance reading based on the current state."""
        dist_right = self.lidar.get_min_distance_right() or 0.0
        dist_back = self.lidar.get_min_distance_back() or 0.0

        if isinstance(self.current_state, ReverseAligningState) and dist_right > 0.0:
            return dist_right
        elif isinstance(self.current_state, ReverseStraightState) and dist_back > 0.0:
            return dist_back
        return 0.0