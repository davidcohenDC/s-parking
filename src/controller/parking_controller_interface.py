from abc import ABC, abstractmethod
from typing import Dict, Any

import carla
from src.controller.vehicle_controller import VehicleController
from src.utils.sensor_buffer import SensorBuffer
from src.sensors.obstacle_detector import ObstacleDetector
from typing import List


class ParkingControllerInterface(ABC):
    """
    Interface defining the methods that parking states need to interact with
    the controller.  This avoids circular dependencies.
    """

    @property
    @abstractmethod
    def ego_vehicle(self) -> carla.Vehicle:
        pass

    @property
    @abstractmethod
    def vehicle_control(self) -> VehicleController:
        pass

    @property
    @abstractmethod
    def config(self) -> Dict[str, Any]:  # or better ParkingConfig
        pass

    @property
    @abstractmethod
    def lidar(self):
        pass

    @property
    @abstractmethod
    def obstacle_detectors(self) -> List[ObstacleDetector]:
        pass

    @property
    @abstractmethod
    def sensor_buffer(self) -> SensorBuffer:
        pass

    @abstractmethod
    def get_relevant_distance(self) -> float:
        pass

    @abstractmethod
    def get_relevant_obstacle_detectors(self) -> List[ObstacleDetector]:
        pass

    @abstractmethod
    def _check_for_obstacles(self) -> bool:
        pass