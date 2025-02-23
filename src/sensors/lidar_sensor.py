import logging
from typing import Any, Optional

import carla
import numpy as np

from src.utils.distance import compute_2d_distance

class LidarSensorManager:
    """
    Wraps a LiDAR sensor and provides utility functions to process its point cloud.
    """
    def __init__(
        self,
        world: carla.World,
        bp_lib: Any,
        ego_vehicle: carla.Actor,
        config: dict,
        transform: carla.Transform,
        name: str = "main"
    ) -> None:
        self.world = world
        self.bp_lib = bp_lib
        self.ego_vehicle = ego_vehicle
        self.config = config
        self.transform = transform
        self.name = name
        self.sensor_actor: Optional[carla.Actor] = None
        self._current_points: Optional[np.ndarray] = None

    def spawn(self) -> None:
        if not self.ego_vehicle:
            raise ValueError(f"[LidarSensorManager-{self.name}] ERROR: Ego vehicle is None.")
        lidar_bp = self.bp_lib.find('sensor.lidar.ray_cast')
        # Set sensor attributes from configuration
        lidar_bp.set_attribute('range', str(self.config['LIDAR_RANGE']))
        lidar_bp.set_attribute('rotation_frequency', str(self.config['LIDAR_ROTATION_FREQUENCY']))
        lidar_bp.set_attribute('channels', str(self.config['LIDAR_CHANNELS']))
        lidar_bp.set_attribute('points_per_second', str(self.config['LIDAR_PPS']))
        lidar_bp.set_attribute('sensor_tick', str(self.config['LIDAR_SENSOR_TICK']))
        lidar_bp.set_attribute('dropoff_general_rate', str(self.config['LIDAR_DROPOFF_GENERAL_RATE']))

        self.sensor_actor = self.world.spawn_actor(lidar_bp, self.transform, attach_to=self.ego_vehicle)
        if not self.sensor_actor:
            raise RuntimeError(f"[LidarSensorManager-{self.name}] ERROR: cannot spawn LiDAR sensor.")
        self.sensor_actor.listen(self._on_lidar_data)
        logging.info(f"[LidarSensorManager-{self.name}] LiDAR spawned and listening.")

    def _on_lidar_data(self, lidar_data: Any) -> None:
        # Convert raw data into an (N,4) NumPy array and store a copy to avoid race conditions.
        self._current_points = np.frombuffer(lidar_data.raw_data, dtype=np.float32).reshape((-1, 4)).copy()

    def get_current_points(self) -> Optional[np.ndarray]:
        """Return a copy of the raw (x,y,z,intensity) points from the last LiDAR frame."""
        return self._current_points.copy() if self._current_points is not None else None

    def destroy(self) -> None:
        if self.sensor_actor and self.sensor_actor.is_alive:
            self.sensor_actor.stop()
            self.sensor_actor.destroy()
            logging.info(f"[LidarSensorManager-{self.name}] LiDAR destroyed.")

    # --- Helper methods to reduce repetition ---

    def _filter_points(self, mask: np.ndarray) -> Optional[np.ndarray]:
        """
        Apply the given mask on a copy of the current points and return the filtered array,
        or None if no points remain.
        """
        if self._current_points is None or len(self._current_points) == 0:
            return None
        pts = self._current_points.copy()
        if mask.shape[0] != pts.shape[0]:
            logging.warning(f"[LidarSensorManager-{self.name}] Mask shape {mask.shape} does not match points shape {pts.shape}.")
            return None
        filtered = pts[mask]
        return filtered if filtered.size > 0 else None

    # --- Distance query methods ---

    def get_min_distance_right(
        self, x_tolerance: float = 0.2, max_dist: float = 2.0, min_z: float = -10.0, max_z: float = 2.0
    ) -> Optional[float]:
        """
        Returns the minimum lateral (Y) distance on the right side.
        """
        if self._current_points is None:
            return None
        pts = self._current_points.copy()
        mask = (
            (np.abs(pts[:, 0]) < x_tolerance) &
            (pts[:, 1] > 0) &
            (compute_2d_distance(pts) < max_dist) &
            (pts[:, 2] > min_z) & (pts[:, 2] < max_z)
        )
        filtered = self._filter_points(mask)
        return float(np.min(filtered[:, 1])) if filtered is not None else None

    def get_min_distance_front(
        self, y_tolerance: float = 1.0, max_dist: float = 3.0, min_z: float = 0.0, max_z: float = 5.0
    ) -> Optional[float]:
        """
        Returns the minimum distance to an obstacle in front (minimum X value).
        """
        if self._current_points is None:
            return None
        pts = self._current_points.copy()
        mask = (
            (pts[:, 0] > 0) &
            (np.abs(pts[:, 1]) < y_tolerance) &
            (compute_2d_distance(pts) < max_dist) &
            (pts[:, 2] > min_z) & (pts[:, 2] < max_z)
        )
        filtered = self._filter_points(mask)
        return float(np.min(filtered[:, 0])) if filtered is not None else None

    def get_min_distance_back(
        self, y_tolerance: float = 0.2, max_dist: float = 3.0, min_z: float = 0.0, max_z: float = 1.0
    ) -> Optional[float]:
        """
        Returns the minimum distance to an obstacle behind (minimum X value).
        """
        if self._current_points is None:
            return None
        pts = self._current_points.copy()
        mask = (
            (pts[:, 0] < 0) &
            (np.abs(pts[:, 1]) < y_tolerance) &
            (compute_2d_distance(pts) < max_dist) &
            (pts[:, 2] > min_z) & (pts[:, 2] < max_z)
        )
        filtered = self._filter_points(mask)
        return float(np.min(filtered[:, 0])) if filtered is not None else None
