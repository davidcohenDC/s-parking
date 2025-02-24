import logging
from typing import Any, Optional, List, Dict

import carla
import numpy as np

from src.utils.distance import compute_2d_distance
from collections import deque  # Import deque


class LidarSensorManager:
    """
    Wraps a LiDAR sensor, provides utility functions to process its point cloud,
    and manages a buffer of recent distance measurements.
    """

    def __init__(
        self,
        world: carla.World,
        bp_lib: Any,
        ego_vehicle: carla.Actor,
        config: dict,
        transform: carla.Transform,
        name: str = "main",
    ) -> None:
        self.world = world
        self.bp_lib = bp_lib
        self.ego_vehicle = ego_vehicle
        self.config = config
        self.transform = transform
        self.name = name
        self.sensor_actor: Optional[carla.Actor] = None
        self._current_points: Optional[np.ndarray] = None

        # Buffering
        self.min_distance_history: deque[Dict[str, float]] = deque(maxlen=self.config.get('LIDAR_BUFFER_SIZE', 10))  # Default to 10 if not in config
        self.min_distance_right_history: deque[float] = deque(maxlen=self.config.get('LIDAR_BUFFER_SIZE', 10))
        self.min_distance_back_history: deque[float] = deque(maxlen=self.config.get('LIDAR_BUFFER_SIZE', 10))

    def spawn(self) -> None:
        # ... (rest of your spawn method, no changes needed here) ...
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
        # Convert raw data and store it.
        self._current_points = np.frombuffer(lidar_data.raw_data, dtype=np.float32).reshape((-1, 4)).copy()

        # Calculate and store minimum distances.  This happens *every time* new data arrives.
        right_dist = self._calculate_min_distance_right()
        back_dist = self._calculate_min_distance_back()

        if right_dist is not None:
          self.min_distance_right_history.append(right_dist)
        if back_dist is not None:
          self.min_distance_back_history.append(back_dist)

        # Storing current frame data for debugging purposes
        current_frame_data = {
            'timestamp': lidar_data.timestamp,
            'frame': lidar_data.frame,
            'transform': lidar_data.transform,
            'min_distance_right': right_dist,
            'min_distance_back': back_dist,
        }
        self.min_distance_history.append(current_frame_data)

    def get_current_points(self) -> Optional[np.ndarray]:
        return self._current_points.copy() if self._current_points is not None else None

    def destroy(self) -> None:
        # ... (rest of your destroy method, no changes needed here) ...
        if self.sensor_actor and self.sensor_actor.is_alive:
            self.sensor_actor.stop()
            self.sensor_actor.destroy()
            logging.info(f"[LidarSensorManager-{self.name}] LiDAR destroyed.")

    def _filter_points(self, mask: np.ndarray) -> Optional[np.ndarray]:
        # ... (no changes needed here) ...
        if self._current_points is None or len(self._current_points) == 0:
            return None
        pts = self._current_points.copy()
        if mask.shape[0] != pts.shape[0]:
            logging.warning(
                f"[LidarSensorManager-{self.name}] Mask shape {mask.shape} does not match points shape {pts.shape}.")
            return None
        filtered = pts[mask]
        return filtered if filtered.size > 0 else None
    # --- Internal Calculation Methods (Private) ---

    def _calculate_min_distance_right(
        self, x_tolerance: float = 0.2, max_dist: float = 2.0, min_z: float = -10.0, max_z: float = 2.0
    ) -> Optional[float]:
        """Calculates the minimum distance to the right (internal use)."""
        if self._current_points is None:
            return None
        pts = self._current_points
        mask = (
            (np.abs(pts[:, 0]) < x_tolerance)
            & (pts[:, 1] > 0)
            & (compute_2d_distance(pts) < max_dist)
            & (pts[:, 2] > min_z)
            & (pts[:, 2] < max_z)
        )
        filtered = self._filter_points(mask)
        return float(np.min(filtered[:, 1])) if filtered is not None else None

    def _calculate_min_distance_front(
        self, y_tolerance: float = 1.0, max_dist: float = 3.0, min_z: float = 0.0, max_z: float = 5.0
    ) -> Optional[float]:
        """Calculates the minimum distance to the front (internal use)."""
        if self._current_points is None:
            return None
        pts = self._current_points
        mask = (
            (pts[:, 0] > 0)
            & (np.abs(pts[:, 1]) < y_tolerance)
            & (compute_2d_distance(pts) < max_dist)
            & (pts[:, 2] > min_z)
            & (pts[:, 2] < max_z)
        )
        filtered = self._filter_points(mask)
        return float(np.min(filtered[:, 0])) if filtered is not None else None

    def _calculate_min_distance_back(
        self, y_tolerance: float = 0.2, max_dist: float = 3.0, min_z: float = 0.0, max_z: float = 1.0
    ) -> Optional[float]:
        """Calculates the minimum distance to the back (internal use)."""
        if self._current_points is None:
            return None
        pts = self._current_points
        mask = (
            (pts[:, 0] < 0)
            & (np.abs(pts[:, 1]) < y_tolerance)
            & (compute_2d_distance(pts) < max_dist)
            & (pts[:, 2] > min_z)
            & (pts[:, 2] < max_z)
        )
        filtered = self._filter_points(mask)
        return float(np.min(filtered[:, 0])) if filtered is not None else None

    # --- Public API for Accessing Buffered Data ---
    def get_history(self) -> List[Dict[str, float]]:
        return list(self.min_distance_history)

    def get_min_distance_right(self) -> Optional[float]:
        """Returns the median of the minimum distances to the right."""
        return np.median(self.min_distance_right_history) if self.min_distance_right_history else None

    def get_min_distance_back(self) -> Optional[float]:
        """Returns the median of the minimum distances to the back."""
        return np.median(self.min_distance_back_history) if self.min_distance_back_history else None

    def get_min_distance_front(self) -> Optional[float]:
        """
        Returns the minimum distance to an obstacle in front (minimum X value).
        There is not a history for front value.
        """
        return self._calculate_min_distance_front()