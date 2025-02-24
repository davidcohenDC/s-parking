import logging
import statistics
import weakref
from collections import deque
from typing import Optional

import carla

class ObstacleDetector:
    def __init__(self,
                 parent_actor: carla.Actor,
                 transform: carla.Transform,
                 config: dict
                 ) -> None:
        self.parent_actor = parent_actor
        self.transform = transform
        self.config = config
        self.history = deque(maxlen=self.config['OBSTACLE_MAX_HISTORY'])
        world = self.parent_actor.get_world()
        bp = world.get_blueprint_library().find('sensor.other.obstacle')

        bp.set_attribute('distance', str(self.config['OBSTACLE_DETECTION_DISTANCE']))
        bp.set_attribute('hit_radius', str(self.config['OBSTACLE_HIT_RADIUS']))
        bp.set_attribute('only_dynamics', str(self.config['OBSTACLE_ONLY_DYNAMICS']))
        bp.set_attribute('sensor_tick', str(self.config['OBSTACLE_SENSOR_TICK']))

        self.sensor = world.spawn_actor(bp, self.transform, attach_to=self.parent_actor)
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: ObstacleDetector._on_obstacle(weak_self, event))


    @classmethod
    def _on_obstacle(cls, weak_self, event) -> None:
        """
        Callback invoked when an obstacle is detected.

        The event (of type carla.ObstacleDetectionEvent) contains:
            - frame: int, the frame number.
            - timestamp: double, simulation time in seconds.
            - transform: carla.Transform, sensor’s world location/rotation.
            - actor: carla.Actor, the parent actor.
            - other_actor: carla.Actor, the detected obstacle.
            - distance: float, distance from the actor to the obstacle.
        """
        self = weak_self()
        if not self:
            return

        # # Retrieve a simple name for the detected obstacle.
        try:
            actor_name = event.other_actor.type_id
        except RuntimeError:
            actor_name = 'unknown'

        # avoid static.road and road
        if actor_name == 'static.road' or actor_name == 'road':
            return

        logging.info(f"[Obstacle Detector] Detected {actor_name} at distance {event.distance:.2f} m.")

        # Append the event details to the history.
        self.history.append({
            'frame': event.frame,
            'timestamp': event.timestamp,
            'transform': event.transform,
            'actor': event.actor,
            'other_actor': event.other_actor,
            'distance': event.distance
        })

    def get_history(self) -> list:
        """
        Returns a copy of the obstacle detection events history.
        """
        return list(self.history)

    def reset_history(self) -> None:
        """
        Clears the obstacle detection events history.
        """
        self.history.clear()

    def get_closest_obstacle(self) -> Optional[dict]:
        """
        Returns the obstacle detection event with the smallest distance,
        filtering out outlier events that are significantly lower than the median.
        Returns None if no events have been recorded.
        """
        if not self.history:
            return None

        # Take a snapshot of the history
        history_list = list(self.history)
        distances = [event['distance'] for event in history_list]

        # Calculate the median distance from the history
        median_distance = statistics.median(distances)

        # Use a configurable ratio to filter out potential outliers.
        # For example, if OBSTACLE_OUTLIER_RATIO=0.8 then events with a distance less than
        # 0.8 * median_distance will be considered outliers.
        outlier_ratio = self.config.get('OBSTACLE_OUTLIER_RATIO', 0.1)
        filtered_events = [e for e in history_list if e['distance'] >= median_distance * outlier_ratio]

        # If filtering removed all events (unlikely), fall back to using the full history.
        if not filtered_events:
            filtered_events = history_list

        # Return the event with the smallest distance among the filtered events.
        return min(filtered_events, key=lambda event: event['distance'])

    def get_closest_obstacle_distance(self) -> Optional[float]:
        """
        Returns the distance to the closest obstacle.
        Returns None if no events have been recorded.
        """
        closest_obstacle = self.get_closest_obstacle()
        if closest_obstacle is None:
            return None
        return closest_obstacle['distance']

    def destroy(self) -> None:
        """
        Stops and destroys the obstacle sensor actor.
        """
        if self.sensor is not None and self.sensor.is_alive:
            self.sensor.stop()
            self.sensor.destroy()
            self.sensor = None

    def __del__(self) -> None:
        """
        Ensures that the sensor is destroyed when the ObstacleDetector object is garbage collected.
        """
        self.destroy()