# collision.py - Collision sensor manager.

# -------------------------------------------------------------------------
# CollisionSensorManager
# -------------------------------------------------------------------------
import logging
from typing import Any, Optional

import carla

class CollisionSensorManager:
    """
    Manages a collision sensor attached to an ego vehicle.
    """
    def __init__(self, world: carla.World, bp_lib: Any, ego_vehicle: carla.Actor) -> None:
        self.world = world
        self.bp_lib = bp_lib
        self.ego_vehicle = ego_vehicle
        self.sensor: Optional[carla.Actor] = None
        self.has_collided = False

    def spawn_collision_sensor(self) -> None:
        if not self.ego_vehicle:
            raise ValueError("[CollisionSensorManager] ERROR: Ego vehicle is None.")
        col_bp = self.bp_lib.find('sensor.other.collision')
        self.sensor = self.world.spawn_actor(col_bp, carla.Transform(
            carla.Location(0, 0, 0),
            carla.Rotation(0, 0, 0)), attach_to=self.ego_vehicle)
        if not self.sensor:
            raise RuntimeError("[CollisionSensorManager] ERROR: cannot spawn collision sensor.")
        self.sensor.listen(self._collision_callback)
        logging.info("[CollisionSensorManager] Collision sensor spawned.")

    def _collision_callback(self, event: Any) -> None:
        self.has_collided = True
        logging.info(f"[CollisionSensorManager] Collision detected: {event}")

    def collided(self) -> bool:
        return self.has_collided
