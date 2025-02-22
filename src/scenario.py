import time
import logging

import numpy as np
import carla
from typing import Optional, List

from src.sensors.lidar_sensor import LidarSensorManager
from sensors.collision_sensor import CollisionSensorManager


class SParkingScenario:
    """
    Sets up and cleans up the parking scenario, including spawning the ego vehicle,
    obstacles, LiDAR sensors, and a collision sensor.
    """
    def __init__(self, config: dict, client: Optional[carla.Client] = None) -> None:
        self.config = config
        self.client = client or carla.Client(
            config.get("carla_host", "127.0.0.1"),
            config.get("carla_port", 2000)
        )
        self.client.set_timeout(10.0)

        # Load the world using unpacked world map configuration.
        self.world = self.client.load_world(**config['world_map'])
        self.bp_lib = self.world.get_blueprint_library()

        self.ego_spawn = config['ego_spawn']
        self.obstacles_spawns = config['obstacles_spawns']

        # Lists for tracking spawned actors and sensors (for cleanup).
        self.actor_list: List[carla.Actor] = []
        self.lidar_sensor: Optional[LidarSensorManager] = None

        self.collision_sensor: Optional[CollisionSensorManager] = None

        self._set_sunny_weather()

    def _set_sunny_weather(self) -> None:
        """Set the weather to sunny."""
        weather = carla.WeatherParameters(
            cloudiness=0.0,
            precipitation=0.0,
            precipitation_deposits=0.0,
            wind_intensity=0.0,
            sun_altitude_angle=90.0,
            sun_azimuth_angle=30.0,
            fog_density=0.0,
            wetness=0.0
        )
        self.world.set_weather(weather)
        logging.info("[SParkingScenario] Weather set to sunny.")

    def setup_scenario(self) -> None:
        """
        Set up the scenario for the simulation run:
          1) Spawn the ego vehicle and obstacles.
          2) Spawn LiDAR sensors.
          3) Spawn the collision sensor.
          4) Wait for all vehicles to stop.
        """
        self._spawn_ego_and_obstacles()
        self._spawn_lidar_sensor()
        self._spawn_collision_sensor()
        self._wait_for_vehicles_stopped()

    def _spawn_actor(self, blueprint: carla.ActorBlueprint, transform: carla.Transform) -> Optional[carla.Actor]:
        """
        Helper to spawn an actor and add it to the actor list.
        """
        actor = self.world.try_spawn_actor(blueprint, transform)
        if actor is not None:
            self.actor_list.append(actor)
        return actor

    def _spawn_ego_and_obstacles(self) -> None:
        """Spawn the ego vehicle and obstacles."""
        # Spawn ego vehicle.
        ego_bp = self.bp_lib.find('vehicle.audi.tt')
        ego_vehicle = self._spawn_actor(ego_bp, self.ego_spawn)
        if not ego_vehicle:
            logging.error("[SParkingScenario] Unable to spawn ego vehicle.")
            return
        logging.info("[SParkingScenario] Ego vehicle spawned.")

        # Spawn obstacles.
        for transform in self.obstacles_spawns:
            obs_bp = self.bp_lib.find('vehicle.audi.tt')
            obs_actor = self._spawn_actor(obs_bp, transform)
            if obs_actor:
                obs_actor.set_autopilot(False)
                obs_actor.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0))
                logging.info(f"[SParkingScenario] Spawned obstacle at {transform.location}.")
            else:
                logging.warning(f"[SParkingScenario] Failed to spawn obstacle at {transform.location}.")

    def _spawn_lidar_sensor(self) -> None:
        """Spawn and attach multiple LiDAR sensors to the ego vehicle."""
        ego_vehicle = self.get_ego_vehicle()
        if not ego_vehicle:
            logging.error("[SParkingScenario] No ego vehicle to attach LiDAR sensors to.")
            return

        # remove lidar sensor if it already exists
        if self.lidar_sensor:
            self.lidar_sensor.destroy()
            self.lidar_sensor = None

        lidar_transform = self.config['lidar_transform']
        self.lidar_sensor= LidarSensorManager(
            world=self.world,
            bp_lib=self.bp_lib,
            ego_vehicle=ego_vehicle,
            config=self.config,
            transform=lidar_transform,
            name="main"
        )

        self.lidar_sensor.spawn()
        if self.lidar_sensor.sensor_actor:
            self.actor_list.append(self.lidar_sensor.sensor_actor)
            logging.info("[SParkingScenario] LiDAR sensor spawned and attached.")


    def _spawn_collision_sensor(self) -> None:
        """Spawn and attach a collision sensor to the ego vehicle."""
        ego_vehicle = self.get_ego_vehicle()
        if not ego_vehicle:
            logging.error("[SParkingScenario] No ego vehicle for collision sensor.")
            return

        self.collision_sensor = CollisionSensorManager(self.world, self.bp_lib, ego_vehicle)
        self.collision_sensor.spawn_collision_sensor()
        if self.collision_sensor.sensor:
            self.actor_list.append(self.collision_sensor.sensor)
            logging.info("[SParkingScenario] Collision sensor spawned.")

    def _wait_for_vehicles_stopped(self) -> None:
        """
        Wait until all vehicles in the actor list have nearly stopped.
        This ensures that all spawned vehicles have stabilized.
        """
        logging.info("[SParkingScenario] Waiting for vehicles to stop...")
        all_stopped = False
        while not all_stopped:
            all_stopped = True
            for actor in self.actor_list:
                if isinstance(actor, carla.Vehicle):
                    vel = actor.get_velocity()
                    speed = np.sqrt(vel.x**2 + vel.y**2 + vel.z**2)
                    if speed > 0.1:
                        all_stopped = False
                        break
            if not all_stopped:
                time.sleep(0.001)
        logging.info("[SParkingScenario] All vehicles are stopped.")

    def get_ego_vehicle(self) -> Optional[carla.Vehicle]:
        """
        Return the ego vehicle from the actor list (first actor with a type_id starting with 'vehicle.').
        """
        for actor in self.actor_list:
            if "vehicle." in actor.type_id:
                return actor
        return None

    def cleanup_all(self) -> None:
        """
        Destroy all spawned actors (vehicles, sensors, obstacles) from the simulation run.
        """
        # Destroy LiDAR sensor
        if self.lidar_sensor:
            self.lidar_sensor.destroy()
            self.lidar_sensor = None


        # Destroy collision sensor
        if self.collision_sensor and self.collision_sensor.sensor:
            if self.collision_sensor.sensor.is_alive:
                self.collision_sensor.sensor.stop()
                self.collision_sensor.sensor.destroy()
        self.collision_sensor = None

        # Destroy all actors
        for actor in self.actor_list:
            if actor.is_alive:
                actor.destroy()
        self.actor_list.clear()
        logging.info("[SParkingScenario] Cleaned up all actors.")

    def __del__(self) -> None:
        if hasattr(self, 'cleanup_all'):
            self.cleanup_all()
