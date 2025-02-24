import time
import logging

import numpy as np
import carla
from typing import Optional, List

from src.sensors.lidar_sensor import LidarSensorManager
from sensors.collision_sensor import CollisionSensorManager
from src.sensors.obstacle_detector import ObstacleDetector


class SParkingScenario:
    """
    Sets up and cleans up the parking scenario, including spawning the ego vehicle,
    obstacles, LiDAR sensors, and a collision sensor.
    """
    def __init__(self, config: dict, client: Optional[carla.Client] = None) -> None:
        self.config = config
        self.client = client or carla.Client(
            config.get("CARLA_HOST", "127.0.0.1"),
            config.get("CARLA_PORT", 2000)
        )
        self.client.set_timeout(10.0)

        # Load the world using unpacked world map configuration.
        self.world = self.client.load_world(**config['WORLD_MAP'])
        self.bp_lib = self.world.get_blueprint_library()

        self.ego_spawn = config['EGO_SPAWN_TRANSFORM']
        self.obstacles_spawns = config['OBSTACLES_SPAWNS_TRANSFORMS']

        # **Store a deep copy of the default obstacle positions**
        self.default_obstacle_positions = [
            carla.Transform(transform.location, transform.rotation) for transform in self.obstacles_spawns
        ]

        # Lists for tracking spawned actors and sensors (for cleanup).
        self.actor_list: List[carla.Actor] = []
        self.lidar_sensor: Optional[LidarSensorManager] = None
        self.obstacle_detectors: List[ObstacleDetector] = []
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
        self._spawn_obstacle_detector()
        self._wait_for_vehicles_stopped()

    def reset_obstacle_positions(self) -> None:
        """
        Resets all obstacles to their default positions before a new scenario setup.
        """
        logging.info("[SParkingScenario] Resetting obstacle positions to default...")

        if not self.default_obstacle_positions:
            logging.warning("[SParkingScenario] No default positions stored for obstacles!")
            return

        # Restore the default obstacle positions
        for i, transform in enumerate(self.default_obstacle_positions):
            self.obstacles_spawns[i].location = carla.Location(
                x=transform.location.x,
                y=transform.location.y,
                z=transform.location.z
            )
            self.obstacles_spawns[i].rotation = carla.Rotation(
                pitch=transform.rotation.pitch,
                yaw=transform.rotation.yaw,
                roll=transform.rotation.roll
            )

        logging.info("[SParkingScenario] Obstacles reset to default positions.")

    def _spawn_actor(self, blueprint: carla.ActorBlueprint, transform: carla.Transform) -> Optional[carla.Actor]:
        """
        Helper to spawn an actor and add it to the actor list.
        Ensures there is no overlap before spawning.
        """
        # Check for existing actors at the spawn location
        for actor in self.world.get_actors():
            if "vehicle." in actor.type_id:
                distance = np.linalg.norm([
                    actor.get_location().x - transform.location.x,
                    actor.get_location().y - transform.location.y
                ])
                if distance < 1.5:  # If too close, prevent spawning
                    logging.warning(
                        f"[SParkingScenario] Too close to another vehicle at {transform.location}. Skipping spawn.")
                    return None

        # Try spawning the actor
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

        self.actor_list.append(ego_vehicle)

        # Spawn obstacles.
        for i, transform in enumerate(self.obstacles_spawns):
            obs_bp = self.bp_lib.find('vehicle.audi.tt')
            obs_actor = self._spawn_actor(obs_bp, transform)
            if obs_actor:
                obs_actor.set_autopilot(False)
                obs_actor.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0))
                obs_actor.set_collisions(True)
                obs_actor.set_simulate_physics(True)
                self.actor_list.append(obs_actor)
                logging.info(f"[SParkingScenario] Spawned obstacle {i} at {transform.location}.")
            else:
                logging.error(
                    f"[SParkingScenario] Failed to spawn obstacle {i} at {transform.location}.")  # Debugging message


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

        lidar_transform = self.config['LIDAR_TRANSFORM']
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

    def move_obstacle(self, offset: float) -> None:
        """
        Moves the back obstacle to a new position instead of respawning it.
        """
        logging.info("[SParkingScenario] Moving back car instead of respawning...")

        if len(self.obstacles_spawns) > 1:
            back_car = None

            # Find the obstacle in the current actors list
            for actor in self.world.get_actors():
                if "vehicle." in actor.type_id and actor != self.get_ego_vehicle():
                    back_car = actor
                    break

            if back_car:
                # Disable physics to force teleportation
                back_car.set_simulate_physics(False)

                # Get the original transform and apply the new offset
                new_transform = back_car.get_transform()
                new_transform.location.x += offset

                # Apply the new position
                back_car.set_transform(new_transform)

                # Re-enable physics
                time.sleep(0.5)  # Allow some processing time before enabling physics again
                back_car.set_simulate_physics(True)

                logging.info(f"[SParkingScenario] Back car moved by {offset:.2f} meters to {new_transform.location}.")
            else:
                logging.warning("[SParkingScenario] No back car found to move!")

        # Wait for CARLA to process the teleportation
        time.sleep(1)

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

    def _spawn_obstacle_detector(self) -> None:
        """Spawn and attach obstacle detectors (front and rear) to the ego vehicle."""
        ego_vehicle = self.get_ego_vehicle()
        if not ego_vehicle:
            logging.error("[SParkingScenario] No ego vehicle for obstacle detectors.")
            return

        # Destroy and clear any previously spawned obstacle detectors.
        for detector in self.obstacle_detectors:
            detector.destroy()
        self.obstacle_detectors.clear()

        # Create the front and rear obstacle detectors using their respective transforms.
        front_right_detector = ObstacleDetector(
            parent_actor=ego_vehicle,
            transform=self.config['OBSTACLE_FRONT_RIGHT_TRANSFORM'],
            config=self.config
        )

        front_center_detector = ObstacleDetector(
            parent_actor=ego_vehicle,
            transform=self.config['OBSTACLE_FRONT_CENTER_TRANSFORM'],
            config=self.config
        )

        front_left_detector = ObstacleDetector(
            parent_actor=ego_vehicle,
            transform=self.config['OBSTACLE_FRONT_LEFT_TRANSFORM'],
            config=self.config
        )

        rear_right_detector = ObstacleDetector(
            parent_actor=ego_vehicle,
            transform=self.config['OBSTACLE_REAR_RIGHT_TRANSFORM'],
            config=self.config
        )

        rear_center_detector = ObstacleDetector(
            parent_actor=ego_vehicle,
            transform=self.config['OBSTACLE_REAR_CENTER_TRANSFORM'],
            config=self.config
        )

        rear_left_detector = ObstacleDetector(
            parent_actor=ego_vehicle,
            transform=self.config['OBSTACLE_REAR_LEFT_TRANSFORM'],
            config=self.config
        )

        # Add both detectors to the list.
        self.obstacle_detectors.extend([
            front_right_detector,
            front_center_detector,
            front_left_detector,
            rear_right_detector,
            rear_center_detector,
            rear_left_detector,
        ])

        # If a detector creates a sensor actor, add it to the actor list.
        for idx, detector in enumerate(self.obstacle_detectors):
            if hasattr(detector, 'sensor') and detector.sensor is not None:
                self.actor_list.append(detector.sensor)
                logging.info(f"[SParkingScenario] Obstacle detector {idx} spawned.")

    def _wait_for_vehicles_stopped(self) -> None:
        """
        Wait until all vehicles in the actor list have nearly stopped.
        This ensures that all spawned vehicles have stabilized.
        """
        logging.info("[SParkingScenario] Waiting for vehicles to stabilize...")

        time.sleep(2)  # Ensures all vehicles spawn correctly before checking movement

        max_attempts = 100  # Maximum iterations to wait
        all_stopped = False
        attempt = 0

        while not all_stopped and attempt < max_attempts:
            all_stopped = True
            for actor in self.actor_list:
                if isinstance(actor, carla.Vehicle):
                    vel = actor.get_velocity()
                    speed = np.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2)
                    if speed > 0.1:
                        all_stopped = False
                        break
            if not all_stopped:
                time.sleep(0.05)  # Increase wait time per iteration
                attempt += 1

        logging.info("[SParkingScenario] All vehicles are stabilized.")

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

        # Destroy obstacle detectors
        for detector in self.obstacle_detectors:
            detector.destroy()


        # Destroy all actors
        for actor in self.actor_list:
            if actor.is_alive:
                actor.destroy()
        self.actor_list.clear()
        logging.info("[SParkingScenario] Cleaned up all actors.")

    def __del__(self) -> None:
        if hasattr(self, 'cleanup_all'):
            self.cleanup_all()
