import os
import time
import numpy as np
import logging
from dotenv import load_dotenv
import carla

# -----------------------------------------------------------------------------
# Load Environment Variables
# -----------------------------------------------------------------------------
load_dotenv()
CARLA_HOST = os.environ.get("CARLA_HOST", "127.0.0.1")
CARLA_PORT = int(os.environ.get("CARLA_PORT", "2000"))

logging.basicConfig(level=logging.INFO, format='[%(levelname)s] %(message)s')

# -----------------------------------------------------------------------------
# Obstacle Detector Configuration
# -----------------------------------------------------------------------------
OBSTACLE_DETECTOR_CONFIG = {
    'OBSTACLE_FRONT_RIGHT_TRANSFORM': carla.Transform(
        carla.Location(x=1.85, y=0.75, z=0.75),
        carla.Rotation(pitch=0.0, yaw=5.0, roll=0.0)
    ),
    'OBSTACLE_FRONT_CENTER_TRANSFORM': carla.Transform(
        carla.Location(x=1.85, y=0.0, z=0.75),
        carla.Rotation(pitch=0.0, yaw=0.0, roll=0.0)
    ),
    'OBSTACLE_FRONT_LEFT_TRANSFORM': carla.Transform(
        carla.Location(x=1.85, y=-0.75, z=0.75),
        carla.Rotation(pitch=0.0, yaw=-5.0, roll=0.0)
    ),
    'OBSTACLE_REAR_RIGHT_TRANSFORM': carla.Transform(
        carla.Location(x=-2.0, y=0.85, z=0.75),
        carla.Rotation(pitch=0.0, yaw=-185.0, roll=0.0)
    ),
    'OBSTACLE_REAR_CENTER_TRANSFORM': carla.Transform(
        carla.Location(x=-2.0, y=0.0, z=0.75),
        carla.Rotation(pitch=0.0, yaw=-180.0, roll=0.0)
    ),
    'OBSTACLE_REAR_LEFT_TRANSFORM': carla.Transform(
        carla.Location(x=-2.0, y=-0.85, z=0.75),
        carla.Rotation(pitch=0.0, yaw=-175.0, roll=0.0)
    ),
    'DETECTION_RANGE': 5.0,  # Detection range for each "beam"
    'BEAM_WIDTH': 0.1 # Visual width
}

# -----------------------------------------------------------------------------
# ObstacleDetectorManager
# -----------------------------------------------------------------------------
class ObstacleDetectorManager:
    def __init__(self, world, ego_vehicle, config):
        self.world = world
        self.ego_vehicle = ego_vehicle
        self.config = config
        self.obstacle_detectors = {}  # Store detector locations

    def spawn_obstacle_detectors(self):
        """Spawns visual representations of the obstacle detectors."""
        if not self.ego_vehicle:
            raise ValueError("Ego vehicle is None. Cannot attach obstacle detectors.")

        ego_transform = self.ego_vehicle.get_transform()

        for name, relative_transform in self.config.items():
            if not name.endswith('_TRANSFORM'):
                continue  # Skip non-transform entries

            # Calculate global transform, similar to LiDAR example
            global_location = ego_transform.transform(relative_transform.location)
            global_rotation = carla.Rotation(
                pitch=ego_transform.rotation.pitch + relative_transform.rotation.pitch,
                yaw=ego_transform.rotation.yaw + relative_transform.rotation.yaw,
                roll=ego_transform.rotation.roll + relative_transform.rotation.roll
            )
            detector_transform = carla.Transform(global_location, global_rotation)
            self.obstacle_detectors[name] = detector_transform


            # Draw a persistent point for the detector's location.
            self.world.debug.draw_point(detector_transform.location,
                                        size=0.2,
                                        color=carla.Color(0, 0, 0),  # Blue for detectors
                                        life_time=10)

            # Draw the detection "beam" as an arrow.
            forward_vector = detector_transform.get_forward_vector()
            beam_end = detector_transform.location + forward_vector * self.config['DETECTION_RANGE']
            self.world.debug.draw_arrow(detector_transform.location, beam_end,
                                        thickness=0.03,
                                        color=carla.Color(0, 0, 0),  # Yellow for beams
                                        life_time=10)
            logging.info(f"Obstacle detector '{name}' spawned and visualized.")


# -----------------------------------------------------------------------------
# MAIN
# -----------------------------------------------------------------------------
def main():
    client = carla.Client(CARLA_HOST, CARLA_PORT)
    client.set_timeout(10.0)
    world = client.get_world()

    blueprint_library = world.get_blueprint_library()

    # Spawn an ego vehicle (for sensor attachment only; the car will not move).
    ego_bp = blueprint_library.find('vehicle.audi.tt')
    spawn_point = world.get_map().get_spawn_points()[0]
    ego_vehicle = world.try_spawn_actor(ego_bp, spawn_point)
    if ego_vehicle is None:
        logging.error("Ego vehicle could not be spawned.")
        return

    logging.info("Ego vehicle spawned successfully.")

    # Set spectator view near the car.
    spectator = world.get_spectator()
    orbit_radius = 7.0
    orbit_height = 4.0
    orbit_angle = 0.0

    # Create and spawn the Obstacle Detectors.
    obstacle_manager = ObstacleDetectorManager(world, ego_vehicle, OBSTACLE_DETECTOR_CONFIG)
    obstacle_manager.spawn_obstacle_detectors()


    try:
        while True:
            # Retrieve the car's current location.
            car_transform = ego_vehicle.get_transform()
            car_location = car_transform.location

            # Calculate new spectator location by orbiting the car.
            orbit_angle += 3.0
            orbit_angle %= 360.0
            rad = np.radians(orbit_angle)
            cam_x = car_location.x + orbit_radius * np.cos(rad)
            cam_y = car_location.y + orbit_radius * np.sin(rad)
            cam_z = car_location.z + orbit_height
            new_location = carla.Location(x=cam_x, y=cam_y, z=cam_z)

            dx = car_location.x - cam_x
            dy = car_location.y - cam_y
            new_yaw = np.degrees(np.arctan2(dy, dx))
            new_rotation = carla.Rotation(pitch=-15, yaw=new_yaw, roll=0)
            new_transform = carla.Transform(new_location, new_rotation)
            #spectator.set_transform(new_transform)  # Uncomment to enable camera movement

            # Optional: print the new spectator transform.
            # print(new_transform)

            time.sleep(0.1)

    except KeyboardInterrupt:
        logging.info("Test interrupted by user.")
    finally:
        logging.info("Cleaning up actors...")
        if ego_vehicle is not None:
            ego_vehicle.destroy()

if __name__ == "__main__":
    main()