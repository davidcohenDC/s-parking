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
# Minimal Configuration for LiDAR Debugging
# -----------------------------------------------------------------------------
CONFIG = {
    'lidar_range': 5.0,
    'lidar_rotation_frequency': 20,
    'lidar_channels': 64,
    'lidar_pps': 64000,
    'lidar_transform': carla.Transform(
        carla.Location(x=2.2, y=0.0, z=1.0),
        carla.Rotation(pitch=-10.0, yaw=0.0, roll=0.0)
    ),
    'lidar_buffer_size': 10,
}

# -----------------------------------------------------------------------------
# AdvancedLidarManager (Minimal Version)
# -----------------------------------------------------------------------------
class AdvancedLidarManager:
    def __init__(self, world, blueprint_library, ego_vehicle, config):
        self.world = world
        self.bp_lib = blueprint_library
        self.ego_vehicle = ego_vehicle
        self.config = config
        self.lidar_actor = None
        self._current_points = None

    def spawn_lidar(self):
        if not self.ego_vehicle:
            raise ValueError("Ego vehicle is None. Cannot attach LiDAR.")

        lidar_bp = self.bp_lib.find('sensor.lidar.ray_cast')
        lidar_bp.set_attribute('range', str(self.config['lidar_range']))
        lidar_bp.set_attribute('rotation_frequency', str(self.config['lidar_rotation_frequency']))
        lidar_bp.set_attribute('channels', str(self.config['lidar_channels']))
        lidar_bp.set_attribute('points_per_second', str(self.config['lidar_pps']))

        lidar_transform = self.config['lidar_transform']
        self.lidar_actor = self.world.spawn_actor(lidar_bp, lidar_transform, attach_to=self.ego_vehicle)
        if not self.lidar_actor:
            raise RuntimeError("Cannot spawn LiDAR sensor.")

        logging.info("LiDAR spawned successfully!")
        # Calculate the sensor's expected global transform.
        ego_transform = self.ego_vehicle.get_transform()
        relative_location = lidar_transform.location
        relative_rotation = lidar_transform.rotation
        global_location = ego_transform.transform(relative_location)
        global_rotation = carla.Rotation(
            pitch=ego_transform.rotation.pitch + relative_rotation.pitch,
            yaw=ego_transform.rotation.yaw + relative_rotation.yaw,
            roll=ego_transform.rotation.roll + relative_rotation.roll
        )
        expected_transform = carla.Transform(global_location, global_rotation)
        # Draw a persistent red point at the sensor's global location.
        self.world.debug.draw_point(expected_transform.location,
                                    size=0.5,
                                    color=carla.Color(255, 0, 0),
                                    life_time=100)
        # Draw an arrow showing the sensor's forward direction.
        forward_vector = expected_transform.get_forward_vector()
        arrow_end = expected_transform.location + forward_vector * 2.0
        self.world.debug.draw_arrow(expected_transform.location, arrow_end,
                                    thickness=0.1,
                                    color=carla.Color(0, 255, 0),
                                    life_time=100)
        self.lidar_actor.listen(self._lidar_callback)

    def _lidar_callback(self, lidar_data):
        pts = np.frombuffer(lidar_data.raw_data, dtype=np.float32)
        pts = np.reshape(pts, (int(pts.shape[0] / 4), 4))
        self._current_points = pts

    def get_min_distance_right(self, max_dist=5.0, min_z=0.0, max_z=2.0, x_tolerance=0.05, beam_x=0.5):
        """
        Compute the lateral minimum distance using a narrow beam on the right.
        """
        if self._current_points is None or len(self._current_points) == 0:
            logging.warning("No LiDAR points received.")
            return None

        pts = self._current_points
        x_vals = pts[:, 0]
        y_vals = pts[:, 1]
        z_vals = pts[:, 2]
        dist2d = np.sqrt(x_vals ** 2 + y_vals ** 2)

        mask_x = (x_vals > beam_x - x_tolerance) & (x_vals < beam_x + x_tolerance)
        mask_dist = dist2d < max_dist
        mask_z = (z_vals > min_z) & (z_vals < max_z)
        mask_right = y_vals > 0
        final_mask = mask_x & mask_dist & mask_z & mask_right
        final_pts = pts[final_mask]

        if len(final_pts) == 0:
            logging.warning("No valid LiDAR points in the right-side beam.")
            return None

        lateral_distance = np.min(final_pts[:, 1])
        logging.info(f"Right-side minimum distance (beam at x={beam_x:.2f} m): {lateral_distance:.2f} m")
        return float(lateral_distance)

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
    # We'll orbit the car. Set initial orbit parameters.
    orbit_radius = 7.0  # meters
    orbit_height = 4.0   # meters above the car
    orbit_angle = 0.0    # degrees, will increment over time

    # Create and spawn the LiDAR sensor.
    lidar_manager = AdvancedLidarManager(world, blueprint_library, ego_vehicle, CONFIG)
    lidar_manager.spawn_lidar()

    try:
        while True:
            # Compute and log LiDAR reading.
            lidar_manager.get_min_distance_right()

            # Retrieve the car's current location.
            car_transform = ego_vehicle.get_transform()
            car_location = car_transform.location

            # Calculate new spectator location by orbiting the car.
            orbit_angle += 3.0  # degrees per iteration
            # Ensure angle stays within 0-360.
            orbit_angle %= 360.0
            # Convert to radians.
            rad = np.radians(orbit_angle)
            # Compute spectator position around the car.
            cam_x = car_location.x + orbit_radius * np.cos(rad)
            cam_y = car_location.y + orbit_radius * np.sin(rad)
            cam_z = car_location.z + orbit_height
            new_location = carla.Location(x=cam_x, y=cam_y, z=cam_z)
            # Compute yaw: angle from camera to car.
            dx = car_location.x - cam_x
            dy = car_location.y - cam_y
            new_yaw = np.degrees(np.arctan2(dy, dx))
            # Set a fixed pitch and roll.
            new_rotation = carla.Rotation(pitch=-15, yaw=new_yaw, roll=0)
            new_transform = carla.Transform(new_location, new_rotation)
            spectator.set_transform(new_transform)

            # Optional: print the new spectator transform.
            print(new_transform)

            time.sleep(0.1)
    except KeyboardInterrupt:
        logging.info("Test interrupted by user.")
    finally:
        logging.info("Cleaning up actors...")
        if lidar_manager.lidar_actor is not None:
            lidar_manager.lidar_actor.destroy()
        if ego_vehicle is not None:
            ego_vehicle.destroy()

if __name__ == "__main__":
    main()
