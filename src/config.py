import os
from dotenv import load_dotenv
import carla

# -----------------------------------------------------------------------------
# Load Environment Variables
# -----------------------------------------------------------------------------
load_dotenv()
CARLA_HOST = os.environ.get("CARLA_HOST", "127.0.0.1")
CARLA_PORT = int(os.environ.get("CARLA_PORT", "2000"))

# -----------------------------------------------------------------------------
# Global Configuration
# -----------------------------------------------------------------------------
CONFIG = {
    # Connection settings
    'carla_host': CARLA_HOST,
    'carla_port': CARLA_PORT,

    # -----------------------------------------------------------------------------
    # Simulation Timing
    # -----------------------------------------------------------------------------
    'num_simulations': 13,          # Number of simulation runs
    'simulation_steps': 100000,     # Number of update steps per simulation run
    'wait_between_simulations': 1,  # Seconds to wait between simulation runs

    # -----------------------------------------------------------------------------
    # World Settings
    # -----------------------------------------------------------------------------
    'world_map': {
        'map_name': 'Town01_Opt',
        'map_layers': carla.MapLayer.Walls
    },

    # -----------------------------------------------------------------------------
    # Spawn Transforms
    # -----------------------------------------------------------------------------
    # Ego vehicle spawn
    'ego_spawn': carla.Transform(
        carla.Location(x=84, y=-0.0, z=2),
        carla.Rotation(yaw=180)
    ),
    # Obstacles spawns
    'obstacles_spawns': [
        carla.Transform(carla.Location(x=86, y=-2.8, z=2), carla.Rotation(yaw=180)),
        carla.Transform(carla.Location(x=96, y=-2.8, z=2), carla.Rotation(yaw=180))
    ],
    # Spectator camera transform
    'spectator_transform': carla.Transform(
        carla.Location(x=102, y=-3, z=15),
        carla.Rotation(pitch=-55, yaw=-180, roll=0)
    ),

    # -----------------------------------------------------------------------------
    # LiDAR Sensor Transforms
    # -----------------------------------------------------------------------------
    'lidar_main_transform': carla.Transform(
        carla.Location(x=0.0, y=0.0, z=1.75),
        carla.Rotation(pitch=0.0, yaw=0.0, roll=0.0)
    ),
    'lidar_front_center_transform': carla.Transform(
        carla.Location(x=1.9, y=0.0, z=0.75),
        carla.Rotation(pitch=0.0, yaw=0.0, roll=0.0)
    ),
    'lidar_rear_left_transform': carla.Transform(
        carla.Location(x=-1.9, y=-0.9, z=0.75),
        carla.Rotation(pitch=0.0, yaw=0.0, roll=0.0)
    ),
    'lidar_rear_right_transform': carla.Transform(
        carla.Location(x=-1.9, y=0.9, z=0.75),
        carla.Rotation(pitch=0.0, yaw=0.0, roll=0.0)
    ),


    # -----------------------------------------------------------------------------
    # Parking Alignment Thresholds
    # -----------------------------------------------------------------------------
    'distance_to_begin_steering': 1.1,
    'distance_to_begin_counter': 2.0,
    'distance_to_stop_reverse': 0.8,
    'distance_to_stop_forward': 0.5,

    # -----------------------------------------------------------------------------
    # Vehicle Dynamics
    # -----------------------------------------------------------------------------
    'reverse_throttle': 0.25,
    'forward_throttle': 0.2,
    'steer_angle_in': 0.95,
    'steer_angle_out': -0.95,
    'arc1_length': 1.8,
    'reverse_straight_length': 2.76,
    'arc2_length': 1.9,

    # -----------------------------------------------------------------------------
    # Sensor Parameters
    # -----------------------------------------------------------------------------
    'sensor_tick': 0.0,
    'dropoff_general_rate': 0.1,
    'min_back_distance': 0.0,
    'forward_min_distance': 1.2,
    'lidar_range': 10.0,
    'lidar_rotation_frequency': 50,
    'lidar_channels': 64,
    'lidar_pps': 200000,

    # -----------------------------------------------------------------------------
    # Smoothing & Hysteresis
    # -----------------------------------------------------------------------------
    'lidar_buffer_size': 1,
    'required_consecutive_frames': 1,
}
