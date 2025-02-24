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
    'CARLA_HOST': CARLA_HOST,
    'CARLA_PORT': CARLA_PORT,

    # -----------------------------------------------------------------------------
    # Simulation Timing
    # -----------------------------------------------------------------------------
    'SIMULATION_NUM_RUNS': 2,          # Number of simulation runs
    'SIMULATION_NUM_STEPS': 950000,     # Number of update steps per simulation run
    'SIMULATION_WAIT_BETWEEN_RUNS': 1,  # Seconds to wait between simulation runs

    # -----------------------------------------------------------------------------
    # World Settings
    # -----------------------------------------------------------------------------
    'WORLD_MAP': {
        'map_name': 'Town01_Opt',
        'map_layers': carla.MapLayer.Walls
    },

    # -----------------------------------------------------------------------------
    # Spawn Transforms
    # -----------------------------------------------------------------------------
    # Ego vehicle spawn
    'EGO_SPAWN_TRANSFORM': carla.Transform(
        carla.Location(x=84, y=0.1, z=2),
        carla.Rotation(yaw=180)
    ),
    # Obstacles spawns
    'OBSTACLES_SPAWNS_TRANSFORMS': [
        carla.Transform(
            carla.Location(x=86, y=-2.7, z=2),
            carla.Rotation(yaw=180)),
        carla.Transform(
            carla.Location(x=96, y=-2.7, z=2),
            carla.Rotation(yaw=180))
    ],
    # Spectator camera transform
    'SPECTATOR_TRANSFORM': carla.Transform(
        carla.Location(x=102, y=-3, z=15),
        carla.Rotation(pitch=-55, yaw=-180, roll=0)
    ),

    # -----------------------------------------------------------------------------
    # LiDAR Sensor Transforms
    # -----------------------------------------------------------------------------
    'LIDAR_TRANSFORM': carla.Transform(
        carla.Location(x=-1.8, y=0.9, z=0.75),
        carla.Rotation(pitch=0.0, yaw=0.0, roll=0.0)
    ),

    'LIDAR_RANGE': 10.0,
    'LIDAR_ROTATION_FREQUENCY': 50,
    'LIDAR_CHANNELS': 64,
    'LIDAR_PPS': 200000,
    'LIDAR_SENSOR_TICK': 0.0,
    'LIDAR_DROPOFF_GENERAL_RATE': 0.1,
    'LIDAR_BUFFER_SIZE': 1,
    'LIDAR_REQUIRED_FRAMES': 1,

    # -----------------------------------------------------------------------------
    # Obstacle Detector Parameters
    # -----------------------------------------------------------------------------

    'OBSTACLE_FRONT_RIGHT_TRANSFORM': carla.Transform(
        carla.Location(x=1.85, y=0.75, z=0.75),
        carla.Rotation(pitch=0.0, yaw=5.0, roll=0.0)
    ),
    'OBSTACLE_FRONT_CENTER_TRANSFORM': carla.Transform(
        carla.Location(x=1.85, y=0.0, z=0.75),
        carla.Rotation(pitch=0.0, yaw=5.0, roll=0.0)
    ),
    'OBSTACLE_FRONT_LEFT_TRANSFORM': carla.Transform(
        carla.Location(x=1.85, y=-0.75, z=0.75),
        carla.Rotation(pitch=0.0, yaw=5.0, roll=0.0)
    ),

    'OBSTACLE_REAR_RIGHT_TRANSFORM': carla.Transform(
        carla.Location(x=-2.0, y=0.85, z=0.75),
        carla.Rotation(pitch=0.0, yaw=-180.0, roll=0.0)
    ),
    'OBSTACLE_REAR_CENTER_TRANSFORM': carla.Transform(
        carla.Location(x=-2.0, y=0.0, z=0.75),
        carla.Rotation(pitch=0.0, yaw=-180.0, roll=0.0)
    ),
    'OBSTACLE_REAR_LEFT_TRANSFORM': carla.Transform(
        carla.Location(x=-2.0, y=-0.85, z=0.75),
        carla.Rotation(pitch=0.0, yaw=-180.0, roll=0.0)
    ),

    'OBSTACLE_MIN_DISTANCE': 0.2,
    'OBSTACLE_DETECTION_DISTANCE': 5.0,
    'OBSTACLE_HIT_RADIUS': 0.4,
    'OBSTACLE_ONLY_DYNAMICS': False,
    'OBSTACLE_SENSOR_TICK': 0.01,
    'OBSTACLE_MAX_HISTORY': 100,
    'OBSTACLE_NUM_FPS_CHECK': 10,
    'OBSTACLE_OUTLIER_RATIO': 0.1,

    # -----------------------------------------------------------------------------
    # Parking Alignment Thresholds
    # -----------------------------------------------------------------------------

    'PARKING_ALIGN_BEGIN_STEERING_DIST': 1.1,
    'PARKING_FORWARD_LENGTH': 0.9,
    'PARKING_FORWARD_MIN_DIST': 0.7,

    # -----------------------------------------------------------------------------
    # Vehicle Dynamics
    # -----------------------------------------------------------------------------
    'VEHICLE_REVERSE_THROTTLE': 0.25,
    'VEHICLE_FORWARD_THROTTLE': 0.2,
    'VEHICLE_STEER_ANGLE_IN': 0.95,
    'VEHICLE_STEER_ANGLE_OUT': -0.95,
    'VEHICLE_ARC1_LENGTH': 1.8,
    'VEHICLE_REVERSE_STRAIGHT_LENGTH': 2.76,
    'VEHICLE_ARC2_LENGTH': 1.9,

    # -----------------------------------------------------------------------------
    # Realigning Forward Straight
    # -----------------------------------------------------------------------------
    'REALIGNMENT_DESIRED_YAW': 180.0,
    'REALIGNMENT_YAW_THRESHOLD': 0.2,
    'REALIGNMENT_CONVERSION_FACTOR': 0.4,
    'REALIGNMENT_MAX_CORRECTION': 1.0,

}
