# S-Parking


[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

S-Parking is a robust autonomous parking simulation project built using the **CARLA Simulator (version 0.9.15)**.  It demonstrates a challenging S-parking maneuver (reverse parallel parking) where an ego vehicle navigates between two parked cars using a sophisticated state machine-based controller, LiDAR, and obstacle detection sensors. The project prioritizes clean code, clear separation of concerns (using **SOLID principles**), and extensibility.
> **Important Note:** This project has been developed and tested with CARLA 0.9.15. While it *may* function with other versions, compatibility is *not* guaranteed.  It is **strongly recommended** to use version 0.9.15.

![S-Parking Demo](media/sparking_demo.gif)  
*Demo: S-Parking simulation showcasing the ego vehicle performing the parking maneuver.*


## Table of Contents

1.  [Project Overview](#project-overview)
2.  [Features](#features)
3.  [Prerequisites](#prerequisites)
4.  [Main Modules Overview](#modules-overview)
5.  [Configuration](#configuration)
6.  [Future Enhancements](#future-enhancements)
7. [Known Limitations](#known-limitations)
8. [Troubleshooting](#troubleshooting)
9. [License](#license)
10. [Acknowledgments](#acknowledgments)

## Project Overview

S-Parking simulates a parking scenario in Carla by:

- **Spawning Actors:** An ego vehicle and obstacles are spawned using predefined transforms.
- **Sensor Integration:** Multiple LiDAR sensors and a collision sensor are attached to the ego vehicle.
- **State Machine Controller:** A parking controller processes sensor data and applies vehicle control commands to perform parking maneuvers.
- **Modular Design:** The code is organized into clearly defined modules to facilitate maintenance and future enhancements.

## Features

*   **Realistic S-Parking Scenario:** Accurately simulates a common and challenging parking situation.
*   **State Machine Controller:** A well-defined `ParkingController` implements a finite state machine (FSM) to orchestrate the parking process. The states include:
    *   `REVERSE_ALIGNING`:  Reverses straight while using LiDAR to maintain a consistent distance from the adjacent vehicle.
    *   `STEER_INTO_SLOT`:  Reverses while initiating a right steering maneuver to enter the parking space.
    *   `REVERSE_STRAIGHT`:  Reverses straight within the confines of the parking space.
    *   `COUNTERSTEER`:  Reverses while applying left steering to straighten the vehicle's alignment.
    *   `FORWARD_STRAIGHT`:  Moves forward to finalize the parking position, incorporating yaw correction as needed.
    *   `STOPPED`:  The terminal state, indicating the vehicle has come to a complete stop.
*   **Sensor Integration:** Effectively utilizes LiDAR and custom obstacle detectors for comprehensive environmental perception.
*   **Asynchronous Sensor Processing:** LiDAR and obstacle sensors leverage CARLA's event-driven system, providing efficient and low-latency data handling. The LiDAR sensor further incorporates internal buffering for smoothed distance measurements.
*   **Highly Configurable Parameters:**  A comprehensive `config.py` file enables easy tuning of all critical parameters, including sensor settings, state transition thresholds, and vehicle control parameters.
*   **Comprehensive Logging:** Extensive logging capabilities facilitate debugging and provide a clear understanding of the system's internal operations.

## Prerequisites

*   **CARLA Simulator (version 0.9.15):**  Download and install CARLA 0.9.15 from the official CARLA website ([https://carla.org/](https://carla.org/)).  Other versions *might* work, but 0.9.15 is strongly recommended.
*   **Python 3.7+:**  Ensure you have Python 3.7 or a later version installed.
*   **Required Python Packages:** Install the necessary packages using pip:

    ```bash
    pip install -r requirements.txt
    ```



## Main Modules Overview

*   **`config.py`:**  Global configuration parameters.  This is the primary place to adjust simulation settings.

*   **`src/main.py`:**  The main script.  Connects to CARLA, sets up the scenario *once*, and runs multiple simulation iterations, resetting the scenario *without* reloading the world between runs.

*   **`src/scenario.py`:**  The `SParkingScenario` class manages the CARLA world, spawns actors (ego vehicle, obstacles, sensors), *resets* the scenario between runs (repositioning actors but *not* destroying them), and provides cleanup functionality.

*   **`src/controller/`:**
    *   `parking_controller.py`:  The `ParkingController` class implements the finite state machine (FSM).  It receives sensor data, updates its internal state, and sends control commands to the `VehicleController`.
    *   `behaviour_states.py`: Contains the individual state classes (e.g., `ReverseAligningState`, `SteerIntoSlotState`, etc.). Each state class encapsulates the logic for a specific phase of the parking maneuver.  State transitions are based on sensor readings, *not* fixed distances.
    *   `parking_controller_interface.py`: Defines an abstract base class (`ParkingControllerInterface`) that specifies the methods states can use to interact with the controller, preventing circular dependencies.
    *   `vehicle_controller.py`: The `VehicleController` class provides simplified methods for controlling the ego vehicle (throttle, steer, brake, reverse).

*   **`src/sensors/`:**
    *   `lidar_sensor.py`: The `LidarSensorManager` class wraps a CARLA LiDAR sensor. It processes the point cloud data *asynchronously* (using CARLA's event-driven `listen()` method) and provides methods for getting the minimum distances to the right, front, and back.  It includes *internal buffering* for smoother distance readings.
    *   `obstacle_detector.py`: The `ObstacleDetector` class uses CARLA's `sensor.other.obstacle` to detect obstacles, also *asynchronously*. It provides methods for getting the closest obstacle.
    *   `collision_sensor.py`: The `CollisionSensorManager` class manages collision detection (currently set up but not actively used in the main logic).

*   **`src/utils/`:**
    *   `distance.py`: Utility functions for distance calculations (e.g., `euclid_dist`).
    *   `normalize_angle.py`: Provides the `normalize_angle` function, which is crucial for handling yaw angles correctly.

## Configuration

The `config.py` file contains all the configurable parameters for the simulation.  **Thoroughly review and adjust these parameters to match your CARLA environment, vehicle characteristics, and desired parking behavior.**  It's crucial to understand the meaning of each parameter and how it affects the simulation.

The configuration is organized into the following sections:

*   **CARLA Connection:** Settings for connecting to the CARLA server.
*   **Simulation Settings:** General parameters controlling the simulation loop.
*   **Spawning:** Transforms (positions and rotations) for spawning the ego vehicle, obstacle vehicles, and setting the spectator camera.
*   **LiDAR Sensor:** Parameters for the LiDAR sensor.
*   **Obstacle Detectors:** Parameters for the obstacle detection sensors.
*   **Parking Controller:** Parameters controlling the behavior of the state machine and vehicle control.

### CARLA Connection

| Parameter       | Type    | Default      | Description                                                                 |
|-----------------|---------|--------------|-----------------------------------------------------------------------------|
| `CARLA_HOST`    | string  | `localhost`  | The hostname or IP address of the CARLA server.                             |
| `CARLA_PORT`    | integer | `2000`       | The port number the CARLA server is listening on.                           |
| `CARLA_TIMEOUT` | float   | `10.0`       | The timeout (in seconds) for connecting to the CARLA server.                |
| `CARLA_MAP`     | string  | `Town04_Opt` | The name of the CARLA map to load (e.g., `'Town04_Opt'`, `'Town05'`, etc.). |

### Simulation Settings

| Parameter                      | Type    | Default | Description                                                                                                           |
|--------------------------------|---------|---------|-----------------------------------------------------------------------------------------------------------------------|
| `FPS`                          | integer | `20`    | The desired frames per second for the main simulation loop.  This controls how often the `ParkingController` updates. |
| `SIMULATION_NUM_RUNS`          | integer | `3`     | The number of simulation runs to perform for each parking gap configuration.                                          |
| `SIMULATION_NUM_STEPS`         | integer | `1000`  | The maximum number of steps (frames) to run in each simulation iteration.                                             |
| `SIMULATION_WAIT_BETWEEN_RUNS` | float   | `2`     | The delay (in seconds) between simulation runs.                                                                       |

### Spawning

| Parameter                     | Type                      | Default (Example) | Description                                                                                                            |
|-------------------------------|---------------------------|-------------------|------------------------------------------------------------------------------------------------------------------------|
| `EGO_SPAWN_TRANSFORM`         | `carla.Transform`         | (See `config.py`) | The initial position and orientation of the ego vehicle.                                                               |
| `OBSTACLES_SPAWNS_TRANSFORMS` | list of `carla.Transform` | (See `config.py`) | A list of transforms, defining the initial positions and orientations of the obstacle vehicles.                        |
| `SPECTATOR_TRANSFORM`         | `carla.Transform`         | (See `config.py`) | The initial position and orientation of the spectator camera.  Adjust this to get a good view of the parking maneuver. |

### LiDAR Sensor

| Parameter                    | Type              | Default           | Description                                                                                               |
|------------------------------|-------------------|-------------------|-----------------------------------------------------------------------------------------------------------|
| `LIDAR_TRANSFORM`            | `carla.Transform` | (See `config.py`) | The position and orientation of the LiDAR sensor *relative to the ego vehicle*.                           |
| `LIDAR_RANGE`                | float             | `50.0`            | The maximum range of the LiDAR sensor (in meters).                                                        |
| `LIDAR_ROTATION_FREQUENCY`   | float             | `20.0`            | The rotation frequency of the LiDAR (in Hz).                                                              |
| `LIDAR_CHANNELS`             | integer           | `32`              | The number of LiDAR channels.                                                                             |
| `LIDAR_PPS`                  | integer           | `100000`          | The number of points per second generated by the LiDAR.                                                   |
| `LIDAR_SENSOR_TICK`          | float             | `0.05`            | The tick rate of the LiDAR sensor (how often it generates data, in seconds). **Crucial for low latency.** |
| `LIDAR_DROPOFF_GENERAL_RATE` | float             | `0.0`             | A parameter controlling the dropoff of LiDAR points with distance (usually keep at 0).                    |
| `LIDAR_BUFFER_SIZE`          | integer           | `10`              | The size of the buffer used to store recent LiDAR distance measurements (for smoothing).                  |
| `LIDAR_REQUIRED_FRAMES`      | integer           | `3`               | Number of consecutive frames.                                                                             |

### Obstacle Detectors

| Parameter                         | Type              | Default (Example) | Description                                                                                                                                     |
|-----------------------------------|-------------------|-------------------|-------------------------------------------------------------------------------------------------------------------------------------------------|
| `OBSTACLE_FRONT_RIGHT_TRANSFORM`  | `carla.Transform` | (See `config.py`) | The transform of the front-right obstacle detector, *relative to the ego vehicle*.                                                              |
| `OBSTACLE_FRONT_CENTER_TRANSFORM` | `carla.Transform` | (See `config.py`) | The transform of the front-center obstacle detector, *relative to the ego vehicle*.                                                             |
| `OBSTACLE_FRONT_LEFT_TRANSFORM`   | `carla.Transform` | (See `config.py`) | The transform of the front-left obstacle detector, *relative to the ego vehicle*.                                                               |
| `OBSTACLE_REAR_RIGHT_TRANSFORM`   | `carla.Transform` | (See `config.py`) | The transform of the rear-right obstacle detector, *relative to the ego vehicle*.                                                               |
| `OBSTACLE_REAR_CENTER_TRANSFORM`  | `carla.Transform` | (See `config.py`) | The transform of the rear-center obstacle detector, *relative to the ego vehicle*.                                                              |
| `OBSTACLE_REAR_LEFT_TRANSFORM`    | `carla.Transform` | (See `config.py`) | The transform of the rear-left obstacle detector, *relative to the ego vehicle*.                                                                |
| `OBSTACLE_MIN_DISTANCE`           | float             | `1.0`             | The minimum distance to an obstacle that will trigger a stop.  **Critical safety parameter.**                                                   |
| `OBSTACLE_DETECTION_DISTANCE`     | float             | `5.0`             | The maximum detection range of the obstacle detectors.                                                                                          |
| `OBSTACLE_HIT_RADIUS`             | float             | `0.5`             | The radius around the obstacle detection point considered a "hit."                                                                              |
| `OBSTACLE_ONLY_DYNAMICS`          | boolean           | `False`           | Whether to detect only dynamic obstacles (`True`) or both static and dynamic obstacles (`False`).  For parking, this should usually be `False`. |
| `OBSTACLE_SENSOR_TICK`            | float             | `0.01`            | The tick rate of the obstacle detection sensors (how often they check for obstacles). **Crucial for low latency.**                              |
| `OBSTACLE_MAX_HISTORY`            | integer           | `20`              | The number of obstacle detection events to store in the history (used for outlier filtering).                                                   |
| `OBSTACLE_OUTLIER_RATIO`          | float             | `0.7`             | A parameter controlling the outlier filtering in the `ObstacleDetector`.  Higher values are less aggressive (filter out fewer points).          |

### Parking Controller Parameters

| Parameter                           | Type    | Default | Description                                                                                                                                                             |
|-------------------------------------|---------|---------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| `PARKING_ALIGN_BEGIN_STEERING_DIST` | float   | `2.5`   | Now it not used                                                                                                                                                         |
| `TARGET_REAR_DISTANCE`              | float   | `0.6`   | **CRITICAL:** The target distance from the *rear center* of the ego vehicle to the obstacle behind it *when the rear of the vehicle is aligned with the parking space*. |
| `REAR_ALIGNMENT_TOLERANCE`          | float   | `0.1`   | The tolerance (in meters) for the rear alignment check.                                                                                                                 |
| `REAR_ALIGN_REQUIRED_FRAMES`        | integer | `3`     | The number of consecutive frames the rear alignment condition must be met before transitioning to the next state.                                                       |
| `VEHICLE_ARC1_LENGTH`               | float   | `3.0`   | Now is not used.                                                                                                                                                        |
| `VEHICLE_REVERSE_STRAIGHT_LENGTH`   | float   | `1.5`   | Now is not used.                                                                                                                                                        |
| `VEHICLE_ARC2_LENGTH`               | float   | `3.5`   | Now is not used.                                                                                                                                                        |
| `PARKING_FORWARD_LENGTH`            | float   | `0.9`   | Now is not used.                                                                                                                                                        |
| `PARKING_FORWARD_MIN_DIST`          | float   | `0.7`   | Minimum distance to stop.                                                                                                                                               |
| `REALIGNMENT_DESIRED_YAW`           | float   | `180.0` | The desired yaw angle of the ego vehicle when parked (in degrees).  For your S-parking scenario, this should be 180.0.                                                  |
| `REALIGNMENT_YAW_THRESHOLD`         | float   | `0.1`   | The maximum allowed yaw error (in radians) before a steering correction is applied during the `FORWARD_STRAIGHT` state.                                                 |
| `REALIGNMENT_CONVERSION_FACTOR`     | float   | `0.3`   | A proportionality constant that determines the steering correction based on the yaw error.                                                                              |
| `REALIGNMENT_MAX_CORRECTION`        | float   | `0.5`   | The maximum steering correction angle (in radians).  This prevents oversteering.                                                                                        |
| `VEHICLE_REVERSE_THROTTLE`          | float   | `0.25`  | The throttle value to use when reversing.                                                                                                                               |
| `VEHICLE_FORWARD_THROTTLE`          | float   | `0.3`   | The throttle value to use when driving forward.                                                                                                                         |
| `VEHICLE_STEER_ANGLE_IN`            | float   | `0.7`   | The steering angle to use when steering into the parking space (`STEER_INTO_SLOT` state).                                                                               |
| `VEHICLE_STEER_ANGLE_OUT`           | float   | `-0.7`  | The steering angle to use when counter-steering (`COUNTERSTEER` state).                                                                                                 |
## Future Enhancements

*   **Improved State Transitions:** Refine the state transition logic to be even more robust.
*   **Dynamic Obstacle Avoidance:** Implement more sophisticated obstacle avoidance that can handle moving obstacles.
*   **Parking Spot Detection:**  Add the ability to automatically detect available parking spots.
*   **Path Planning:** Integrate a path planning algorithm (e.g., A*, RRT) for more complex maneuvers.
*   **Machine Learning:** Explore using machine learning.
*   **Testing Framework:** Develop a comprehensive testing framework.


## Known Limitations

*   **CARLA Version:** Developed and tested with CARLA 0.9.15.
*   **Simplified Vehicle Dynamics:** Uses CARLA's built-in vehicle physics.
*    **No Recovery Mechanism:** Currently, there is no implemented logic for recovering from errors during the parking maneuver.

## Troubleshooting

*   **"Cannot connect to CARLA server":**  Ensure the CARLA server is running, and check `CARLA_HOST` and `CARLA_PORT`.
*   **Vehicle not moving:**  Verify `world.tick()` is called.  Check throttle values in `VehicleController`.
*   **Vehicle colliding:** Adjust `OBSTACLE_MIN_DISTANCE`, sensor parameters, and state transition thresholds.
*   **Unexpected State Transitions:** Use logging to trace state transitions and sensor readings. Pay close attention to `TARGET_REAR_DISTANCE` and other threshold values.
*   **Import Errors:** Check your Python environment and package installations.  Look for circular imports.
*   **Vehicle Turning the Wrong Way:** Carefully review the `_correct_yaw()` method in your state classes and the `drive()` method in `VehicleController`. Ensure the signs of the steering corrections are correct, and that `REALIGNMENT_DESIRED_YAW` is set appropriately (180.0 degrees for your scenario).
*  **Lidar not giving correct value:** Check the filter parameters and position on `LidarSensorManager` class.


## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Acknowledgments

This project was created by **David Cohen** and **Michele Monti**.