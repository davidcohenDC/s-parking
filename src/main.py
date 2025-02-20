import time
import logging

from config import CONFIG
from scenario import SParkingScenario
from src.controller.controller import ParkingController


def run_simulation_run(sim_index: int, scenario: SParkingScenario, config: dict) -> None:
    """
    Run a single simulation iteration:
      1. Cleanup previous actors.
      2. Set up the scenario.
      3. Instantiate the parking controller.
      4. Run the simulation steps and check for collisions.
    """
    logging.info(f"--- Starting Simulation Run {sim_index + 1}/{config['num_simulations']} ---")

    # Cleanup from previous run, then set up scenario
    scenario.cleanup_all()
    scenario.setup_scenario()

    ego_vehicle = scenario.get_ego_vehicle()
    if not ego_vehicle:
        logging.error("[MAIN] No ego vehicle. Skipping run.")
        return

    controller = ParkingController(
        ego_vehicle=ego_vehicle,
        lidar_sensors=scenario.lidar_sensors,
        config=config
    )

    # Run simulation steps
    for step in range(config['simulation_steps']):
        controller.update()
        if scenario.collision_sensor and scenario.collision_sensor.collided():
            logging.error("[MAIN] Collision detected, ending run.")
            break

    logging.info("[MAIN] Simulation run finished.")
    time.sleep(config['wait_between_simulations'])


def main() -> None:
    # Initialize logging (global configuration)
    logging.basicConfig(level=logging.INFO, format='[%(levelname)s] %(message)s')

    scenario = SParkingScenario(CONFIG)
    time.sleep(2)  # Allow time for initialization

    # Adjust the spectator camera once
    spectator = scenario.world.get_spectator()
    spectator.set_transform(CONFIG['spectator_transform'])

    num_runs = CONFIG.get('num_simulations', 1)
    for sim_index in range(num_runs):
        run_simulation_run(sim_index, scenario, CONFIG)

    scenario.cleanup_all()
    logging.info("[MAIN] All simulation runs complete.")


if __name__ == "__main__":
    main()
