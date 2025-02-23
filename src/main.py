import time
import logging

from config import CONFIG
from scenario import SParkingScenario
from controller.controller import ParkingController


def run_simulation_run(sim_index: int, scenario: SParkingScenario, config: dict) -> None:
    """
    Run a single simulation iteration:
      1. Cleanup previous actors.
      2. Set up the scenario.
      3. Instantiate the parking controller.
      4. Run the simulation steps and check for collisions.
    """
    logging.info(f"--- Starting Simulation Run {sim_index + 1}/{config['SIMULATION_NUM_RUNS']} ---")

    # Cleanup from previous run, then set up scenario
    scenario.cleanup_all()
    scenario.setup_scenario()

    ego_vehicle = scenario.get_ego_vehicle()
    if not ego_vehicle:
        logging.error("[MAIN] No ego vehicle. Skipping run.")
        return

    controller = ParkingController(
        ego_vehicle=ego_vehicle,
        lidar_sensor=scenario.lidar_sensor,
        obstacle_detectors=scenario.obstacle_detectors,
        config=config
    )

    # Run simulation steps
    for step in range(config['SIMULATION_NUM_STEPS']):
        controller.update()
    logging.info("[MAIN] Simulation run finished.")
    time.sleep(config['SIMULATION_WAIT_BETWEEN_RUNS'])


def main() -> None:
    # Initialize logging (global configuration)
    logging.basicConfig(level=logging.INFO, format='[%(levelname)s] %(message)s')

    scenario = SParkingScenario(CONFIG)
    time.sleep(2)  # Allow time for initialization

    # Adjust the spectator camera once
    spectator = scenario.world.get_spectator()
    spectator.set_transform(CONFIG['SPECTATOR_TRANSFORM'])

    num_runs = CONFIG.get('SIMULATION_NUM_RUNS', 1)
    for sim_index in range(num_runs):
        run_simulation_run(sim_index, scenario, CONFIG)

    scenario.cleanup_all()
    logging.info("[MAIN] All simulation runs complete.")


if __name__ == "__main__":
    main()
