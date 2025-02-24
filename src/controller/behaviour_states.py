# src/controller/behaviour_states.py
import carla
import logging
from abc import ABC, abstractmethod
from typing import Optional

from src.controller.parking_controller_interface import ParkingControllerInterface  # Import the interface
from src.utils.normalize_angle import normalize_angle
from enum import Enum, auto

class ParkingState(Enum):
    REVERSE_ALIGNING = auto()
    STEER_INTO_SLOT = auto()
    REVERSE_STRAIGHT = auto()
    COUNTERSTEER = auto()
    FORWARD_STRAIGHT = auto()
    STOPPED = auto()

class ParkingStateBase(ABC):
    """Abstract base class for parking states."""

    def __init__(self, controller: ParkingControllerInterface):  # Use the interface
        self.controller = controller
        self.distance_on_arc: float = 0.0
        self.prev_location: Optional[carla.Location] = None

    def _reset_phase(self) -> None:
        """Reset phase-specific tracking variables."""
        self.distance_on_arc = 0.0
        self.prev_location = None


    @abstractmethod
    def update(self, dist_step: float) -> Optional[ParkingState]:
        """
        Perform actions and state transitions based on the current state.
        Returns the new state if a transition is needed, otherwise None.
        """
        pass

    def enter_state(self):
        """Called when this state is entered."""
        self._reset_phase()
        pass  # Optional: Add any setup needed when entering the state

    def exit_state(self):
        """Called when exiting this state."""
        pass  # Optional: Add any cleanup needed when exiting the state


class ReverseAligningState(ParkingStateBase):

    def __init__(self, controller: ParkingControllerInterface):
        super().__init__(controller)
        self.consec_count: int = 0

    def update(self, dist_step: float) -> Optional[ParkingState]:
        self.controller.vehicle_control.reverse_straight()
        dist_smoothed = self.controller.get_relevant_distance()

        if not self._is_obstacle_far_enough(dist_smoothed): # Use helper method
            self.consec_count = 0
            self.distance_on_arc += dist_step
            return None


        # Obstacle is far enough, increment counter
        self.consec_count += 1
        if self.consec_count < self.controller.config.get('LIDAR_REQUIRED_FRAMES'):
            # Not enough consecutive frames, keep reversing
            self.distance_on_arc += dist_step
            return None

        # Enough consecutive frames, transition!
        return ParkingState.STEER_INTO_SLOT

    def _reset_phase(self) -> None:
        super()._reset_phase()
        self.consec_count = 0

    def _is_obstacle_far_enough(self, dist_smoothed):
        """Checks if the obstacle is far enough to start steering."""
        threshold = self.controller.config.get('PARKING_ALIGN_BEGIN_STEERING_DIST', 1.0) # Use config
        return dist_smoothed > threshold



class SteerIntoSlotState(ParkingStateBase):
    def update(self, dist_step: float) -> Optional[ParkingState]:
        self.controller.vehicle_control.reverse_steer_right()
        self.distance_on_arc += dist_step
        if self.distance_on_arc >= self.controller.config.get('VEHICLE_ARC1_LENGTH'):
            return ParkingState.REVERSE_STRAIGHT
        return None


class ReverseStraightState(ParkingStateBase):
    def update(self, dist_step: float) -> Optional[ParkingState]:
        self.controller.vehicle_control.reverse_straight()
        self.distance_on_arc += dist_step
        if self.distance_on_arc >= self.controller.config.get('VEHICLE_REVERSE_STRAIGHT_LENGTH'):
            return ParkingState.COUNTERSTEER
        return None


class CountersteerState(ParkingStateBase):
    def update(self, dist_step: float) -> Optional[ParkingState]:
        self.controller.vehicle_control.counter_steer()
        self.distance_on_arc += dist_step
        if self.distance_on_arc >= self.controller.config.get('VEHICLE_ARC2_LENGTH'):
            return ParkingState.FORWARD_STRAIGHT
        return None


class ForwardStraightState(ParkingStateBase):
    def update(self, dist_step: float) -> Optional[ParkingState]:
        current_yaw = self.controller.ego_vehicle.get_transform().rotation.yaw
        yaw_error = normalize_angle(self.controller.config.get('REALIGNMENT_DESIRED_YAW') - current_yaw)

        if abs(yaw_error) > self.controller.config.get('REALIGNMENT_YAW_THRESHOLD'):
            raw_correction = yaw_error * self.controller.config.get('REALIGNMENT_CONVERSION_FACTOR')
            correction = max(-self.controller.config.get('REALIGNMENT_MAX_CORRECTION'), min(self.controller.config.get('REALIGNMENT_MAX_CORRECTION'), raw_correction))
            logging.info(f"[ParkingController] Realigning: yaw_error={yaw_error:.2f}, correction={correction:.2f}")
            self.controller.vehicle_control.drive(
                throttle=self.controller.config.get('VEHICLE_FORWARD_THROTTLE'),
                steer=correction,
                reverse=False
            )
        else:
            self.controller.vehicle_control.forward_straight()

        self.distance_on_arc += dist_step
        if self.distance_on_arc >= self.controller.config.get('PARKING_FORWARD_LENGTH'):
            return ParkingState.STOPPED
        return None


class StoppedState(ParkingStateBase):
    def update(self, dist_step: float) -> Optional[ParkingState]:
        self.controller.vehicle_control.stop()
        return None  # Stays in STOPPED state