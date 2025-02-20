# parking_state.py
from enum import Enum

# Define the parking states
class ParkingState(Enum):
    IDLE = 0
    REVERSE_STRAIGHT_1 = 1
    STEER_INTO_SLOT = 2
    REVERSE_STRAIGHT_2 = 3
    COUNTERSTEER = 4
    FORWARD_STRAIGHT = 5
    STOPPED = 6
