import logging
from typing import Optional
import carla

from src.config import CONFIG

class VehicleController:
    """
    Provides high-level vehicle control methods by encapsulating
    the CARLA VehicleControl commands.
    """

    def __init__(self, vehicle: carla.Vehicle) -> None:
        self.vehicle = vehicle
        self.reverse_throttle: float = CONFIG['VEHICLE_REVERSE_THROTTLE']
        self.steer_angle_in: float = CONFIG['VEHICLE_STEER_ANGLE_IN']
        self.steer_angle_out: float = CONFIG['VEHICLE_STEER_ANGLE_OUT']
        self.forward_throttle: float = CONFIG['VEHICLE_FORWARD_THROTTLE']

    def drive(
        self,
        throttle: float,
        steer: float,
        reverse: bool = False,
        brake: float = 0.0
    ) -> carla.VehicleControl:
        """
        Apply a drive command to the vehicle.

        :param throttle: Throttle value (0.0 to 1.0).
        :param steer: Steering value (-1.0 to 1.0).
        :param reverse: Whether the vehicle should be in reverse.
        :param brake: Brake value (0.0 to 1.0).
        :return: The VehicleControl command that was applied.
        """
        control = carla.VehicleControl(throttle=throttle, steer=steer, reverse=reverse, brake=brake)

        self.vehicle.apply_control(control)
        return control

    def reverse_straight(self, throttle: Optional[float] = None) -> carla.VehicleControl:
        """
        Go straight in reverse.

        If a custom throttle is provided, it is used; otherwise, the default
        reverse throttle from the configuration is used.

        :param throttle: Optional custom throttle for reverse driving.
        :return: The VehicleControl command that was applied.
        """
        if throttle is None:
            throttle = self.reverse_throttle

        return self.drive(throttle=throttle, steer=0.0, reverse=True)

    def reverse_steer_right(self, throttle: Optional[float] = None, steer: Optional[float] = None) -> None:
        """
        Reverse and steer right.

        If custom throttle or steer values are provided, they are used; otherwise,
        the defaults from the configuration are used.
        """
        if throttle is None:
            throttle = self.reverse_throttle
        if steer is None:
            steer = self.steer_angle_in
        self.drive(throttle=throttle, steer=steer, reverse=True)


    def counter_steer(self, throttle: Optional[float] = None, steer: Optional[float] = None) -> None:
        """
        Counter steer the vehicle.

        If custom throttle or steer values are provided, they are used; otherwise,
        the defaults from the configuration are used.
        """
        if throttle is None:
            throttle = self.reverse_throttle
        if steer is None:
            steer = self.steer_angle_out
        self.drive(throttle=throttle, steer=steer, reverse=True)


    def forward_straight(self, throttle: Optional[float] = None) -> None:
        """
        Go straight forward.

        If a custom throttle is provided, it is used; otherwise, the default
        forward throttle from the configuration is used.
        """
        if throttle is None:
            throttle = self.forward_throttle
        self.drive(throttle=throttle, steer=0.0, reverse=False)


    def stop(self) -> None:
        """
        Stop the vehicle immediately.
        """
        self.drive(throttle=0.0, steer=0.0, reverse=False, brake=1.0)

