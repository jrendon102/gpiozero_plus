#!/usr/bin/env python3
"""
A Python class for controlling motors using gpiozero library.

This script defines a class, MotorController, to control motors using the gpiozero library.
You can set the speed of the individual motors to allow maneuverability in any direction.

Author: Julian A Rendon
Copyright (c) 2023
License: MIT License
Last Updated: September 15, 2023

"""
from gpiozero import Robot
from gpiozero.pins.pigpio import PiGPIOFactory

from gpiozero_plus.enums import Motion


class MotorController:
    """
    Controller for motors attached to the Yahboom G1 Tank Expansion Board.

    Parameters:
        `left_motor`:
            Pins associated with the left motor.

        `right_motor`:
            Pins associated with the right motor.

        `curve_scale`:
            Amount of curvature when moving in a CURVED motion (default is 0.6).
    """

    def __init__(self, left_motor: int, right_motor: int, curve_scale: float = 0.6) -> None:
        self.__factory = PiGPIOFactory()
        self.__motors = Robot(left=left_motor, right=right_motor, pin_factory=self.__factory)
        self.__curve_scale = curve_scale
        self.__motion_type = None

    def run(self, linear_x: float, angular_z: float) -> None:
        """
        Adjust motor speed based on the provided velocities.

        Args:
            `linear_x (float)`: Linear velocity in the x direction.

            `angular_z (float)`: Angular velocity in the z direction.

        Returns:
            None
        """

        # Keep velocity between 0.0 and 1.0
        linear_x, angular_z = self.constrain_vel_range(linear_x, angular_z)

        if linear_x > 0.0 and angular_z > 0.0:
            self.__motors.forward(angular_z, curve_left=self.__curve_scale)
            self.__motion_type = Motion.LEFT_FWD

        elif linear_x > 0.0 and angular_z < 0.0:
            self.__motors.forward(angular_z, curve_right=self.__curve_scale)
            self.__motion_type = Motion.RIGHT_FWD

        elif linear_x < 0.0 and angular_z > 0.0:
            self.__motors.backward(abs(angular_z), curve_right=self.__curve_scale)
            self.__motion_type = Motion.RIGHT_BK

        elif linear_x < 0.0 and angular_z < 0.0:
            self.__motors.backward(abs(angular_z), curve_right=self.__curve_scale)
            self.__motion_type = Motion.LEFT_BK

        elif linear_x > 0:
            self.__motors.forward(linear_x)
            self.__motion_type = Motion.STR_FWD

        elif linear_x < 0:
            self.__motors.backward(abs(linear_x))
            self.__motion_type = Motion.STR_BK

        elif angular_z > 0:
            self.__motors.left(angular_z)
            self.__motion_type = Motion.ROT_LEFT

        elif angular_z < 0:
            self.__motors.right(abs(angular_z))
            self.__motion_type = Motion.ROT_RIGHT

        else:
            self.__motors.stop()
            self.__motion_type = Motion.NO_MOTION

    def constrain_vel_range(self, linear_x: float, angular_z: float) -> "tuple[float, float]":
        """
        Constrain the linear and angular velocity values to a range of -1 to 1.

        Args:
            `linear_x (float)`: The linear velocity value to be constrained.
            `angular_z (float)`: The angular velocity value to be constrained.

        Returns:
            `tuple[float, float]`: A tuple containing the constrained linear x and angular z velocity
            values.
        """
        constrained_linear_x = max(min(linear_x, 1.0), -1.0)
        constrained_angular_z = max(min(angular_z, 1.0), -1.0)
        return constrained_linear_x, constrained_angular_z

    def get_motion_type(self) -> Motion:
        """
        Get the current motion type.

        Returns:
            The current motion type, or None if there's no motion.
        """
        return self.__motion_type

    def disconnect(self) -> None:
        """
        Stop the motors and release motor pins.

        Returns:
            None
        """
        self.__motors.stop()
        self.__factory.close()
