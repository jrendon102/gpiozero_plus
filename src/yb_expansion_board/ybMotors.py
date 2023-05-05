#!/usr/bin/env python3
"""
This script utilizes the gpiozero library to access the GPIO pins on the Raspberry Pi. The purpose
is to allow communication with motors attached to the Yahboom G1 Tank Expansion Board. The speed
of the motors are varied to allow maneuverability in any direction.
"""
from gpiozero import Robot
from gpiozero.pins.pigpio import PiGPIOFactory

from yb_expansion_board.navigation import constrain_vel_range
from yb_expansion_board.robotEnums import MotionType

# Type of motion.
CURVILINEAR = MotionType.CURVILINEAR.value
LINEAR = MotionType.LINEAR.value
NO_MOTION = MotionType.NO_MOTION.value
ROTATIONAL = MotionType.ROTATIONAL.value


class YBMotor:
    """
    This class allows communication with hardware so that the robot can navigate in any direction.
    Navigation is possible by varying speed of individual motors.

    :param `left_motor`:
        Pins associated with the left motor.
    :param `right_motor`:
        Pins associated with the right motor.
    :param `curve_scale`:
        Amount of curvature when moving in a CURVILINEAR motion.
        `Default` set to 0.6 which was determined to be acceptable.
    """

    # TODO: M-15:Remove hardcoded curve scale.
    #       The curve_scale should be set in a config file.
    def __init__(
        self, left_motor: int, right_motor: int, curve_scale: float = 0.6
    ) -> None:
        # Set up pin factory for better control of GPIO pins.
        self.factory = PiGPIOFactory()
        self.motors = Robot(
            left=left_motor, right=right_motor, pin_factory=self.factory
        )
        self.curve_scale = curve_scale  # (m/s)

    def run(self, linear_x: float, angular_z: float, motion_type: str) -> None:
        """
        Adjust motor speed based on the motion type and communicate with the hardware.

        Args:
            `linear_x (float)`: Linear velocity in the x direction.
            `angular_z (float)`: Angular velocity in the z direction.
            `motion_type (str)`: Type of motion. If not provided, it will be determined
            automatically.

        Returns:
            `None`
        """
        linear_x, angular_z = constrain_vel_range(linear_x, angular_z)

        if motion_type is CURVILINEAR:
            if linear_x > 0.0 and angular_z < 0.0:
                self.motors.forward(speed=abs(angular_z), curve_right=self.curve_scale)

            elif linear_x > 0.0 and angular_z > 0.0:
                self.motors.forward(speed=angular_z, curve_left=self.curve_scale)

            elif linear_x < 0.0 and angular_z > 0.0:
                self.motors.backward(speed=angular_z, curve_right=self.curve_scale)

            elif linear_x < 0.0 and angular_z < 0.0:
                self.motors.backward(speed=abs(angular_z), curve_left=self.curve_scale)

        elif motion_type is LINEAR:
            if linear_x > 0:
                self.motors.forward(speed=linear_x)
            else:
                self.motors.backward(speed=abs(linear_x))

        elif motion_type is ROTATIONAL:
            if angular_z > 0:
                self.motors.left(speed=angular_z)
            else:
                self.motors.right(speed=abs(angular_z))

        else:
            self.motors.stop()

    def disconnect(self) -> None:
        """
        Stop the motors and release the motor pins.
        """
        self.motors.stop()
        self.factory.close()
