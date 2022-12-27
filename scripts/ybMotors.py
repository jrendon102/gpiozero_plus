#!/usr/bin/env python3
"""
This script utilizes the gpiozero library to access the GPIO pins on the Raspberry Pi. The purpose
is to allow communication with motors attached to the Yahboom G1 Tank Expansion Board. The speed
of the motors are varied to allow maneuverability in any direction.
"""
from gpiozero import Robot
from gpiozero.pins.pigpio import PiGPIOFactory

from motion import determine_motion
from robotEnums import MotionType

# Type of motion.
CURVILINEAR = MotionType.CURVILINEAR.value
LINEAR = MotionType.LINEAR.value
NO_MOTION = MotionType.NO_MOTION.value
ROTATIONAL = MotionType.ROTATIONAL.value


class YBMotor:
    """
    This class allows communication with hardware so that the robot can navigate in any direction.
    Navigation is possible by varying speed of individual motors.
    """

    def __init__(
        self, left_motor: int, right_motor: int, curve_scale: float = 0.6
    ) -> None:
        """
        :param `left_motor`:
            Pins associated with the left motor.
        :param `right_motor`:
            Pins associated with the right motor.
        :param `curve_scale`:
            Amount of curvature when moving in a CURVILINEAR motion.
            `Default` set to 0.6 which was determined to be acceptable.
        """

        # Set up pin factory for better control of GPIO pins.
        self.factory = PiGPIOFactory()
        # Set up left & right motors. Pins are set in config file prior to launch.
        self.motors = Robot(
            left=left_motor, right=right_motor, pin_factory=self.factory
        )

        # CURVILINEAR motion scale (m/s)
        self.curve_scale = curve_scale

    def run(self, linear_x: float, angular_z: float) -> None:
        """
        Communicate with hardware and vary speeds of motors according to the type of motion that
        was provided.

        :param linear_x:
            The linear velocity in the x direction.
        :param angular_z:
            The angular velocity in the z direction.
        """
        # Determine the direction of motion to navigate towards according to the linear and
        # angular velocities.
        motion_type = determine_motion(linear_x, angular_z)

        # Drive along a curved path.
        if motion_type is CURVILINEAR:
            # Curve forwards towards the right (frame of robot)
            if linear_x > 0.0 and angular_z < 0.0:
                self.motors.forward(speed=abs(angular_z), curve_right=self.curve_scale)

            # Curve forwards towards the left (frame of robot)
            elif linear_x > 0.0 and angular_z > 0.0:
                self.motors.forward(speed=angular_z, curve_left=self.curve_scale)

            # Curve backwards towards the right (frame of robot)
            elif linear_x < 0.0 and angular_z > 0.0:
                self.motors.backward(speed=angular_z, curve_right=self.curve_scale)

            # Curve backwards towards the left (frame of robot)
            elif linear_x < 0.0 and angular_z < 0.0:
                self.motors.backward(speed=abs(angular_z), curve_left=self.curve_scale)

        # Drive linearly either forwards or backwards.
        elif motion_type is LINEAR:
            if linear_x > 0:
                self.motors.forward(speed=linear_x)
            else:
                self.motors.backward(speed=abs(linear_x))

        # Rotate about z-axis.
        elif motion_type is ROTATIONAL:
            if angular_z > 0:
                self.motors.left(speed=angular_z)
            else:
                self.motors.right(speed=abs(angular_z))

        # Stop motors
        else:
            self.motors.stop()

    def disconnect(self) -> None:
        """
        Stop the motors and release the motor pins.
        """
        self.motors.stop()
        self.factory.close()
