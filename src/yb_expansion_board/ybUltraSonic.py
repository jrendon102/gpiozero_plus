#!/usr/bin/env python3
"""
This script utilizes the gpiozero library to access the GPIO pins on the Raspberry Pi. The purpose
is to allow communication with ultrasonic sensor attached to the Yahboom G1 Tank Expansion Board.
"""

from gpiozero import DistanceSensor
from gpiozero.pins.pigpio import PiGPIOFactory


class YBUltrasonic:
    """
    This class allows communication with the Ultrasonic interface on the Yahboom G1 Tank expansion
    board. It sets up the Ultrasonic sensor and calculates the distance between itself and an object.
    Purpose is to provide functionality for object detection.

    :param `echo_pin`:
        Echo pin number. This is connected to SDA.
    :param `trig_pin`:
        Trig pin number. This is connected to SCL.
    """

    def __init__(self, echo_pin: int, trig_pin: int) -> None:
        # Set up pin_factory to get better control of the GPIO pins.
        self.factory = PiGPIOFactory()

        # Set up Ultrasonic Sensor
        self.sensor = DistanceSensor(
            echo=echo_pin,
            trigger=trig_pin,
            pin_factory=self.factory,
        )

    def get_distance(self) -> float:
        """
        Returns the distance between sensor and object.
        """
        return self.sensor.distance

    def disconnect(self) -> None:
        """
        Release the pins.
        """
        self.factory.close()
