#!/usr/bin/env python3
"""
A Python class for controlling an ultrasonic sensor using gpiozero library.

This script defines a class, Ultrasonic, for interfacing with an ultrasonic sensor.
It provides methods to get the distance measured by the sensor, retrieve sensor information,
and disconnect the sensor.

Author: Julian A Rendon
Copyright (c) 2023
License: MIT License
Last Updated: September 15, 2023
"""

from gpiozero import DistanceSensor
from gpiozero.pins.pigpio import PiGPIOFactory


class Ultrasonic:
    """
    A class for controlling an ultrasonic sensor using gpiozero library.

    Args:
        echo_pin (int): The GPIO pin connected to the sensor's echo pin.
        trig_pin (int): The GPIO pin connected to the sensor's trigger pin.
        max_distance (float): The maximum distance that the sensor can measure.
        threshold_distance (float): The distance that triggers the "in range" event.

    Attributes:
        sensor (DistanceSensor): The DistanceSensor instance.
    """

    def __init__(
        self,
        echo_pin: int,
        trig_pin: int,
        max_distance: float,
        threshold_distance: float,
    ) -> None:
        """
        Initializes the Ultrasonic instance.

        Parameters:
            `echo_pin (int)`: The GPIO pin connected to the sensor's echo pin.

            `trig_pin (int)`: The GPIO pin connected to the sensor's trigger pin.

            `max_distance (float)`: The maximum distance that the sensor can measure.

            `threshold_distance (float)`: The distance that triggers the "in range" event.
        """
        self.__factory = PiGPIOFactory()
        self.__sensor = DistanceSensor(
            echo=echo_pin,
            trigger=trig_pin,
            max_distance=max_distance,
            threshold_distance=threshold_distance,
            pin_factory=self.__factory,
        )

    def get_distance(self) -> float:
        """
        Get the distance measured by the sensor if an object is in range.

        Returns:
            `float`: The measured distance in meters, or `None` if no object is in range.
        """
        if self.__sensor.in_range:
            return self.__sensor.distance

    def get_sensor_info(self) -> dict:
        """
        Get information about the sensor configuration.

        Returns:
            `dict`: A dictionary containing sensor information including `max_distance`,
                  `threshold_distance`, `value`, `trigger_pin`, and `echo_pin`.
        """
        info = {
            "max_distance": self.__sensor.max_distance,
            "threshold_distance": self.__sensor.threshold_distance,
            "value": self.__sensor.value,
            "trigger_pin": self.__sensor.sensor.trigger,
            "echo_pin": self.__sensor.echo,
        }
        return info

    def disconnect(self) -> None:
        """
        Disconnect the ultrasonic sensor and release the GPIO pins.
        """
        self.__sensor.close()
