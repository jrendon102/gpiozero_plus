#!/usr/bin/python3
"""
A Python class for controlling a fan using the gpiozero library.

This script defines a class, Fan, that allows you to control a fan using the gpiozero library.
You can set the initial state of the fan and toggle it on or off.

Author: Julian A Rendon
Copyright (c) 2023 
License: MIT License
Last Updated: September 15, 2023
"""

from gpiozero import DigitalOutputDevice
from gpiozero.pins.pigpio import PiGPIOFactory


class Fan:
    """
    A class for controlling a fan using the gpiozero library.

    Parameters:
        `fan_pin (int)`: The GPIO pin connected to the fan.

        `initial_enabled (bool, optional)`: The initial state of the fan (default is False, i.e., fan is off).
    """

    def __init__(self, fan_pin: int, initial_enabled: bool = False) -> None:
        """
        Initializes the Fan instance.
        """
        self.__factory = PiGPIOFactory()
        self.__fan = DigitalOutputDevice(pin=fan_pin, active_high=False, pin_factory=self.__factory)
        self.set_fan_state(enabled=initial_enabled)

    def set_fan_state(self, enabled: bool) -> None:
        """
        Sets the state of the fan.

        Parameters:
            `enabled (bool)`: True to turn the fan on, False to turn it off.
        """
        if enabled:
            self.__fan.on()
        else:
            self.__fan.off()

    def disconnect(self) -> None:
        """
        Disconnects the fan and releases the GPIO pin.
        """
        self.__fan.off()
        self.__factory.close()
