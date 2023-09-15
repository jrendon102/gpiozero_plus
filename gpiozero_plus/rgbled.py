#!/usr/bin/env python3
"""
A Python class for controlling an RGB LED using gpiozero library.

This script defines a class, RGBLEDController, to control an RGB LED using the gpiozero library.
You can set the initial color and brightness, change colors, adjust brightness, toggle the LED state,
and create smooth color transitions.

Author: Julian A Rendon
Copyright (c) 2023
License: MIT License
Last Updated: September 15, 2023
"""

import time

from gpiozero import RGBLED
from gpiozero.pins.pigpio import PiGPIOFactory

DEFAULT_COLOR = (0, 0, 0)
DEFAULT_BRIGHTNESS = 0.0


class RGBLEDController:
    """
    A class for controlling an RGB LED using gpiozero library.

    Parameters:
        `red_pin`: The GPIO pin for the red LED component (default is RED_PIN).

        `green_pin`: The GPIO pin for the green LED component (default is GREEN_PIN).

        `blue_pin`: The GPIO pin for the blue LED component (default is BLUE_PIN).

        `initial_color`: The initial RGB color as a tuple (red, green, blue). Default is (0, 0, 0).

        `initial_brightness`: The initial brightness value between 0 (off) and 1 (full brightness). Default is 0.0.

    """

    def __init__(
        self,
        red_pin: int,
        green_pin: int,
        blue_pin: int,
        initial_color: tuple = DEFAULT_COLOR,
        initial_brightness: float = DEFAULT_BRIGHTNESS,
    ) -> None:
        """
        Initializes the RGBLEDController instance.
        """
        self.__pin_factory = PiGPIOFactory()
        self.__rgb_led = RGBLED(
            red_pin,
            green_pin,
            blue_pin,
            pin_factory=self.__pin_factory,
        )
        if initial_brightness != DEFAULT_BRIGHTNESS:
            self.change_color(initial_color, initial_brightness)
        else:
            self.__rgb_led.color = initial_color

    def toggle(self):
        """
        Toggles the state of the RGB LED.
        """
        self.__rgb_led.color = DEFAULT_COLOR

    def adjust_brightness(self, rgb_led: tuple, brightness: float) -> None:
        """
        Adjusts the brightness of an RGB color.

        Args
            `rgb_led`: The RGB color as a tuple (red, green, blue).

            `brightness`: The brightness value between 0 (off) and 1 (full brightness).

        Returns:
            None
        """
        return tuple(value * brightness for value in rgb_led)

    def change_color(self, color: tuple, brightness: float = 1.0) -> None:
        """
        Changes the color of the RGB LED.

        Args:
            `color`: The target RGB color as a tuple (red, green, blue).

            `brightness`: The brightness value between 0 (off) and 1 (full brightness).

            `raises ValueError`: If the brightness value is outside the valid range [0, 1].

        Returns:
            None

        """
        if 0.0 <= brightness <= 1.0:
            color = self.adjust_brightness(color, brightness)
            self.__rgb_led.color = color
        else:
            raise ValueError("Invalid brightness value")

    def cycle_colors(self, color_list: list, interval: float) -> None:
        """
        Cycles through a list of colors at a specified interval.

        Args:
            `color_list`: A list of RGB colors to cycle through.

            `interval`: The time interval (in seconds) between color changes.

        Returns:
            None
        """
        for color in color_list:
            self.change_color(color)
            time.sleep(interval)

    def fade_to_color(
        self,
        from_color: tuple,
        to_color: tuple,
        duration: float,
        brightness: float = 1.0,
    ) -> None:
        """
        Fades the RGB LED smoothly from one color to another over a specified duration.

        Args:
            `from_color`: The starting RGB color as a tuple (red, green, blue).

            `to_color`: The target RGB color as a tuple (red, green, blue).

            `duration`: The duration of the smooth fade effect in seconds.

        Returns:
            None
        """
        num_steps = 100 * duration
        delay = duration / num_steps
        from_r, from_g, from_b = from_color
        to_r, to_g, to_b = to_color

        for step in range(num_steps + 1):
            brightness = step / num_steps
            r = from_r + (to_r - from_r) * brightness
            g = from_g + (to_g - from_g) * brightness
            b = from_b + (to_b - from_b) * brightness
            self.change_color((r, g, b))
            time.sleep(delay)

    def disconnect(self) -> None:
        """
        Disconnects the RGB LED.

        Returns:
            None
        """
        self.__rgb_led.close()
