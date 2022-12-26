#!/usr/bin/env python3
"""
This script utilizes the gpiozero library to access the GPIO pins on the Raspberry Pi. The purpose
is to allow communication with the RGB LED interface on the Yahboom G1 Tank expansion board.
"""
from gpiozero import RGBLED
from gpiozero.pins.pigpio import PiGPIOFactory

from robotEnums import LEDMode, Colors

# Modes for different LED settings.
FLASHLIGHT = LEDMode.FLASHLIGHT_MODE.value
OFF = LEDMode.OFF_MODE.value
TEST = LEDMode.TEST_MODE.value


# Colors
BLACK = Colors.BLACK.value
BLUE = Colors.BLUE.value
GREEN = Colors.GREEN.value
RED = Colors.RED.value
WHITE = Colors.WHITE.value


class YBLED:
    """
    This class allows communication with the RGB LED interface on the Yahboom G1 Tank expansion
    board. It sets up the RGB LED and changes the color and brightness of the LED.

    :param red_led:
        The pin # for the red led.
    :param green_led:
        The pin # for the green led.
    :param blue_led:
        The pin # for the blue led.
    """

    def __init__(self, red_led: int, green_led: int, blue_led: int) -> None:

        # Set up pin_factory to get better control of GPIO pin.
        self.factory = PiGPIOFactory()

        # Set up LEDs according to their respective pin numbers.
        self.rgb_led = RGBLED(
            red=red_led, green=green_led, blue=blue_led, pin_factory=self.factory
        )

    def adjust_brightness(self, led_color: tuple, scalar: float) -> tuple:
        """
        Adjust the brightness of the LED.

        :param led_color:
            The LED color.

        returns the LED color with adjusted brightness as a `tuple`.
        """
        color = tuple(value * scalar for value in led_color)
        return color

    def change_color(self, led_mode: str, brightness: float = None) -> None:
        """
        Vary the power output to the RGB LEDs depending on the led mode that was set. This will
        change the color of the LEDs based what kind of interactions the robot is having with a
        user or environment.

        :param led_mode:
            The mode that determines what colors the LED should be.
        :param brightness:
            Scalar that determines how bright the RGB LED should be.Value should be a number (float)
            between 0 and 1.
        """

        # Initialize color as WHITE at full brightness (1,1,1)
        color = Colors.WHITE.value

        if led_mode == TEST:
            color = self.adjust_brightness(RED, brightness) if brightness else RED
            self.rgb_led.blink(
                on_time=0.5,
                off_time=0.5,
                on_color=color,
                n=1,
                background=False,
            )
            # Green
            color = self.adjust_brightness(GREEN, brightness) if brightness else GREEN
            self.rgb_led.blink(
                on_time=0.5,
                off_time=0.5,
                on_color=color,
                n=1,
                background=False,
            )
            # Blue
            color = self.adjust_brightness(BLUE, brightness) if brightness else BLUE
            self.rgb_led.blink(
                on_time=0.5,
                off_time=0.5,
                on_color=color,
                n=1,
                background=False,
            )
            # Blink White n times
            self.rgb_led.blink(
                on_time=0.01,
                on_color=color,
                off_time=0.1,
                n=3,
                background=False,
            )

        elif led_mode == FLASHLIGHT:
            self.rgb_led.value = (
                self.adjust_brightness(WHITE, brightness) if brightness else WHITE
            )

        elif led_mode == OFF:
            self.rgb_led.value = BLACK

        # TODO: Create a unique pattern for IDLE.
        # elif led_mode == IDLE:
        #     pass

    def disconnect(self) -> None:
        """
        Turn off the RGB LED and release the pins.
        """
        self.change_color(OFF)
        self.factory.close()
