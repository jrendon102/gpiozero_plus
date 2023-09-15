#!/usr/bin/python3
"""
This script utilizes the gpiozero library to access the GPIO pins on the Raspberry Pi. The purpose
is to allow communication with the FAN interface on the Yahboom G1 Tank expansion board.
"""

from gpiozero import DigitalOutputDevice
from gpiozero.pins.pigpio import PiGPIOFactory


class YBFan:
    """
    This class allows communication with the FAN interface on the Yahboom G1 Tank expansion
    board. It sets up the FAN and turns it off or on.

    :param `fan_pin`:
        The pin # for the fan.
    :param `initial_mode`:
        The initial mode of the fan. `Default` set to `False` (off).
    """

    def __init__(self, fan_pin: int, initial_mode: bool = False) -> None:
        # Set up pin_factory to get better control of the GPIO pins.
        self.factory = PiGPIOFactory()

        # Set up Fan and have it initially ON when started.
        self.ybfan = DigitalOutputDevice(
            pin=fan_pin, active_high=False, pin_factory=self.factory
        )

        self.control_fan(fan_enable=initial_mode)

    def control_fan(self, fan_enable: bool):
        """
        Turns the fan OFF or ON.

        :param `fan_enable`:
            Whether the fan should be turned off or on.
        """

        if fan_enable:
            self.ybfan.on()
        else:
            self.ybfan.off()

    def disconnect(self) -> None:
        """
        Turn off the RGB LED and release the pins.
        """
        self.ybfan.off()
        self.factory.close()
