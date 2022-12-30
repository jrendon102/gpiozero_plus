#!/usr/bin/env python3
"""
This script utilizes the gpiozero library to access the GPIO pins on the Raspberry Pi. The purpose
is to allow communication with ultra sonic sensor(s) attached to the Yahboom G1 Tank Expansion Board.
"""

from gpiozero import DistanceSensor
from gpiozero.pins.pigpio import PiGPIOFactory

# Set up pin factory for better control of GPIO pins.
FACTORY = PiGPIOFactory()


def get_distance(echo_pin: int, trig_pin: int) -> float:
    """
    Sets up the ultrasonic senor and returns the distance between sensor and object.
    """

    ultra_sonic_sensor = DistanceSensor(
        echo=echo_pin, trigger=trig_pin, pin_factory=FACTORY
    )
    return ultra_sonic_sensor.distance()
