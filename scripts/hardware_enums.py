#!/usr/bin/env python3
"""
This provides useful enum classes for various applications. Reduces the amount of repetitive
hardcoding throughout package.
"""
from enum import Enum


class Message(Enum):
    """This Enum class is useful when logging common messages."""

    PARAM = "Could not get parameter values."
    ROSINIT = "Error occurred when initializing ROS state."


class MotionType(Enum):
    """This Enum class is useful when referring to robot navigation."""

    CURVILINEAR = "CURVILINEAR"
    LINEAR = "LINEAR"
    ROTATIONAL = "ROTATIONAL"
    NO_MOTION = "NO MOTION"


class LEDMode(Enum):
    """This Enum class is useful when referring to LED modes."""

    FLASHLIGHT_MODE = "FLASHLIGHT_MODE"  # Turns on LEDs to full brightness.
    OFF_MODE = "OFF"
