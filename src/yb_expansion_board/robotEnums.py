#!/usr/bin/env python3
"""
This provides useful enum classes for various applications. Reduces the amount of repetitive
hardcoding throughout package.
"""
from enum import Enum


class Message(Enum):
    """
    An enumeration of common logging messages.

    Constants:
        `PARAM` (str):  Message when ROSParamException occurs.
        `ROSINIT` (str): Message when ROSInitException occurs.
    """

    PARAM = "Could not get parameter values to set up hardware."
    ROSINIT = "Error occurred when initializing ROS state."


class MotionType(Enum):
    """
    An enumeration of robot motion types.

    Constants:
        `CURVILINEAR` (str): Curvilinear motion. (xz-axis)
        `LINEAR` (str): Linear motion. (xz-axis)
        `ROTATIONAL` (str): Rotational motion. (xz-axis)
        `NO_MOTION` (str): No motion. (xz-axis)
    """

    CURVILINEAR = "CURVILINEAR"
    LINEAR = "LINEAR"
    ROTATIONAL = "ROTATIONAL"
    NO_MOTION = "NO MOTION"


class CollisionType(Enum):
    """
    An enumeration of collision types.

    Constants:
        FORWARD_COLLISION (str): Forward collision.
        REAR_COLLISION (str): Rear collision.
    """

    FORWARD_COLLISION = "FORWARD COLLISION"
    REAR_COLLISION = "REAR COLLISION"


class LEDMode(Enum):
    """
    An enumeration of LED modes.

    Constants:
        `FLASHLIGHT` (str): RGB LED acts as flashlight and provides full brightness (1,1,1).
        `IDLE_MODE` (str): Color pattern for RBG LED during idle operations.
        `OFF_MODE` (str): RGB LED turned off.
        `TEST_MODE` (str): Test the RGB LED by testing red, blue and green intensities.
    """

    FLASHLIGHT_MODE = "FLASHLIGHT"
    IDLE_MODE = "IDLE"
    OFF_MODE = "OFF"
    TEST_MODE = "TEST"


class Colors(Enum):
    """
    An enumeration of RGB color values.

    Constants:
        `RED` (Tuple[float, float, float]): The RGB value for red.
        `GREEN` (Tuple[float, float, float]): The RGB value for green.
        `BLUE` (Tuple[float, float, float]): The RGB value for blue.
        `YELLOW` (Tuple[float, float, float]): The RGB value for yellow.
        `CYAN` (Tuple[float, float, float]): The RGB value for cyan.
        `MAGENTA` (Tuple[float, float, float]): The RGB value for magenta.
        `WHITE` (Tuple[float, float, float]): The RGB value for white.
        `BLACK` (Tuple[float, float, float]): The RGB value for black.
    """

    RED = (1, 0, 0)
    GREEN = (0, 1, 0)
    BLUE = (0, 0, 1)
    YELLOW = (1, 1, 0)
    CYAN = (0, 1, 1)
    MAGENTA = (1, 0, 1)
    WHITE = (1, 1, 1)
    BLACK = (0, 0, 0)
