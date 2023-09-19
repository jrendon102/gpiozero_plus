#!/usr/bin/env python3
"""
This file contains enumerations for various use cases.

Author: Julian A Rendon
Copyright (c) 2023
License: MIT License
Last Updated: September 19, 2023
"""

from enum import Enum


class Motion(Enum):
    """
    An enumeration representing different motion types for a robot.

    Constants:
        - `STR_FWD` (str): Straight forward motion.
        - `STR_BK` (str): Straight backward motion.
        - `LEFT_FWD` (str): Left forward motion.
        - `RIGHT_FWD` (str): Right forward motion.
        - `LEFT_BK` (str): Left backward motion.
        - `RIGHT_BK` (str): Right backward motion.
        - `ROT_LEFT` (str): Rotational left motion.
        - `ROT_RIGHT` (str): Rotational right motion.
        - `NO_MOTION` (str): No motion.
    """

    STR_FWD = "STRAIGHT_FORWARD"
    STR_BK = "STRAIGHT_BACKWARD"
    LEFT_FWD = "LEFT_FORWARD"
    RIGHT_FWD = "RIGHT_FORWARD"
    LEFT_BK = "LEFT_BACKWARD"
    RIGHT_BK = "RIGHT_BACKWARD"
    ROT_LEFT = "ROTATIONAL_LEFT"
    ROT_RIGHT = "ROTATIONAL_RIGHT"
    NO_MOTION = "NO_MOTION"


class Colors(Enum):
    """
    An enumeration of RGB color values.

    Constants:
        - `BLACK` (Tuple[float, float, float]): The RGB value for black.
        - `WHITE` (Tuple[float, float, float]): The RGB value for white.
        - `RED` (Tuple[float, float, float]): The RGB value for red.
        - `GREEN` (Tuple[float, float, float]): The RGB value for green.
        - `BLUE` (Tuple[float, float, float]): The RGB value for blue.
        - `YELLOW` (Tuple[float, float, float]): The RGB value for yellow.
        - `CYAN` (Tuple[float, float, float]): The RGB value for cyan.
        - `MAGENTA` (Tuple[float, float, float]): The RGB value for magenta.
        - `ORANGE` (Tuple[float, float, float]): The RGB value for orange.
        - `PURPLE` (Tuple[float, float, float]): The RGB value for purple.
        - `PINK` (Tuple[float, float, float]): The RGB value for pink.
        - `LIME` (Tuple[float, float, float]): The RGB value for lime.
        - `GOLD` (Tuple[float, float, float]): The RGB value for gold.
        - `SILVER` (Tuple[float, float, float]): The RGB value for silver.
        - `GRAY` (Tuple[float, float, float]): The RGB value for gray.
        - `BROWN` (Tuple[float, float, float]): The RGB value for brown.
        - `NAVY` (Tuple[float, float, float]): The RGB value for navy.
        - `TEAL` (Tuple[float, float, float]): The RGB value for teal.
        - `OLIVE` (Tuple[float, float, float]): The RGB value for olive.
        - `MAROON` (Tuple[float, float, float]): The RGB value for maroon.
        - `INDIGO` (Tuple[float, float, float]): The RGB value for indigo.
        - `TURQUOISE` (Tuple[float, float, float]): The RGB value for turquoise.
        - `SALMON` (Tuple[float, float, float]): The RGB value for salmon.
        - `VIOLET` (Tuple[float, float, float]): The RGB value for violet.
        - `BEIGE` (Tuple[float, float, float]): The RGB value for beige.
    """

    BLACK = (0, 0, 0)
    WHITE = (1, 1, 1)
    RED = (1, 0, 0)
    GREEN = (0, 1, 0)
    BLUE = (0, 0, 1)
    YELLOW = (1, 1, 0)
    CYAN = (0, 1, 1)
    MAGENTA = (1, 0, 1)
    ORANGE = (1, 0.5, 0)
    PURPLE = (0.5, 0, 0.5)
    PINK = (1, 0.5, 0.5)
    LIME = (0.5, 1, 0)
    GOLD = (1, 0.843, 0)
    SILVER = (0.753, 0.753, 0.753)
    GRAY = (0.5, 0.5, 0.5)
    BROWN = (0.647, 0.165, 0.165)
    NAVY = (0, 0, 0.5)
    TEAL = (0, 0.5, 0.5)
    OLIVE = (0.5, 0.5, 0)
    MAROON = (0.5, 0, 0)
    INDIGO = (0.294, 0, 0.509)
    TURQUOISE = (0.251, 0.878, 0.816)
    SALMON = (0.918, 0.6, 0.6)
    VIOLET = (0.933, 0.51, 0.933)
    BEIGE = (0.961, 0.961, 0.863)
