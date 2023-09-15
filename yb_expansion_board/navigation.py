#!/usr/bin/env python3
"""
This script provides functions which describe different types of motion during
navigation.
"""
from yb_expansion_board.robotEnums import MotionType

# Type of motion.
CURVILINEAR = MotionType.CURVILINEAR.value
LINEAR = MotionType.LINEAR.value
NO_MOTION = MotionType.NO_MOTION.value
ROTATIONAL = MotionType.ROTATIONAL.value


def eval_motion(linear_x: float, angular_z: float) -> str:
    """
    Evaluate the motion of a robot based on its linear and angular velocity values.

    Args:
        `linear_x (float)`: The linear velocity value of the robot.
        `angular_z (float)`: The angular velocity value of the robot.

    Returns:
        A string indicating the type of motion of the robot. Possible values are:
        - `"CURVILINEAR"`: Indicates both linear and angular velocities are non-zero.
        - `"LINEAR"`: Indicates only the linear velocity is non-zero.
        - `"ROTATIONAL"`: Indicates only the angular velocity is non-zero.
        - `"NO_MOTION"`: Indicates both linear and angular velocities are zero.
    """
    if linear_x and angular_z:
        return CURVILINEAR
    if linear_x and not angular_z:
        return LINEAR
    if not linear_x and angular_z:
        return ROTATIONAL
    return NO_MOTION


def constrain_vel_range(linear_x: float, angular_z: float) -> "tuple[float, float]":
    """
    Constrain the linear and angular velocity values to a range of -1 to 1.

    Args:
        `linear_x (float)`: The linear velocity value to be constrained.
        `angular_z (float)`: The angular velocity value to be constrained.

    Returns:
        `tuple[float, float]`: A tuple containing the constrained linear and angular velocity
        values.
    """
    constrained_linear_x = max(min(linear_x, 1.0), -1.0)
    constrained_angular_z = max(min(angular_z, 1.0), -1.0)
    return constrained_linear_x, constrained_angular_z
