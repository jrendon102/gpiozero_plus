#!/usr/bin/env python3
"""
This script provides functions which describe different types of motion during
navigation.
"""
from robotEnums import MotionType

# Type of motion.
CURVILINEAR = MotionType.CURVILINEAR.value
LINEAR = MotionType.LINEAR.value
NO_MOTION = MotionType.NO_MOTION.value
ROTATIONAL = MotionType.ROTATIONAL.value


def determine_motion(linear_x: float, angular_z: float) -> str:
    """
    Determine the direction of motion depending on the linear x velocity and angular z
    velocity.

    :param linear_x:
        The linear velocity in the x direction.
    :param angular_z:
        The angular velocity in the z direction.
    """
    if linear_x and angular_z:
        return CURVILINEAR
    if linear_x and not angular_z:  # Angular z velocity is 0.0 m/s
        return LINEAR
    if not linear_x and angular_z:  # linear x velocity is 0.0 m/s
        return ROTATIONAL
    return NO_MOTION  # Both linear x and Angular z velocities are 0.0 m/s
