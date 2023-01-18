#!/usr/bin/env python3
"""
ROS wrapper node that communicates with the motor/driver interface on the Yahboom G1 Tank.
"""
import sys
from threading import Lock

import rospy
from geometry_msgs.msg import Twist

from yb_expansion_board.robotEnums import Message, MotionType
from yb_expansion_board.ybMotors import YBMotor
from expansion_board_driver.msg import Collision

# Type of motion.
CURVILINEAR = MotionType.CURVILINEAR.value
LINEAR = MotionType.LINEAR.value
NO_MOTION = MotionType.NO_MOTION.value
ROTATIONAL = MotionType.ROTATIONAL.value

# Log Messages
ERR_ROSINIT = Message.ROSINIT.value
ERR_PARAM = Message.PARAM.value


class MotorControl:
    """
    This class controls the motors of a robot to navigate in any direction.
    """

    def __init__(self) -> None:
        rospy.loginfo("Setting up motors.")
        try:
            self.motors = YBMotor(
                left_motor=tuple(rospy.get_param("/hardware/motor/left_rear")),
                right_motor=tuple(rospy.get_param("/hardware/motor/right_rear")),
            )
        except KeyError:
            rospy.logerr(ERR_PARAM)
            sys.exit()

        # Subscribers
        self.rear_collision_sub = rospy.Subscriber(
            "/rear_collision",
            Collision,
            self.rear_collision_callback,
        )
        self.vel_sub = rospy.Subscriber(
            "/cmd_vel",
            Twist,
            self.velocity_callback,
        )

        # Create lock to prevent race conditions.
        self.motor_lock = Lock()

        # Initialize linear and angular velocities to 0.0 (m/s)
        self.linear_x = 0.0
        self.angular_z = 0.0

        # Initially, collision is not an issue until we start moving.
        self.collision = False

        # Rate is set to allow motors to process the recent velocity
        # changes. (Hz)
        self.rate = rospy.Rate(15)

    def rear_collision_callback(self, data: Collision) -> None:
        """
        Callback function that receives incoming data from ultrasonic
        sensor.

        :param data:
            Information received from ultrasonic sensor.
        """

        object_distance = data.distance
        min_distance = data.min_collision_dist
        with self.motor_lock:
            if object_distance < min_distance:
                self.collision = True
                self.linear_x = self.angular_z = 0.0

    def velocity_callback(self, vel: Twist) -> None:
        """
        Callback function that receives incoming velocity.

        :param vel:
            Velocity which stores the linear and angular velocities in the
            x, y and z directions.
        """

        with self.motor_lock:
            # We need to keep the velocities between 0.0 and 1.0 or the run() function will
            # crash.
            if not self.collision:
                self.linear_x = vel.linear.x if abs(vel.linear.x) < 1 else 1
                self.angular_z = vel.angular.z if abs(vel.angular.z) < 1 else 1

    # TODO: Add a timer to check how long its been since we were in teleop mode without a
    # new message received. If time is exceeded then set teleop mode to false.
    def loop(self) -> None:
        """
        Loop that changes the motor speed.
        """
        while not rospy.is_shutdown():
            with self.motor_lock:
                self.motors.run(self.linear_x, self.angular_z)
                self.linear_x = self.angular_z = 0.0
                self.collision = False
            self.rate.sleep()


if __name__ == "__main__":
    try:
        rospy.init_node("motor_controller_node")
        motorControl = MotorControl()
        motorControl.loop()
        rospy.spin()
        rospy.loginfo("Releasing pins and shutting down motors.")
        motorControl.motors.disconnect()
    except rospy.ROSInitException:
        rospy.logerr(ERR_ROSINIT)
