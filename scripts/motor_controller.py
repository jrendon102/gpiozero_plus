#!/usr/bin/env python3
"""
ROS wrapper node that communicates with the motor/driver interface on the Yahboom G1 Tank.
"""
import sys
from threading import Lock

import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist

from robotEnums import Message, MotionType
from ybMotors import YBMotor

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
                left_motor=tuple(rospy.get_param("/hardware/left_motor")),
                right_motor=tuple(rospy.get_param("/hardware/right_motor")),
            )
        except KeyError:
            rospy.logerr(ERR_PARAM)
            sys.exit()

        # Publisher
        self.teleop_pub = rospy.Publisher("/teleop_mode", Bool, queue_size=1)

        # Subscribers
        self.teleop_vel_sub = rospy.Subscriber(
            "/teleop_cmd_vel", Twist, self.teleop_vel_callback
        )
        self.vel_sub = rospy.Subscriber("/cmd_vel", Twist, self.velocity_callback)

        # Create lock to prevent race conditions.
        self.vel_lock = Lock()

        # Initialize linear and angular velocities to 0.0 (m/s)
        self.linear_x = 0.0
        self.angular_z = 0.0

        # Initialize teleop mode message as False.
        self.teleop_msg = Bool()
        self.teleop_msg.data = False

        # Rate which determines when to stop motors if no velocity messages have been received.
        self.rate = rospy.Rate(1)

    def teleop_vel_callback(self, vel: Twist) -> None:
        """
        Callback function that receives the velocity messages when teleoperting.

        :param `vel`:
            Velocity which stores the linear and angular velocities in the
            x, y and z directions.
        """

        with self.vel_lock:
            self.linear_x = vel.linear.x if abs(vel.linear.x) < 1 else 1
            self.angular_z = vel.angular.z if abs(vel.angular.z) < 1 else 1
            self.teleop_msg.data = True

    def velocity_callback(self, vel: Twist) -> None:
        """
        Callback function that receives incoming velocity.

        :param vel:
            Velocity which stores the linear and angular velocities in the
            x, y and z directions.
        """

        # If currently in teleop mode we want to skip this. Teleop mode will take precedence.
        if not self.teleop_msg.data:
            with self.vel_lock:
                # Keep the velocities between 0.0 and 1.0.
                self.linear_x = vel.linear.x if abs(vel.linear.x) < 1 else 1
                self.angular_z = vel.angular.z if abs(vel.angular.z) < 1 else 1

    # FIXME: Add a timer to check how long its been since we were in teleop mode without a
    # new message received. If time is exceeded then set teleop mode to false.
    def loop(self) -> None:
        """
        Loop that changes the motor speed. Publishes whether in teleop mode or not.
        """
        while not rospy.is_shutdown():
            with self.vel_lock:
                self.motors.run(self.linear_x, self.angular_z)
                self.linear_x = self.angular_z = 0.0
                self.teleop_pub.publish(self.teleop_msg)
                self.teleop_msg.data = False
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
