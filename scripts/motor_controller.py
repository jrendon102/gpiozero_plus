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
from mach1_msgs.msg import Collision

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
                left_motor=tuple(rospy.get_param("/exp_board/motor/left_rear")),
                right_motor=tuple(rospy.get_param("/exp_board/motor/right_rear")),
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
        self.joy_vel_sub = rospy.Subscriber(
            "joy_cmd_vel",  # Originally cmd_vel but remapped.
            Twist,
            self.joystick_vel_callback,
        )

        self.joystick_teleop = None
        self.collision = False

        # Create lock to prevent race conditions.
        self.motor_lock = Lock()

        # Initialize linear and angular velocities to 0.0 (m/s)
        self.linear_x = 0.0
        self.angular_z = 0.0

        # Sets the rate for incoming messages (Hz)
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

    def joystick_vel_callback(self, vel: Twist) -> None:
        """
        Callback function that receives incoming joystick
        velocity.

        :param vel:
            Velocity which stores the linear and angular velocities in the
            x, y and z directions.
        """
        if not self.collision and self.joystick_teleop is not None:
            with self.motor_lock:
                # Keep the velocities between 0.0 and 1.0 to prevent code from crashing.
                self.linear_x = vel.linear.x if abs(vel.linear.x) < 1 else 1
                self.angular_z = vel.angular.z if abs(vel.angular.z) < 1 else 1
                self.joystick_teleop = True

    def velocity_callback(self, vel: Twist) -> None:
        """
        Callback function that receives incoming velocity.

        :param vel:
            Velocity which stores the linear and angular velocities in the
            x, y and z directions.
        """
        # TODO: Add a timer to check how long it has been since a new message teleop
        # message. If time is exceeded then set teleop to false.
        if not self.collision and not self.joystick_teleop:
            with self.motor_lock:
                # Keep the velocities between 0.0 and 1.0 to prevent code from crashing.
                self.linear_x = vel.linear.x if abs(vel.linear.x) < 1 else 1
                self.angular_z = vel.angular.z if abs(vel.angular.z) < 1 else 1

    def loop(self) -> None:
        """
        Loop that changes the motor speed.
        """
        while not rospy.is_shutdown():
            with self.motor_lock:
                self.motors.run(self.linear_x, self.angular_z)
                self.linear_x = self.angular_z = 0.0
                self.collision = self.joystick_teleop = False
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
