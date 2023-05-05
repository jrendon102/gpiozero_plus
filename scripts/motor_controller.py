#!/usr/bin/env python3
"""
ROS wrapper node that communicates with the motor/driver interface on the Yahboom G1 Tank.
"""
import sys
from threading import Lock

import rospy

from mach1_msgs.msg import VelStatus
from yb_expansion_board.navigation import eval_motion
from yb_expansion_board.robotEnums import Message, MotionType
from yb_expansion_board.ybMotors import YBMotor

# Initial Motion
NO_MOTION = MotionType.NO_MOTION.value

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
        self.vel_sub = rospy.Subscriber(
            "/vel_status",
            VelStatus,
            self.velocity_callback,
        )

        self.linear_x = self.angular_z = 0.0 # (m/s)
        self.motion_type = NO_MOTION

        # Create lock to prevent race conditions.
        self.motor_lock = Lock()

        # Sets the rate for incoming messages (Hz)
        self.rate = rospy.Rate(10)

    def velocity_callback(self, vel: VelStatus) -> None:
        """
        Update the robot's velocity and motion type based on incoming velocity
        information.

        Parameters:
            `vel (VelStatus)`: The incoming velocity message.

        Returns:
            `None`
        """
        with self.motor_lock:
            self.linear_x = vel.twist_msg.linear.x
            self.angular_z = vel.twist_msg.angular.z
            self.motion_type = eval_motion(self.linear_x, self.angular_z)

    def loop(self) -> None:
        """
        Control the robot's motion in a loop until shutdown.

        Returns:
            `None`
        """
        while not rospy.is_shutdown():
            # Set linear and angular velocities to 0.0 to stop motors in case
            #   vel msg's are no longer being published and last vel msg
            #   received from callback was non-zero.
            with self.motor_lock:
                self.motors.run(self.linear_x, self.angular_z, self.motion_type)
                self.linear_x = self.angular_z = 0.0
                self.motion_type = NO_MOTION
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
