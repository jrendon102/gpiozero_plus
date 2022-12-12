#!/usr/bin/env python3
"""
This script utilizes the gpiozero library to access the GPIO pins on the Raspberry Pi. The purpose
is to allow communication with motors attached to the Yahboom G1 Tank Expansion Board. The speed
of the motors are varied to allow maneuverability of the robot in any direction.
"""
import sys
import rospy
from geometry_msgs.msg import Twist
from gpiozero import Robot
from gpiozero.pins.pigpio import PiGPIOFactory
from hardware_enums import Message, MotionType

# Set up the pin_factory for better control of GPIO pins.
FACTORY = PiGPIOFactory()

# Type of motion.
CURVILINEAR = MotionType.CURVILINEAR.value
LINEAR = MotionType.LINEAR.value
NO_MOTION = MotionType.NO_MOTION.value
ROTATIONAL = MotionType.ROTATIONAL.value

# Log Messages
ERR_ROSINIT = Message.ROSINIT.value
ERR_PARAM = Message.PARAM.value


class YBMotor:
    """
    This class allows communication with hardware so that the robot can navigate in any direction.
    Navigation is possible by varying speed of individual motors.
    """

    def __init__(self, curve_scale: float = 0.6) -> None:
        """
        :param curve_scale:
            Amount of curvature when moving in a CURVILINEAR motion.
            Default set to 0.6 which was determined to be acceptable.
        """
        # Set up left & right motors. Pins are set in config file prior to launch.
        try:
            self.left_motor = tuple(rospy.get_param("/hardware/left_motor"))
            self.right_motor = tuple(rospy.get_param("/hardware/right_motor"))
        except KeyError:
            rospy.logerr(ERR_PARAM)
            sys.exit()
        self.motors = Robot(
            left=self.left_motor, right=self.right_motor, pin_factory=FACTORY
        )
        # Subscriber
        self.vel_sub = rospy.Subscriber("/cmd_vel", Twist, self.motor_callback)

        # Initialize linear and angular speed to 0.0 (m/s)
        self.linear = 0.0
        self.angular = 0.0

        # CURVILINEAR motion scale (m/s)
        self.curve_scale = curve_scale

        # Robot initial state is NO MOTION
        self.motion_type = NO_MOTION

    def motor_callback(self, vel: Twist) -> None:
        """
        Callback function that subscribes to specific rostopic. The type of motion that the robot
        moves is determined by the linear and angular velocity values that are received.

        :param vel:
            ROS message of type Twist.
        """
        self.linear = vel.linear.x if abs(vel.linear.x) < 1 else 1
        self.angular = vel.angular.z if abs(vel.angular.z) < 1 else 1

        # Determine type of motion.
        if self.linear and self.angular:
            self.motion_type = CURVILINEAR
        elif self.linear and not self.angular:
            self.motion_type = LINEAR
        elif not self.linear and self.angular:
            self.motion_type = ROTATIONAL
        else:
            self.motion_type = NO_MOTION

        # Communicate with hardware.
        self.run()

    def run(self) -> None:
        """
        Communicate with hardware and move robot according to the type of motion that was
        determined.
        """
        # Robot motion is along a curved path.
        if self.motion_type is CURVILINEAR:
            # Curve forwards towards the right (frame of robot)
            if self.linear > 0.0 and self.angular < 0.0:
                self.motors.forward(
                    speed=abs(self.angular), curve_right=self.curve_scale
                )
            # Curve forwards towards the left (frame of robot)
            elif self.linear > 0.0 and self.angular > 0.0:
                self.motors.forward(speed=self.angular, curve_left=self.curve_scale)
            # Curve backwards towards the right (frame of robot)
            elif self.linear < 0.0 and self.angular > 0.0:
                self.motors.backward(speed=self.angular, curve_right=self.curve_scale)
            # Curve backwards towards the left (frame of robot)
            elif self.linear < 0.0 and self.angular < 0.0:
                self.motors.backward(
                    speed=abs(self.angular), curve_left=self.curve_scale
                )
        # Drive robot linearly either forwards or backwards.
        elif self.motion_type is LINEAR:
            if self.linear > 0:
                self.motors.forward(speed=self.linear)
            else:
                self.motors.backward(speed=abs(self.linear))
        # Rotate robot about z-axis.
        elif self.motion_type is ROTATIONAL:
            if self.angular > 0:
                self.motors.left(speed=self.angular)
            else:
                self.motors.right(speed=abs(self.angular))
        # Stop motors
        else:
            self.motors.stop()
        msg = (
            f"Motion type:[{self.motion_type}]. Linear velocity:[{self.linear:.3f}] m/s. "
            f"Angular velocity: [{self.angular:.3f}] rad/s."
        )
        rospy.loginfo(msg)


if __name__ == "__main__":
    try:
        rospy.init_node("ybmotors_node")
        rospy.loginfo("Setting up motors.")
        YBMotor()
        rospy.spin()
    except rospy.ROSInitException:
        rospy.logerr(ERR_ROSINIT)
    rospy.loginfo("Releasing motor pins.")
    FACTORY.close()  # Release pins.
    rospy.loginfo("Motors are shutdown.")
