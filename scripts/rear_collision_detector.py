#!/usr/bin/env python3
"""
ROS wrapper node that communicates with the Ultrasonic Sensor interface on the Yahboom G1 Tank.
"""
import sys
from threading import Lock

import rospy
from geometry_msgs.msg import Twist

from yb_expansion_board.robotEnums import Message, CollisionType
from yb_expansion_board.ybUltraSonic import YBUltrasonic
from mach1_msgs.msg import Collision

# Log messages
ERR_ROSINIT = Message.ROSINIT.value
ERR_PARAM = Message.PARAM.value


# Collision Types
REAR = CollisionType.REAR_COLLISION.value


class ReverseSafetySensor:
    """
    This class sets up the Ultrasonic Sensor on the Yahboom G1 Tank. Used to detect objects
    for obstacle avoidance.
    """

    def __init__(self) -> None:
        try:
            # Set up Ultrasonic sensor
            rospy.loginfo("Setting up ultrasonic sensor.")
            self.collision_sensor = YBUltrasonic(
                echo_pin=rospy.get_param("/exp_board/ultrasonic/echo"),
                trig_pin=rospy.get_param("/exp_board/ultrasonic/trig"),
            )
            self.collision_threshold = rospy.get_param("/exp_board/ultrasonic/min_dist")
        except KeyError:
            rospy.logerr(f"{ERR_PARAM}")
            sys.exit()

        # Publisher
        self.rear_collision_pub = rospy.Publisher(
            "/rear_collision",
            Collision,
            queue_size=1,
        )

        # Subscriber
        self.velocity_sub = rospy.Subscriber(
            "/cmd_vel",
            Twist,
            self.velocity_callback,
        )

        self.velocity_sub = rospy.Subscriber(
            "/joy_cmd_vel",
            Twist,
            self.joystick_vel_callback,
        )

        self.collision_lock = Lock()

    def joystick_vel_callback(self, vel: Twist) -> None:
        """
        Callback function that receives incoming joystick
        velocity.

        :param vel:
            Velocity which stores the linear and angular velocities in the
            x, y and z directions.
        """
        with self.collision.lock:
            linear_x = vel.linear.x
            self.run(linear_x)

    def velocity_callback(self, vel: Twist) -> None:
        """
        Callback function that receives incoming velocity.

        :param vel:
            Stores the linear and angular velocities in the x, y and z directions.
        """
        with self.collision_lock:
            linear_x = vel.linear.x
            self.run(linear_x)

    def run(self, velocity: float) -> None:
        """
        Determines if there is a possible collision when navigating in reverse.

        :param velocity:
            Linear velocity in the x.
        """
        collision_msg = Collision()
        with self.collision_lock:
            if velocity < 0:
                collision_msg.collisionType = REAR
                collision_msg.distance = self.collision_sensor.get_distance()
                collision_msg.min_collision_dist = self.collision_threshold
                self.rear_collision_pub.publish(collision_msg)
                rospy.loginfo(f"Distance:[{collision_msg.distance:.6f}] m")


if __name__ == "__main__":
    try:
        rospy.init_node("rear_collision_detector_node")
        detection = ReverseSafetySensor()
        rospy.spin()
        rospy.loginfo("Releasing pins.")
        detection.collision_sensor.disconnect()
    except rospy.ROSInitException:
        rospy.logerr(ERR_ROSINIT)
