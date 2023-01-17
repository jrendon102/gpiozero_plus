#!/usr/bin/env python3
"""
ROS wrapper node that communicates with the Ultrasonic Sensor interface on the Yahboom G1 Tank.
"""
import sys

import rospy
from geometry_msgs.msg import Twist

from yb_expansion_board.robotEnums import Message, CollisionType
from yb_expansion_board.ybUltraSonic import YBUltrasonic
from expansion_board_driver.msg import Collision

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
                echo_pin=rospy.get_param("/hardware/ultrasonic/echo"),
                trig_pin=rospy.get_param("/hardware/ultrasonic/trig"),
            )
        except KeyError:
            rospy.logerr(f"{ERR_PARAM}")
            sys.exit()

        # Publisher
        self.rear_collision_pub = rospy.Publisher(
            "/collision_detection", Collision, queue_size=1
        )

        # Subscriber
        self.velocity_sub = rospy.Subscriber("/cmd_vel", Twist, self.velocity_callback)

    def velocity_callback(self, vel) -> None:
        """
        Callback function that receives incoming velocity.

        :param vel:
            Stores the linear and angular velocities in the x, y and z directions.
        """
        collision_msg = Collision()
        if vel.linear.x < 0:
            collision_msg.collisionType = REAR
            collision_msg.distance = self.collision_sensor.get_distance()
            self.rear_collision_pub.publish(collision_msg)
            rospy.loginfo(f"Distance:[{collision_msg.distance:.5f}] m")


if __name__ == "__main__":
    try:
        rospy.init_node("rear_collision_detector_node")
        detection = ReverseSafetySensor()
        rospy.spin()
        rospy.loginfo("Releasing pins.")
        detection.collision_sensor.disconnect()
    except rospy.ROSInitException:
        rospy.logerr(ERR_ROSINIT)
