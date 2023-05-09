#!/usr/bin/env python3
"""
ROS wrapper node that communicates with the Ultrasonic Sensor interface on the Yahboom G1 Tank.
"""
import sys
from threading import Lock

import rospy

from yb_expansion_board.ybUltraSonic import YBUltrasonic
from yb_expansion_board.robotEnums import Message, CollisionType
from mach1_msgs.msg import Collision, VelStatus

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
        self.sensor_status_pub = rospy.Publisher(
            "/ultra_sonic_status",
            Collision,
            queue_size=1,
        )
        # Subscriber
        self.vel_status_sub = rospy.Subscriber(
            "/vel_status",
            VelStatus,
            self.vel_status_callback,
        )

        self.sensor_msg = Collision()
        self.sensor_msg.collisionType = "NONE"
        self.sensor_msg.min_collision_dist = self.collision_threshold

        # Create lock to prevent race conditions.
        self.sensor_lock = Lock()
        # Sets the rate for incoming messages (Hz)
        self.rate = rospy.Rate(10)

    def vel_status_callback(self, vel: VelStatus) -> None:
        """
        Update the robot's velocity and motion type based on incoming velocity
        information. Determines if there is a possible collision when navigating
        in reverse.

        Args:
            `vel (VelStatus)`: The incoming velocity message.

        Returns:
            `None`
        """
        with self.sensor_lock:
            self.sensor_msg.distance = self.collision_sensor.get_distance()
            if vel.twist_msg.linear.x < 0.0:
                self.sensor_msg.collisionType = REAR
                rospy.loginfo(f"Distance:[{self.sensor_msg.distance:.6f}] m")

    # TODO: M-8:Collision Warning Response Time
    def loop(self) -> None:
        while not rospy.is_shutdown():
            with self.sensor_lock:
                self.sensor_msg.distance = self.collision_sensor.get_distance()
                self.sensor_msg.min_collision_dist = self.collision_threshold
                self.sensor_status_pub.publish(self.sensor_msg)
                self.sensor_msg.collisionType = "NONE"
            self.rate.sleep()


if __name__ == "__main__":
    try:
        rospy.init_node("rear_collision_detector_node")
        detection = ReverseSafetySensor()
        detection.loop()
        rospy.spin()
        rospy.loginfo("Releasing pins.")
        detection.collision_sensor.disconnect()
    except rospy.ROSInitException:
        rospy.logerr(ERR_ROSINIT)
