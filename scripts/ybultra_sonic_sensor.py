#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

from msg import Collision
from robotEnums import Message, CollisionType
from ybUltraSonicSensor import get_distance

# Log messages
ERR_ROSINIT = Message.ROSINIT.value
ERR_PARAM = Message.PARAM.value


# Collision Types
REAR = CollisionType.REAR_COLLISION.value


class UltraSonicSensor:
    def __init__(self) -> None:
        try:
            self.echo_pin = rospy.get_param("/hardware/ultrasonic/echo")
            self.trig_pin = rospy.get_param("/hardware/ultrasonic/trig")
        except KeyError:
            rospy.logerr(ERR_PARAM)

        # Publisher
        self.obj_dist_pub = rospy.Publisher("/object_distance", Collision, queue_size=1)

        # Subscriber
        self.vel_sub = rospy.Subscriber("/cmd_vel", Twist, self.motor_callback)

        self.distance = None

        # Rate for publishing messages
        self.obj_dist_time = rospy.Rate(0.20)

    def motor_callback(self, vel: Twist):
        linear_x = vel.linear.x
        if linear_x < 0:
            collision_detection = Collision()

            collision_detection.distance = get_distance(self.echo_pin, self.trig_pin)
            collision_detection.collisionType = REAR

            rospy.loginfo(f"Objgect distance: [{collision_detection.distance:.3f}] m")
            self.obj_dist_pub.publish(collision_detection)


if __name__ == "__main__":
    try:
        rospy.init_node("yb_ultrasonic_node")
        rospy.loginfo("Setting up ultrasonic sensor.")
        UltraSonicSensor()
        rospy.spin()
    except rospy.ROSInitException:
        rospy.logerr(ERR_ROSINIT)


# class YBUltrasonic:
#     def __init__(self) -> None:
#         try:
#             self.echo_pin = rospy.get_param("/hardware/ultrasonic/echo")
#             self.trig_pin = rospy.get_param("/hardware/ultrasonic/trig")
#         except KeyError:
#             rospy.logwarn(ERR_PARAM)

#         self.sensor = DistanceSensor(
#             echo=self.echo_pin, trigger=self.trig_pin, pin_factory=FACTORY
#         )
#         self.rate = rospy.Rate(10)
# self.run()

# def run(self) -> None:
#     while not rospy.is_shutdown():
#         self.rate.sleep()
#         print(f"Distance to nearest object is: [{self.sensor.distance:.3f} m]")


# def get_distance(echo_pin: int, trig_pin: int, time: float):
#     ultra_sonic_sensor = DistanceSensor(
#         echo=echo_pin, trigger=trig_pin, pin_factory=FACTORY
#     )
#     while True:
#         ultra_sonic_sensor.distance()
#         sleep(time)
