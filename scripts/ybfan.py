#!/usr/bin/env python3
"""
This script utilizes the gpiozero library to access the gpio pins on the Raspberry Pi. The purpose
is to allow communication with the fan interface on the Yaboom expansion board. The fan is used to
cool down the Raspberry Pi while the robot is operating.
"""
import rospy
from gpiozero import DigitalOutputDevice
from gpiozero.pins.pigpio import PiGPIOFactory
from hardware_enums import Message
from std_msgs.msg import Bool

# Log Messages.
ERR_ROSINIT = Message.ROSINIT.value
ERR_PARAM = Message.PARAM.value

# Set up pin_factory to get better control of GPIO pins.
FACTORY = PiGPIOFactory()


class YBFan:
    """This Class allows communication with hardware so that the fan can be toggled ON or OFF."""

    def __init__(self) -> None:
        # Get fan pin number.
        try:
            self.ybfan_pin = rospy.get_param("/hardware/fan_pin")
        except KeyError:
            rospy.logerr(ERR_PARAM)

        # Set up Fan and initially have it ON.
        self.ybfan = DigitalOutputDevice(
            pin=self.ybfan_pin, active_high=False, pin_factory=FACTORY
        )
        self.ybfan.on()

        # Subscriber
        self.ybfan_sub = rospy.Subscriber("/fan_status", Bool, self.fan_callback)

    def fan_callback(self, status: Bool) -> None:
        """
        Callback function to turn the fan on or off depending on depending on what message was
        received.

        :param status:
            Incoming ROS message which determines the state of the fan.
        """
        msg = "OFF"
        if status.data:
            self.ybfan.on()
            msg = "ON"
        else:
            self.ybfan.off()
        rospy.loginfo(f"Fan toggled {msg}.")


if __name__ == "__main__":
    try:
        rospy.init_node("ybfan_node")
        rospy.loginfo("Setting up fan.")
        YBFan()
        rospy.spin()
    except rospy.ROSInitException:
        rospy.logerr(ERR_ROSINIT)
    rospy.loginfo("Releasing fan pin.")
    FACTORY.close()  # Release pin.
    rospy.loginfo("Fan is OFF.")
