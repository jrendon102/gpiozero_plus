#!/usr/bin/env python3
"""
ROS wrapper node that communicates with the RGB LED interface on the Yahboom G1 Tank.
"""
import sys
from threading import Lock

import rospy
from std_msgs.msg import Float32, String

from robotEnums import LEDMode, Message
from ybLED import YBLED

# Log Messages
ERR_ROSINIT = Message.ROSINIT.value
ERR_PARAM = Message.PARAM.value

# Luminosity value thresholds for handling hysteresis.
MAX_THRESHOLD = 0.50
MIN_THRESHOLD = 0.15

# TODO: Once IDLE mode is created, set DEFAULT to IDLE.
# Modes for different LED settings.
FLASHLIGHT = LEDMode.FLASHLIGHT_MODE.value
TEST = LEDMode.TEST_MODE.value
DEFAULT = LEDMode.OFF_MODE.value


class LEDControl:
    """
    This class sets up the RGB LEDs on the Yahboom G1 Tank and changes the color and brightness.
    The LEDs are dependent on the led mode that was determined during operational use.
    """

    def __init__(self) -> None:
        try:
            # Set up RGB LED
            rospy.loginfo("Setting up RGB LEDs.")
            self.rgb_led = YBLED(
                red_led=rospy.get_param("/hardware/rgb_leds/red"),
                green_led=rospy.get_param("/hardware/rgb_leds/green"),
                blue_led=rospy.get_param("/hardware/rgb_leds/blue"),
            )
        except KeyError:
            rospy.logerr(f"{ERR_PARAM}")
            sys.exit()

        # Publisher
        self.led_mode_pub = rospy.Publisher("/led_mode", String, queue_size=1)

        # Rate (Hz)
        self.rate = rospy.Rate(1)

        # Subscriber
        self.luminosity_sub = rospy.Subscriber(
            "/luminosity", Float32, self.luminosity_value_callback
        )

        # Create lock to prevent race conditions.
        self.led_mode_lock = Lock()

        # Initialize the current and new led modes.
        self.current_led_mode = DEFAULT
        self.new_led_mode = TEST

        # Brightness scale
        self.brightness_scale = 0.03

        # Log message.
        self.log_msg = f"LED MODE:[{self.new_led_mode}]."

    # FIXME: Hysteresis is causing robot to have a "disco party".
    def luminosity_value_callback(self, luminosity_value: Float32):
        """
        Callback function to determine if environmental lighting conditions are too dark or too
        bright.

        :param luminosity_value:
            Incoming ROS message which provides luminosity value from camera video feeds.
        """
        with self.led_mode_lock:
            # Lighting conditions are too dark.
            if luminosity_value.data < MAX_THRESHOLD:
                self.new_led_mode = FLASHLIGHT
                self.log_msg = (
                    f"LED MODE:[{self.new_led_mode}]. Lighting conditions worsened."
                )
            # Lighting conditions have improved.
            elif luminosity_value.data > MIN_THRESHOLD:
                self.new_led_mode = DEFAULT
                self.log_msg = (
                    f"LED MODE:[{self.new_led_mode}]. Lighting conditions improved."
                )
            self.brightness_scale = 0.07

    def loop(self) -> None:
        """
        Loop that changes the color of the RGB LED. Only changes the LED color when there is a new
        led mode. If no callback has changed the led mode, the led mode will be switched to DEFAULT.
        Publish led mode at constant rate.
        """
        while not rospy.is_shutdown():
            with self.led_mode_lock:
                if self.new_led_mode != self.current_led_mode:
                    rospy.loginfo(self.log_msg)
                    self.rgb_led.change_color(self.new_led_mode, self.brightness_scale)
                self.current_led_mode = self.new_led_mode
            self.new_led_mode = DEFAULT
            self.log_msg = f"LED MODE:[{self.new_led_mode}]."
            self.led_mode_pub.publish(self.current_led_mode)
            self.rate.sleep()


if __name__ == "__main__":
    try:
        rospy.init_node("led_controller_node")
        led_control = LEDControl()
        led_control.loop()
        rospy.spin()
        rospy.loginfo("Releasing led pins and powering of LED.")
        led_control.rgb_led.disconnect()
    except rospy.ROSInitException:
        rospy.logerr(ERR_ROSINIT)
