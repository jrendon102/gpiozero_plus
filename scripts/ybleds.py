#!/usr/bin/env python3
"""
This script utilizes the gpiozero library to access the GPIO pins on the Raspberry Pi. The purpose
is to allow communication with the RGB LED interface on the Yahboom G1 Tank expansion board.
"""
import rospy
from gpiozero import PWMLED
from gpiozero.pins.pigpio import PiGPIOFactory
from hardware_enums import LEDMode, Message
from std_msgs.msg import Float32

# Set up pin_factory to get better control of GPIO pin.
FACTORY = PiGPIOFactory()

# Log Messages
ERR_ROSINIT = Message.ROSINIT.value
ERR_PARAM = Message.PARAM.value

# Luminosity value thresholds for handling hysteresis.
MAX_THRESHOLD = 0.50
MIN_THRESHOLD = 0.15

# Modes for different LED settings.
FLASHLIGHT = LEDMode.FLASHLIGHT_MODE.value
OFF = LEDMode.OFF_MODE.value


class YBLED:
    """
    This class allows communication with the RGB LED interface on the Yahboom G1 Tank expansion
    board.
    """

    def __init__(self) -> None:
        try:
            # Set up LEDs according to their respective pin numbers.
            self.red_led = PWMLED(
                pin=rospy.get_param("/hardware/rgb_leds/red"), pin_factory=FACTORY
            )
            self.green_led = PWMLED(
                pin=rospy.get_param("/hardware/rgb_leds/green"), pin_factory=FACTORY
            )
            self.blue_led = PWMLED(
                pin=rospy.get_param("/hardware/rgb_leds/blue"), pin_factory=FACTORY
            )
        except KeyError:
            rospy.logwarn(ERR_PARAM)

        # Store LEDs in dict for easy access/control.
        self.rgb_leds = {
            "red": self.red_led,
            "green": self.green_led,
            "blue": self.blue_led,
        }

        # Subscriber
        self.luminosity_sub = rospy.Subscriber(
            "/luminosity", Float32, self.luminosity_value_callback
        )

        # Initialize current LED mode
        self.current_led_mode = OFF

    def luminosity_value_callback(self, luminosity_value: Float32):
        """
        Callback function to determine if environmental lighting conditions are too dark or too
        bright

        :param luminosity_value:
            Incoming ROS message which provides luminosity value from camera video feeds.
        """
        # Lighting conditions are too dark.
        if luminosity_value.data < MAX_THRESHOLD:
            new_led_mode = FLASHLIGHT
            msg = f"Lighting conditions worsened. LEDs set to {new_led_mode}"
        # Lighting conditions have improved.
        elif luminosity_value.data > MIN_THRESHOLD:
            new_led_mode = OFF
            msg = f"Lighting conditions improved. LEDs set to {new_led_mode}"
        self.change_mode(led_mode=new_led_mode, log_msg=msg)

    def change_mode(self, led_mode, log_msg) -> None:
        """
        Vary the power output to the RGB LEDs depending on the led mode that was set.
        """
        if self.current_led_mode != led_mode:
            # RGB LED values all set to 255.
            if led_mode == FLASHLIGHT:
                for led in self.rgb_leds:
                    self.rgb_leds[led].value = 1.0
            # LEDs turned off.
            elif led_mode == OFF:
                for led in self.rgb_leds:
                    self.rgb_leds[led].value = 0.0
            rospy.loginfo(log_msg)
        # Update current_led_mode.
        self.current_led_mode = led_mode


if __name__ == "__main__":
    try:
        rospy.init_node("ybleds_node")
        rospy.loginfo("Setting up RGB LEDs.")
        YBLED()
        rospy.spin()
    except rospy.ROSInitException:
        rospy.logerr(ERR_ROSINIT)
    rospy.loginfo("Releasing led pins.")
    FACTORY.close()
    rospy.loginfo("RGB LEDs powered off.")
