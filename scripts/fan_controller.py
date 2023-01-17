#!/usr/bin/env python3
"""
ROS wrapper node that communicates with the Fan interface on the Yahboom G1 Tank.
"""

import sys

import rospy

from yb_expansion_board.robotEnums import Message
from yb_expansion_board.ybFan import YBFan
from expansion_board_driver.srv import enable_fan, enable_fanResponse

# Log Messages
ERR_ROSINIT = Message.ROSINIT.value
ERR_PARAM = Message.PARAM.value


class FAN:
    """
    This class sets up the Fan on the Yahboom G1 Tank and turns it either on or off.
    """

    def __init__(self) -> None:
        rospy.loginfo("Setting up Fan.")
        try:
            self.fan = YBFan(
                fan_pin=rospy.get_param("/hardware/fan/pin"),
                initial_mode=rospy.get_param("/hardware/fan/enabled"),
            )
        except KeyError:
            rospy.logerr(f"{ERR_PARAM}")
            sys.exit()

        # Service
        self.fan_service = rospy.Service("enable_fan", enable_fan, self.fan_callback)

    def fan_callback(self, state):
        """
        Callback function to determine if the fan should be turned OFF or ON.
        :param `state`:
            Incoming ROS request.
        """
        rospy.loginfo(f"FanEnabled:[{state.enabled}]")
        message = "Fan turned OFF."
        if state.enabled:
            self.fan.control_fan(state.enabled)
            message = "Fan turned ON."
        else:
            self.fan.control_fan(state.enabled)
        return enable_fanResponse(message)


if __name__ == "__main__":
    try:
        rospy.init_node("fan_controller_node")
        fan_controller = FAN()
        rospy.spin()
        rospy.loginfo("Turning off Fan and releasing pin.")
        fan_controller.fan.disconnect()
    except rospy.ROSInitException:
        rospy.logerr(ERR_ROSINIT)
