#!/usr/bin/env python3
"""
Rospy service module to change mode the internal light
"""

from typing import Dict
import requests
import rospy
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse


class LightsService:
    """Rospy Server Class for changing internal lights"""

    def __init__(self) -> None:
        rospy.init_node("internal_lights_server")
        self.server: rospy.Service = rospy.Service(
            "internal_lights", SetBool, self.handle_srv_req
        )
        self.response: SetBoolResponse = SetBoolResponse()
        self.url: str = "http://10.0.0.80/ios/change_output"
        self.payload_on: Dict[str, str] = {
            "output_name": "light_base",
            "new_value": "on",
            "action": "change",
        }
        self.payload_off: Dict[str, str] = {
            "output_name": "light_base",
            "new_value": "off",
            "action": "change",
        }

    def handle_srv_req(self, req: SetBoolRequest) -> SetBoolResponse:
        """Handle request to switch state of internal lights
        Args:
            req (SetBoolRequest): True: turn on; False: turn off
        Returns:
            SetBoolResponse: {Success: response of post, Message: String indicating state}
        """
        if req.data:
            res = requests.post(self.url, data=self.payload_on, timeout=1)
        else:
            res = requests.post(self.url, data=self.payload_off, timeout=1)
        self.response.success = res.ok
        self.response.message = "Light is on" if req.data else "Light is off"
        return self.response


if __name__ == "__main__":
    light_server: LightsService = LightsService()
    rospy.spin()
