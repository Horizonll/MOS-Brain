# interfaces/action.py
#
#   @description :   The py files to call actions
#
#   @interfaces :
#       1. class Action
#

import os
from booster_robotics_sdk_python import B1LocoClient, \
                                        ChannelFactory
import sys, time, random

class Action:
    # @public variants:
    #   None
    #
    # @public methods;
    #   cmd_vel(vel_x, vel_y, vel_theta) : publish a velocity command
    #   
    # @private variants:
    #   _config             dictionary of configurations 
    #   _client             The publisher
    # 
    # @private methods
    #

    def __init__(self, config = {}): 
        self._config = config.get("interface", {}).get("action", {})
        ChannelFactory.Instance().Init(0, \
                self._config.get("network_interface", "192.168.10.102"))
        self._client = B1LocoClient()
        self._client.Init()
        time.sleep(0.4)

    # cmd_vel(vel_x, vel_y, vel_theta)
    #   publish a velocity command
    def cmd_vel(self, vel_x, vel_y, vel_theta):
        vel_x *= self._config.get("walk_vel_x", 1)
        vel_y *= self._config.get("walk_vel_y", 1)
        vel_theta *= self._config.get("walk_vel_theta", 1)
        print(vel_x)
        self._client.Move(vel_x, vel_y, vel_theta)

if __name__ == "__main__":
    print("Your are testing the action interfaces.")
    print("Notice the robot may active unindently.")
    print("Type yes to continue: ", end = "")
    if(input() != "yes"):
        exit()
    action = Action()
    action.cmd_vel(0.8, 0, 0)
    time.sleep(3)
    action.cmd_vel(0, 0.1, 0)
    time.sleep(3)
    action.cmd_vel(0.8, 0, 0)
    time.sleep(3)
    action.cmd_vel(0, 0, 0.3)
    time.sleep(3)
    action.cmd_vel(0, 0, 0)
