# decider.py
#
#   @description : The entry py file for decision on clients
#

import json
import math
import time
import signal
import socket
import asyncio
import logging
import threading
import websockets
import numpy as np
from transitions import Machine

# ROS
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist

# Submodules
import configuration
from angle import angle

# interfaces with other components
import interfaces.action
import interfaces.vision
import interfaces.network


class Agent:
    # @public variants:
    #       VARIANTS             TYPE                 DESCRIPTION
    #   config              dictionary      configurations such as vel, etc.
    #   command             dictionary          current running command
    #
    # @public methods:
    #   cmd_vel(vel_x : float, vel_y : float, vel_theta : float)
    #   kick()
    #   run()
    #
    # @public methods to get varants:
    #       METHODS             TYPE                DESCRIPTION
    #   get_self_pos()          numpy.array     self position in map
    #   get_self_yaw()          angle           self angle in deg, [-180, 180)
    #   get_ball_pos()          numpy.array     ball postiion from robot
    #   get_ball_pos_in_vis()   numpy.array     ball position in **vision**
    #   get_ball_pos_in_map()   numpy.array     ball position in global 
    #   get_if_ball()           bool            whether can find ball
    #   get_ball_distance()     float           the distance to the ball
    #   get_neck()              angle           the neck angle, left and right
    #   get_head()              angle           the head angle, up and down
    # 
    # @private variants:
    #   _action         The interface to kick and walk
    #   _vision         The interface to head, neck and camera
    #   _state_machine  A dictionary of state machines
    # 
    # @private methods:
    #   None

    def __init__(self):
        print("[+] Initializing Agent")
        rospy.init_node("decider")
        
        # initializing public variants
        config = configuration.load_config()
        self.command = {
            "command": "stop",
            "data": {},
            "send_time": time.time(),
        }

        print("[*] Registering interfaces")
        self._action     = action.Action(self)
        self._vision     = vision.Vision(self)
        print("[*] Initializing sub-statemahcines")
        self._state_machine["kick"]     = KickStateMachine(self)
        self._state_machine["go_back_to_field"]  = GoBackToFieldStateMachine(self, \
                                            0, 4500, 300)
        self._find_ball_state_machine   = FindBallStateMachine(self)
        self._chase_ball_state_machine  = ChaseBallStateMachine(self)
        self._dribble_state_machine     = DribbleStateMachine(self)
        

        print("Agent instance initialization complete.")


    # following are some simple encapsulation of interfaces
    def cmd_vel(vel_x: float, vel_y: float, vel_theta: float):
        self._action.cmd_vel(x, y, theta)
    def kick():
        self._action.do_kick()
    def get_self_pos():
        return self._vision.self_pos
    def get_self_yaw():
        return self._vision.self_yaw
    def get_ball_pos():
        return self._vision.get_ball_pos()
    def get_ball_pos_in_vis():
        return self._vision.get_ball_pos_in_vis()
    def get_ball_pos_in_map():
        return self._vision.get_ball_pos_in_map()
    def get_if_ball():
        return self._vision.get_if_ball()
    def get_ball_distance():
        return self._vision.ball_distance
    def get_neck():
        return self._vision.neck
    def get_head():
        return self._vision.head


def main():
    print("[+] Decider started")
    agent = Agent()

    try:
        def sigint_handler(sig, frame):
            print("\nExiting gracefully...")
            exit(0)

        signal.signal(signal.SIGINT, sigint_handler)
        while True:
            agent.run()
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nProgram interrupted by user")
        exit()
    finally:
        pass


if __name__ == "__main__":
    main()
