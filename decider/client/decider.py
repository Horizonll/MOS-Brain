# decider.py
#
#   @description : The entry py file for decision on clients
#

import os
import json
import math
import time
import signal
import asyncio
import numpy as np
import threading
import logging
import sub_statemachines
import sub_statemachines

logging.basicConfig(
    level=logging.DEBUG,
    format='%(asctime)s - %(levelname)s - %(filename)s:%(funcName)s(): %(message)s'
)

# ROS
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist

# Submodules
import configuration
from angle import angle

# Interfaces with other components
import interfaces.action
import interfaces.vision

# Network
from robot_client import RobotClient
from receiver import Receiver

class Agent:
    # @public variants:
    #           NONE
    #           NONE
    #
    # @public methods:
    #   cmd_vel(vel_x : float, vel_y : float, vel_theta : float)
    #   kick()
    #   look_at([head, neck]: float list[2])
    #       disable automatically tracking and force to look_at
    #       use (NaN, NaN) to enable tracking
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
    #   _config         dictionary: configurations such as vel, etc.
    #   _command        dictionary: current recieved command
    #   _lst_command    dictionary: last command
    #   _info           dictionary: current running command str
    #   _action         The interface to kick and walk
    #   _vision         The interface to head, neck and camera
    #   _state_machine  A dictionary of state machines
    # 
    # @private methods:
    #   None

    _command = {
        "command": "None",
        "args": {},
        "timestamp": time.time(),
    }

    def __init__(self):
        rospy.init_node("decider", log_level=rospy.DEBUG)
        rospy.loginfo("Initializing the Agent instance")
        
        self.if_local_test = False

        # Initializing public variables
        self._config = configuration.load_config()
        self.id = 1
        self._command = {
            "command": "stop",
            "data": {},
            "timestamp": time.time(),
        }
        self._lst_command = self._command
        
        rospy.loginfo("Registering interfaces")
        # action: provide functions to control the robot, such as cmd_vel 
        # and kick
        self._action = interfaces.action.Action(self._config)
        # vision: provide functions to get information from the robot, 
        # such as self position and ball position
        self._vision = interfaces.vision.Vision(self._config)
        # robot_client: provide functions to communicate with the server
        self._robot_client = RobotClient(self)
        self.receiver = Receiver(self.get_config()["team"], self.get_config()["id"])


        # Initialize state machines by importing all python files in 
        # sub_statemachines and create a dictionary from command to 
        # statemachine.run

        rospy.loginfo("Initializing sub-state machines")
        # py_files = Agent._get_python_files("sub_statemachines/")
        # print(py_files)
        
        self.chase_ball_state_machine = sub_statemachines.ChaseBallStateMachine(self)
        self.find_ball_state_machine = sub_statemachines.FindBallStateMachine(self)
        self.kick_state_machine = sub_statemachines.KickStateMachine(self)
        self.go_back_to_field_state_machine = sub_statemachines.GoBackToFieldStateMachine(self)
        self.dribble_state_machine = sub_statemachines.DribbleStateMachine(self)


        self._state_machine_runners = {
            "chase_ball": self.chase_ball_state_machine.run,
            "find_ball": self.find_ball_state_machine.run,
            "kick": self.kick_state_machine.run,
            "go_back_to_field": self.go_back_to_field_state_machine.run,
            "dribble": self.dribble_state_machine.run,
            "stop": self.stop,
        }

        rospy.loginfo("Agent instance initialization completed")

    def run(self):
        if self.receiver.game_state == 'STATE_SET':
            self.stop()
        elif self.receiver.game_state == 'STATE_READY':
            self._state_machine_runners['go_back_to_field']()
        elif ((not self.get_if_ball()) and (self._command["command"] == 'chase_ball' or \
                                          self._command["command"] == 'kick')):
            self._state_machine_runners['find_ball']()
        elif self._state_machine_runners.get(self._command["command"]):
            self._state_machine_runners[self._command["command"]]()
        else:
            logging.debug(f"State machine '{self._command['command']}' not found.")
            self.stop()


    # The following are some simple encapsulations of interfaces
    # The implementation of the apis may be change in the future
    def cmd_vel(self, vel_x: float, vel_y: float, vel_theta: float):
        self._action.cmd_vel(vel_x, vel_y, vel_theta)
        rospy.loginfo(f"Setting the robot's speed: linear velocity x={vel_x}, "
                + "y={vel_y}, angular velocity theta={vel_theta}")

    def look_at(self, args):
        self._vision.look_at(args)
    
    def stop(self, sleep_time=0):
        self._action.cmd_vel(0, 0, 0)
        time.sleep(sleep_time)

    def kick(self):
        self._action.do_kick()
        rospy.loginfo("Executing the kicking action")

    def get_command(self):
        return self._command

    def get_self_pos(self):
        return self._vision.self_pos

    def get_self_yaw(self):
        return self._vision.self_yaw

    def get_ball_pos(self):
        if self.get_if_ball():
            return self._vision.get_ball_pos()
        else:
            return [None, None]
    
    def get_ball_angle(self):
        ball_pos = self.get_ball_pos()
        ball_x = ball_pos[0]
        ball_y = ball_pos[1]
        if ball_pos is not None:
            if ball_x == 0 and ball_y == 0:
                return None
            else:
                angle_rad = - math.atan2(ball_x, ball_y)
                return angle_rad
        else:
            return None

    def get_ball_pos_in_vis(self):
        return self._vision.get_ball_pos_in_vis()

    def get_ball_pos_in_map(self):
        return self._vision.get_ball_pos_in_map()

    def get_if_ball(self):
        return self._vision.get_if_ball()


    def get_if_close_to_ball(self):
        if self.get_if_ball():
            return 0.1 <= self.get_ball_distance() <= 0.4
        else:
            return False

    def get_ball_distance(self):
        if self.get_if_ball():
            return self._vision.ball_distance
        else:
            return 1e6

    def get_neck(self):
        return self._vision.neck

    def get_head(self):
        return self._vision.head
    
    def get_config(self):
        return self._config


def main():
    rospy.loginfo("Decider started")
    agent = Agent()

    try:
        def sigint_handler(sig, frame):
            rospy.loginfo("User interrupted the program, exiting gracefully...")
            exit(0)

        signal.signal(signal.SIGINT, sigint_handler)
        while True:
            agent.run()
            time.sleep(0.1)
    except KeyboardInterrupt:
        rospy.loginfo("Program interrupted by the user")
        exit()
    finally:
        pass


if __name__ == "__main__":
    main()
