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


class Agent:
    # @public variants:
    #       VARIANTS             TYPE                 DESCRIPTION
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
        logging.info("Initializing the Agent instance")
        rospy.init_node("decider")

        # Initializing public variables
        self._config = configuration.load_config()
        self._command = {
            "command": "stop",
            "args": {},
            "timestamp": time.time(),
        }
        self._lst_command = self._command
        
        logging.info("Registering interfaces")
        # action: provide functions to control the robot, such as cmd_vel 
        # and kick
        self._action = interfaces.action.Action(self._config)
        # vision: provide functions to get information from the robot, 
        # such as self position and ball position
        self._vision = interfaces.vision.Vision(self._config)
        # robot_client: provide functions to communicate with the server
        self._robot_client = RobotClient(self)

        # Initialize state machines by importing all python files in 
        # sub_statemachines and create a dictionary from command to 
        # statemachine.run

        logging.info("Initializing sub-state machines")
        py_files = Agent._get_python_files("sub_statemachines/")
        self._state_machine = {}
        for py_file in py_files:
            logging.info("found : " + py_file)
            eval_str = "import " + py_file
            exec(eval_str)
            module_name = py_file.split('.')[-1] # also class name
            eval_str = "self._state_machine[\"" + module_name + "\"] = " \
                        + py_file + "." + module_name
            exec(eval_str) 

        logging.info("Agent instance initialization completed")

    def run(self):
        if(self._command == self._lst_command):
            self._execute(self._command["command"], "run")
            return

        old_state_machine = self._lst_command["command"]
        new_state_machine = self._command["command"]
        self._execute(old_state_machine, "stop", new_state_machine)
        self._execute(new_state_machine, "start", 
                      new_command["args"], old_state_machine)
        self._execute(new_state_machine, "run")
    

    # private: _execute: call state machines method
    def _execute(self, statemachine, func_name, *args):
        if(statemachine == "stop"):
            return
        eval_str = "self._statemachine[" + statemachine + "]." \
                    + func_name + "("
        for index, arg in enumerate(args):
            if(index != 0):
                eval_str += ","
            eval_str += str(arg)
        eval_str += ")"
        logging.debug("run " + eval_str)
        try:
            eval(eval_str)
        except Exception as  e:
            pass


    # private: _get_python_files: find all python file in a directory
    @staticmethod
    def _get_python_files(start_dir):
        py_files = []
        for root, dirs, files in os.walk(start_dir):
            for filename in files:
                if filename.endswith('.py'):
                    full_path = os.path.join(root, filename)
                py_files.append(full_path[0:-3].replace('/', '.'))
        return py_files


    # The following are some simple encapsulations of interfaces
    def cmd_vel(self, vel_x: float, vel_y: float, vel_theta: float):
        self._action.cmd_vel(vel_x, vel_y, vel_theta)
        logging.info(f"Setting the robot's speed: linear velocity x={vel_x}, "
                + "y={vel_y}, angular velocity theta={vel_theta}")

    def kick(self):
        self._action.do_kick()
        logging.info("Executing the kicking action")

    def get_self_pos(self):
        return self._vision.self_pos

    def get_self_yaw(self):
        return self._vision.self_yaw

    def get_ball_pos(self):
        return self._vision.get_ball_pos()

    def get_ball_pos_in_vis(self):
        return self._vision.get_ball_pos_in_vis()

    def get_ball_pos_in_map(self):
        return self._vision.get_ball_pos_in_map()

    def get_if_ball(self):
        return self._vision.get_if_ball()

    def get_ball_distance(self):
        return self._vision.ball_distance

    def get_neck(self):
        return self._vision.neck

    def get_head(self):
        return self._vision.head


def main():
    logging.info("Decider started")
    agent = Agent()

    try:
        def sigint_handler(sig, frame):
            logging.info("User interrupted the program, exiting gracefully...")
            exit(0)

        signal.signal(signal.SIGINT, sigint_handler)
        while True:
            agent.run()
            time.sleep(1)
    except KeyboardInterrupt:
        logging.info("Program interrupted by the user")
        exit()
    finally:
        pass


if __name__ == "__main__":
    main()
