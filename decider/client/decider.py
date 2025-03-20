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
    #   _command        dictionary: current running command
    #   _action         The interface to kick and walk
    #   _vision         The interface to head, neck and camera
    #   _state_machine  A dictionary of state machines
    # 
    # @private methods:
    #   None

    def __init__(self):
        print("[+] decider.py: Agent.__init__(): Initializing Agent")
        rospy.init_node("decider")
        
        # initializing public variants
        self._config = configuration.load_config()
        self._command = {
            "command": "None",
            "args": {},
            "timestamp": time.time(),
        }

        print("[*] decider.py: Agent.__init__(): Registering interfaces")
        self._action     = interfaces.action.Action(self._config)
        self._vision     = interfaces.vision.Vision(self._config)
        self._network    = interfaces.network.Network(self._config)

        print("[*] decider.py: Agent.__init__(): Initializing sub-statemahcines")
        py_files = Agent._get_python_files("sub_statemachines/")
        for py_file in py_files:
            print("[+] decider.py: Agent.__init__(): found : " + py_file)
            eval_str = "import " + py_file
            eval(eval_str)
            module_name = py_file.split('.')[-1] # also class name
            eval_str = "self._state_machine[" + module_name + "]=" \
                        + py_file + "." + module_name
            eval(eval_str)

        print("[+] decider.py: Agent.__init__(): " \
            + "Agent instance initialization complete.")


    def run():
        data_str = self._network.receive()
        if(data_str == None):
            self._execute(self._command["command"], "run")
            return

        # change state machine
        data = json.loads(data_str)
        for robot in data['robots']:
            if(robot["id"] == self._config["id"]):
                new_command = robot
                break

        old_state_machine = self._command["command"]
        new_state_machine = new_command["command"]
        self._execute(old_state_machine, "stop", new_state_machine)
        self._execute(new_state_machine, "start", 
                      new_command["args"], old_state_machine)
        self._execute(new_state_machine, "run")
        self._command = new_command
    

    # private: _execute: call state machines method
    def _execute(statemachine, func_name, *args):
        if(statemachin == "stop"):
            return
        eval_str = "self._statemachine[statemachine]." \
                    + func_name + "("
        for arg in args:
            eval_str += str(arg)
        eval_str += ")"
        eval(eval_str) 


    # private: _get_python_files: find all python file in a directory
    @classmethod
    def _get_python_files(start_dir):
        py_files = []
        for root, dirs, files in os.walk(start_dir):
            for filename in files:
                if filename.endswith('.py'):
                    full_path = os.path.join(root, filename)
                py_files.append(full_path[0:-3].replace('\\', '.')
        return py_files


    # following are some simple encapsulation of interfaces
    def cmd_vel(self, vel_x: float, vel_y: float, vel_theta: float):
        self._action.cmd_vel(x, y, theta)
    def kick(self):
        self._action.do_kick()
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
    print("[+] decider.py: main(): Decider started")
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
