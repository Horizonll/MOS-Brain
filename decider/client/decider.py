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
from typing import Optional

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
        self.id = self._config["id"]
        # last command time initialize to 1970-01-01 00:00:00
        self._last_command_time = time.mktime(time.strptime("1970-01-01 00:00:00", "%Y-%m-%d %H:%M:%S"))
        self._command = {
            "command": "stop",
            "data": {},
            "timestamp": time.time(),
        }
        self._lst_command = self._command

        self._robots_data = {}
        
        rospy.loginfo("Registering interfaces")
        # action: provide functions to control the robot, such as cmd_vel 
        # and kick
        self._action = interfaces.action.Action(self._config)
        # vision: provide functions to get information from the robot, 
        # such as self position and ball position
        self._vision = interfaces.vision.Vision(self._config)

        self.receiver = Receiver(self.get_config()["team"], self.get_config()["id"]-1)

        # robot_client: provide functions to communicate with the server
        self._robot_client = RobotClient(self)


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
        state = self.receiver.game_state
        penalized_time = self.receiver.penalized_time
        if penalized_time > 0:
            rospy.loginfo(f"Stopping: Player is penalized for {penalized_time} seconds")
            self.stop()
            return
        elif state in ['STATE_SET', 'STATE_FINISHED', 'STATE_INITIAL']:
            rospy.loginfo(f"Stopping: Game state is {state}")
            self.stop()
        elif state == 'STATE_READY':
            rospy.loginfo("Running: go_back_to_field (STATE_READY)")
            self._state_machine_runners['go_back_to_field']()
        elif time.time() - self._last_command_time > 5:
            if not self.get_if_ball():
                rospy.loginfo("Running: find_ball (lost command, no ball)")
                self._state_machine_runners['find_ball']()
            elif self.get_ball_distance() > 0.8:
                rospy.loginfo("Running: chase_ball (lost command, far ball)")
                self._state_machine_runners['chase_ball']()
            else:
                rospy.loginfo("Running: dribble (lost command, close ball)")
                self._state_machine_runners['dribble']()
        else:
            cmd = self._command["command"]
            if not self.get_if_ball() and cmd in ['chase_ball', 'kick', 'dribble']:
                rospy.loginfo(f"Running: find_ball (server cmd {cmd}, no ball)")
                self._state_machine_runners['find_ball']()
            elif cmd in self._state_machine_runners:
                rospy.loginfo(f"Running: {cmd} (server command)")
                self._state_machine_runners[cmd]()
            else:
                rospy.loginfo(f"Error: State machine {cmd} not found. Stopping.")
                self.stop()


    # The following are some simple encapsulations of interfaces
    # The implementation of the apis may be change in the future
    def cmd_vel(self, vel_x: float, vel_y: float, vel_theta: float):
        self._action.cmd_vel(vel_x, vel_y, vel_theta)
        rospy.loginfo(f"Setting the robot's speed: linear velocity x={vel_x}, "
                + f"y={vel_y}, angular velocity theta={vel_theta}")

    def look_at(self, args):
        self._vision.look_at(args)
    
    def stop(self, sleep_time=0):
        self._action.cmd_vel(0, 0, 0)
        time.sleep(sleep_time)

    def kick(self):
        self._action.do_kick()
        rospy.loginfo("Executing the kicking action")

    def get_robots_data(self):
        return self._robots_data

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
        if ball_pos is not None and ball_x is not None and ball_y is not None:
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

    def get_ball_pos_in_map_from_other_robots(self) -> Optional[np.ndarray]:
        """
        Calculate averaged ball position from connected robots with valid detections

        Returns:
            np.ndarray | None: Averaged ball position in map coordinates (x, y), 
            returns None if no valid data
        """
        print("Calculating ball position in map from other robots")

        valid_positions = []  # Stores valid (x,y) coordinates

        # Iterate through all robot data
        for robot_id, robot_data in self.get_robots_data().items():
            print(f"Robot ID: {robot_id}, Data: {robot_data}")
            # robot id 转换为 int
            robot_id = int(robot_id)
            # Skip self and disconnected robots
            if robot_id == self.id or robot_data.get('status') != 'connected':
                continue

            # Check ball detection status
            if robot_data.get("data").get('if_ball', False):
                # Extract coordinates
                ballx = robot_data['data'].get('ballx')
                bally = robot_data['data'].get('bally')

                # Validate coordinate existence
                if None not in (ballx, bally):
                    try:
                        # Convert to float array
                        pos = np.array([float(ballx), float(bally)], dtype=np.float32)
                        valid_positions.append(pos)
                        logging.debug(f"Valid position from {robot_id}: {pos}")
                    except (TypeError, ValueError) as e:
                        logging.warning(f"Invalid coordinates from {robot_id}: {e}")

        # Calculate simple average
        if valid_positions:
            avg_pos = np.mean(valid_positions, axis=0)
            logging.debug(f"Fused position from {len(valid_positions)} robots: {avg_pos}")
            return avg_pos

        logging.info("No valid ball positions from peer robots")
        return None
    
    def get_ball_angle_from_other_robots(self) -> Optional[float]:
        """
        Calculate averaged ball angle from connected robots with valid detections

        Returns:
            float | None: Averaged ball angle in radians, returns None if no valid data
        """
        ball_pos_in_map = self.get_ball_pos_in_map_from_other_robots()
        logging.debug(f"Ball position in map from other robots: {ball_pos_in_map}")
        logging.debug(f"Self position: {self.get_self_pos()}")
        if ball_pos_in_map is not None:
            # Calculate angle in radians
            ball_pos_relative = ball_pos_in_map - np.array(self.get_self_pos())
            print(f"Ball position relative to self: {ball_pos_relative}")
            angle_rad = math.atan2(ball_pos_relative[1], ball_pos_relative[0])
            print(f"Ball angle in radians: {angle_rad}")
            angle_relative = angle_rad - (self.get_self_yaw() / 180 * np.pi) - np.pi / 2
            print(f"Ball angle relative to self: {angle_relative}")
            # Normalize angle to [-pi, pi)
            angle_relative = (angle_relative + math.pi) % (2 * math.pi) - math.pi

            print(f"Ball angle from other robots: {angle_relative}")

            return angle_relative
        return None

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
