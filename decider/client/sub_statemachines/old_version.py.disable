# old_version.py
# State machine for robot using old system

import threading
import numpy as np
import rospy
import time
import math
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from decision_subscriber import Decision_Pos, Decision_Vision, Decision_Motion
from receiver import Receiver
from utils import config
from transitions import Machine

class old_version:
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
    def __init__(self, agent):
        self.agent = agent
        self.states = ["can_not_find_ball", "chase_ball", "dribble","find_ball","go_back_to_field","kick"]
        self.transitions = [
            {
                "trigger": "go_back_to_field",
                "source": "can_not_find_ball",
                "dest": "find_ball",
            },
        ]
        self.machine = Machine(
            model=self,
            states=self.states,
            initial="",
            transitions=self.transitions,
        )
        self.lock = threading.Lock()

    def run(self):
        pass
    
    def start(self, args, last_statemachine):
        pass
    
    def stop(self, next_statemachine):
        pass