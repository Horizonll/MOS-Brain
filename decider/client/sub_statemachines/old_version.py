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
    def __init__(self, agent):
        self.agent = agent
        self.states = ["", "", ""]
        self.transitions = [
            {
                "trigger": "",
                "source": "",
                "dest": "",
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