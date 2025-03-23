# Dribble
#

import time
import math
from transitions import Machine

class dribble:
    """
    朝球门带球
    TODO: 处理丢球情况，检查方向
    """

    def __init__(self, agent):
        self.agent = agent
        self.states = ["forward", "adjust", "finished"]
        self.transitions = [
            {
                "trigger": "dribble",
                "source": "forward",
                "dest": "adjust",
                "conditions": "bad_position",
                "after": "adjust",
            },
            {
                "trigger": "dribble",
                "source": "adjust",
                "dest": "forward",
                "conditions": "good_position",
                "after": "forward",
            },
            {
                "trigger": "dribble",
                "source": "*",
                "dest": "finished",
                "conditions": "finished",
            },
        ]
        self.machine = Machine(
            model=self,
            states=self.states,
            initial="forward",
            transitions=self.transitions,
        )
        self.direction = True  # FIXME: True: right, False: left

    def run(self):
        while self.state != "finished":
            print("Dribbling...")
            self.machine.model.trigger("dribble")

    def forward(self):
        self.agent.speed_controller(0.5, 0, 0)
        time.sleep(0.5)

    def adjust(self):
        while not self.good_angel():  # 调整角度
            if self.direction:
                self.agent.speed_controller(0, 0, 0.1)
            else:
                self.agent.speed_controller(0, 0, -0.1)
            time.sleep(0.5)
        while self.agent.ball_y > 600:  # 后退
            self.agent.speed_controller(0, -0.1, 0)
            time.sleep(0.5)
        while abs(self.agent.ball_x - 640) > 10:  # 左右调整
            if self.agent.ball_x < 640:
                self.agent.speed_controller(0, 0.1, 0)
            else:
                self.agent.speed_controller(0, -0.1, 0)
            time.sleep(0.5)

    def good_position(self):
        return self.good_angel() and self.agent.ball_y < 600 and abs(self.agent.ball_x - 640) < 10

    def bad_position(self):
        return not self.good_position()

    def good_angel(self):
        rad1 = math.atan((self.agent.pos_x - 1300) / (4500 - self.agent.pos_y))  # FIXME:球门左侧
        ang_tar1 = rad1 * 180 / math.pi
        rad2 = math.atan((self.agent.pos_x + 1300) / (4500 - self.agent.pos_y))  # FIXME:球门右侧
        ang_tar2 = rad2 * 180 / math.pi
        if ang_tar1 < self.agent.pos_yaw < ang_tar2:
            return True
        else:
            self.direction = ang_tar1 > self.agent.pos_yaw
            return False

    def finished(self):
        return not self.agent.ifBall

    def start(self, args, last_statemachine):
        pass
    
    def stop(self, next_statemachine):
        pass


