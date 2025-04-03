import time
import math
from transitions import Machine


class DribbleStateMachine:
    """
    朝球门带球
    TODO: 处理丢球情况，检查方向
    """

    def __init__(self, agent):
        """
        初始化朝球门带球状态机
        :param agent: 代理对象
        """
        self.agent = agent
        self._config = self.agent.get_config()
        self.rotate_start_time = 0  # 记录旋转开始时间
        self.states = ["forward", "position_adjust", "angle_adjust", "finished"]
        self.transitions = [
            {
                "trigger": "dribble",
                "source": "forward",
                "dest": "position_adjust",
                "conditions": "lost_ball",
            },
            {
                "trigger": "dribble",
                "source": ["angle_adjust", "position_adjust"],
                "dest": "forward",
                "conditions": "good_position",
                "after": "dribble_forward",
            },
            {
                "trigger": "dribble",
                "source": "*",
                "dest": "finished",
                "conditions": "finished",
            },
            {
                "trigger": "dribble",
                "source": "forward",
                "dest": "forward",
                "conditions": "ok_to_forward",
                "after": "dribble_forward",
            },
            {
                "trigger": "dribble",
                "source": "finished",
                "dest": "position_adjust",
                "conditions": "not_finished",
            },
            {
                "trigger": "dribble",
                "source": "angle_adjust",
                "dest": "position_adjust",
                "conditions": "bad_position",
                "after": "adjust_position",
            },
            {
                "trigger": "dribble",
                "source": "position_adjust",
                "dest": "angle_adjust",
                "conditions": "bad_angle",
                "after": "adjust_angle",
            },
        ]
        self.machine = Machine(
            model=self,
            states=self.states,
            initial="forward",
            transitions=self.transitions,
        )
        self.direction = True  # FIXME: True: right, False: left
        print(f"[DRIBBLE FSM] Initialized. Starting state: {self.state}")

    def run(self):
        """
        运行状态机
        """
        # 如果没有球，直接return
        if not self.agent.ball_in_sight():
            print("No ball in sight.")
            return

        self.calculate_angle()

        self.machine.model.trigger("dribble")
        print("[DRIBBLE FSM] Running...")

    def forward(self):
        """
        向前带球
        """
        print("[DRIBBLE FSM] Moving forward...")
        self.agent.cmd_vel(0.5, 0, 0)
        time.sleep(0.5)
        print("[DRIBBLE FSM] Forward movement done")

    def adjust_position(self):
        """
        调整位置
        """
        print("[DRIBBLE FSM] Adjusting position...")
        if self.angle_ball_to_goal_rad > self.angle_to_goal_rad:
            self.agent.cmd_vel(0, 0.1, 0)
        else:
            self.agent.cmd_vel(0, -0.1, 0)
        time.sleep(0.5)
        print("[DRIBBLE FSM] Position adjustment done")

    def adjust_angle(self):
        """
        调整角度
        """
        print("[DRIBBLE FSM] Adjusting angle...")
        if self.agent.ball_x < 640:
            self.agent.cmd_vel(0, 0, -0.1)
        else:
            self.agent.cmd_vel(0, 0, 0.1)
        time.sleep(0.5)
        print("[DRIBBLE FSM] Angle adjustment done")

    def adjust_angle1(self):
        """
        另一种调整角度的方法
        """
        print("[DRIBBLE FSM] Adjusting angle (method 1)...")
        if not self.good_angle():  # 调整角度
            if self.direction:
                self.agent.cmd_vel(0, 0, 0.1)
            else:
                self.agent.cmd_vel(0, 0, -0.1)
        elif self.agent.ball_y > 600:  # 后退
            self.agent.cmd_vel(0, -0.1, 0)
        elif abs(self.agent.ball_x - 640) > 10:  # 左右调整
            if self.agent.ball_x < 640:
                self.agent.cmd_vel(0, 0.1, 0)
            else:
                self.agent.cmd_vel(0, -0.1, 0)
        time.sleep(0.5)
        print("[DRIBBLE FSM] Angle adjustment (method 1) done")

    def good_position(self):
        """
        检查是否处于好的位置
        :return: True 表示位置好，False 表示位置不好
        """
        result = abs(self.angle_to_goal_rad - self.angle_ball_to_goal_rad) < (20 * math.pi / 180) and abs(
            self.agent.get_self_yaw() / math.pi * 180 - self.angle_to_goal_rad) < (20 * math.pi / 180)
        print(f"[DRIBBLE FSM] Good position: {'Yes' if result else 'No'}")
        return result

    def bad_position(self):
        """
        检查是否处于不好的位置
        :return: True 表示位置不好，False 表示位置好
        """
        result = not self.good_position()
        print(f"[DRIBBLE FSM] Bad position: {'Yes' if result else 'No'}")
        return result

    def good_angle(self):
        """
        检查角度是否合适
        :return: True 表示角度合适，False 表示角度不合适
        """
        rad1 = math.atan((self.agent.get_self_pos()[0] - 1300) / (4500 - self.agent.get_self_pos()[1]))  # FIXME:球门左侧
        ang_tar1 = rad1 * 180 / math.pi
        rad2 = math.atan((self.agent.get_self_pos()[0] + 1300) / (4500 - self.agent.get_self_pos()[1]))  # FIXME:球门右侧
        ang_tar2 = rad2 * 180 / math.pi
        result = ang_tar1 < self.agent.get_self_yaw() < ang_tar2
        if not result:
            self.direction = ang_tar1 > self.agent.get_self_yaw()
        print(f"[DRIBBLE FSM] Good angle: {'Yes' if result else 'No'}")
        return result

    def bad_angle(self):
        """
        检查角度是否不合适
        :return: True 表示角度不合适，False 表示角度合适
        """
        result = not self.good_angle()
        print(f"[DRIBBLE FSM] Bad angle: {'Yes' if result else 'No'}")
        return result

    def finished(self):
        """
        检查是否完成带球
        :return: True 表示完成，False 表示未完成
        """
        result = self.good_position() and self.agent.ball_y_in_map > 3000
        print(f"[DRIBBLE FSM] Finished: {'Yes' if result else 'No'}")
        return result

    def not_finished(self):
        """
        检查是否未完成带球
        :return: True 表示未完成，False 表示完成
        """
        result = not self.finished()
        print(f"[DRIBBLE FSM] Not finished: {'Yes' if result else 'No'}")
        return result

    def ok_to_forward(self):
        """
        检查是否可以继续向前
        :return: True 表示可以，False 表示不可以
        """
        result = self.agent.get_self_pos()[1] < 3000 and abs(self.agent.get_self_yaw() / math.pi * 180 - self.angle_to_goal_rad) < (20 * math.pi / 180)
        print(f"[DRIBBLE FSM] Ok to forward: {'Yes' if result else 'No'}")
        return result

    def lost_ball(self):
        """
        检查是否丢球
        :return: True 表示丢球，False 表示未丢球
        """
        result = not (self.ok_to_forward() or self.finished())
        print(f"[DRIBBLE FSM] Lost ball: {'Yes' if result else 'No'}")
        return result

    def calculate_angle(self):
        """
        计算角度
        """
        print("[DRIBBLE FSM] Calculating angles...")
        if self.agent.get_self_pos()[0] > 0:
            if self.agent.get_self_pos()[0] > 1500:
                angle_to_goal_rad = math.atan((self.agent.get_self_pos()[0] - 1300) / (4500 - self.agent.get_self_pos()[1]))
            else:
                angle_to_goal_rad = 0.0
        else:
            if self.agent.get_self_pos()[0] < -1500:
                angle_to_goal_rad = math.atan((self.agent.get_self_pos()[0] + 1300) / (4500 - self.agent.get_self_pos()[1]))
            else:
                angle_to_goal_rad = 0.0

        if self.agent.ball_x_in_map > 0:
            if self.agent.ball_x_in_map > 1500:
                angle_ball_to_goal = math.atan((self.agent.ball_x_in_map - 1300) / (4500 - self.agent.ball_y_in_map))
            else:
                angle_ball_to_goal = 0.0
        else:
            if self.agent.ball_x_in_map < -1500:
                angle_ball_to_goal = math.atan((self.agent.ball_x_in_map + 1300) / (4500 - self.agent.ball_y_in_map))
            else:
                angle_ball_to_goal = 0.0

        self.angle_to_goal_rad = angle_to_goal_rad
        self.angle_ball_to_goal_rad = angle_ball_to_goal
        print("[DRIBBLE FSM] Angles calculated")