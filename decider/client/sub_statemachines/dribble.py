import time
import math
import numpy as np
import rospy
from transitions import Machine


class DribbleStateMachine:
    """
    朝某个方向带球
    """

    def __init__(self, agent):
        """
        初始化朝球门带球状态机
        :param agent: 代理对象
        """
        self.agent = agent
        self._config = self.agent.get_config()
        self.read_params()  # 新增参数读取方法

        self.states = ["forward", "pos_to_ball_adjust", "yaw_adjust", "horizontal_position_adjust"]
        self.transitions = [
            {
                "trigger": "dribble",
                "source": "forward",
                "dest": "forward",
                "conditions": ["not_lost_ball", "not_lost_yaw"],
                "after": "dribble_forward",
            },
            {
                "trigger": "dribble",
                "source": "forward",
                "dest": "horizontal_position_adjust",
                "conditions": "lost_ball_x",
                "after": "adjust_horizontal_position",
            },
            {
                "trigger": "dribble",
                "source": ["yaw_adjust", "horizontal_position_adjust"],
                "dest": "pos_to_ball_adjust",
                "conditions": ["lost_ball"],
            },
            {
                "trigger": "dribble",
                "source": ["forward"],
                "dest": "pos_to_ball_adjust",
                "conditions": ["lost_yaw"],
            },
            {
                "trigger": "dribble",
                "source": ["pos_to_ball_adjust"],
                "dest": "pos_to_ball_adjust",
                "conditions": "bad_pos_to_ball",
                "after": "adjust_pos_to_ball",
            },
            {
                "trigger": "dribble",
                "source": ["pos_to_ball_adjust"],
                "dest": "yaw_adjust",
                "conditions": "good_pos_to_ball",
                "after": "stop_moving",
            },
            {
                "trigger": "dribble",
                "source": ["yaw_adjust"],
                "dest": "yaw_adjust",
                "conditions": "bad_yaw_angle",
                "after": "adjust_yaw_angle",
            },
            {
                "trigger": "dribble",
                "source": ["yaw_adjust"],
                "dest": "horizontal_position_adjust",
                "conditions": "good_yaw_angle",
                "after": "adjust_horizontal_position",
            },
            {
                "trigger": "dribble",
                "source": ["horizontal_position_adjust"],
                "dest": "horizontal_position_adjust",
                "conditions": "bad_horizontal_position",
                "after": "adjust_horizontal_position",
            },
            {
                "trigger": "dribble",
                "source": ["horizontal_position_adjust"],
                "dest": "forward",
                "conditions": "good_horizontal_position",
                "after": "stop_moving",
            },
        ]
        self.machine = Machine(
            model=self,
            states=self.states,
            initial="pos_to_ball_adjust",
            transitions=self.transitions,
            after_state_change=self.print_state,
        )
        self.direction = True  # FIXME: True: right, False: left
        rospy.loginfo(f"[DRIBBLE FSM] Initialized. Starting state: {self.state}")

    def run(self):
        """
        运行状态机
        """
        # 如果没有球，直接return
        if not self.agent.get_if_ball():
            rospy.logwarn("No ball in sight.")
            self.agent.cmd_vel(0, 0, 0)
            return

        self.calculate_angle()

        # 计算目标角度
        self.aim_yaw = self.agent.get_command().get('data').get('aim_yaw', None)

        if self.aim_yaw is None:
            if self.agent.get_self_pos()[1] > self._config.get("dribble", {}).get("dribble_0_y", 3700):
                self.aim_yaw = 0.0
            else:
                self.aim_yaw = self.calc_angle_to_goal_degree()
            rospy.loginfo(f"[DRIBBLE FSM] Calculated aim_yaw: {self.aim_yaw:.2f}°")

        self.machine.model.trigger("dribble")
        rospy.loginfo("[DRIBBLE FSM] Running...")
        rospy.loginfo(f"[DRIBBLE FSM] Current state: {self.state}")

    def print_state(self):
        """
        打印当前状态
        """
        rospy.loginfo(f"[DRIBBLE FSM] Current state: {self.state}")

    def forward(self):
        """
        向前带球
        """
        rospy.loginfo("[DRIBBLE FSM] Moving forward...")

        vel_x = self.forward_vel
        self.agent.cmd_vel(vel_x, 0, (self.agent.get_self_yaw()-self.aim_yaw)/30)
        rospy.loginfo("[DRIBBLE FSM] Forward movement done")

    def stop_moving(self):
        """
        停止移动
        """
        rospy.loginfo("[DRIBBLE FSM] Stopping...")
        self.agent.cmd_vel(0, 0, 0)
        time.sleep(0.4)
        rospy.loginfo("[DRIBBLE FSM] Stopped")

    def adjust_pos_to_ball(self):
        """
        调整机器人位置以靠近球
        """
        rospy.loginfo("[DRIBBLE FSM] Adjusting position to ball...")
        target_angle_rad = self.agent.get_ball_angle()
        ball_distance = self.agent.get_ball_distance()

        if abs(target_angle_rad) > self.angle_to_ball_adjust_threshold_rad:
            rospy.loginfo(
                f"[DRIBBLE FSM] target_angle_rad ({target_angle_rad}) > {self.angle_to_ball_adjust_threshold_rad}. ball_distance: {ball_distance}. Rotating..."
            )
            self.agent.cmd_vel(
                0, 0, np.sign(target_angle_rad) * self.rotate_vel_theta
            )
        elif ball_distance > self.max_ball_distance_m:
            rospy.loginfo(
                f"[DRIBBLE FSM] ball_distance ({ball_distance}) > {self.max_ball_distance_m}. Moving backward..."
            )
            self.agent.cmd_vel(
                self.forward_vel,
                0,
                0,
            )
        elif ball_distance < self.min_ball_distance_m:
            rospy.loginfo(
                f"[DRIBBLE FSM] ball_distance ({ball_distance}) < 0.35. Moving forward..."
            )
            self.agent.cmd_vel(
                -self.forward_vel,
                0,
                0,
            )

        rospy.loginfo("[DRIBBLE FSM] Position adjustment step completed.")

    def good_pos_to_ball(self):
        """
        检查球相对机器人的位置是否合适
        :return: True 表示位置合适，False 表示位置不合适
        """
        result = self.min_ball_distance_m < self.agent.get_ball_distance() < self.max_ball_distance_m and self.good_angle_to_ball()
        rospy.loginfo(f"[DRIBBLE FSM] Good position to ball: {'Yes' if result else 'No'}")
        return result
    
    def bad_pos_to_ball(self):
        """
        检查球相对机器人的位置是否不合适
        :return: True 表示位置不合适，False 表示位置合适
        """
        result = not self.good_pos_to_ball()
        rospy.loginfo(f"[DRIBBLE FSM] Bad position to ball: {'Yes' if result else 'No'}")
        return result
    
    def adjust_horizontal_position(self):
        """
        Adjust the robot's horizontal position to approach the ball
        调整机器人左右位置以靠近球
        """
        rospy.loginfo("[DRIBBLE FSM] Starting horizontal position adjustment...")

        # neck_angle = self.agent.get_ball_angle()
        ball_x = self.agent.get_ball_pos()[0]
        if ball_x > 0:
            rospy.loginfo(f"[DRIBBLE FSM] Moving left (Ball x: {ball_x})")
            self.agent.cmd_vel(
                0,
                -self.horizontal_adjust_vel_y,
                0
            )
        elif ball_x < 0:
            rospy.loginfo(f"[DRIBBLE FSM] Moving right (Ball x: {ball_x})")
            self.agent.cmd_vel(
                0,
                self.horizontal_adjust_vel_y,
                0
            )

    def adjust_ball_angle(self):
        """
        调整角度
        """
        rospy.loginfo("[DRIBBLE FSM] Adjusting angle...")
        target_angle_rad = self.agent.get_ball_angle()

        rospy.loginfo(f"[DRIBBLE FSM] Adjusting angle... Current angle: {target_angle_rad}")
        self.agent.cmd_vel(
            0, 0, np.sign(target_angle_rad) * self.rotate_vel_theta
        )

    def bad_angle_to_ball(self):
        """
        检查球相对机器人的角度是否合适
        :return: True 表示角度不合适，False 表示角度合适
        """
        target_angle_rad = self.agent.get_ball_angle()
        result =  not abs(target_angle_rad) > self.angle_to_ball_adjust_threshold_rad
        rospy.loginfo(f"[DRIBBLE FSM] Bad angle to ball: {'Yes' if result else 'No'}")
        return result

    def good_angle_to_ball(self):
        """
        检查球相对机器人的角度是否合适
        :return: True 表示角度合适，False 表示角度不合适
        """
        target_angle_rad = self.agent.get_ball_angle()
        result = abs(target_angle_rad) < self.angle_to_ball_adjust_threshold_rad
        rospy.loginfo(f"[DRIBBLE FSM] Good angle to ball: {'Yes' if result else 'No'}")
        return result

    def good_yaw_angle(self):
        """
        检查是否处于好的角度
        :return: True 表示角度好，False 表示角度不好
        """
        """Check if angle is within acceptable range"""
        ang_tar = self.aim_yaw
        ang_delta = ang_tar - self.agent.get_self_yaw()
        result = abs(ang_delta) < self.good_angle_to_goal_threshold_degree

        rospy.loginfo(
            f"[ANGLE CHECK] Angle delta: {abs(ang_delta):.2f}° (OK? {'Yes' if result else 'No'})"
        )
        return result

    def bad_yaw_angle(self):
        """
        检查是否处于不好的角度
        :return: True 表示角度不好，False 表示角度好
        """
        """Check if angle is outside acceptable range"""
        result = not self.good_yaw_angle()
        rospy.loginfo(
            f"[ANGLE CHECK] Bad yaw angle: {'Yes' if result else 'No'} (angle delta: {abs(self.aim_yaw - self.agent.get_self_yaw()):.2f}°)"
        )
        return result

    def adjust_yaw_angle(self):
        """Adjust robot's yaw angle relative to the goal"""
        rospy.loginfo("[DRIBBLE FSM] Starting yaw angle adjustment...")

        # Calculate target yaw angle using the robot's position
        target_angle_deg = self.aim_yaw
        current_yaw = self.agent.get_self_yaw()
        yaw_delta = target_angle_deg - current_yaw

        rospy.loginfo(
            f"[DRIBBLE FSM] Target yaw: {target_angle_deg:.2f}°, Current yaw: {current_yaw:.2f}°, Delta: {yaw_delta:.2f}°"
        )

        if yaw_delta > self.good_angle_to_goal_threshold_degree:
            rospy.loginfo(f"[DRIBBLE FSM] Rotating CCW (Δ={yaw_delta:.2f}°)")
            self.agent.cmd_vel(0, -self.adjust_angle_to_goal_vel_y, self.adjust_angle_to_goal_vel_theta)
        elif yaw_delta < -self.good_angle_to_goal_threshold_degree:
            rospy.loginfo(f"[DRIBBLE FSM] Rotating CW (Δ={yaw_delta:.2f}°)")
            self.agent.cmd_vel(0, self.adjust_angle_to_goal_vel_y, -self.adjust_angle_to_goal_vel_theta)
        else:
            rospy.loginfo("[DRIBBLE FSM] Yaw angle is within acceptable range.")

    def good_horizontal_position(self):
        """
        检查是否处于好的位置
        :return: True 表示位置好，False 表示位置不好
        """
        ball_x = self.agent.get_ball_pos()[0]
        result = abs(ball_x) < self.good_horizontal_position_to_ball_threshold_mm
        rospy.loginfo(f"[DRIBBLE FSM] Good position: {'Yes' if result else 'No'}")
        return result

    def bad_horizontal_position(self):
        """
        检查是否处于不好的位置
        :return: True 表示位置不好，False 表示位置好
        """
        ball_x = self.agent.get_ball_pos()[0]
        result = abs(ball_x) > self.good_horizontal_position_to_ball_threshold_mm
        rospy.loginfo(f"[DRIBBLE FSM] Bad position: {'Yes' if result else 'No'}")
        return result

    def lost_ball(self):
        """
        检查是否丢球
        :return: True 表示丢球，False 表示未丢球
        """
        # neck_angle = self.agent.get_ball_angle()
        # ball_x = self.agent.get_ball_pos()[0]
        ball_distance = self.agent.get_ball_distance()
        # ball_x_lost = abs(ball_x) > 100
        ball_distance_lost = ball_distance > self.lost_ball_distance_threshold_m # 0.6
        # yaw_angle = self.agent.get_self_yaw()
        # yaw_angle_lost = abs(yaw_angle) > 30 * math.pi / 180
        result = ball_distance_lost
        # if yaw_angle_lost:
        #     print(f"[DRIBBLE FSM] Yaw angle lost: {yaw_angle:.2f} rad")

        rospy.loginfo(f"[DRIBBLE FSM] Ball distance: {ball_distance:.2f} m")
        rospy.loginfo(f"[DRIBBLE FSM] Lost ball: {'Yes' if result else 'No'}")
        return result

    def lost_yaw(self):
        """
        检查是否丢球
        :return: True 表示丢球，False 表示未丢球
        """
        # neck_angle = self.agent.get_ball_angle()
        yaw_angle = self.aim_yaw - self.agent.get_self_yaw()
        yaw_angle_lost = abs(yaw_angle) > self.lost_angle_to_target_threshold_degree # 45
        result = yaw_angle_lost
        rospy.loginfo(f"[DRIBBLE FSM] Lost ball yaw: {'Yes' if result else 'No'}")
        return result

    def lost_ball_x(self):
        """
        检查是否丢球
        :return: True 表示丢球，False 表示未丢球
        """
        # neck_angle = self.agent.get_ball_angle()
        ball_x = self.agent.get_ball_pos()[0]
        ball_x_lost = abs(ball_x) > self.lost_ball_x_threshold_mm  # 80
        result = ball_x_lost
        rospy.loginfo(f"[DRIBBLE FSM] Lost ball x: {'Yes' if result else 'No'}")
        return result

    def not_lost_ball(self):
        """
        检查是否未丢球
        :return: True 表示未丢球，False 表示丢球
        """
        result = not self.lost_ball() and not self.lost_ball_x()
        rospy.loginfo(f"[DRIBBLE FSM] Not lost ball: {'Yes' if result else 'No'}")
        return result
    
    def not_lost_yaw(self):
        """
        检查是否未丢球
        :return: True 表示未丢球，False 表示丢球
        """
        result = not self.lost_yaw()
        rospy.loginfo(f"[DRIBBLE FSM] Not lost yaw: {'Yes' if result else 'No'}")
        return result

    def calculate_angle(self):
        """
        计算角度
        """
        rospy.loginfo("[DRIBBLE FSM] Calculating angles...")

        if self.agent.get_self_pos()[0] > 0:
            if self.agent.get_self_pos()[0] > self.goal_center_bias_mm:
                angle_to_goal_rad = math.atan((self.agent.get_self_pos()[0] - self.goal_center_bias_mm) / (5000 - self.agent.get_self_pos()[1]))
            else:
                angle_to_goal_rad = 0.0
        else:
            if self.agent.get_self_pos()[0] < -self.goal_center_bias_mm:
                angle_to_goal_rad = math.atan((self.agent.get_self_pos()[0] + self.goal_center_bias_mm) / (5000 - self.agent.get_self_pos()[1]))
            else:
                angle_to_goal_rad = 0.0

        if self.agent.get_ball_pos_in_map()[0] > 0:
            if self.agent.get_ball_pos_in_map()[0] > self.goal_center_bias_mm:
                angle_ball_to_goal = math.atan((self.agent.get_ball_pos_in_map()[0] - self.goal_center_bias_mm) / (5000 - self.agent.get_ball_pos_in_map()[1]))
            else:
                angle_ball_to_goal = 0.0
        else:
            if self.agent.get_ball_pos_in_map()[0] < -self.goal_center_bias_mm:
                angle_ball_to_goal = math.atan((self.agent.get_ball_pos_in_map()[0] + self.goal_center_bias_mm) / (5000 - self.agent.get_ball_pos_in_map()[1]))
            else:
                angle_ball_to_goal = 0.0

        self.angle_to_goal_rad = angle_to_goal_rad
        self.angle_ball_to_goal_rad = angle_ball_to_goal
        rospy.loginfo("[DRIBBLE FSM] Angles calculated")

    def dribble_forward(self):
        """
        向前带球
        """
        rospy.loginfo("[DRIBBLE FSM] Dribbling forward...")
        vel_x = self.forward_vel
        vel_y = 0.0
        vel_theta = 0.0

        # if self.direction:
        #     vel_y = -self._config.get("walk_vel_y", 0.05)
        #     vel_theta = -self._config.get("walk_vel_theta", 0.3)
        # else:
        #     vel_y = self._config.get("walk_vel_y", 0.05)
        #     vel_theta = self._config.get("walk_vel_theta", -0.3)

        self.agent.cmd_vel(vel_x, vel_y, vel_theta)
        rospy.loginfo("[DRIBBLE FSM] Dribbling forward done")

    def calc_angle_to_goal_degree(self):
        """
        计算朝球门的角度
        """
        if self.agent.get_self_pos()[0] > 0:
            if self.agent.get_self_pos()[0] > self.goal_center_bias_mm:
                angle_to_goal_rad = math.atan((self.agent.get_self_pos()[0] - self.goal_center_bias_mm) / (5000 - self.agent.get_self_pos()[1]))
            else:
                angle_to_goal_rad = 0.0
        else:
            if self.agent.get_self_pos()[0] < -self.goal_center_bias_mm:
                angle_to_goal_rad = math.atan((self.agent.get_self_pos()[0] + self.goal_center_bias_mm) / (5000 - self.agent.get_self_pos()[1]))
            else:
                angle_to_goal_rad = 0.0

        return angle_to_goal_rad * 180 / math.pi
    
    def read_params(self):

        self.angle_to_ball_adjust_threshold_rad = self._config.get("dribble", {}).get(
            "angle_to_ball_adjust_threshold_degree", 15
        ) * math.pi / 180
        self.good_angle_to_goal_threshold_degree = self._config.get("dribble", {}).get(
            "good_angle_to_goal_threshold_degree", 10
        )
        self.goal_center_bias_mm = self._config.get("dribble", {}).get("goal_center_bias_mm", 0)
        
        self.dribble_stop_angle_threshold_rad = 0.2
        self.min_ball_distance_m = self._config.get("dribble", {}).get(
            "min_ball_distance_m", 0.35
        )
        self.max_ball_distance_m = self._config.get("dribble", {}).get(
            "max_ball_distance_m", 0.55
        )

        self.good_horizontal_position_to_ball_threshold_mm = self._config.get("dribble", {}).get(
            "good_horizontal_position_to_ball_threshold_mm", 50
        )

        self.lost_angle_to_target_threshold_degree = self._config.get("dribble", {}).get(
            "lost_angle_to_target_threshold_degree", 20
        )

        self.lost_ball_x_threshold_mm = self._config.get("dribble", {}).get(
            "lost_ball_x_threshold_mm", 80
        )

        self.lost_ball_distance_threshold_m = self._config.get("dribble", {}).get(
            "lost_ball_distance_threshold_m", 0.6
        )

        self.forward_vel = self._config.get("dribble", {}).get("walk_vel_x", 0.1)
        self.backward_vel = self._config.get("dribble", {}).get("back_vel", 0.05)
        self.horizontal_adjust_vel_y = self._config.get("dribble", {}).get("horizontal_adjust_vel_y", 0.3)
        self.horizontal_adjust_vel_theta = self._config.get("dribble", {}).get("horizontal_adjust_vel_theta", 0.3)
        self.rotate_vel_theta = self._config.get("dribble", {}).get("rotate_vel_theta", 0.3)
        self.adjust_angle_to_goal_vel_y = self._config.get("dribble", {}).get("adjust_angle_to_goal_vel_y", 0.3)
        self.adjust_angle_to_goal_vel_theta = self._config.get("dribble", {}).get("adjust_angle_to_goal_vel_theta", 0.3)
    
    # def good_angle(self):
    #     """
    #     检查角度是否合适
    #     :return: True 表示角度合适，False 表示角度不合适
    #     """
    #     rad1 = math.atan((self.agent.get_self_pos()[0] - self.goal_center_bias_mm) / (5000 - self.agent.get_self_pos()[1]))  # FIXME:球门左侧
    #     ang_tar1 = rad1 * 180 / math.pi
    #     rad2 = math.atan((self.agent.get_self_pos()[0] + self.goal_center_bias_mm) / (5000 - self.agent.get_self_pos()[1]))  # FIXME:球门右侧
    #     ang_tar2 = rad2 * 180 / math.pi
    #     result = ang_tar1 < self.agent.get_self_yaw() < ang_tar2
    #     if not result:
    #         self.direction = ang_tar1 > self.agent.get_self_yaw()
    #     print(f"[DRIBBLE FSM] Good angle: {'Yes' if result else 'No'}")
    #     return result

    # def bad_angle(self):
    #     """
    #     检查角度是否不合适
    #     :return: True 表示角度不合适，False 表示角度合适
    #     """
    #     result = not self.good_angle()
    #     print(f"[DRIBBLE FSM] Bad angle: {'Yes' if result else 'No'}")
    #     return result

    # def ok_to_forward(self):
    #     """
    #     检查是否可以继续向前
    #     :return: True 表示可以，False 表示不可以
    #     """
    #     result = self.agent.get_self_pos()[1] < 4000 and abs(self.agent.get_self_yaw() / math.pi * 180 - self.aim_yaw) < (20 * math.pi / 180)
    #     print(f"[DRIBBLE FSM] Ok to forward: {'Yes' if result else 'No'}")
    #     return result
