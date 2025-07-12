import time
import math
import numpy as np
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
        self.logger = self.agent.get_logger().get_child("dribble_fsm")
        self._config = self.agent.get_config()
        self.read_params()  # 新增参数读取方法

        self.states = ["forward", "pos_to_ball_adjust", "yaw_adjust", "horizontal_position_adjust"]
        self.transitions = [
            {
                "trigger": "dribble",
                "source": "forward",
                "dest": "forward",
                "conditions": ["not_lost_ball_distance", "not_lost_yaw"],
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
                "source": ["forward"],
                "dest": "pos_to_ball_adjust",
                "conditions": ["lost_yaw"]
            },
            {
                "trigger": "dribble",
                "source": ["forward"],
                "dest": "pos_to_ball_adjust",
                "conditions": ["lost_ball_x"]
            },
            {
                "trigger": "dribble",
                "source": ["horizontal_position_adjust"],
                "dest": "pos_to_ball_adjust",
                "conditions": ["lost_ball_distance"],
            },
            # {
            #     "trigger": "dribble",
            #     "source": ["yaw_adjust"],
            #     "dest": "pos_to_ball_adjust",
            #     "conditions": "bad_distance_for_yaw_adjust",
            #     "after": "adjust_pos_to_ball",
            # },
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
        self.logger.debug(f"[DRIBBLE FSM] Initialized. Starting state: {self.state}")

    def run(self, aim_yaw=None):
        """
        运行状态机
        """
        self.agent.move_head(math.inf, math.inf)
        # 如果没有球，直接return
        if not self.agent.get_if_ball():
            self.logger.warn("No ball in sight.")
            self.agent.cmd_vel(0, 0, 0)
            return

        self.calculate_angle()

        # 计算目标角度
        self.aim_yaw = self.agent.get_command().get('data').get('aim_yaw', None)

        if self.aim_yaw is None:
            if self.agent.get_self_pos()[1] > self._config.get("field_size", [9,6]).get(self.agent.league)[0] * self.blind_dribble_thres_percent / 2:
                self.aim_yaw = 0.0
            else:
                self.aim_yaw = self.calc_angle_to_goal_degree()
            self.logger.debug(f"[DRIBBLE FSM] Calculated aim_yaw: {self.aim_yaw:.2f}°")

        if aim_yaw is not None:
            self.aim_yaw = aim_yaw
            self.logger.debug(f"[DRIBBLE FSM] Using provided aim_yaw: {self.aim_yaw:.2f}°")

        if self.obstacle_avoidance:
            self_yaw = self.agent.get_self_yaw()
            self.aim_yaw = self_yaw + self.agent.get_obstacle_avoidance_angle_degree(self.aim_yaw-self_yaw)

        self.machine.model.trigger("dribble")
        self.logger.debug("[DRIBBLE FSM] Running...")
        self.logger.debug(f"[DRIBBLE FSM] Current state: {self.state}")

    def print_state(self):
        """
        打印当前状态
        """
        self.logger.debug(f"[DRIBBLE FSM] Current state: {self.state}")

    # def forward(self):
    #     """
    #     向前带球
    #     """
    #     self.logger.debug("[DRIBBLE FSM] Moving forward...")

    #     vel_x = self.adjust_pos_to_ball_vel_x
    #     self.agent.cmd_vel(vel_x, 0, (self.agent.get_self_yaw()-self.aim_yaw)/30)
    #     self.logger.debug("[DRIBBLE FSM] Forward movement done")

    def stop_moving(self):
        """
        停止移动
        """
        self.logger.debug("[DRIBBLE FSM] Stopping...")
        self.agent.cmd_vel(0, 0, 0)
        self.logger.debug("[DRIBBLE FSM] Stopped")

    def adjust_pos_to_ball(self):
        """
        调整机器人位置以靠近球
        """
        self.logger.debug("[DRIBBLE FSM] Adjusting position to ball...")
        target_angle_rad = self.agent.get_ball_angle()
        ball_distance = self.agent.get_ball_distance()

        if abs(target_angle_rad) > self.angle_to_ball_adjust_threshold_rad:
            self.logger.debug(
                f"[DRIBBLE FSM] target_angle_rad ({target_angle_rad}) > {self.angle_to_ball_adjust_threshold_rad}. ball_distance: {ball_distance}. Rotating..."
            )
            self.agent.cmd_vel(
                0, 0, np.sign(target_angle_rad) * self.rotate_vel_theta
            )
        elif ball_distance < self.max_ball_distance_m + 0.5:
            self.logger.debug(
                f"[DRIBBLE FSM] ball_distance ({ball_distance}) > {self.max_ball_distance_m}. Moving forward..."
            )
            self.agent.cmd_vel(
                0.5*self.adjust_pos_to_ball_vel_x,
                0,
                0,
            )
        elif ball_distance > self.max_ball_distance_m:
            self.logger.debug(
                f"[DRIBBLE FSM] ball_distance ({ball_distance}) > {self.max_ball_distance_m}. Moving forward..."
            )
            self.agent.cmd_vel(
                self.adjust_pos_to_ball_vel_x,
                0,
                0,
            )
        elif ball_distance < self.min_ball_distance_m:
            self.logger.debug(
                f"[DRIBBLE FSM] ball_distance ({ball_distance}) < 0.35. Moving backward..."
            )
            self.agent.cmd_vel(
                -self.adjust_pos_to_ball_vel_x,
                0,
                0,
            )

        self.logger.debug("[DRIBBLE FSM] Position adjustment step completed.")

    def good_pos_to_ball(self):
        """
        检查球相对机器人的位置是否合适
        :return: True 表示位置合适，False 表示位置不合适
        """
        result = self.min_ball_distance_m < self.agent.get_ball_distance() < self.max_ball_distance_m and self.good_angle_to_ball()
        self.logger.debug(f"[DRIBBLE FSM] Good position to ball: {'Yes' if result else 'No'}")
        return result
    
    def bad_pos_to_ball(self):
        """
        检查球相对机器人的位置是否不合适
        :return: True 表示位置不合适，False 表示位置合适
        """
        result = not self.good_pos_to_ball()
        self.logger.debug(f"[DRIBBLE FSM] Bad position to ball: {'Yes' if result else 'No'}")
        return result

    def bad_distance_for_yaw_adjust(self):
        """
        检查是否需要调整角度
        :return: True 表示需要调整角度，False 表示不需要调整角度
        """
        ball_distance = self.agent.get_ball_distance()
        result = ball_distance < self.min_ball_distance_m or ball_distance > self.max_ball_distance_m
        self.logger.debug(f"[DRIBBLE FSM] Bad distance for yaw adjust: {'Yes' if result else 'No'}")
        return result
    
    def adjust_horizontal_position(self):
        """
        Adjust the robot's horizontal position to approach the ball
        调整机器人左右位置以靠近球
        """
        self.logger.debug("[DRIBBLE FSM] Starting horizontal position adjustment...")

        # neck_angle = self.agent.get_ball_angle()
        ball_x = self.agent.get_ball_pos()[0] - self.camera_bias
        feet_center = (self.good_horizontal_position_to_ball_upper_threshold_m + self.good_horizontal_position_to_ball_lower_threshold_m) / 2
        if ball_x > 0:
            self.logger.debug(f"[DRIBBLE FSM] Moving left (Ball x: {ball_x})")
            if ball_x > feet_center:
                self.agent.cmd_vel(
                    0,
                    -self.horizontal_adjust_vel_y,
                    0
                )
            else:
                self.agent.cmd_vel(
                    0,
                    self.horizontal_adjust_vel_y,
                    0
                )
        elif ball_x < 0:
            self.logger.debug(f"[DRIBBLE FSM] Moving right (Ball x: {ball_x})")
            if ball_x < -feet_center:
                self.agent.cmd_vel(
                    0,
                    self.horizontal_adjust_vel_y,
                    0
                )
            else:
                self.agent.cmd_vel(
                    0,
                    -self.horizontal_adjust_vel_y,
                    0
                )

    def adjust_ball_angle(self):
        """
        调整角度
        """
        self.logger.debug("[DRIBBLE FSM] Adjusting angle...")
        target_angle_rad = self.agent.get_ball_angle()

        self.logger.debug(f"[DRIBBLE FSM] Adjusting angle... Current angle: {target_angle_rad}")
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
        self.logger.debug(f"[DRIBBLE FSM] Bad angle to ball: {'Yes' if result else 'No'}")
        return result

    def good_angle_to_ball(self):
        """
        检查球相对机器人的角度是否合适
        :return: True 表示角度合适，False 表示角度不合适
        """
        target_angle_rad = self.agent.get_ball_angle()
        result = abs(target_angle_rad) < self.angle_to_ball_adjust_threshold_rad
        self.logger.debug(f"[DRIBBLE FSM] Good angle to ball: {'Yes' if result else 'No'}")
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

        self.logger.debug(
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
        self.logger.debug(
            f"[ANGLE CHECK] Bad yaw angle: {'Yes' if result else 'No'} (angle delta: {abs(self.aim_yaw - self.agent.get_self_yaw()):.2f}°)"
        )
        return result
    


    def adjust_yaw_angle(self):
        """Adjust robot's yaw angle relative to the goal"""
        self.logger.debug("[DRIBBLE FSM] Starting yaw angle adjustment...")

        # Calculate target yaw angle using the robot's position
        target_angle_deg = self.aim_yaw
        current_yaw = self.agent.get_self_yaw()
        yaw_delta = target_angle_deg - current_yaw
        yaw_delta_rad = yaw_delta * math.pi / 180
        yaw_delta_rad_normalized = self.agent.angle_normalize(yaw_delta_rad)
        yaw_delta = yaw_delta_rad_normalized * 180 / math.pi
        ball_y_distance = self.agent.get_ball_pos()[1] if not self.agent.get_ball_pos()[1] is None else 0.5
        max_y_vel = self.agent.get_config().get("max_walk_vel_y")
        max_theta_vel = self.agent.get_config().get("max_walk_vel_theta")
        ratio = max_y_vel / max_theta_vel * 0.3
        


        self.logger.debug(
            f"[DRIBBLE FSM] Target yaw: {target_angle_deg:.2f}°, Current yaw: {current_yaw:.2f}°, Delta: {yaw_delta:.2f}°"
        )
        
        if yaw_delta > self.good_angle_to_goal_threshold_degree:
            self.logger.debug(f"[DRIBBLE FSM] Rotating CCW (Δ={yaw_delta:.2f}°)")
            self.agent.cmd_vel(0, -ball_y_distance * self.adjust_angle_to_goal_vel_theta / ratio, self.adjust_angle_to_goal_vel_theta)
        elif yaw_delta < -self.good_angle_to_goal_threshold_degree:
            self.logger.debug(f"[DRIBBLE FSM] Rotating CW (Δ={yaw_delta:.2f}°)")
            self.agent.cmd_vel(0, ball_y_distance * self.adjust_angle_to_goal_vel_theta / ratio, -self.adjust_angle_to_goal_vel_theta)
        else:
            self.logger.debug("[DRIBBLE FSM] Yaw angle is within acceptable range.")

    def good_horizontal_position(self):
        """
        检查是否处于好的位置
        :return: True 表示位置好，False 表示位置不好
        """
        ball_x = self.agent.get_ball_pos()[0] - self.camera_bias
        result = self.good_horizontal_position_to_ball_lower_threshold_m < abs(ball_x) < self.good_horizontal_position_to_ball_upper_threshold_m
        self.logger.debug(f"[DRIBBLE FSM] Good position: {'Yes' if result else 'No'}")
        return result

    def bad_horizontal_position(self):
        """
        检查是否处于不好的位置
        :return: True 表示位置不好，False 表示位置好
        """
        result = not self.good_horizontal_position()
        self.logger.debug(f"[DRIBBLE FSM] Bad position: {'Yes' if result else 'No'}")
        return result

    def lost_ball_distance(self):
        """
        检查是否丢球
        :return: True 表示丢球，False 表示未丢球
        """
        # neck_angle = self.agent.get_ball_angle()
        # ball_x = self.agent.get_ball_pos()[0] - self.camera_bias
        ball_distance = self.agent.get_ball_distance()
        # ball_x_lost = abs(ball_x) > self.lost_ball_x_threshold_m  # 0.08
        ball_distance_lost = ball_distance > self.lost_ball_distance_threshold_m
        # yaw_angle = self.agent.get_self_yaw()
        # yaw_angle_lost = abs(yaw_angle) > 30 * math.pi / 180
        result = ball_distance_lost
        # if yaw_angle_lost:
        #     print(f"[DRIBBLE FSM] Yaw angle lost: {yaw_angle:.2f} rad")

        self.logger.debug(f"[DRIBBLE FSM] Ball distance: {ball_distance:.2f} m")
        self.logger.debug(f"[DRIBBLE FSM] Lost ball distance: {'Yes' if result else 'No'}")
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
        self.logger.debug(f"[DRIBBLE FSM] Lost ball yaw: {'Yes' if result else 'No'}")
        return result

    def lost_ball_x(self):
        """
        检查是否丢球
        :return: True 表示丢球，False 表示未丢球
        """
        # neck_angle = self.agent.get_ball_angle()
        ball_x = self.agent.get_ball_pos()[0] - self.camera_bias
        ball_x_lost = abs(ball_x) > self.lost_ball_x_threshold_m  # 80
        result = ball_x_lost
        self.logger.debug(f"[DRIBBLE FSM] Lost ball x: {'Yes' if result else 'No'}")
        return result

    def not_lost_ball_x(self):
        return not self.lost_ball_x()

    def not_lost_ball_distance(self):
        """
        检查是否未丢球
        :return: True 表示未丢球，False 表示丢球
        """
        result = not self.lost_ball_distance()
        self.logger.debug(f"[DRIBBLE FSM] Not lost ball distance: {'Yes' if result else 'No'}")
        return result
    
    def not_lost_yaw(self):
        """
        检查是否未丢球
        :return: True 表示未丢球，False 表示丢球
        """
        result = not self.lost_yaw()
        self.logger.debug(f"[DRIBBLE FSM] Not lost yaw: {'Yes' if result else 'No'}")
        return result

    def calculate_angle(self):
        """
        计算角度
        """
        self.logger.debug("[DRIBBLE FSM] Calculating angles...")

        goal_y_coord = self._config.get("field_size", {}).get(self.agent.league, [9, 6])[0] / 2

        if self.agent.get_self_pos()[0] > 0:
            if self.agent.get_self_pos()[0] > self.goal_center_bias_m:
                angle_to_goal_rad = math.atan((self.agent.get_self_pos()[0] - self.goal_center_bias_m) / (goal_y_coord - self.agent.get_self_pos()[1]))
            else:
                angle_to_goal_rad = 0.0
        else:
            if self.agent.get_self_pos()[0] < -self.goal_center_bias_m:
                angle_to_goal_rad = math.atan((self.agent.get_self_pos()[0] + self.goal_center_bias_m) / (goal_y_coord - self.agent.get_self_pos()[1]))
            else:
                angle_to_goal_rad = 0.0

        if self.agent.get_ball_pos_in_map()[0] > 0:
            if self.agent.get_ball_pos_in_map()[0] > self.goal_center_bias_m:
                angle_ball_to_goal = math.atan((self.agent.get_ball_pos_in_map()[0] - self.goal_center_bias_m) / (goal_y_coord - self.agent.get_ball_pos_in_map()[1]))
            else:
                angle_ball_to_goal = 0.0
        else:
            if self.agent.get_ball_pos_in_map()[0] < -self.goal_center_bias_m:
                angle_ball_to_goal = math.atan((self.agent.get_ball_pos_in_map()[0] + self.goal_center_bias_m) / (goal_y_coord - self.agent.get_ball_pos_in_map()[1]))
            else:
                angle_ball_to_goal = 0.0

        self.angle_to_goal_rad = angle_to_goal_rad
        self.angle_ball_to_goal_rad = angle_ball_to_goal
        self.logger.debug("[DRIBBLE FSM] Angles calculated")


    def dribble_forward(self):
        """
        向前带球
        """
        self.logger.debug("[DRIBBLE FSM] Dribbling forward...")
        vel_x = self.adjust_pos_to_ball_vel_x
        ball_x = self.agent.get_ball_pos()[0] - self.camera_bias
        feet_center = (self.good_horizontal_position_to_ball_upper_threshold_m + self.good_horizontal_position_to_ball_lower_threshold_m) / 2
        if ball_x > 0:
            if ball_x > feet_center:
                vel_theta = -self.dribble_forward_vel_theta * 0.2
            else:
                vel_theta = self.dribble_forward_vel_theta * 0.1
        elif ball_x < 0:
            if ball_x < -feet_center:
                vel_theta = self.dribble_forward_vel_theta * 0.2
            else:
                vel_theta = -self.dribble_forward_vel_theta * 0.1
        else:
            vel_theta = 0.0

        vel_y = 0.0

        self.agent.cmd_vel(vel_x, vel_y, vel_theta)
        self.logger.debug("[DRIBBLE FSM] Dribbling forward done")


    def calc_angle_to_goal_degree(self):
        """
        计算朝球门的角度
        """
        goal_y_coord = self._config.get("field_size", {}).get(self.agent.league, [9, 6])[0] / 2
        
        if self.agent.get_self_pos()[0] > 0:
            if self.agent.get_self_pos()[0] > self.goal_center_bias_m:
                angle_to_goal_rad = math.atan((self.agent.get_self_pos()[0] - self.goal_center_bias_m) / (goal_y_coord - self.agent.get_self_pos()[1]))
            else:
                angle_to_goal_rad = 0.0
        else:
            if self.agent.get_self_pos()[0] < -self.goal_center_bias_m:
                angle_to_goal_rad = math.atan((self.agent.get_self_pos()[0] + self.goal_center_bias_m) / (goal_y_coord - self.agent.get_self_pos()[1]))
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
        self.goal_center_bias_m = self._config.get("dribble", {}).get("goal_center_bias_m", 0)
        
        self.dribble_stop_angle_threshold_rad = 0.2
        self.min_ball_distance_m = self._config.get("dribble", {}).get(
            "min_ball_distance_m", 0.35
        )
        self.max_ball_distance_m = self._config.get("dribble", {}).get(
            "max_ball_distance_m", 0.55
        )

        self.good_horizontal_position_to_ball_upper_threshold_m = self._config.get("dribble", {}).get(
            "good_horizontal_position_to_ball_threshold_m", 0.09
        )

        self.good_horizontal_position_to_ball_lower_threshold_m = self._config.get("dribble", {}).get(
            "good_horizontal_position_to_ball_threshold_m", 0.04
        )

        self.lost_angle_to_target_threshold_degree = self._config.get("dribble", {}).get(
            "lost_angle_to_target_threshold_degree", 20
        )

        self.lost_ball_x_threshold_m = self._config.get("dribble", {}).get(
            "lost_ball_x_threshold_m", 0.08
        )

        self.lost_ball_distance_threshold_m = self._config.get("dribble", {}).get(
            "lost_ball_distance_threshold_m", 0.6
        )
      
    
        self.adjust_pos_to_ball_vel_x = self._config.get("dribble", {}).get("adjust_pos_to_ball_vel_x", 0.1)
        self.backward_vel = self._config.get("dribble", {}).get("back_vel", 0.05)
        self.horizontal_adjust_vel_y = self._config.get("dribble", {}).get("horizontal_adjust_vel_y", 1.0)
        self.dribble_forward_vel_theta = self._config.get("dribble", {}).get("dribble_forward_vel_theta", 1)
        self.rotate_vel_theta = self._config.get("dribble", {}).get("rotate_vel_theta", 0.3)
        self.adjust_angle_to_goal_vel_theta = self._config.get("dribble", {}).get("adjust_angle_to_goal_vel_theta", 1.0)
        self.camera_bias = self._config.get("dribble", {}).get("camera_bias", 0.06)
        self.blind_dribble_thres_percent = self._config.get("dribble", {}).get("blind_dribble_thres_percent", 1.0)
        self.obstacle_avoidance = self._config.get("dribble", {}).get("obstacle_avoidance", True)
