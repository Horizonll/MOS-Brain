import math
import time
from transitions import Machine
import numpy as np
import rospy


class GoalkeeperStateMachine:
    """
    守门员
    """

    def __init__(self, agent):
        """
        初始化守门员状态机
        :param agent: 代理对象
        """
        self.agent = agent
        self._config = self.agent.get_config()
        self.read_params()  # 新增参数读取方法

        self.states = ["pos_to_ball_adjust", "yaw_adjust", "horizontal_position_adjust"]
        self.transitions = [
            {
                "trigger": "goalkeeper",
                "source": "horizontal_position_adjust",
                "dest": "horizontal_position_adjust",
                "conditions": ["not_lost_ball", "not_lost_yaw"],
                "after": "stop_moving",
            },
            {
                "trigger": "goalkeeper",
                "source": "horizontal_position_adjust",
                "dest": "horizontal_position_adjust",
                "conditions": "lost_ball_x",
                "after": "adjust_horizontal_position",
            },
            {
                "trigger": "goalkeeper",
                "source": ["yaw_adjust"],
                "dest": "yaw_adjust",
                "conditions": "bad_yaw_angle",
                "after": "adjust_yaw_angle",
            },
            {
                "trigger": "goalkeeper",
                "source": ["yaw_adjust"],
                "dest": "horizontal_position_adjust",
                "conditions": "good_yaw_angle",
                "after": "adjust_horizontal_position",
            },
            {
                "trigger": "goalkeeper",
                "source": ["horizontal_position_adjust"],
                "dest": "horizontal_position_adjust",
                "conditions": "bad_horizontal_position",
                "after": "adjust_horizontal_position",
            },
        ]
        self.machine = Machine(
            model=self,
            states=self.states,
            initial="horizontal_position_adjust",
            transitions=self.transitions,
            after_state_change=self.print_state,
        )
        self.direction = True  # FIXME: True: right, False: left
        rospy.loginfo(f"[GOALKEEPER FSM] Initialized. Starting state: {self.state}")

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
        self.aim_yaw = self._config.get("goalkeeper", {}).get('aim_yaw', None)

        if self.aim_yaw is None: 
            self.aim_yaw = 180.0
            rospy.loginfo(f"[GOALKEEPER FSM] Calculated aim_yaw: {self.aim_yaw:.2f}°")

        self.machine.model.trigger("goalkeeper")
        rospy.loginfo("[GOALKEEPER FSM] Running...")
        rospy.loginfo(f"[GOALKEEPER FSM] Current state: {self.state}")

    def print_state(self):
        """
        打印当前状态
        """
        rospy.loginfo(f"[GOALKEEPER FSM] Current state: {self.state}")

    def stop_moving(self):
        """
        停止移动
        """
        rospy.loginfo("[GOALKEEPER FSM] Stopping...")
        self.agent.cmd_vel(0, 0, 0)
        time.sleep(0.4)
        rospy.loginfo("[GOALKEEPER FSM] Stopped")

    def near_ball_to_goal(self):
        """
        检查球相对球门的位置是否接近
        :return: True 表示位置接近，False 表示位置不接近
        """
        ball_y = self.agent.get_ball_pos()[1]
        result = self.min_ball_distance_m < ball_y < self.max_ball_distance_m
        rospy.loginfo(f"[DRIBBLE FSM] Near ball to goal: {'Yes' if result else 'No'}")
        return result
    
    def far_ball_to_goal(self):
        """
        检查球相对球门的位置是否不接近
        :return: True 表示位置不接近，False 表示位置接近
        """
        result = not self.near_pos_to_ball()
        rospy.loginfo(f"[DRIBBLE FSM] Far ball to goal: {'Yes' if result else 'No'}")
        return result
    
    def adjust_horizontal_position(self):
        """
        Adjust the robot's horizontal position to defend the ball
        调整机器人左右位置以阻挡球
        """
        rospy.loginfo("[GOALKEEPER FSM] Starting horizontal position adjustment...")

        # neck_angle = self.agent.get_ball_angle()
        ball_x = self.agent.get_ball_pos()[0]
        if ball_x > 0:
            rospy.loginfo(f"[GOALKEEPER FSM] Moving left (Ball x: {ball_x})")
            self.agent.cmd_vel(
                0,
                -self.horizontal_adjust_vel_y,
                0
            )
        elif ball_x < 0:
            rospy.loginfo(f"[GOALKEEPER FSM] Moving right (Ball x: {ball_x})")
            self.agent.cmd_vel(
                0,
                self.horizontal_adjust_vel_y,
                0
            )

    def adjust_ball_angle(self):
        """
        调整角度
        """
        rospy.loginfo("[GOALKEEPER FSM] Adjusting angle...")
        target_angle_rad = self.agent.get_ball_angle()

        rospy.loginfo(f"[GOALKEEPER FSM] Adjusting angle... Current angle: {target_angle_rad}")
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
        rospy.loginfo(f"[GOALKEEPER FSM] Bad angle to ball: {'Yes' if result else 'No'}")
        return result

    def good_angle_to_ball(self):
        """
        检查球相对机器人的角度是否合适
        :return: True 表示角度合适，False 表示角度不合适
        """
        target_angle_rad = self.agent.get_ball_angle()
        result = abs(target_angle_rad) < self.angle_to_ball_adjust_threshold_rad
        rospy.loginfo(f"[GOALKEEPER FSM] Good angle to ball: {'Yes' if result else 'No'}")
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
        rospy.loginfo("[GOALKEEPER FSM] Starting yaw angle adjustment...")

        # Calculate target yaw angle using the robot's position
        target_angle_deg = self.aim_yaw
        current_yaw = self.agent.get_self_yaw()
        yaw_delta = target_angle_deg - current_yaw

        rospy.loginfo(
            f"[GOALKEEPER FSM] Target yaw: {target_angle_deg:.2f}°, Current yaw: {current_yaw:.2f}°, Delta: {yaw_delta:.2f}°"
        )

        if yaw_delta > self.good_angle_to_goal_threshold_degree:
            rospy.loginfo(f"[GOALKEEPER FSM] Rotating CCW (Δ={yaw_delta:.2f}°)")
            self.agent.cmd_vel(0, -self.adjust_angle_to_goal_vel_y, self.adjust_angle_to_goal_vel_theta)
        elif yaw_delta < -self.good_angle_to_goal_threshold_degree:
            rospy.loginfo(f"[GOALKEEPER FSM] Rotating CW (Δ={yaw_delta:.2f}°)")
            self.agent.cmd_vel(0, self.adjust_angle_to_goal_vel_y, -self.adjust_angle_to_goal_vel_theta)
        else:
            rospy.loginfo("[GOALKEEPER FSM] Yaw angle is within acceptable range.")

    def good_horizontal_position(self):
        """
        检查是否处于好的位置
        :return: True 表示位置好，False 表示位置不好
        """
        ball_x = self.agent.get_ball_pos()[0]
        result = abs(ball_x) < self.good_horizontal_position_to_ball_threshold_mm
        rospy.loginfo(f"[GOALKEEPER FSM] Good position: {'Yes' if result else 'No'}")
        return result

    def bad_horizontal_position(self):
        """
        检查是否处于不好的位置
        :return: True 表示位置不好，False 表示位置好
        """
        ball_x = self.agent.get_ball_pos()[0]
        result = abs(ball_x) > self.good_horizontal_position_to_ball_threshold_mm
        rospy.loginfo(f"[GOALKEEPER FSM] Bad position: {'Yes' if result else 'No'}")
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

        rospy.loginfo(f"[GOALKEEPER FSM] Ball distance: {ball_distance:.2f} m")
        rospy.loginfo(f"[GOALKEEPER FSM] Lost ball: {'Yes' if result else 'No'}")
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
        rospy.loginfo(f"[GOALKEEPER FSM] Lost ball yaw: {'Yes' if result else 'No'}")
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
        rospy.loginfo(f"[GOALKEEPER FSM] Lost ball x: {'Yes' if result else 'No'}")
        return result

    def not_lost_ball(self):
        """
        检查是否未丢球
        :return: True 表示未丢球，False 表示丢球
        """
        result = not self.lost_ball() and not self.lost_ball_x()
        rospy.loginfo(f"[GOALKEEPER FSM] Not lost ball: {'Yes' if result else 'No'}")
        return result
    
    def not_lost_yaw(self):
        """
        检查是否未丢球
        :return: True 表示未丢球，False 表示丢球
        """
        result = not self.lost_yaw()
        rospy.loginfo(f"[GOALKEEPER FSM] Not lost yaw: {'Yes' if result else 'No'}")
        return result

    def calculate_angle(self):
        """
        计算角度
        """
        rospy.loginfo("[GOALKEEPER FSM] Calculating angles...")

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
        rospy.loginfo("[GOALKEEPER FSM] Angles calculated")

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

        self.angle_to_ball_adjust_threshold_rad = self._config.get("goalkeeper", {}).get(
            "angle_to_ball_adjust_threshold_degree", 15
        ) * math.pi / 180
        self.good_angle_to_goal_threshold_degree = self._config.get("goalkeeper", {}).get(
            "good_angle_to_goal_threshold_degree", 10
        )
        self.goal_center_bias_mm = self._config.get("goalkeeper", {}).get("goal_center_bias_mm", 0)
        
        self.dribble_stop_angle_threshold_rad = 0.2
        self.min_ball_distance_m = self._config.get("goalkeeper", {}).get(
            "min_ball_distance_m", 0.0
        )
        self.max_ball_distance_m = self._config.get("goalkeeper", {}).get(
            "max_ball_distance_m", 0.55
        )

        self.min_ball_distance_goal = self._config.get("goalkeeper", {}).get(
            "min_ball_distance_goal", 0.0
        )
        self.max_ball_distance_goal = self._config.get("goalkeeper", {}).get(
            "max_ball_distance_goal", 0.55
        )

        self.good_horizontal_position_to_ball_threshold_mm = self._config.get("goalkeeper", {}).get(
            "good_horizontal_position_to_ball_threshold_mm", 50
        )

        self.lost_angle_to_target_threshold_degree = self._config.get("goalkeeper", {}).get(
            "lost_angle_to_target_threshold_degree", 20
        )

        self.lost_ball_x_threshold_mm = self._config.get("goalkeeper", {}).get(
            "lost_ball_x_threshold_mm", 80
        )

        self.lost_ball_distance_threshold_m = self._config.get("goalkeeper", {}).get(
            "lost_ball_distance_threshold_m", 0.6
        )

        self.forward_vel = self._config.get("goalkeeper", {}).get("walk_vel_x", 0.1)
        self.backward_vel = self._config.get("goalkeeper", {}).get("back_vel", 0.05)
        self.horizontal_adjust_vel_y = self._config.get("goalkeeper", {}).get("horizontal_adjust_vel_y", 0.03)
        self.horizontal_adjust_vel_theta = self._config.get("goalkeeper", {}).get("horizontal_adjust_vel_theta", 0.2)
        self.rotate_vel_theta = self._config.get("goalkeeper", {}).get("rotate_vel_theta", 0.3)
        self.adjust_angle_to_goal_vel_y = self._config.get("goalkeeper", {}).get("adjust_angle_to_goal_vel_y", 0.08)
        self.adjust_angle_to_goal_vel_theta = self._config.get("goalkeeper", {}).get("adjust_angle_to_goal_vel_theta", 0.3)