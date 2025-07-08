import math
from math import inf
import time
from transitions import Machine
import numpy as np
import logging


class GoalkeeperStateMachine:
    def __init__(self, agent):
        """Initialize the goalkeeper state machine with an agent"""
        self.agent = agent
        self.logger = self.agent.get_logger().get_child("goalkeeper_fsm")

        logging.getLogger('transitions').setLevel(logging.WARNING)

        self._config = self.agent.get_config()
        self.read_params()

        # Define states and transitions
        self.states = ["goalkeep", "charge_out", "clearance", "save"]
        self.transitions = [
            {
                "trigger": "keepgoal",
                "source": ["goalkeep", "charge_out", "clearance", "save"],
                "dest": "save",
                "conditions": "goal_risk_high",
                "after": "save_the_ball",
            },
            {
                "trigger": "keepgoal",
                "source": ["goalkeep", "charge_out", "clearance", "save"],
                "dest": "goalkeep",
                "conditions": "ball_in_safe_area",
                "after": "keep_the_goal",
            },
            {
                "trigger": "keepgoal",
                "source": ["goalkeep", "charge_out", "clearance"],
                "dest": "charge_out",
                "conditions": ["not_close_to_ball", "ball_in_dangerous_area"],
                "after": "do_charge_out",
            },
            {
                "trigger": "keepgoal",
                "source": ["goalkeep", "clearance", "charge_out"],
                "dest": "clearance",
                "conditions": ["close_to_ball", "ball_in_dangerous_area"],
                "after": "do_clearance",
            }
        ]

        # Initialize state machine
        self.machine = Machine(
            model=self,
            states=self.states,
            initial="goalkeep",
            transitions=self.transitions,
        )
        self.logger.info(f"[GOALKEEPER FSM] Initialized. Starting state: {self.state}")

    def read_params(self):
        """从配置中读取参数"""
        goalkeeper_config = self._config.get("goalkeeper", {})
        self.logger.info(f"[GOALKEEPER FSM] Reading goalkeeper parameters: {goalkeeper_config}")
        self.close_to_ball_threshold = goalkeeper_config.get("close_to_ball_threshold", 0.45)
        self.close_to_our_goal_threshold = goalkeeper_config.get("close_to_our_goal_threshold", 1.0)
        self.rotate_vel_theta = goalkeeper_config.get("rotate_vel_theta", 1)
        self.walk_vel_x = goalkeeper_config.get("walk_vel_x", 1)
        self.walk_vel_y = goalkeeper_config.get("walk_vel_y", 1)
        self.no_saving_area_width_m = goalkeeper_config.get("no_saving_area_width_m", 0.5)
        self.ball_acceleration = goalkeeper_config.get("ball_acceleration", 0.6)
        self.unreachable_area_lower_bound_m = goalkeeper_config.get("unreachable_area_lower_bound_m", 1.5)
        self.early_time = goalkeeper_config.get("early_time", 0.5)

    def close_to_ball(self):
        """Check if the agent is close to the ball"""
        if self.agent.get_ball_distance() is None:
            return False
        distance_close = self.agent.get_ball_distance() < 1.0

        return distance_close

    def not_close_to_ball(self):
        """Check if the agent is NOT close to the ball"""
        return not self.close_to_ball()
    
    def ball_in_safe_area(self):
        """Check if the ball is in a safe area (not close to the goal)"""
        ball_pos_in_map = self.agent.get_ball_pos_in_map()
        if ball_pos_in_map is None:
            self.logger.warning("[GOALKEEPER FSM] Ball position in map is None, assuming safe area.")
            return True
        self.logger.info(f"[GOALKEEPER FSM] Ball position in map: {ball_pos_in_map}")
        safe_area_threshold = self._config.get("safe_area_threshold", -1) + 0.3
        safe_area = ball_pos_in_map[1] > safe_area_threshold
        return safe_area
    
    def ball_in_dangerous_area(self):
        """Check if the ball is in a dangerous area (close to the goal)"""
        ball_pos_in_map = self.agent.get_ball_pos_in_map()
        if ball_pos_in_map is None:
            self.logger.warning("[GOALKEEPER FSM] Ball position in map is None, assuming not dangerous area.")
            return False
        self.logger.info(f"[GOALKEEPER FSM] Ball position in map: {ball_pos_in_map}")
        safe_area_threshold = self._config.get("safe_area_threshold", -1)
        dangerous_area = ball_pos_in_map[1] <= safe_area_threshold
        return dangerous_area
    
    def keep_the_goal(self):
        """Return to the goal"""
        self.logger.info("[GOALKEEPER FSM] Keeping the goal...")

        angle_to_our_goal = self.agent.get_angle_to_our_goal()
        reversed_angle = self.agent.angle_normalize(angle_to_our_goal + math.pi)

        # Normalize angle difference to [-pi, pi]
        yaw = self.agent.get_self_yaw() / 180 * math.pi
        angle_diff = reversed_angle - yaw
        angle_diff = self.agent.angle_normalize(angle_diff)

        self.logger.info(f"[GOALKEEPER FSM] Angle to our goal: {angle_to_our_goal}, Reversed angle: {reversed_angle}, Yaw: {yaw}, Angle diff: {angle_diff}")

        if abs(angle_diff) > math.pi/6:
            theta = np.sign(angle_diff) * self.rotate_vel_theta * 0.5
        elif abs(angle_diff) > math.pi/8:
            theta = np.sign(angle_diff) * self.rotate_vel_theta * 0.3
        else:
            theta = 0

        y_vel = 0

        if self.agent.get_distance_to_our_goal() > self.close_to_our_goal_threshold:
            x_vel = -self.walk_vel_x * 0.7
        else:
            x_vel = 0
            if abs(yaw) > math.pi / 15:
                theta = -self.rotate_vel_theta if yaw > 0 else self.rotate_vel_theta
            else:
                theta = 0
                if abs(self.agent.get_self_pos()[0]) > 0.5:
                    y_vel = self.walk_vel_y if self.agent.get_self_pos()[0] > 0 else -self.walk_vel_y

        self.agent.cmd_vel(
            x_vel,
            y_vel,
            theta
        )

    def do_charge_out(self):
        """Charge out towards the ball"""
        self.logger.info("[GOALKEEPER FSM] Charging out towards the ball...")
        self.agent._state_machine_runners['chase_ball']()

    def do_clearance(self):
        """Perform a clearance action"""
        self.logger.info("[GOALKEEPER FSM] Performing clearance action...")
        
        yaw = self.agent.get_self_yaw()
        if yaw < math.pi / 2 and yaw > -math.pi / 2:
            yaw = 0
        elif yaw >= math.pi / 2:
            yaw = math.pi / 2
        elif yaw <= -math.pi / 2:
            yaw = -math.pi / 2

        yaw = 0

        self.agent._state_machine_runners['dribble'](aim_yaw=yaw)

    def run(self):
        """Main execution loop for the state machine"""
        self.agent.move_head(inf, inf)
        command = self.agent.get_command()["command"]
        self.logger.info(
            f"[GK FSM] agent.command: {command}, state: {self.state}"
        )

        self.chase_distance = self.agent.get_command().get("data", {}).get("chase_distance", 0.45)

        self.logger.info(f"\n[GK FSM] Current state: {self.state}")
        self.logger.info(f"[GK FSM] Triggering 'keepgoal' transition")
        self.machine.model.trigger("keepgoal")

    def stop_moving(self):
        """Stop the agent's movement"""
        self.logger.info("[GK FSM] Stopping movement...")
        self.agent.cmd_vel(0, 0, 0)
        self.logger.info("[GK FSM] Movement stopped.")

    def stop_moving_and_set_head(self):
        """Stop the agent's movement and set head position"""
        self.logger.info("[GK FSM] Stopping movement and setting head position...")
        self.agent.cmd_vel(0, 0, 0)
        self.logger.info("[GK FSM] Movement stopped and head position set.")

    def goal_risk_high(self):
        ball_pos = self.agent.get_ball_pos()
        ball_distance = self.agent.get_ball_distance()
        if ball_pos is None:
            self.logger.info("[GOALKEEPER FSM] Ball position is None, assuming no goal risk.")
            return False
        ball_vx, ball_vy, x_at_y0, t_delta = self.calculate_ball_velocity_and_prediction()

        if ball_vx is None or ball_vy is None:
            self.logger.info("[GOALKEEPER FSM] Insufficient data to calculate ball velocity and prediction.")
            return False
        
        if x_at_y0 is None or t_delta is None:
            self.logger.info("[GOALKEEPER FSM] no goal risk.")
            return False

        if abs(x_at_y0) < self.no_saving_area_width_m:
            self.logger.info("[GOALKEEPER FSM] Ball is outside saving area, assuming no goal risk.")
            return False

        if abs(x_at_y0) > self.unreachable_area_lower_bound_m:
            self.logger.info("[GOALKEEPER FSM] Ball is in unreachable area, assuming no goal risk.")
            return False

        if t_delta > self.early_time:
            self.logger.info("[GOALKEEPER FSM] Ball is too late, assuming no goal risk.")
            return False

        if abs(ball_pos[0]) < self.close_to_ball_threshold_x_m and abs(ball_pos[1]) < self.close_to_ball_threshold_y_m:
            self.logger.info("[GOALKEEPER FSM] Ball is close, assuming no goal risk.")
            return False

        return True

    def save_the_ball(self):
        """Save the ball from going into the goal"""
        self.logger.info("[GOALKEEPER FSM] Saving the ball...")

        ball_vx, ball_vy, x_at_y0, t_delta = self.calculate_ball_velocity_and_prediction()
        if x_at_y0 is None:
            self.logger.info("[GOALKEEPER FSM] No valid prediction for ball position, cannot save the ball.")
            pass
        else:
            self.logger.info(f"[GOALKEEPER FSM] Ball will reach y=0 at x={x_at_y0} in {t_delta:.2f} seconds.")
            if x_at_y0 < 0:
                self.agent.save_ball("1") # Save the left side
            else:
                self.agent.save_ball("2") # Save the right side

    def calculate_ball_velocity_and_prediction(self):
        """
        假设球做匀减速运动，计算速度并预测球在y=0时的坐标和时间
        
        Returns:
            tuple: (vx, vy, x_at_y0, t_delta)
                vx, vy: 球在x和y方向上的速度分量，单位为m/s
                x_at_y0: 球在y=0时的x坐标，单位为m
                t_delta: 从当前时间到球到达y=0位置所需的时间，单位为s
                如果历史数据不足或无法预测，返回(None, None, None, None)
        """
        current_time = time.time()
        recent_positions = []
        
        # 获取最近0.3秒内的所有球位置（已为相对坐标）
        for item in self.get_ball_history():
            if current_time - item['timestamp'] <= 0.3:
                recent_positions.append(item)
        
        # 至少需要两个点来计算速度
        if len(recent_positions) < 2:
            return (None, None, None, None)
        
        # 按时间排序
        recent_positions.sort(key=lambda x: x['timestamp'])
        
        # 提取时间和位置数据
        times = np.array([item['timestamp'] for item in recent_positions])
        positions = np.array([item['pos'][:2] for item in recent_positions])  # 只取x和y坐标
        
        # 计算时间差（相对于第一个点）
        t_rel = times - times[0]
        
        # 使用匀减速模型计算初始速度（假设加速度为self.ball_acceleration，方向与速度相反）
        # 位移公式：s = v0*t + 0.5*a*t² → v0 = (s - 0.5*a*t²) / t
        
        # 取最后一个点（当前状态）计算初始速度
        last_idx = len(recent_positions) - 1
        dt = t_rel[last_idx]
        
        if dt <= 0:
            return (None, None, None, None)
        
        # 计算位移
        dx = positions[last_idx, 0] - positions[0, 0]
        dy = positions[last_idx, 1] - positions[0, 1]
        
        # 计算初始速度分量（考虑加速度方向）
        vx0 = (dx - 0.5 * self.ball_acceleration * np.sign(dx) * dt**2) / dt
        vy0 = (dy - 0.5 * self.ball_acceleration * np.sign(dy) * dt**2) / dt
        
        # 当前位置（最新记录）
        current_pos = positions[last_idx]
        
        # 预测y=0时的时间（解二次方程：0 = y0 + vy0*t + 0.5*a*t²）
        a_y = -self.ball_acceleration * np.sign(vy0)  # 加速度方向与速度相反
        y0 = current_pos[1]
        
        # 判别式
        discriminant = vy0**2 - 4 * (0.5 * a_y) * y0
        
        if discriminant < 0:
            return (vx0, vy0, None, None)  # 无解（球不会到达y=0）
        
        sqrt_d = np.sqrt(discriminant)
        
        # 两个解
        t1 = (-vy0 + sqrt_d) / (2 * (0.5 * a_y))
        t2 = (-vy0 - sqrt_d) / (2 * (0.5 * a_y))
        
        # 选择正的解（未来时间）
        valid_solutions = [t for t in [t1, t2] if t > 0]
        
        if not valid_solutions:
            return (vx0, vy0, None, None)  # 没有未来解
        
        # 取最小的正解（最近的未来时间）
        t_delta = min(valid_solutions)
        
        # 预测x坐标（考虑匀减速）
        a_x = -self.ball_acceleration * np.sign(vx0)  # 加速度方向与速度相反
        x_at_y0 = current_pos[0] + vx0 * t_delta + 0.5 * a_x * t_delta**2
        
        return (vx0, vy0, x_at_y0, t_delta)
    
