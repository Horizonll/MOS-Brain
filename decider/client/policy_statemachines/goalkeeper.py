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
                "source": ["goalkeep", "charge_out", "clearance"],
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
        self.logger.debug(f"[GOALKEEPER FSM] Initialized. Starting state: {self.state}")

    def read_params(self):
        """从配置中读取参数"""
        goalkeeper_config = self._config.get("goalkeeper", {})
        self.logger.debug(f"[GOALKEEPER FSM] Reading goalkeeper parameters: {goalkeeper_config}")
        self.close_to_ball_threshold = goalkeeper_config.get("close_to_ball_threshold", 0.45)
        self.close_to_our_goal_threshold = goalkeeper_config.get("close_to_our_goal_threshold", 1.0)
        self.rotate_vel_theta = goalkeeper_config.get("rotate_vel_theta", 1)
        self.walk_vel_x = goalkeeper_config.get("walk_vel_x", 1)
        self.walk_vel_y = goalkeeper_config.get("walk_vel_y", 1)
        self.no_saving_area_width_m = goalkeeper_config.get("no_saving_area_width_m", 0.5)
        self.ball_acceleration = goalkeeper_config.get("ball_acceleration", 0.6)
        self.unreachable_area_lower_bound_m = goalkeeper_config.get("unreachable_area_lower_bound_m", 1.5)
        self.early_time = goalkeeper_config.get("early_time", 0.5)
        self.close_to_ball_threshold_x_m = goalkeeper_config.get("close_to_ball_threshold_x_m", 0.5)
        self.close_to_ball_threshold_y_m = goalkeeper_config.get("close_to_ball_threshold_y_m", 0.5)
        self.unreachable_area_y_distance_m = goalkeeper_config.get("unreachable_area_y_distance_m", 3.0)
        self.self._ignore_ball_vel = goalkeeper_config.get("ignore_ball_vel", 3.0)

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
        self.logger.debug(f"[GOALKEEPER FSM] Ball position in map: {ball_pos_in_map}")
        safe_area_threshold = self._config.get("safe_area_threshold", -1) + 0.3
        safe_area = ball_pos_in_map[1] > safe_area_threshold
        return safe_area
    
    def ball_in_dangerous_area(self):
        """Check if the ball is in a dangerous area (close to the goal)"""
        ball_pos_in_map = self.agent.get_ball_pos_in_map()
        if ball_pos_in_map is None:
            self.logger.warning("[GOALKEEPER FSM] Ball position in map is None, assuming not dangerous area.")
            return False
        self.logger.debug(f"[GOALKEEPER FSM] Ball position in map: {ball_pos_in_map}")
        safe_area_threshold = self._config.get("safe_area_threshold", -1)
        dangerous_area = ball_pos_in_map[1] <= safe_area_threshold
        return dangerous_area
    
    def keep_the_goal(self):
        """Return to the goal"""
        self.logger.debug("[GOALKEEPER FSM] Keeping the goal...")

        angle_to_our_goal = self.agent.get_angle_to_our_goal()
        reversed_angle = self.agent.angle_normalize(angle_to_our_goal + math.pi)

        # Normalize angle difference to [-pi, pi]
        yaw = self.agent.get_self_yaw() / 180 * math.pi
        angle_diff = reversed_angle - yaw
        angle_diff = self.agent.angle_normalize(angle_diff)

        self.logger.debug(f"[GOALKEEPER FSM] Angle to our goal: {angle_to_our_goal}, Reversed angle: {reversed_angle}, Yaw: {yaw}, Angle diff: {angle_diff}")

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
        self.logger.debug("[GOALKEEPER FSM] Charging out towards the ball...")
        self.agent._state_machine_runners['chase_ball']()

    def do_clearance(self):
        """Perform a clearance action"""
        self.logger.debug("[GOALKEEPER FSM] Performing clearance action...")
        
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
        self.logger.debug(
            f"[GK FSM] agent.command: {command}, state: {self.state}"
        )

        self.chase_distance = self.agent.get_command().get("data", {}).get("chase_distance", 0.45)

        self.logger.debug(f"\n[GK FSM] Current state: {self.state}")
        self.logger.debug(f"[GK FSM] Triggering 'keepgoal' transition")
        self.machine.model.trigger("keepgoal")

    def stop_moving(self):
        """Stop the agent's movement"""
        self.logger.debug("[GK FSM] Stopping movement...")
        self.agent.cmd_vel(0, 0, 0)
        self.logger.debug("[GK FSM] Movement stopped.")

    def stop_moving_and_set_head(self):
        """Stop the agent's movement and set head position"""
        self.logger.debug("[GK FSM] Stopping movement and setting head position...")
        self.agent.cmd_vel(0, 0, 0)
        self.logger.debug("[GK FSM] Movement stopped and head position set.")

    def goal_risk_high(self):
        ball_pos = np.array([self.agent.get_ball_pos()[0], self.agent.get_ball_pos()[1]])
        ball_distance = self.agent.get_ball_distance()
        if_ball = self.agent.get_if_ball()
        if ball_pos is None or ball_pos[0] is None or ball_pos[1] is None:
            self.logger.debug("[GOALKEEPER FSM] Ball position is None, assuming no goal risk.")
            return False

        ball_pos = ball_pos
        ball_vx, ball_vy, x_at_y0, t_delta = self.calculate_ball_velocity_and_prediction()

        if not if_ball:
            self.logger.debug("[GOALKEEPER FSM] Not holding the ball, assuming no goal risk.")
            return False

        if ball_vx is None or ball_vy is None:
            self.logger.debug("[GOALKEEPER FSM] Insufficient data to calculate ball velocity and prediction.")
            return False
        
        if x_at_y0 is None or t_delta is None:
            self.logger.debug("[GOALKEEPER FSM] no goal risk.")
            return False

        if abs(x_at_y0) < self.no_saving_area_width_m:
            self.logger.debug("[GOALKEEPER FSM] Ball is outside saving area, assuming no goal risk.")
            return False

        if abs(x_at_y0) > self.unreachable_area_lower_bound_m:
            self.logger.debug("[GOALKEEPER FSM] Ball is in unreachable area, assuming no goal risk.")
            return False

        if t_delta > self.early_time:
            self.logger.debug("[GOALKEEPER FSM] Ball is too late, assuming no goal risk.")
            return False

        if abs(ball_pos[0]) < self.close_to_ball_threshold_x_m and abs(ball_pos[1]) < self.close_to_ball_threshold_y_m:
            self.logger.debug("[GOALKEEPER FSM] Ball is close, assuming no goal risk.")
            return False

        if ball_pos[1] > self.unreachable_area_y_distance_m:
            self.logger.debug("[GOALKEEPER FSM] Ball is too far, assuming no goal risk.")
            return False

        return True

    def save_the_ball(self):
        """Save the ball from going into the goal"""
        self.logger.debug("[GOALKEEPER FSM] Saving the ball...")

        ball_vx, ball_vy, x_at_y0, t_delta = self.calculate_ball_velocity_and_prediction()
        if x_at_y0 is None:
            self.logger.debug("[GOALKEEPER FSM] No valid prediction for ball position, cannot save the ball.")
            pass
        else:
            self.logger.debug(f"[GOALKEEPER FSM] Ball will reach y=0 at x={x_at_y0} in {t_delta:.2f} seconds.")
            if x_at_y0 < 0:
                self.agent.save_ball(1) # Save the left side
            else:
                self.agent.save_ball(2) # Save the right side

    def calculate_ball_velocity_and_prediction(self):
        """
        假设球做匀减速运动，使用线性拟合计算速度并预测球在y=0时的坐标和时间

        Returns:
            tuple: (vx, vy, x_at_y0, t_delta)
                vx, vy: 球在x和y方向上的速度分量，单位为m/s
                x_at_y0: 球在y=0时的x坐标，单位为m
                t_delta: 从当前时间到球到达y=0位置所需的时间，单位为s
                如果历史数据不足或无法预测，返回(None, None, None, None)
        """
        # 历史中最近的一帧设为当前帧
        current_time = self.agent.get_ball_history()[-1]['timestamp'] if self.agent.get_ball_history() else time.time()
        recent_positions = []

        self.logger.debug(f"current_time: {current_time}")

        # 获取最近0.8秒内的所有球位置（已为相对坐标）
        for item in self.agent.get_ball_history():
            self.logger.debug("===============================================")
            self.logger.debug(f"[GOALKEEPER FSM] Ball history item: {item['timestamp']} position{ item['pos']}")
            if current_time - item['timestamp'] <= 0.5:  # 限制在0.5秒内提高精度
                recent_positions.append(item)

        # 至少需要三个点来进行线性拟合（两个点只能计算平均速度）
        if len(recent_positions) < 3:
            return (None, None, None, None)

        # 按时间排序
        recent_positions.sort(key=lambda x: x['timestamp'])

        # 提取时间和位置数据
        times = np.array([item['timestamp'] for item in recent_positions])
        positions = np.array([item['pos'][:2] for item in recent_positions]) # 只取x和y坐标

        # 计算时间差（相对于第一个点）
        t_rel = times - times[0]

        # 使用线性拟合计算速度
        # 位移公式：s = v0*t + 0.5*a*t²
        # 对于短时间间隔，可以近似为：s ≈ v0*t (忽略加速度影响)
        # 因此可以通过线性回归拟合 s = v0*t 来估算初速度

        # 构建线性方程组 Ax = b
        A = np.vstack([t_rel, np.ones_like(t_rel)]).T  # 设计矩阵 [t, 1]

        try:
            # 拟合x方向速度
            vx, _ = np.linalg.lstsq(A, positions[:, 0], rcond=None)[0]
            # 拟合y方向速度
            vy, _ = np.linalg.lstsq(A, positions[:, 1], rcond=None)[0]
        except np.linalg.LinAlgError:
            return (None, None, None, None)

        # 忽略太大的速度（可能是异常值）
        if math.sqrt(vx**2 + vy**2) > self._ignore_ball_vel:  # 假设10m/s为异常速度
            self.logger.warning("[GOALKEEPER FSM] Detected abnormal ball velocity, returning None.")
            return (None, None, None, None)

        # 当前位置（最新记录）
        current_pos = positions[-1]

        # 预测y=0时的时间（解二次方程：0 = y0 + vy*t + 0.5*a*t²）
        # 加速度方向应与速度方向相反
        a_y = -self.ball_acceleration * np.sign(vy)
        y0 = current_pos[1]

        # 若加速度为0，避免除零
        if np.isclose(a_y, 0):
            if np.isclose(vy, 0):
                return (vx, vy, None, None)
            t_delta = -y0 / vy
            if t_delta < 0:
                return (vx, vy, None, None)
            # 匀速运动，无需判断停止
        else:
            # 判别式
            discriminant = vy**2 - 4 * (0.5 * a_y) * y0

            if discriminant < 0:
                return (vx, vy, None, None)  # 无解（球不会到达y=0）

            sqrt_d = np.sqrt(discriminant)

            # 两个解
            t1 = (-vy + sqrt_d) / (2 * (0.5 * a_y))
            t2 = (-vy - sqrt_d) / (2 * (0.5 * a_y))

            # 选择正的解（未来时间）
            valid_solutions = [t for t in [t1, t2] if t > 0]

            if not valid_solutions:
                return (vx, vy, None, None)  # 没有未来解

            # 取最小的正解（最近的未来时间）
            t_delta = min(valid_solutions)

            # 计算球在y方向上减速到停止所需时间
            t_stop = abs(vy) / abs(a_y) if a_y != 0 else float('inf')

            # 如果t_delta > t_stop，说明球停下前到不了y=0
            if t_delta > t_stop + 1e-6:  # 加上微小容差
                return (vx, vy, None, None)

        # 预测x坐标（考虑匀减速）
        a_x = -self.ball_acceleration * np.sign(vx)
        x_at_y0 = current_pos[0] + vx * t_delta + 0.5 * a_x * t_delta**2

        return (vx, vy, x_at_y0, t_delta)
    
