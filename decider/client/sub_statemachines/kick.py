import math
from math import inf
import time
from transitions import Machine


class KickStateMachine:
    def __init__(self, agent):
        """Initialize the kick state machine with an agent"""
        self.agent = agent
        self.logger = self.agent.get_logger().get_child("kick_fsm")
        self._config = self.agent.get_config()
        self.read_params()  # 初始化参数
        
        # Define states and transitions
        self.states = ["angle_adjust", "horizontal_adjust", "back_forth_adjust", "finished"]
        self.transitions = [
            {
                "trigger": "adjust_position",
                "source": ["angle_adjust"],
                "dest": "angle_adjust",
                "conditions": "not_good_angle",
                "after": ["adjust_angle"]
            },
            {
                "trigger": "adjust_position",
                "source": ["horizontal_adjust", "back_forth_adjust"],
                "dest": "angle_adjust",
                "conditions": "really_not_good_angle",
                "after": ["stop", "adjust_angle"],
            },
            {
                "trigger": "adjust_position",
                "source": "angle_adjust",
                "dest": "back_forth_adjust",
                "conditions": "good_angle",
                "after": "stop"
            },
            {
                "trigger": "adjust_position",
                "source": "back_forth_adjust",
                "dest": "back_forth_adjust",
                "conditions": "not_good_back_forth",
                "after": "adjust_back_forth",
            },
            {
                "trigger": "adjust_position",
                "source": "back_forth_adjust",
                "dest": "horizontal_adjust",
                "conditions": "good_back_forth",
                "after": "stop"
            },
            {
                "trigger": "adjust_position",
                "source": ["horizontal_adjust"],
                "dest": "horizontal_adjust",
                "conditions": "not_good_position_horizontally",
                "after": "adjust_horizontally",
            },
            {
                "trigger": "adjust_position",
                "source": "horizontal_adjust",
                "dest": "finished",
                "conditions": "good_position_horizontally",
            },
            {
                "trigger": "adjust_position",
                "source": "finished",
                "dest": "angle_adjust",
                "conditions": "not_finished",
                "after": "stop",
            },
        ]

        # Initialize state machine
        self.machine = Machine(
            model=self,
            states=self.states,
            initial="angle_adjust",
            transitions=self.transitions,
        )
        self.logger.debug(f"[KICK FSM] Initialized. Starting state: {self.state}")

    def run(self):
        """Main execution loop for the state machine"""
        self.logger.debug("[KICK FSM] Starting kick sequence...")
        self.logger.debug(f"[KICK FSM] if_ball: {self.agent.get_if_ball()} state: {self.state}")
        self.agent.move_head(inf, inf)

        ######################
        field_length = self.agent.get_config().get("field_size", {}).get(self.agent.league, "kid")[0]
        if self.agent.get_self_pos()[1] > field_length/2:
            target_angle_rad = 0.0
        else:
            target_angle_rad = math.atan2(self.agent.get_self_pos()[0], field_length/2 - self.agent.get_self_pos()[1])
        self.ang_tar = math.degrees(self.agent.angle_normalize(target_angle_rad))
        self.ang_delta = self.agent.angle_normalize((self.ang_tar - self.agent.get_self_yaw()) / 180 * math.pi) * 180 / math.pi
        ######################

        if (self.agent.get_ball_distance() > self.lost_ball_distance_threshold_m) and (self.state != "finished"):
            self.logger.debug("[KICK FSM] Ball is too far. Stopping robot.")
            self.agent.stop()
            return

        if (
            self.state != "finished"
            and self.agent.get_if_ball()
        ):
            self.logger.debug(f"\n[KICK FSM] Current state: {self.state}")
            self.logger.debug(f"[KICK FSM] Triggering 'adjust_position' transition")
            self.adjust_position()  # Changed from trigger to direct call

        if (
            self.state == "finished"
        ):
            self.logger.debug("\n[KICK FSM] Positioning complete! Executing kick...")
            self.agent.cmd_vel(0, 0, 0)
            self.agent.move_head(0.0, 0.0)
            self.agent.kick()
            time.sleep(5)
            self.agent.move_head(inf, inf)
            self.logger.debug("[KICK FSM] Kick executed successfully!")
            self.adjust_position()

    def stop(self):
        """Stop the robot's movement"""
        self.logger.debug("[KICK FSM] Stopping robot...")
        self.agent.stop(1)
        self.logger.debug("[KICK FSM] Robot stopped.")

    def adjust_angle(self):
        """Adjust robot's angle relative to goal"""
        self.logger.debug("[ANGLE ADJUST] Starting angle adjustment...")

        self.logger.debug(
            f"[ANGLE ADJUST] Target angle: {self.ang_tar:.2f}°, Current yaw: {self.agent.get_self_yaw():.2f}°, Delta: {self.ang_delta:.2f}°"
        )

        ball_y_distance = self.agent.get_ball_pos()[1] if not self.agent.get_ball_pos()[1] is None else 0.5
        max_y_vel = self.agent.get_config().get("max_walk_vel_y")
        max_theta_vel = self.agent.get_config().get("max_walk_vel_theta")
        ratio = max_y_vel / max_theta_vel * self.moonwalk_vel_ratio

        if self.ang_delta > self.good_angle_threshold_degree:
            self.logger.debug(f"[ANGLE ADJUST] Rotating CCW (Δ={self.ang_delta:.2f}°)")
            self.agent.cmd_vel(0, -ball_y_distance * self.rotate_vel_theta / ratio, self.rotate_vel_theta)
        elif self.ang_delta < -self.good_angle_threshold_degree:
            self.logger.debug(f"[ANGLE ADJUST] Rotating CW (Δ={self.ang_delta:.2f}°)")
            self.agent.cmd_vel(0, ball_y_distance * self.rotate_vel_theta / ratio, -self.rotate_vel_theta)

    def good_angle(self):
        """Check if angle is within acceptable range"""

        result = abs(self.ang_delta) < self.good_angle_threshold_degree

        self.logger.debug(
            f"[ANGLE CHECK] Angle delta: {abs(self.ang_delta):.2f}° (OK? {'Yes' if result else 'No'})"
        )
        return result
    
    def really_not_good_angle(self):
        """Check if angle is outside large acceptable range"""

        result = abs(self.ang_delta) >= self.really_bad_angle_threshold_degree

        self.logger.debug(
            f"[ANGLE CHECK] Large angle delta: {abs(self.ang_delta):.2f}° (Bad? {'Yes' if result else 'No'})"
        )
        return result
    
    def not_good_angle(self):
        """Check if angle is not within acceptable range"""
        return not self.good_angle()
    
    def not_finished(self):
        """Check if positioning is not finished"""
        return not (self.good_angle() and self.good_back_forth())

    def adjust_horizontally(self):
        """Adjust left-right position relative to ball"""
        self.logger.debug("\n[LR ADJUST] Starting lateral adjustment...")
        ball_x = self.agent.get_ball_pos()[0]
        center = (self.horizontal_position_lower_threshold_m + self.horizontal_position_upper_threshold_m) / 2
        if abs(ball_x - center) < 0.2:
            y_vel = 0.5 * self.lateral_vel
        else:
            y_vel = self.lateral_vel

        if ball_x > center:
            self.logger.debug(f"[KICK LR ADJUST] Moving left (Ball X: {ball_x:.2f}mm)")
            self.agent.cmd_vel(0, - y_vel, 0)
        elif ball_x < center:
            self.logger.debug(f"[KICK LR ADJUST] Moving right (Ball X: {ball_x:.2f}mm)")
            self.agent.cmd_vel(0, y_vel, 0)

    def good_position_horizontally(self):
        """Check if left-right position is correct"""
        ball_x = self.agent.get_ball_pos()[0]
        result = self.horizontal_position_lower_threshold_m < ball_x < self.horizontal_position_upper_threshold_m

        self.logger.debug(
            f"[LR CHECK] Ball X offset: {ball_x:.2f}mm (OK? {'Yes' if result else 'No'})"
        )
        return result
    
    def not_good_position_horizontally(self):
        """Check if left-right position is not correct"""
        return not self.good_position_horizontally()
    
    def adjust_back_forth(self):
        """Adjust forward-backward position relative to ball"""
        self.logger.debug("\n[FB ADJUST] Starting forward adjustment...")
        ball_y = self.agent.get_ball_pos()[1]
        ball_x = self.agent.get_ball_pos()[0] - self.camera_bias

        horizontal_center = (self.horizontal_position_lower_threshold_m + self.horizontal_position_upper_threshold_m) / 2

        y_vel = 0.0
        # if ball_x > horizontal_center:
        #     y_vel = - self.lateral_vel
        # else:
        #     y_vel = self.lateral_vel

        if ball_y > self.max_kick_y_distance_m:
            self.logger.debug(f"[FB ADJUST] Moving forward (Distance: {ball_y:.2f}m)")
            self.agent.cmd_vel(self.forward_vel, y_vel, 0)
        elif ball_y < self.min_kick_y_distance_m:
            self.logger.debug(f"[FB ADJUST] Moving backward (Distance: {ball_y:.2f}m)")
            self.agent.cmd_vel(-self.back_vel, y_vel, 0)

    def good_back_forth(self):
        """Check if forward position is correct"""
        ball_y = self.agent.get_ball_pos()[1]
        result = (self.min_kick_y_distance_m < ball_y < self.max_kick_y_distance_m)

        self.logger.debug(
            f"[FB CHECK] Ball Distance: {ball_y:.2f}m (OK? {'Yes' if result else 'No'})"
        )
        return result
    
    def not_good_back_forth(self):
        """Check if forward position is not correct"""
        return not self.good_back_forth()

    def read_params(self):
        """读取配置参数"""
        dribble_config = self._config.get("kick", {})
        self.good_angle_threshold_degree = dribble_config.get("good_angle_threshold_degree", 10)
        self.really_bad_angle_threshold_degree = dribble_config.get("really_bad_angle_threshold_degree", 30)
        self.horizontal_position_upper_threshold_m = dribble_config.get("horizontal_position_upper_threshold_m", 0.3)
        self.horizontal_position_lower_threshold_m = dribble_config.get("horizontal_position_lower_threshold_m", 0.3)
        self.horizontal_adjust_threshold_rad = math.radians(dribble_config.get("horizontal_adjust_threshold_degree", 5))
        
        self.min_kick_y_distance_m = dribble_config.get("min_kick_y_distance_m", 0.12)
        self.max_kick_y_distance_m = dribble_config.get("max_kick_y_distance_m", 0.16)
        self.lost_ball_distance_threshold_m = dribble_config.get("lost_ball_distance_threshold_m", 0.8)
        
        self.forward_vel = dribble_config.get("forward_vel", 0.3)
        self.back_vel = dribble_config.get("back_vel", 0.3)
        self.lateral_vel = dribble_config.get("lateral_vel", 0.05)
        self.rotate_vel_theta = dribble_config.get("rotate_vel_theta", 0.3)
        self.horizontal_adjust_forward_vel = dribble_config.get("horizontal_adjust_forward_vel", 0.05)
        self.adjust_angle_vel_y = dribble_config.get("adjust_angle_vel_y", 0.05)
        self.blind_dribble_thres_percent = dribble_config.get("blind_dribble_thres_percent", 1.0)
        self.camera_bias = dribble_config.get("camera_bias", 0.035)


    # 配置文件（JSON格式）
    """
    "kick": {
        "good_angle_threshold_degree": 10,
        "really_bad_angle_threshold_degree": 30,
        "horizontal_position_threshold_mm": 30,
        "horizontal_adjust_threshold_degree": 5,
        "min_kick_y_distance_m": 0.3,
        "max_kick_y_distance_m": 0.35,
        "lost_ball_distance_threshold_m": 0.8,
        "forward_vel": 0.3,
        "lateral_vel": 0.05,
        "rotate_vel_theta": 0.3,
        "horizontal_adjust_forward_vel": 0.05,
        "adjust_angle_vel_y": 0.05
    }
    """
