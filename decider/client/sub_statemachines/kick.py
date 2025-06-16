import math
import time
from transitions import Machine
import rospy


class KickStateMachine:
    def __init__(self, agent):
        """Initialize the kick state machine with an agent"""
        self.agent = agent
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
                "dest": "horizontal_adjust",
                "conditions": "good_angle",
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
                "dest": "back_forth_adjust",
                "conditions": "good_position_horizontally",
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
                "dest": "finished",
                "conditions": "good_back_forth",
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
        rospy.loginfo(f"[KICK FSM] Initialized. Starting state: {self.state}")

    def run(self):
        """Main execution loop for the state machine"""
        rospy.loginfo("[KICK FSM] Starting kick sequence...")
        rospy.loginfo(f"[KICK FSM] if_ball: {self.agent.get_if_ball()} state: {self.state}")
        self.agent.look_at([None, None])
        if (self.agent.get_ball_distance() > self.lost_ball_distance_threshold_m) and (self.state != "finished"):
            rospy.loginfo("[KICK FSM] Ball is too far. Stopping robot.")
            self.agent.stop()
            return

        if (
            self.state != "finished"
            and self.agent.get_if_ball()
        ):
            rospy.loginfo(f"\n[KICK FSM] Current state: {self.state}")
            rospy.loginfo(f"[KICK FSM] Triggering 'adjust_position' transition")
            self.adjust_position()  # Changed from trigger to direct call

        if (
            self.state == "finished"
        ):
            rospy.loginfo("\n[KICK FSM] Positioning complete! Executing kick...")
            self.agent.cmd_vel(0, 0, 0)
            self.agent.look_at([0.0, 0.0])
            time.sleep(2)
            self.agent.kick()
            time.sleep(2)
            self.agent.look_at([None, None])
            rospy.loginfo("[KICK FSM] Kick executed successfully!")
            self.adjust_position()

    def stop(self):
        """Stop the robot's movement"""
        rospy.loginfo("[KICK FSM] Stopping robot...")
        self.agent.stop(1)
        rospy.loginfo("[KICK FSM] Robot stopped.")

    def adjust_angle(self):
        """Adjust robot's angle relative to goal"""
        rospy.loginfo("[ANGLE ADJUST] Starting angle adjustment...")
        
        if self.agent.get_self_pos()[1] > 4000:
            target_angle_rad = 0.0
        else:
            target_angle_rad = math.atan2(self.agent.get_self_pos()[0], 5000 - self.agent.get_self_pos()[1])
        ang_tar = math.degrees(target_angle_rad)
        ang_delta = ang_tar - self.agent.get_self_yaw()

        rospy.loginfo(
            f"[ANGLE ADJUST] Target angle: {ang_tar:.2f}°, Current yaw: {self.agent.get_self_yaw():.2f}°, Delta: {ang_delta:.2f}°"
        )

        if ang_delta > self.good_angle_threshold_degree:
            rospy.loginfo(f"[ANGLE ADJUST] Rotating CCW (Δ={ang_delta:.2f}°)")
            self.agent.cmd_vel(0, -self.rotate_vel_y, self.rotate_vel_theta)
        elif ang_delta < -self.good_angle_threshold_degree:
            rospy.loginfo(f"[ANGLE ADJUST] Rotating CW (Δ={ang_delta:.2f}°)")
            self.agent.cmd_vel(0, self.rotate_vel_y, -self.rotate_vel_theta)

    def good_angle(self):
        """Check if angle is within acceptable range"""
        if self.agent.get_self_pos()[1] > 4000:
            target_angle_rad = 0.0
        else:
            target_angle_rad = math.atan2(self.agent.get_self_pos()[0], 5000 - self.agent.get_self_pos()[1])
        ang_tar = math.degrees(target_angle_rad)
        ang_delta = ang_tar - self.agent.get_self_yaw()
        result = abs(ang_delta) < self.good_angle_threshold_degree

        rospy.loginfo(
            f"[ANGLE CHECK] Angle delta: {abs(ang_delta):.2f}° (OK? {'Yes' if result else 'No'})"
        )
        return result
    
    def really_not_good_angle(self):
        """Check if angle is outside large acceptable range"""
        if self.agent.get_self_pos()[1] > 4000:
            target_angle_rad = 0.0
        else:
            target_angle_rad = math.atan2(self.agent.get_self_pos()[0], 5000 - self.agent.get_self_pos()[1])
        ang_tar = math.degrees(target_angle_rad)
        ang_delta = ang_tar - self.agent.get_self_yaw()
        result = abs(ang_delta) >= self.really_bad_angle_threshold_degree

        rospy.loginfo(
            f"[ANGLE CHECK] Large angle delta: {abs(ang_delta):.2f}° (Bad? {'Yes' if result else 'No'})"
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
        rospy.loginfo("\n[LR ADJUST] Starting lateral adjustment...")
        
        if not self.good_position_horizontally():
            ball_angle = self.agent.get_ball_angle()
            if ball_angle > self.horizontal_adjust_threshold_rad:
                rospy.loginfo(f"[LR ADJUST] Moving left (Ball Angle: {math.degrees(ball_angle):.2f}°)")
                self.agent.cmd_vel(0, self.horizontal_adjust_forward_vel, 0)
            elif ball_angle < -self.horizontal_adjust_threshold_rad:
                rospy.loginfo(f"[LR ADJUST] Moving right (Ball Angle: {math.degrees(ball_angle):.2f}°)")
                self.agent.cmd_vel(0, -self.horizontal_adjust_forward_vel, 0)

    def good_position_horizontally(self):
        """Check if left-right position is correct"""
        ball_x = self.agent.get_ball_pos()[0]
        result = abs(ball_x) < self.horizontal_position_threshold_mm

        rospy.loginfo(
            f"[LR CHECK] Ball X offset: {ball_x:.2f}mm (OK? {'Yes' if result else 'No'})"
        )
        return result
    
    def not_good_position_horizontally(self):
        """Check if left-right position is not correct"""
        return not self.good_position_horizontally()
    
    def adjust_back_forth(self):
        """Adjust forward-backward position relative to ball"""
        rospy.loginfo("\n[FB ADJUST] Starting forward adjustment...")
        ball_distance = self.agent.get_ball_distance()
        
        if ball_distance > self.max_kick_distance_m:
            rospy.loginfo(f"[FB ADJUST] Moving forward (Distance: {ball_distance:.2f}m)")
            self.agent.cmd_vel(self.forward_vel, 0, 0)
        elif ball_distance < self.min_kick_distance_m:
            rospy.loginfo(f"[FB ADJUST] Moving backward (Distance: {ball_distance:.2f}m)")
            self.agent.cmd_vel(-self.lateral_vel, 0, 0)

    def good_back_forth(self):
        """Check if forward position is correct"""
        ball_distance = self.agent.get_ball_distance()
        result = (self.min_kick_distance_m < ball_distance < self.max_kick_distance_m)

        rospy.loginfo(
            f"[FB CHECK] Ball Distance: {ball_distance:.2f}m (OK? {'Yes' if result else 'No'})"
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
        self.horizontal_position_threshold_mm = dribble_config.get("horizontal_position_threshold_mm", 30)
        self.horizontal_adjust_threshold_rad = math.radians(dribble_config.get("horizontal_adjust_threshold_degree", 5))
        
        self.min_kick_distance_m = dribble_config.get("min_kick_distance_m", 0.3)
        self.max_kick_distance_m = dribble_config.get("max_kick_distance_m", 0.35)
        self.lost_ball_distance_threshold_m = dribble_config.get("lost_ball_distance_threshold_m", 0.8)
        
        self.forward_vel = dribble_config.get("forward_vel", 0.3)
        self.lateral_vel = dribble_config.get("lateral_vel", 0.05)
        self.rotate_vel_theta = dribble_config.get("rotate_vel_theta", 0.3)
        self.horizontal_adjust_forward_vel = dribble_config.get("horizontal_adjust_forward_vel", 0.05)
        self.rotate_vel_y = dribble_config.get("rotate_vel_y", 0.05)


    # 配置文件（JSON格式）
    """
    "kick": {
        "good_angle_threshold_degree": 10,
        "really_bad_angle_threshold_degree": 30,
        "horizontal_position_threshold_mm": 30,
        "horizontal_adjust_threshold_degree": 5,
        "min_kick_distance_m": 0.3,
        "max_kick_distance_m": 0.35,
        "lost_ball_distance_threshold_m": 0.8,
        "forward_vel": 0.3,
        "lateral_vel": 0.05,
        "rotate_vel_theta": 0.3,
        "horizontal_adjust_forward_vel": 0.05,
        "rotate_vel_y": 0.05
    }
    """
