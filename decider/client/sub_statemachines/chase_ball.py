import math
import time
from transitions import Machine
import numpy as np


class ChaseBallStateMachine:
    def __init__(self, agent):
        """Initialize the chase ball state machine with an agent"""
        self.agent = agent
        self._config = self.agent.get_config()
        self.read_params()

        # Define states and transitions
        self.states = ["rotate", "forward", "arrived"]
        self.transitions = [
            {
                "trigger": "chase_ball",
                "source": ["rotate", "forward", "arrived"],
                "dest": "arrived",
                "conditions": "close_to_ball",
                "after": "stop_moving_and_set_head",
            },
            {
                "trigger": "chase_ball",
                "source": ["rotate", "forward", "arrived"],
                "dest": "rotate",
                "conditions": ["not_close_to_ball", "large_angle"],
                "after": "rotate_to_ball",
            },
            {
                "trigger": "chase_ball",
                "source": ["rotate", "forward", "arrived"],
                "dest": "forward",
                "conditions": ["not_close_to_ball", "small_angle"],
                "after": "move_forward_to_ball",
            },
        ]

        # Initialize state machine
        self.machine = Machine(
            model=self,
            states=self.states,
            initial="rotate",
            transitions=self.transitions,
        )
        print(f"[CHASE BALL FSM] Initialized. Starting state: {self.state}")

    def read_params(self):
        """从配置中读取参数"""
        chase_config = self._config.get("chase", {})
        self.close_angle_threshold_rad = chase_config.get("close_angle_threshold_rad", 0.1)
        self.walk_vel_x = chase_config.get("walk_vel_x", 0.3)
        self.walk_vel_theta = chase_config.get("walk_vel_theta", 0.3)
        self.default_chase_distance = chase_config.get("default_chase_distance", 0.45)

    def close_to_ball(self):
        """Check if the agent is close to the ball (距离+角度检查)"""
        if self.chase_distance is not None:
            distance_close = self.chase_distance > self.agent.get_ball_distance()
        else:
            distance_close = self.agent.get_if_close_to_ball()
        if self.agent.get_ball_angle() is None:
            return False
        angle_close = abs(self.agent.get_ball_angle()) < self.close_angle_threshold_rad
        result = distance_close and angle_close
        print(
            f"[CHASE BALL FSM] Close to ball? Distance: {distance_close}, Angle: {angle_close}, Result: {result}"
        )
        return result

    def not_close_to_ball(self):
        """Check if the agent is NOT close to the ball"""
        return not self.close_to_ball()

    def large_angle(self):
        """Check if the angle to the ball is large"""
        if self.agent.get_ball_angle() is None:
            return False
        target_angle_rad = self.agent.get_ball_angle()
        return abs(target_angle_rad) > self.close_angle_threshold_rad

    def small_angle(self):
        """Check if the angle to the ball is small"""
        if self.agent.get_ball_angle() is None:
            return False
        target_angle_rad = self.agent.get_ball_angle()
        return abs(target_angle_rad) < self.close_angle_threshold_rad * 0.5

    def run(self):
        """Main execution loop for the state machine"""
        self.agent.look_at([None, None])
        command = self.agent.get_command()["command"]
        print(
            f"[CHASE BALL FSM] agent.command: {command}, state: {self.state}"
        )
        # if no ball, then stop
        if not self.agent.get_if_ball():
            print("[CHASE BALL FSM] No ball in sight. Stopping.")
            self.stop_moving()
            return

        self.chase_distance = self.agent.get_command().get("data", {}).get("chase_distance", 0.45)

        print(f"\n[CHASE BALL FSM] Current state: {self.state}")
        print(f"[CHASE BALL FSM] Triggering 'chase_ball' transition")
        self.machine.model.trigger("chase_ball")

    def rotate_to_ball(self):
        """Rotate the agent towards the ball"""
        print("[CHASE BALL FSM] Starting to rotate towards the ball...")
        target_angle_rad = self.agent.get_ball_angle()
        if self.agent.get_if_ball() == False:
            print("noball,cant rotate")
            return
        self.agent.cmd_vel(
            0,
            0,
            np.sign(target_angle_rad) * self.walk_vel_theta
        )
        print("[CHASE BALL FSM] Rotation step completed.")

    def move_forward_to_ball(self):
        """Move the agent forward towards the ball"""
        print("[CHASE BALL FSM] Starting to move forward towards the ball...")
        self.agent.cmd_vel(
            self.walk_vel_x,
            0,
            0
        )
        print("[CHASE BALL FSM] Forward movement step completed.")

    def stop_moving(self):
        """Stop the agent's movement"""
        print("[CHASE BALL FSM] Stopping movement...")
        self.agent.cmd_vel(0, 0, 0)
        print("[CHASE BALL FSM] Movement stopped.")

    def stop_moving_and_set_head(self):
        """Stop the agent's movement and set head position"""
        print("[CHASE BALL FSM] Stopping movement and setting head position...")
        self.agent.cmd_vel(0, 0, 0)
        print("[CHASE BALL FSM] Movement stopped and head position set.")

    
