import math
from math import inf
import time
from transitions import Machine
import numpy as np
import logging


class ChaseBallStateMachine:
    def __init__(self, agent):
        """Initialize the chase ball state machine with an agent"""
        self.agent = agent
        self.logger = self.agent.get_logger().get_child("chase_ball_fsm")

        logging.getLogger('transitions').setLevel(logging.WARNING)

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
        self.logger.debug(f"[CHASE BALL FSM] Initialized. Starting state: {self.state}")

    def read_params(self):
        """从配置中读取参数"""
        chase_config = self._config.get("chase", {})
        self.close_angle_threshold_rad = chase_config.get("close_angle_threshold_rad", 0.1)
        self.walk_vel_x = chase_config.get("walk_vel_x", 0.3)
        self.walk_vel_theta = chase_config.get("walk_vel_theta", 0.3)
        self.default_chase_distance = self._config.get("close_to_ball_threshold", 0.5) - 0.05
        self.obstacle_avoidance = chase_config.get("obstacle_avoidance", True)
        self.face_to_ball_threshold_degree = chase_config.get("face_to_ball_threshold_degree", 20)

    def close_to_ball(self):
        """Check if the agent is close to the ball (距离+角度检查)"""
        distance_close = self.chase_distance > self.agent.get_ball_distance()
        if self.agent.get_ball_angle() is None:
            return False
        angle_close = abs(self.agent.get_ball_angle()) < self.close_angle_threshold_rad
        result = distance_close and angle_close
        self.logger.debug(
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
        self.logger.debug(f"[CHASE BALL FSM] Large angle check: {abs(target_angle_rad) > self.close_angle_threshold_rad}")
        return abs(target_angle_rad) > self.close_angle_threshold_rad

    def small_angle(self):
        """Check if the angle to the ball is small"""
        if self.agent.get_ball_angle() is None:
            return False
        target_angle_rad = self.agent.get_ball_angle()
        return abs(target_angle_rad) < self.close_angle_threshold_rad * 0.5

    def run(self):
        """Main execution loop for the state machine"""
        self.agent.move_head(inf, inf)
        command = self.agent.get_command()["command"]
        self.logger.debug(
            f"[CHASE BALL FSM] agent.command: {command}, state: {self.state}"
        )
        # if no ball, then stop
        if not self.agent.get_if_ball():
            self.logger.warn("[CHASE BALL FSM] No ball in sight. Stopping.")
            self.stop_moving()
            return

        self.chase_distance = self.agent.get_command().get("data", {}).get("chase_distance", self.default_chase_distance)

        self.logger.debug(f"\n[CHASE BALL FSM] Current state: {self.state}")
        self.logger.debug(f"[CHASE BALL FSM] Triggering 'chase_ball' transition")
        self.machine.model.trigger("chase_ball")

    def rotate_to_ball(self):
        """Rotate the agent towards the ball"""
        self.logger.debug("[CHASE BALL FSM] Starting to rotate towards the ball...")
        target_angle_rad = self.agent.get_ball_angle()
        if self.agent.get_if_ball() == False:
            self.logger.warn("[CHASE BALL FSM] Noball,cant rotate")
            return
        self.agent.cmd_vel(
            0,
            0,
            np.sign(target_angle_rad) * self.walk_vel_theta
        )
        self.logger.debug("[CHASE BALL FSM] Rotation step completed.")

    def move_forward_to_ball(self):
        """Move the agent forward towards the ball"""
        self.logger.debug("[CHASE BALL FSM] Starting to move forward towards the ball...")

        if self.obstacle_avoidance:
            # Check for obstacles and adjust y velocity accordingly
            self.logger.debug("[CHASE BALL FSM] Using obstacle avoidance...")
            x_vel, y_vel, theta_vel = self.agent.get_obstacle_avoidance_velocity()
            if x_vel > self.walk_vel_x or x_vel is None:
                x_vel = self.walk_vel_x
            if theta_vel is None:
                self.logger.warn("[CHASE BALL FSM] Obstacle avoidance failed, using default theta velocity.")
                theta_vel = 0.0
        else:
            # No obstacle avoidance, use default y velocity
            theta_vel = 0.0
            y_vel = 0.0
            x_vel = self.walk_vel_x

        self.agent.cmd_vel(
            x_vel,
            y_vel,
            self.agent.get_ball_angle()/np.pi * self.walk_vel_theta * 2 + theta_vel
        )
        self.logger.debug("[CHASE BALL FSM] Forward movement step completed.")

    def stop_moving(self):
        """Stop the agent's movement"""
        self.logger.debug("[CHASE BALL FSM] Stopping movement...")
        self.agent.cmd_vel(0, 0, 0)
        self.logger.debug("[CHASE BALL FSM] Movement stopped.")

    def stop_moving_and_set_head(self):
        """Stop the agent's movement and set head position"""
        target_angle_degree = self.agent.get_ball_angle() * 180 / np.pi if self.agent.get_ball_angle() is not None else 0
        self.logger.debug("[CHASE BALL FSM] Stopping movement and setting head position...")
        if abs(target_angle_degree) > self.face_to_ball_threshold_degree:
            self.agent.cmd_vel(0, 0, np.sign(target_angle_degree) * self.walk_vel_theta)
        else:
            self.agent.cmd_vel(0, 0, 0)
        self.logger.debug("[CHASE BALL FSM] Movement stopped and head position set.")

    
