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
        self.states = ["goalkeep", "charge_out", "clearance"]
        self.transitions = [
            {
                "trigger": "keepgoal",
                "source": ["goalkeep", "charge_out", "clearance"],
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
                "source": ["goalkeep", "forward", "arrived"],
                "dest": "forward",
                "conditions": ["close_to_ball", "ball_in_dangerous_area"],
                "after": "do_clearance",
            },
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
        chase_config = self._config.get("goalkeeper", {})
        self.close_to_ball_threshold = chase_config.get("close_to_ball_threshold", 0.45)
        

    def close_to_ball(self):
        """Check if the agent is close to the ball (距离+角度检查)"""

        if self.agent.get_ball_angle() is None:
            return False
        angle_close = abs(self.agent.get_ball_angle()) < self.close_angle_threshold_rad
        result = distance_close and angle_close
        self.logger.info(
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
        self.logger.info(f"[CHASE BALL FSM] Large angle check: {abs(target_angle_rad) > self.close_angle_threshold_rad}")
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
        self.logger.info(
            f"[CHASE BALL FSM] agent.command: {command}, state: {self.state}"
        )
        # if no ball, then stop
        if not self.agent.get_if_ball():
            self.logger.warn("[CHASE BALL FSM] No ball in sight. Stopping.")
            self.stop_moving()
            return

        self.chase_distance = self.agent.get_command().get("data", {}).get("chase_distance", 0.45)

        self.logger.info(f"\n[CHASE BALL FSM] Current state: {self.state}")
        self.logger.info(f"[CHASE BALL FSM] Triggering 'chase_ball' transition")
        self.machine.model.trigger("chase_ball")

    def rotate_to_ball(self):
        """Rotate the agent towards the ball"""
        self.logger.info("[CHASE BALL FSM] Starting to rotate towards the ball...")
        target_angle_rad = self.agent.get_ball_angle()
        if self.agent.get_if_ball() == False:
            self.logger.warn("[CHASE BALL FSM] Noball,cant rotate")
            return
        self.agent.cmd_vel(
            0,
            0,
            np.sign(target_angle_rad) * self.walk_vel_theta
        )
        self.logger.info("[CHASE BALL FSM] Rotation step completed.")

    def move_forward_to_ball(self):
        """Move the agent forward towards the ball"""
        self.logger.info("[CHASE BALL FSM] Starting to move forward towards the ball...")
        self.agent.cmd_vel(
            self.walk_vel_x,
            0,
            self.agent.get_ball_angle()/np.pi * self.walk_vel_theta * 5
        )
        self.logger.info("[CHASE BALL FSM] Forward movement step completed.")

    def stop_moving(self):
        """Stop the agent's movement"""
        self.logger.info("[CHASE BALL FSM] Stopping movement...")
        self.agent.cmd_vel(0, 0, 0)
        self.logger.info("[CHASE BALL FSM] Movement stopped.")

    def stop_moving_and_set_head(self):
        """Stop the agent's movement and set head position"""
        self.logger.info("[CHASE BALL FSM] Stopping movement and setting head position...")
        self.agent.cmd_vel(0, 0, 0)
        self.logger.info("[CHASE BALL FSM] Movement stopped and head position set.")

    
