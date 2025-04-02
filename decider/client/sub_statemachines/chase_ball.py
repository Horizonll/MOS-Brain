# chase_ball.py
# State machine for robot chasing ball behavior

import math
import time
from transitions import Machine
import numpy as np


class ChaseBallStateMachine:
    def __init__(self, agent):
        """Initialize the chase ball state machine with an agent"""
        self.agent = agent
        self._config = self.agent.get_config()

        # Define states and transitions
        self.states = ["chase", "arrived"]
        self.transitions = [
            {
                "trigger": "chase_ball",
                "source": ["chase", "arrived"],
                "dest": "arrived",
                "conditions": "close_to_ball",
                "after": "stop_moving_and_set_head",
            },
            {
                "trigger": "chase_ball",
                "source": ["chase", "arrived"],
                "dest": "chase",
                "conditions": "not_close_to_ball",
                "after": "move_to_ball",
            }
        ]

        # Initialize state machine
        self.machine = Machine(
            model=self,
            states=self.states,
            initial="chase",
            transitions=self.transitions,
        )
        print(f"[CHASE BALL FSM] Initialized. Starting state: {self.state}")

    def close_to_ball(self):
        """Check if the agent is close to the ball"""
        result = self.agent.get_if_close_to_ball()

        target_angle_rad = (
            self.agent.get_neck()
        ) * (self.agent.get_ball_distance())

        print(
            f"[CHASE BALL FSM] Checking if close to ball. Result: {'Yes' if result else 'No'} distance: {self.agent.get_ball_distance()}"
        )
        return result and (target_angle_rad < 0.1)
    
    def not_close_to_ball(self):
        """Check if the agent is close to the ball"""
        result = self.agent.get_if_close_to_ball()

        target_angle_rad = (
            self.agent.get_neck()
        ) * (self.agent.get_ball_distance())

        print(
            f"[CHASE BALL FSM] Checking if close to ball. Result: {'Yes' if result else 'No'} distance: {self.agent.get_ball_distance()}"
        )
        return not (result and (target_angle_rad < 0.1))

    def run(self):
        """Main execution loop for the state machine"""
        command = self.agent.get_command()["command"]
        print(
            f"[CHASE BALL FSM] agent.command: {command}, state: {self.state}"
        )
        # if no ball, then stop
        if not self.agent.get_if_ball():
            print("[CHASE BALL FSM] No ball in sight. Stopping.")
            # self.agent.head_set(head=0.5, neck=0)
            self.stop_moving()
            return

        print(f"\n[CHASE BALL FSM] Current state: {self.state}")
        print(f"[CHASE BALL FSM] Triggering 'chase_ball' transition")
        self.machine.model.trigger("chase_ball")

    def move_to_ball(self, ang=0.1):
        """Move the agent towards the ball"""
        print("[CHASE BALL FSM] Starting to move towards the ball...")
        ball_pos_in_map = self.agent.get_ball_pos_in_map()
        pos = self.agent.get_self_pos()
        yaw = self.agent.get_self_yaw()
        # target_angle_rad = (
        #     -math.atan(
        #         (ball_pos_in_map[0] - pos[0])
        #         / (ball_pos_in_map[1] - pos[1])
        #     )
        #     - yaw * math.pi / 180
        # ) * (self.agent.get_ball_distance()) * 1.5
        target_angle_rad = (
            self.agent.get_neck()
        ) * (self.agent.get_ball_distance())

        if abs(target_angle_rad) > ang:
            print(
                f"[CHASE BALL FSM] target_angle_rad ({target_angle_rad}) > {ang}. ball_distance: {self.agent.get_ball_distance()}. Rotating..."
            )
            self.agent.cmd_vel(
                0, 0, np.sign(target_angle_rad) * self._config.get("walk_vel_theta", 0.3)
            )
        elif abs(target_angle_rad) <= ang:
            print(
                f"[CHASE BALL FSM] target_angle_rad ({target_angle_rad}) <= {ang}. ball_distance: {self.agent.get_ball_distance()} Moving forward..."
            )
            self.agent.cmd_vel(
                self._config.get("walk_vel_x", 0.3),
                0,
                # 2.5 * self.agent.cam_neck * self._config.get("walk_vel_theta", 0.3),
                0,
            )
            time.sleep(0.1)
        time.sleep(0.5)
        print("[CHASE BALL FSM] Movement step completed.")

    def stop_moving(self):
        """Stop the agent's movement"""
        print("[CHASE BALL FSM] Stopping movement...")
        self.agent.cmd_vel(0, 0, 0)
        print("[CHASE BALL FSM] Movement stopped.")

    def stop_moving_and_set_head(self):
        """Stop the agent's movement and set head position"""
        print("[CHASE BALL FSM] Stopping movement and setting head position...")
        self.agent.cmd_vel(0, 0, 0)
        # self.agent.head_set(head=0.9, neck=0)
        print("[CHASE BALL FSM] Movement stopped and head position set.")


# find_ball.py
