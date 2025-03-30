# chase_ball.py
# State machine for robot chasing ball behavior

import math
import time
from transitions import Machine
import numpy as np
from configuration import configuration


class ChaseBallStateMachine:
    def __init__(self, agent):
        """Initialize the chase ball state machine with an agent"""
        self.agent = agent

        # Define states and transitions
        self.states = ["chase", "arrived"]
        self.transitions = [
            {
                "trigger": "chase_ball",
                "source": "chase",
                "dest": "arrived",
                "conditions": "close_to_ball",
                "prepare": "move_to_ball",
                "after": "stop_moving_and_set_head",
            },
            {
                "trigger": "chase_ball",
                "source": "arrived",
                "dest": "chase",
                "conditions": "not_close_to_ball",
                "after": "set_head_high",
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
        result = self.agent.close_to_ball()
        print(
            f"[CHASE BALL FSM] Checking if close to ball. Result: {'Yes' if result else 'No'} distance: {self.agent.ball_distance}"
        )
        return result
    
    def not_close_to_ball(self):
        """Check if the agent is close to the ball"""
        result = self.agent.close_to_ball()
        print(
            f"[CHASE BALL FSM] Checking if close to ball. Result: {'Yes' if result else 'No'} distance: {self.agent.ball_distance}"
        )
        return not result

    def run(self):
        """Main execution loop for the state machine"""
        print("[CHASE BALL FSM] Starting ball chasing sequence...")
        print(
            f"[CHASE BALL FSM] agent.command: {self.agent.command}, agent.info: {self.agent.info}, state: {self.state}"
        )
        self.machine.model.trigger("chase_ball")
        if (
            self.state != "arrived"
        ):
            # if no ball, then stop
            if not self.agent.ifBall:
                print("[CHASE BALL FSM] No ball in sight. Stopping.")
                self.agent.head_set(head=0.5, neck=0)
                self.stop_moving()
                return

            print(f"\n[CHASE BALL FSM] Current state: {self.state}")
            print(f"[CHASE BALL FSM] Triggering 'chase_ball' transition")
            

        if self.state == "arrived" and self.agent.command == self.agent.info:
            print("\n[CHASE BALL FSM] Arrived at the ball!")

    def move_to_ball(self, ang=0.25):
        """Move the agent towards the ball"""
        print("[CHASE BALL FSM] Starting to move towards the ball...")
        target_angle_rad = (
            -math.atan(
                (self.agent.ball_x_in_map - self.agent.pos_x)
                / (self.agent.ball_y_in_map - self.agent.pos_y)
            )
            - self.agent.pos_yaw * math.pi / 180
        ) * (self.agent.ball_distance)

        if self.agent.ball_distance < 1:
            self.agent.head_set(head=0.5, neck=0)
        else:
            self.agent.head_set(head=0, neck=0)

        if abs(target_angle_rad) > ang:
            print(
                f"[CHASE BALL FSM] target_angle_rad ({target_angle_rad}) > {ang}. ball_distance: {self.agent.ball_distance}. Rotating..."
            )
            self.agent.speed_controller(
                0, 0, np.sign(self.agent.ball_distance) * configuration.walk_theta_vel
            )
        elif abs(target_angle_rad) <= ang:
            print(
                f"[CHASE BALL FSM] target_angle_rad ({target_angle_rad}) <= {ang}. ball_distance: {self.agent.ball_distance} Moving forward..."
            )
            self.agent.speed_controller(
                configuration.walk_x_vel,
                0,
                # 2.5 * self.agent.cam_neck * configuration.walk_theta_vel,
                0,
            )
            time.sleep(0.1)
        time.sleep(0.5)
        print("[CHASE BALL FSM] Movement step completed.")

    def stop_moving(self):
        """Stop the agent's movement"""
        print("[CHASE BALL FSM] Stopping movement...")
        self.agent.speed_controller(0, 0, 0)
        print("[CHASE BALL FSM] Movement stopped.")

    def stop_moving_and_set_head(self):
        """Stop the agent's movement and set head position"""
        print("[CHASE BALL FSM] Stopping movement and setting head position...")
        self.agent.speed_controller(0, 0, 0)
        self.agent.head_set(head=0.9, neck=0)
        print("[CHASE BALL FSM] Movement stopped and head position set.")

    def set_head_high(self):
        print("[CHASE BALL FSM] setting head position high...")
        self.agent.head_set(head=0.2, neck=0)
        print("[CHASE BALL FSM] head position set.")


# find_ball.py
