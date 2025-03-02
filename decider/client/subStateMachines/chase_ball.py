# chase_ball.py
# State machine for robot chasing ball behavior

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
            f"[CHASE BALL FSM] Checking if close to ball. Result: {'Yes' if result else 'No'}"
        )
        return result

    def run(self):
        """Main execution loop for the state machine"""
        print("[CHASE BALL FSM] Starting ball chasing sequence...")
        print(
            f"[CHASE BALL FSM] agent.command: {self.agent.command}, agent.info: {self.agent.info}, state: {self.state}"
        )
        while (
            self.state != "arrived" and self.agent.command["command"] == self.agent.info
        ):
            print(f"\n[CHASE BALL FSM] Current state: {self.state}")
            print(f"[CHASE BALL FSM] Triggering 'chase_ball' transition")
            self.machine.model.trigger("chase_ball")

        if self.state == "arrived" and self.agent.command == self.agent.info:
            print("\n[CHASE BALL FSM] Arrived at the ball!")

    def move_to_ball(self, ang=0.25):
        """Move the agent towards the ball"""
        print("[CHASE BALL FSM] Starting to move towards the ball...")
        if abs(self.agent.cam_neck) > ang:
            print(
                f"[CHASE BALL FSM] agent.cam_neck ({self.agent.cam_neck}) > {ang}. Rotating..."
            )
            self.agent.speed_controller(
                0, 0, np.sign(self.agent.cam_neck) * configuration.walk_theta_vel
            )
        elif abs(self.agent.cam_neck) <= ang:
            print(
                f"[CHASE BALL FSM] agent.cam_neck ({self.agent.cam_neck}) <= {ang}. Moving forward..."
            )
            self.agent.speed_controller(
                configuration.walk_x_vel,
                0,
                2.5 * self.agent.cam_neck * configuration.walk_theta_vel,
            )
            time.sleep(0.1)
        print("[CHASE BALL FSM] Movement step completed.")
