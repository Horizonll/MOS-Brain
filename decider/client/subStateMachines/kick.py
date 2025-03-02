# kick.py
# State machine for robot kicking behavior

import config
import math
import time
from transitions import Machine


class KickStateMachine:
    def __init__(self, agent):
        """Initialize the kick state machine with an agent"""
        self.agent = agent

        # Define states and transitions
        self.states = ["angel", "lr", "fb", "finished"]
        self.transitions = [
            {
                "trigger": "adjust_position",
                "source": "angel",
                "dest": "lr",
                "conditions": "good_angel",
                "prepare": "adjust_angel",
            },
            {
                "trigger": "adjust_position",
                "source": "lr",
                "dest": "fb",
                "conditions": "good_lr",
                "prepare": "adjust_lr",
            },
            {
                "trigger": "adjust_position",
                "source": "fb",
                "dest": "finished",
                "conditions": "good_fb",
                "prepare": "adjust_fb",
            },
        ]

        # Initialize state machine
        self.machine = Machine(
            model=self,
            states=self.states,
            initial="angel",
            transitions=self.transitions,
        )
        print(f"[KICK FSM] Initialized. Starting state: {self.state}")

    def run(self):
        """Main execution loop for the state machine"""
        print("[KICK FSM] Starting kick sequence...")
        print(f"[KICK FSM] ifBall: {self.agent.ifBall} state: {self.state}")
        while self.state != "finished" and self.agent.ifBall and self.agent.command["command"] == self.agent.info:
            print(f"\n[KICK FSM] Current state: {self.state}")
            print(f"[KICK FSM] Triggering 'adjust_position' transition")
            self.adjust_position()  # Changed from trigger to direct call
            time.sleep(2)

        if self.state == "finished" and self.agent.info == self.agent.command["command"]:
            print("\n[KICK FSM] Positioning complete! Executing kick...")
            self.agent.speed_controller(0, 0, 0)
            self.agent.head_set(head=0.1, neck=0)
            time.sleep(1)
            self.agent.doKick()
            time.sleep(2)
            print("[KICK FSM] Kick executed successfully!")

    def adjust_angel(self):
        """Adjust robot's angle relative to goal"""
        print("[ANGLE ADJUST] Starting angle adjustment...")
        self.agent.head_set(0.05, 0)

        # Calculate target angle using ball position

        # 
        target_angle_rad = math.atan((self.agent.pos_x - 0) / (4500 - self.agent.pos_y))
        ang_tar = target_angle_rad * 180 / math.pi
        ang_delta = ang_tar - self.agent.pos_yaw

        print(f"[ANGLE ADJUST] Target angle: {ang_tar:.2f}°, Current yaw: {self.agent.pos_yaw:.2f}°, Delta: {ang_delta:.2f}°")

        if ang_delta > 10:
            print(f"[ANGLE ADJUST] Rotating CCW (Δ={ang_delta:.2f})")
            self.agent.speed_controller(0, -0.05, 0.3)
        elif ang_delta < -10:
            print(f"[ANGLE ADJUST] Rotating CW (Δ={ang_delta:.2f})")
            self.agent.speed_controller(0, 0.05, 0.3)

    def good_angel(self):
        """Check if angle is within acceptable range"""
        target_angle_rad = math.atan((self.agent.pos_x - 0) / (4500 - self.agent.pos_y))
        ang_tar = target_angle_rad * 180 / math.pi
        ang_delta = ang_tar - self.agent.pos_yaw
        result = abs(ang_delta) < 10

        print(f"[ANGLE CHECK] Angle delta: {abs(ang_delta):.2f}° (OK? {'Yes' if result else 'No'})")
        return result

    def adjust_lr(self):
        """Adjust left-right position relative to ball"""
        print("\n[LR ADJUST] Starting lateral adjustment...")
        self.agent.head_set(head=0.9, neck=0)
        self.agent.stop(1)

        no_ball_count = 0
        t0 = time.time()
        print("[LR ADJUST] Scanning for ball...")

        # while self.agent.loop() and (self.agent.ball_x < 600 or self.agent.ball_x == 0):
        while (self.agent.ball_x < 600 or self.agent.ball_x == 0):
            if time.time() - t0 > 10 or no_ball_count > 5:
                print("[LR ADJUST] Timeout or lost ball during adjustment!")
                return

            if not self.agent.ifBall:
                no_ball_count += 1
                print(f"[LR ADJUST] Lost ball ({no_ball_count}/5)")
                time.sleep(0.7)
                continue

            print(f"[LR ADJUST] Moving right (Current X: {self.agent.ball_x})")
            self.agent.speed_controller(0, 0.6 * config.walk_y_vel, 0)

        while self.agent.loop() and (self.agent.ball_x > 660 or self.agent.ball_x == 0):
            if time.time() - t0 > 10 or no_ball_count > 5:
                print("[LR ADJUST] Timeout or lost ball during adjustment!")
                return

            if not self.agent.ifBall:
                no_ball_count += 1
                print(f"[LR ADJUST] Lost ball ({no_ball_count}/5)")
                time.sleep(0.7)
                continue

            print(f"[LR ADJUST] Moving left (Current X: {self.agent.ball_x})")
            self.agent.speed_controller(0, -0.6 * config.walk_y_vel, 0)

        self.agent.stop(0.5)
        print("[LR ADJUST] Lateral adjustment completed")

    def good_lr(self):
        """Check if left-right position is correct"""
        result = 600 <= self.agent.ball_x <= 660
        print(f"[LR CHECK] Ball X: {self.agent.ball_x} (OK? {'Yes' if result else 'No'})")
        return result

    def adjust_fb(self):
        """Adjust forward-backward position relative to ball"""
        print("\n[FB ADJUST] Starting forward adjustment...")
        self.agent.stop(0.5)
        t0 = time.time()
        no_ball_count = 0

        while self.agent.loop() and (self.agent.ball_y < 420 or self.agent.ball_y == 0):
            if time.time() - t0 > 10 or no_ball_count > 5:
                print("[FB ADJUST] Timeout or lost ball during adjustment!")
                return

            if not self.agent.ifBall:
                no_ball_count += 1
                print(f"[FB ADJUST] Lost ball ({no_ball_count}/5)")
                time.sleep(0.7)
                continue

            print(f"[FB ADJUST] Moving forward (Current Y: {self.agent.ball_y})")
            self.agent.speed_controller(0.5 * config.walk_x_vel, 0, 0)

        self.agent.stop(0.5)
        print("[FB ADJUST] Forward adjustment completed")

    def good_fb(self):
        """Check if forward position is correct"""
        result = 420 <= self.agent.ball_y
        self.agent.ready_to_kick = result
        print(f"[FB CHECK] Ball Y: {self.agent.ball_y} (OK? {'Yes' if result else 'No'})")
        return result
