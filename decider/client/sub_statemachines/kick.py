# kick.py
# State machine for robot kicking behavior

import math
import time
from transitions import Machine


class KickStateMachine:
    def __init__(self, agent):
        """Initialize the kick state machine with an agent"""
        self.agent = agent
        self._config = self.agent.get_config()
        # Define states and transitions
        self.states = ["angle_adjust", "horizontal_adjust", "back_forth_adjust", "finished"]
        self.transitions = [
            {
                "trigger": "adjust_position",
                "source": "angle_adjust",
                "dest": "angle_adjust",
                "conditions": "not_good_angle",
                "after": "adjust_angle",
            },
            {
                "trigger": "adjust_position",
                "source": "angle_adjust",
                "dest": "horizontal_adjust",
                "conditions": "good_angle",
            },
            {
                "trigger": "adjust_position",
                "source": "horizontal_adjust",
                "dest": "horizontal_adjust",
                "conditions": "not_good_position_horizontally",
                "after": "adjust_horizontally",
            },
            {
                "trigger": "adjust_position",
                "source": "horizontal_adjust",
                "dest": "back_forth_adjust",
                "conditions": "good_position_horizontally",
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
        print(f"[KICK FSM] Initialized. Starting state: {self.state}")

    def run(self):
        """Main execution loop for the state machine"""
        print("[KICK FSM] Starting kick sequence...")
        print(f"[KICK FSM] ifBall: {self.agent.get_if_ball()} state: {self.state}")
        if (
            self.state != "finished"
            and self.agent.get_if_ball()
        ):
            print(f"\n[KICK FSM] Current state: {self.state}")
            print(f"[KICK FSM] Triggering 'adjust_position' transition")
            self.adjust_position()  # Changed from trigger to direct call

        if (
            self.state == "finished"
        ):
            print("\n[KICK FSM] Positioning complete! Executing kick...")
            self.agent.cmd_vel(0, 0, 0)
            # self.agent.head_set(head=0.1, neck=0)
            time.sleep(1)
            self.agent.kick()
            time.sleep(2)
            print("[KICK FSM] Kick executed successfully!")
            self.adjust_position()

    def stop(self):
        """Stop the robot's movement"""
        print("[KICK FSM] Stopping robot...")
        self.agent.stop()
        print("[KICK FSM] Robot stopped.")

    def adjust_angle(self):
        """Adjust robot's angle relative to goal"""
        print("[ANGLE ADJUST] Starting angle adjustment...")
        # self.agent.head_set(0.7, 0)

        # Calculate target angle using ball position

        #
        target_angle_rad = math.atan((self.agent.get_self_pos()[0] - 0) / (4500 - self.agent.get_self_pos()[1]))
        ang_tar = target_angle_rad * 180 / math.pi
        ang_delta = ang_tar - self.agent.get_self_yaw()

        print(
            f"[ANGLE ADJUST] Target angle: {ang_tar:.2f}°, Current yaw: {self.agent.get_self_yaw():.2f}°, Delta: {ang_delta:.2f}°"
        )

        if ang_delta > 10:
            print(f"[ANGLE ADJUST] Rotating CCW (Δ={ang_delta:.2f})")
            self.agent.cmd_vel(0, -0.05, 0.3)
        elif ang_delta < -10:
            print(f"[ANGLE ADJUST] Rotating CW (Δ={ang_delta:.2f})")
            self.agent.cmd_vel(0, 0.05, -0.3)

    def good_angle(self):
        """Check if angle is within acceptable range"""
        target_angle_rad = math.atan((self.agent.get_self_pos()[0] - 0) / (4500 - self.agent.get_self_pos()[1]))
        ang_tar = target_angle_rad * 180 / math.pi
        ang_delta = ang_tar - self.agent.get_self_yaw()
        result = abs(ang_delta) < 10

        print(
            f"[ANGLE CHECK] Angle delta: {abs(ang_delta):.2f}° (OK? {'Yes' if result else 'No'})"
        )
        return result
    
    def not_good_angle(self):
        """Check if angle is not within acceptable range"""
        return not self.good_angle()
    
    def not_finished(self):
        """Check if angle is not within acceptable range"""
        return (not self.good_angle()) or not self.good_back_forth()

    def adjust_horizontally(self):
        """Adjust left-right position relative to ball"""
        print("\n[LR ADJUST] Starting lateral adjustment...")
        # self.agent.head_set(head=0.9, neck=0)
        self.agent.stop(1)

        no_ball_count = 0
        t0 = time.time()

        while not self.good_position_horizontally():
            if time.time() - t0 > 10 or no_ball_count > 5:
                print("[LR ADJUST] Timeout or lost ball during adjustment!")
                return

            if not self.agent.get_if_ball():
                no_ball_count += 1
                print(f"[LR ADJUST] Lost ball ({no_ball_count}/5)")
                time.sleep(0.2)
                continue

            if self.agent.get_neck() > -0.05:
                print(f"[LR ADJUST] Moving left (Ball(Neck) Angle: {self.agent.get_neck()})")
                self.agent.cmd_vel(0, 0.6 * self._config.get("walk_vel_y", 0.05), 0)
            elif self.agent.get_neck() < -0.25:
                print(f"[LR ADJUST] Moving right (Ball(Neck) Angle: {self.agent.get_neck()})")
                self.agent.cmd_vel(0, - 0.6 * self._config.get("walk_vel_y", 0.05), 0)

            

        self.agent.stop()
        print("[LR ADJUST] Lateral adjustment completed")

    def good_position_horizontally(self):
        """Check if left-right position is correct"""
        result = -0.25 <= self.agent.get_neck() <= -0.05
        print(
            f"[LR CHECK] Ball(Neck) Angle: {self.agent.get_neck()} (OK? {'Yes' if result else 'No'})"
        )
        return result
    
    def not_good_position_horizontally(self):
        """Check if left-right position is not correct"""
        return not self.good_position_horizontally()
    
    # def adjust_horizontally(self):
    #     """Adjust left-right position relative to ball"""
    #     print("\n[LR ADJUST] Starting lateral adjustment...")
    #     # self.agent.head_set(head=0.9, neck=0)
    #     self.agent.stop(1)

    #     no_ball_count = 0
    #     t0 = time.time()
    #     print("[LR ADJUST] Scanning for ball...")

    #     # while self.agent.loop() and (self.agent.get_ball_pos_in_vis()[0] < 600 or self.agent.get_ball_pos_in_vis()[0] == 0):
    #     while self.agent.get_ball_pos_in_vis()[0] < 600 or self.agent.get_ball_pos_in_vis()[0] == 0:
    #         if time.time() - t0 > 10 or no_ball_count > 5:
    #             print("[LR ADJUST] Timeout or lost ball during adjustment!")
    #             return

    #         if not self.agent.get_if_ball():
    #             no_ball_count += 1
    #             print(f"[LR ADJUST] Lost ball ({no_ball_count}/5)")
    #             time.sleep(0.7)
    #             continue

    #         print(f"[LR ADJUST] Moving right (Current X: {self.agent.get_ball_pos_in_vis()[0]})")
    #         self.agent.cmd_vel(0, 0.6 * self._config.get("walk_vel_y", 0.05), 0)

    #     while self.agent.loop() and (self.agent.get_ball_pos_in_vis()[0] > 660 or self.agent.get_ball_pos_in_vis()[0] == 0):
    #         if time.time() - t0 > 10 or no_ball_count > 5:
    #             print("[LR ADJUST] Timeout or lost ball during adjustment!")
    #             return

    #         if not self.agent.get_if_ball():
    #             no_ball_count += 1
    #             print(f"[LR ADJUST] Lost ball ({no_ball_count}/5)")
    #             time.sleep(0.7)
    #             continue

    #         print(f"[LR ADJUST] Moving left (Current X: {self.agent.get_ball_pos_in_vis()[0]})")
    #         self.agent.cmd_vel(0, -0.6 * self._config.get("walk_vel_y", 0.05), 0)

    #     self.agent.stop(0.5)
    #     print("[LR ADJUST] Lateral adjustment completed")

    # def good_position_horizontally(self):
    #     """Check if left-right position is correct"""
    #     result = 600 <= self.agent.get_ball_pos_in_vis()[0] <= 660
    #     print(
    #         f"[LR CHECK] Ball X: {self.agent.get_ball_pos_in_vis()[0]} (OK? {'Yes' if result else 'No'})"
    #     )
    #     return result

    def adjust_back_forth(self):
        """Adjust forward-backward position relative to ball"""
        print("\n[FB ADJUST] Starting forward adjustment...")
        self.agent.stop(0.5)
        t0 = time.time()
        no_ball_count = 0

        while not self.good_back_forth():
            if time.time() - t0 > 10 or no_ball_count > 5:
                print("[FB ADJUST] Timeout or lost ball during adjustment!")
                return

            if not self.agent.get_if_ball():
                no_ball_count += 1
                print(f"[FB ADJUST] Lost ball ({no_ball_count}/5)")
                time.sleep(0.7)
                continue

            print(f"[FB ADJUST] Moving forward (Current Distance: {self.agent.get_ball_distance()})")
            self.agent.cmd_vel(0.5 * self._config.get("walk_vel_x", 0.3), 0, 0)

        self.agent.stop(0.5)
        print("[FB ADJUST] Forward adjustment completed")

    def good_back_forth(self):
        """Check if forward position is correct"""
        result = (self.agent.get_ball_distance() < 0.4)
        self.agent.ready_to_kick = result
        print(
            f"[FB CHECK] Ball Distance: {self.agent.get_ball_distance()} (OK? {'Yes' if result else 'No'})"
        )
        return result
    
    def not_good_back_forth(self):
        """Check if forward position is not correct"""
        return not self.good_back_forth()
