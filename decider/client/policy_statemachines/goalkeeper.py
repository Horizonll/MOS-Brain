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
                "source": ["goalkeep", "clearance", "charge_out"],
                "dest": "clearance",
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
        goalkeeper_config = self._config.get("goalkeeper", {})
        self.logger.info(f"[GOALKEEPER FSM] Reading goalkeeper parameters: {goalkeeper_config}")
        self.close_to_ball_threshold = goalkeeper_config.get("close_to_ball_threshold", 0.45)
        self.close_to_our_goal_threshold = goalkeeper_config.get("close_to_our_goal_threshold", 1.0)
        self.rotate_vel_theta = goalkeeper_config.get("rotate_vel_theta", 1)
        self.walk_vel_x = goalkeeper_config.get("walk_vel_x", 1)
        self.walk_vel_y = goalkeeper_config.get("walk_vel_y", 1)

    def close_to_ball(self):
        """Check if the agent is close to the ball"""
        if self.agent.get_ball_distance() is None:
            return False
        distance_close = self.agent.get_ball_distance() < 1.0

        return distance_close

    def not_close_to_ball(self):
        """Check if the agent is NOT close to the ball"""
        return not self.close_to_ball()
    
    def ball_in_safe_area(self):
        """Check if the ball is in a safe area (not close to the goal)"""
        ball_pos_in_map = self.agent.get_ball_pos_in_map()
        if ball_pos_in_map is None:
            self.logger.warning("[GOALKEEPER FSM] Ball position in map is None, assuming safe area.")
            return True
        self.logger.info(f"[GOALKEEPER FSM] Ball position in map: {ball_pos_in_map}")
        safe_area_threshold = self._config.get("safe_area_threshold", -1) + 0.3
        safe_area = ball_pos_in_map[1] > safe_area_threshold
        return safe_area
    
    def ball_in_dangerous_area(self):
        """Check if the ball is in a dangerous area (close to the goal)"""
        ball_pos_in_map = self.agent.get_ball_pos_in_map()
        if ball_pos_in_map is None:
            self.logger.warning("[GOALKEEPER FSM] Ball position in map is None, assuming not dangerous area.")
            return False
        self.logger.info(f"[GOALKEEPER FSM] Ball position in map: {ball_pos_in_map}")
        safe_area_threshold = self._config.get("safe_area_threshold", -1)
        dangerous_area = ball_pos_in_map[1] <= safe_area_threshold
        return dangerous_area
    
    def keep_the_goal(self):
        """Return to the goal"""
        self.logger.info("[GOALKEEPER FSM] Keeping the goal...")

        angle_to_our_goal = self.agent.get_angle_to_our_goal()
        reversed_angle = self.agent.angle_normalize(angle_to_our_goal + math.pi)

        # Normalize angle difference to [-pi, pi]
        yaw = self.agent.get_self_yaw() / 180 * math.pi
        angle_diff = reversed_angle - yaw
        angle_diff = self.agent.angle_normalize(angle_diff)

        self.logger.info(f"[GOALKEEPER FSM] Angle to our goal: {angle_to_our_goal}, Reversed angle: {reversed_angle}, Yaw: {yaw}, Angle diff: {angle_diff}")

        if abs(angle_diff) > math.pi/6:
            theta = np.sign(angle_diff) * self.rotate_vel_theta * 0.5
        elif abs(angle_diff) > math.pi/8:
            theta = np.sign(angle_diff) * self.rotate_vel_theta * 0.3
        else:
            theta = 0

        y_vel = 0

        if self.agent.get_distance_to_our_goal() > self.close_to_our_goal_threshold:
            x_vel = -self.walk_vel_x * 0.7
        else:
            x_vel = 0
            if abs(yaw) > math.pi / 15:
                theta = -self.rotate_vel_theta if yaw > 0 else self.rotate_vel_theta
            else:
                theta = 0
                if abs(self.agent.get_self_pos()[0]) > 0.5:
                    y_vel = self.walk_vel_y if self.agent.get_self_pos()[0] > 0 else -self.walk_vel_y

        self.agent.cmd_vel(
            x_vel,
            y_vel,
            theta
        )

    def do_charge_out(self):
        """Charge out towards the ball"""
        self.logger.info("[GOALKEEPER FSM] Charging out towards the ball...")
        self.agent._state_machine_runners['chase_ball']()

    def do_clearance(self):
        """Perform a clearance action"""
        self.logger.info("[GOALKEEPER FSM] Performing clearance action...")
        
        yaw = self.agent.get_self_yaw()
        if yaw < math.pi / 2 and yaw > -math.pi / 2:
            yaw = 0
        elif yaw >= math.pi / 2:
            yaw = math.pi / 2
        elif yaw <= -math.pi / 2:
            yaw = -math.pi / 2

        yaw = 0

        self.agent._state_machine_runners['dribble'](aim_yaw=yaw)

    def run(self):
        """Main execution loop for the state machine"""
        self.agent.move_head(inf, inf)
        command = self.agent.get_command()["command"]
        self.logger.info(
            f"[GK FSM] agent.command: {command}, state: {self.state}"
        )

        self.chase_distance = self.agent.get_command().get("data", {}).get("chase_distance", 0.45)

        self.logger.info(f"\n[GK FSM] Current state: {self.state}")
        self.logger.info(f"[GK FSM] Triggering 'keepgoal' transition")
        self.machine.model.trigger("keepgoal")

    def stop_moving(self):
        """Stop the agent's movement"""
        self.logger.info("[GK FSM] Stopping movement...")
        self.agent.cmd_vel(0, 0, 0)
        self.logger.info("[GK FSM] Movement stopped.")

    def stop_moving_and_set_head(self):
        """Stop the agent's movement and set head position"""
        self.logger.info("[GK FSM] Stopping movement and setting head position...")
        self.agent.cmd_vel(0, 0, 0)
        self.logger.info("[GK FSM] Movement stopped and head position set.")

    
