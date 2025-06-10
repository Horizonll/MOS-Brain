# decider.py
#
#   @description : The entry py file for decision-making on clients
#

import argparse
import os
import json
import math
import time
import signal
import asyncio
import numpy as np
import threading
import logging
import sub_statemachines
import sys
from typing import Optional, Dict, Any, Callable, List

logging.basicConfig(
    level=logging.DEBUG,
    format='%(asctime)s - %(levelname)s - %(filename)s:%(funcName)s(): %(message)s'
)

# ROS 2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist

# Submodules
import configuration
from angle import angle

# Interfaces with other components
import interfaces.action
import interfaces.vision

# Network
from robot_client import RobotClient
from receiver import Receiver

class Agent(Node):
    """
    Main agent class responsible for decision-making and robot control.
    Manages state machines and handles communication with other components.
    """
    
    def __init__(self):
        """Initialize the Agent instance and set up ROS 2 node and components."""
        super().__init__('decider')
        self.get_logger().info("Initializing the Agent instance")
        
        # Flags and configuration
        self.if_local_test = False
        self.attack_method = "dribble"
        
        # Load configuration and initialize parameters
        self._config = configuration.load_config()
        self.id = self._config["id"]
        self.read_params()
        
        # Initialize command tracking
        self._last_command_time = self._get_initial_time()
        self._command = self._create_initial_command()
        self._lst_command = self._command
        
        # Initialize data storage
        self._robots_data = {}
        self._last_play_time = time.time()
        
        # Initialize interfaces
        self.get_logger().info("Registering interfaces")
        self._action = interfaces.action.Action(self._config)
        self._vision = interfaces.vision.Vision(self)
        self.receiver = Receiver(self.get_config()["team"], self.get_config()["id"]-1)
        self._robot_client = RobotClient(self)
        
        # Initialize state machines
        self.get_logger().info("Initializing sub-state machines")
        self._initialize_state_machines()
        
        self.get_logger().info(f"Agent instance initialization completed, sleeping for {self.start_wait_time}")
        
        self.decider_start_time = time.time()
        self.penalize_end_time = self._get_initial_time()
        
        # 创建定时器，替代ROS 1中的while循环
        timer_period = 1.0 / self._config.get("loop_rate", 10.0)  # 默认10Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        """定时器回调函数，替代ROS 1中的主循环"""
        try:
            if self._config.get("debug_mode", False):
                self.debug_run()
            else:
                self.run()
        except Exception as e:
            self.get_logger().error(f"Error in timer callback: {e}")

    def _get_initial_time(self) -> float:
        """Return initial timestamp set to 1970-01-01 00:00:00."""
        return time.mktime(time.strptime("1970-01-01 00:00:00", "%Y-%m-%d %H:%M:%S"))

    def _create_initial_command(self) -> Dict[str, Any]:
        """Create initial command structure."""
        return {
            "command": "stop",
            "data": {},
            "timestamp": time.time(),
        }

    def _initialize_state_machines(self) -> None:
        """Initialize all state machines and map commands to their runners."""
        self.chase_ball_state_machine = sub_statemachines.ChaseBallStateMachine(self)
        self.find_ball_state_machine = sub_statemachines.FindBallStateMachine(self)
        self.kick_state_machine = sub_statemachines.KickStateMachine(self)
        self.go_back_to_field_state_machine = sub_statemachines.GoBackToFieldStateMachine(self)
        self.dribble_state_machine = sub_statemachines.DribbleStateMachine(self)
        
        self._state_machine_runners = {
            "chase_ball": self.chase_ball_state_machine.run,
            "find_ball": self.find_ball_state_machine.run,
            "kick": self.kick_state_machine.run,
            "go_back_to_field": self.go_back_to_field_state_machine.run,
            "dribble": self.dribble_state_machine.run,
            "stop": self.stop,
        }

    def run(self) -> None:
        """Main decision-making loop based on game state and commands."""
        try:
            state = self.receiver.game_state
            penalized_time = self.receiver.penalized_time
            self.get_logger().info(f"penalized_time = {penalized_time}")
            
            # Handle penalized state
            if penalized_time > 0:
                self.get_logger().info(f"Stopping: Player is penalized for {penalized_time} seconds")
                self.stop()
                self.penalize_end_time = time.time()
                return
                
            # Handle game states other than playing
            if state != "STATE_PLAYING":
                self._last_play_time = time.time()
                
            if state in ['STATE_SET', 'STATE_FINISHED', 'STATE_INITIAL', None]:
                if state is None:
                    self.decider_start_time = time.time()
                self.get_logger().info(f"Stopping: Game state is {state}")
                self.stop()
                return
                
            # Handle ready state
            if state == 'STATE_READY':
                self.get_logger().info("Running: go_back_to_field (STATE_READY)")
                self._state_machine_runners['go_back_to_field']()
                return
                
            # Handle playing state with various conditions
            current_time = time.time()
            
            if current_time - self.decider_start_time < self.start_walk_into_field_time:
                self._state_machine_runners['go_back_to_field']()
            elif current_time - self.penalize_end_time < self.start_walk_into_field_time:
                self._state_machine_runners['go_back_to_field']()
            elif current_time - self._last_play_time < 10.0 and not self.receiver.kick_off:
                self._state_machine_runners['chase_ball']()
            elif current_time - self._last_command_time > self.offline_time:
                self._handle_offline_state()
            else:
                self._handle_server_command()
                
        except Exception as e:
            self.get_logger().error(f"Error in decider run: {e}")

    def _handle_offline_state(self) -> None:
        """Handle state when command is offline based on ball detection."""
        if not self.get_if_ball():
            self.get_logger().info("Running: find_ball (lost command, no ball)")
            self._state_machine_runners['find_ball']()
        elif self.get_ball_distance() > 0.6:
            self.get_logger().info("Running: chase_ball (lost command, far ball)")
            self._state_machine_runners['chase_ball']()
        else:
            self.get_logger().info("Running: dribble (lost command, close ball)")
            if self.if_can_kick:
                self._state_machine_runners['kick']()
            else:
                self._state_machine_runners['dribble']()

    def _handle_server_command(self) -> None:
        """Handle state based on server command and ball detection."""
        cmd = self._command["command"]
        
        if not self.get_if_ball() and cmd in ['chase_ball', 'kick', 'dribble']:
            self.get_logger().info(f"Running: find_ball (server cmd {cmd}, no ball)")
            self._state_machine_runners['find_ball']()
        elif cmd in self._state_machine_runners:
            self.get_logger().info(f"Running: {cmd} (server command)")
            
            if cmd == 'kick':
                self._handle_kick_command()
            else:
                self._state_machine_runners[cmd]()
        else:
            self.get_logger().error(f"Error: State machine {cmd} not found. Stopping.")
            self.stop()

    def _handle_kick_command(self) -> None:
        """Handle kick command with special logic for dribbling vs kicking."""
        self.get_logger().info(f"=======================> cmd = kick")
        self_pos_y = self.get_self_pos()[1]
        
        if self_pos_y < self.dribble_to_kick[0] or self_pos_y > self.dribble_to_kick[1]:
            self.get_logger().info(f"=======================> cmd = kick case 1")
            self.attack_method = "kick"
        if self_pos_y > self.kick_to_dribble[0] and self_pos_y < self.kick_to_dribble[1]:
            self.get_logger().info(f"=======================> cmd = kick case 2")
            self.attack_method = "dribble"
            
        if self.attack_method == "dribble" or not self.if_can_kick:
            self.get_logger().info(f"=======================> cmd = kick action 1")
            self._state_machine_runners['dribble']()
        else:
            self.get_logger().info(f"=======================> cmd = kick action 2")
            self._state_machine_runners["kick"]()

    def debug_run(self) -> None:
        """Debug mode execution loop."""
        try:
            cmd = self._command["command"]
            self.get_logger().info(f"Debug_mode: {cmd}")
            
            if cmd in self._state_machine_runners:
                self.get_logger().info(f"Running: {cmd} (server command)")
                self._state_machine_runners[cmd]()
            else:
                self.get_logger().error(f"Error: State machine {cmd} not found. Stopping.")
                self.stop()
                
        except Exception as e:
            self.get_logger().error(f"Error in decider run: {e}")
            self._state_machine_runners['find_ball']()        

    def read_params(self) -> None:
        """Read configuration parameters with default values."""
        self.offline_time = self._config.get("offline_time", 5)
        self.if_can_kick = self._config.get("if_can_kick", False)
        
        self.start_wait_time = self._config.get("start_wait_time", 3)
        self.dribble_to_kick = self._config.get("dribble_to_kick", [-2300, 2300])
        self.kick_to_dribble = self._config.get("kick_to_dribble", [-1700, 1700])
        
        self.start_walk_into_field_time = self._config.get("start_walk_into_field_time", 3)

    # Control interface methods
    def cmd_vel(self, vel_x: float, vel_y: float, vel_theta: float) -> None:
        """Set robot velocity with scaling from configuration."""
        vel_x *= self._config.get("max_walk_vel_x", 0.25)
        vel_y *= self._config.get("max_walk_vel_y", 0.1)
        vel_theta *= self._config.get("max_walk_vel_theta", 0.5)
        self._action.cmd_vel(vel_x, vel_y, vel_theta)
        self.get_logger().info(f"Setting the robot's speed: linear velocity x={vel_x}, y={vel_y}, angular velocity theta={vel_theta}")

    def look_at(self, args) -> None:
        """Set robot's head and neck to look at specified position."""
        self._vision.look_at(args)

    def move_head(self, pitch: float, yaw: float) -> None:
        """Set robot's head and neck angles."""
        self._action._move_head(pitch, yaw)
        self.get_logger().info(f"Setting the robot's head to {pitch} and neck to {yaw}")

    def stop(self, sleep_time: float = 0) -> None:
        """Stop the robot's movement and optionally sleep."""
        self._action.cmd_vel(0, 0, 0)
        time.sleep(sleep_time)

    def kick(self) -> None:
        """Execute the kicking action."""
        self._action.do_kick()
        self.get_logger().info("Executing the kicking action")

    # Data retrieval methods
    def get_robots_data(self) -> Dict:
        """Return data about all robots."""
        return self._robots_data

    def get_command(self) -> Dict:
        """Return current command."""
        return self._command

    def get_self_pos(self) -> np.ndarray:
        """Return self position in map coordinates."""
        return self._vision.self_pos

    def get_self_yaw(self) -> angle:
        """Return self orientation angle."""
        return self._vision.self_yaw

    def get_ball_pos(self) -> List[Optional[float]]:
        """Return ball position relative to robot or [None, None] if not found."""
        if self.get_if_ball():
            return self._vision.get_ball_pos()
        else:
            return [None, None]

    def get_ball_angle(self) -> Optional[float]:
        """Calculate and return ball angle relative to robot."""
        ball_pos = self.get_ball_pos()
        ball_x = ball_pos[0]
        ball_y = ball_pos[1]
        
        if ball_pos is not None and ball_x is not None and ball_y is not None:
            if ball_x == 0 and ball_y == 0:
                return None
            else:
                angle_rad = - math.atan2(ball_x, ball_y)
                return angle_rad
        else:
            return None

    def get_ball_pos_in_vis(self) -> np.ndarray:
        """Return ball position in vision coordinates."""
        return self._vision.get_ball_pos_in_vis()

    def get_ball_pos_in_map(self) -> np.ndarray:
        """Return ball position in global map coordinates."""
        return self._vision.get_ball_pos_in_map()

    def get_if_ball(self) -> bool:
        """Return whether ball is detected."""
        return self._vision.get_if_ball()

    def get_if_close_to_ball(self) -> bool:
        """Return whether robot is close to the ball."""
        if self.get_if_ball():
            return 0.1 <= self.get_ball_distance() <= 0.4
        else:
            return False

    def get_ball_distance(self) -> float:
        """Return distance to ball or a large value if not detected."""
        if self.get_if_ball():
            return self._vision.ball_distance
        else:
            return 1e6

    def get_neck(self) -> angle:
        """Return neck angle."""
        return self._vision.neck

    def get_head(self) -> angle:
        """Return head angle."""
        return self._vision.head

    def get_ball_pos_in_map_from_other_robots(self) -> Optional[np.ndarray]:
        """
        Calculate averaged ball position from connected robots with valid detections

        Returns:
            np.ndarray | None: Averaged ball position in map coordinates (x, y), 
            returns None if no valid data
        """
        self.get_logger().info("Calculating ball position in map from other robots")
        
        valid_positions = []  # Stores valid (x,y) coordinates
        
        # Iterate through all robot data
        for robot_id, robot_data in self.get_robots_data().items():
            self.get_logger().info(f"Robot ID: {robot_id}, Data: {robot_data}")
            # Convert robot id to int
            robot_id = int(robot_id)
            # Skip self and disconnected robots
            if robot_id == self.id or robot_data.get('status') != 'connected':
                continue
                
            # Check ball detection status
            if robot_data.get("data").get('if_ball', False):
                # Extract coordinates
                ballx = robot_data['data'].get('ballx')
                bally = robot_data['data'].get('bally')
                
                # Validate coordinate existence
                if None not in (ballx, bally):
                    try:
                        # Convert to float array
                        pos = np.array([float(ballx), float(bally)], dtype=np.float32)
                        valid_positions.append(pos)
                        logging.debug(f"Valid position from {robot_id}: {pos}")
                    except (TypeError, ValueError) as e:
                        logging.warning(f"Invalid coordinates from {robot_id}: {e}")
        
        # Calculate simple average
        if valid_positions:
            avg_pos = np.mean(valid_positions, axis=0)
            logging.debug(f"Fused position from {len(valid_positions)} robots: {avg_pos}")
            return avg_pos
            
        logging.info("No valid ball positions from peer robots")
        return None
    
    def get_ball_angle_from_other_robots(self) -> Optional[float]:
        """
        Calculate averaged ball angle from connected robots with valid detections

        Returns:
            float | None: Averaged ball angle in radians, returns None if no valid data
        """
        ball_pos_in_map = self.get_ball_pos_in_map_from_other_robots()
        logging.debug(f"Ball position in map from other robots: {ball_pos_in_map}")
        logging.debug(f"Self position: {self.get_self_pos()}")
        
        if ball_pos_in_map is not None:
            # Calculate angle in radians
            ball_pos_relative = ball_pos_in_map - np.array(self.get_self_pos())
            self.get_logger().info(f"Ball position relative to self: {ball_pos_relative}")
            angle_rad = math.atan2(ball_pos_relative[1], ball_pos_relative[0])
            self.get_logger().info(f"Ball angle in radians: {angle_rad}")
            angle_relative = angle_rad - (self.get_self_yaw() / 180 * np.pi) - np.pi / 2
            self.get_logger().info(f"Ball angle relative to self: {angle_relative}")
            
            # Normalize angle to [-pi, pi)
            angle_relative = (angle_relative + math.pi) % (2 * math.pi) - math.pi
            
            self.get_logger().info(f"Ball angle from other robots: {angle_relative}")
            
            return angle_relative
            
        return None

    def get_config(self) -> Dict:
        """Return agent configuration."""
        return self._config


def main(args=None):
    """Main entry point for the decider program."""
    print("Decider started")
    
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Decider program arguments')
    parser.add_argument('--debug', action='store_true',
                        help='Enable debug mode')
    parser.add_argument('--rate', type=float, default=10.0,
                        help='Loop rate in Hz (default: 10.0)')
    cmd_args = parser.parse_args()
    
    # Initialize ROS 2
    rclpy.init(args=args)
    
    # Create and run agent
    agent = Agent()
    
    try:
        # Spin the node to execute callbacks
        rclpy.spin(agent)
    except KeyboardInterrupt:
        print("Program interrupted by the user")
    finally:
        # Cleanup
        agent.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()