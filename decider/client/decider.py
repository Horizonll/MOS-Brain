# decider.py
#
#   @description : The entry py file for decision-making on clients
#

import argparse
import os
import json
import math
from pathlib import Path
import time
import signal
import asyncio
import traceback
import numpy as np
import threading
import logging
import rclpy.logging
import sub_statemachines
import policy_statemachines
import sys
from typing import Optional, Dict, Any, Callable, List
from datetime import datetime

# # 确保日志目录存在
# log_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'log')
# if not os.path.exists(log_dir):
#     os.makedirs(log_dir)

# # 生成带时间戳的日志文件名
# timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
# log_filename = f"client_{timestamp}.log"
# log_filepath = os.path.join(log_dir, log_filename)

# # 配置日志输出到文件和控制台
# logging.basicConfig(
#     level=logging.DEBUG,
#     format='%(asctime)s - %(levelname)s - %(filename)s:%(funcName)s(): %(message)s',
#     handlers=[
#         logging.FileHandler(log_filepath),
#         logging.StreamHandler(sys.stdout)
#     ]
# )

# ROS 2
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# Submodules
import configuration
from angle import angle

# Interfaces with other components
import interfaces.action
import interfaces.vision

# Network
import network
from receiver import Receiver

class Agent(Node):
    """
    Main agent class responsible for decision-making and robot control.
    Manages state machines and handles communication with other components.
    """
    
    def __init__(self, args=None):
        """Initialize the Agent instance and set up ROS 2 node and components."""
        super().__init__('decider')
        self.get_logger().info("Initializing the Agent instance")

        # Parse command line arguments
        self.debug_mode = args.debug if args else False
        self.loop_rate = args.rate if args else 1.0
        
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
        self._action = interfaces.action.Action(self)
        self._vision = interfaces.vision.Vision(self)
        self.receiver = Receiver(
            team=self.get_config()["team_id"], 
            player=self.get_config()["id"]-1, 
            logger=self.get_logger().get_child("receiver")
        )
        self.receiver.start()
        self._robot_client = network.Network(self)
        self._robot_client.start_send_loop()
        self._robot_client.start_receive_loop()
        
        # Initialize state machines
        self.get_logger().info("Initializing sub-state machines")
        self._initialize_state_machines()
        
        self.get_logger().info(f"Agent instance initialization completed, sleeping for {self.start_wait_time}")
        
        self.decider_start_time = time.time()
        self.penalize_end_time = self._get_initial_time()

        # relocalization
        self.relocalize()
        
        # 创建定时器，替代ROS 1中的while循环
        timer_period = 1.0 / self.loop_rate  # 默认10Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        """定时器回调函数，替代ROS 1中的主循环"""
        try:
            self.get_logger().info(f"Command: {self._command}")
            if self.debug_mode:
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
        self.goalkeeper_state_machine = policy_statemachines.GoalkeeperStateMachine(self)

        self._state_machine_runners = {
            "chase_ball": self.chase_ball_state_machine.run,
            "find_ball": self.find_ball_state_machine.run,
            "kick": self.kick_state_machine.run,
            "go_back_to_field": self.go_back_to_field_state_machine.run,
            "dribble": self.dribble_state_machine.run,
            "stop": self.stop,
            "goalkeeper": self.goalkeeper_state_machine.run,
        }

    def run(self) -> None:
        """Main decision-making loop based on game state and commands."""
        try:
            state = self.receiver._game_state
            # penalized_time = self.receiver.penalized_time
            penalty = self.receiver._penalty
            self.get_logger().info(f"penalized = {penalty}")

            if state == "STATE_INITIAL":
                self.relocalize()
            
            # Handle penalized state
            if penalty == 34:
                self.get_logger().info(f"Stopping: Player is penalized for {penalty} seconds")
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
            traceback.print_exc()

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
        print("================server cmd = ", cmd)
        
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
            # self.get_logger().info(f"ball angle: {self.get_ball_angle()}")
            # self.get_logger().info(f"ball pos: {self.get_ball_pos()}")
            self.get_logger().info(f"self_pos: {self.get_self_pos()}")
            self.get_logger().info(f"self_yaw: {self.get_self_yaw()}")
            cmd = self._command["command"]
            # self.get_logger().info(f"Debug_mode: {cmd}")
            
            if cmd in self._state_machine_runners:
                self.get_logger().info(f"Running: {cmd} (server command)")
                self._state_machine_runners[cmd]()
                pass
            else:
                self.get_logger().error(f"Error: State machine {cmd} not found. Stopping.")
                self.stop()
                
        except Exception as e:
            #print traceback
            import traceback
            traceback.print_exc()

            self.get_logger().error(f"Error in decider debug run: {e}, traceback: {traceback.format_exc()}")  

    def read_params(self) -> None:
        """Read configuration parameters with default values."""
        self.offline_time = self._config.get("offline_time", 5)
        self.if_can_kick = self._config.get("if_can_kick", False)
        
        self.start_wait_time = self._config.get("start_wait_time", 3)

        self.league = self._config.get("league", "kid")
        self.dribble_to_kick = self._config.get("dribble_to_kick", {}).get(self.league, [-2.3, 2.3])
        self.kick_to_dribble = self._config.get("kick_to_dribble", {}).get(self.league, [-1.7, 1.7])
        
        self.start_walk_into_field_time = self._config.get("start_walk_into_field_time", 3)

    def relocalize(self):
        """Perform relocalization to reset vision data."""
        x = self._config.get("initial_pos", {}).get("x", 0.0)
        y = self._config.get("initial_pos", {}).get("y", 0.0)
        theta = self._config.get("initial_pos", {}).get("theta", 0.0)

        self.get_logger().info(f"Relocalizing to position: x={x}, y={y}, theta={theta}")
        
        # Reset vision data
        self._vision.relocal(x, y, theta)

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
        self._action.cmd_vel(0.0, 0.0, 0.0)
        time.sleep(sleep_time)

    def kick(self) -> None:
        """Execute the kicking action."""
        self._action.do_kick()
        self.get_logger().info("Executing the kicking action")

    # 归一化角度(-pi,pi)
    def angle_normalize(self, angle: float) -> float:
        """Normalize angle to the range (-pi, pi)."""
        self.get_logger().info("Normalizing angle")
        if angle is None:
            return None
        angle = angle % (2 * math.pi)
        if angle > math.pi:
            angle -= 2 * math.pi
        elif angle < -math.pi:
            angle += 2 * math.pi
        self.get_logger().debug(f"Normalized angle: {angle}")
        return angle
        

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
        
        if ball_pos is not None and ball_x is not None and ball_y is not None and self.get_if_ball():
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
        if not self.get_if_ball():
            self.get_logger().warning("Ball not detected, returning None")
            return None
        return self._vision.get_ball_pos_in_map()

    def get_if_ball(self) -> bool:
        """Return whether ball is detected."""
        return self._vision.get_if_ball()

    def get_if_close_to_ball(self) -> bool:
        """Return whether robot is close to the ball."""
        if self.get_if_ball():
            return self.get_ball_distance() < self._config.get("close_to_ball_threshold", 0.5)
        else:
            return False

    def get_ball_distance(self) -> float:
        """Return distance to ball or a large value if not detected."""
        if self.get_if_ball():
            return self._vision.ball_distance
        else:
            return 1e6
        
    def get_angle_to_our_goal(self) -> float:
        """Calculate angle to our goal based on self position."""
        self_pos = self.get_self_pos()
        goal_x = 0.0
        goal_y = - self._config["field_size"].get(self.league,[9,6])[0] / 2
        self.get_logger().info(f"Calculating angle to our goal: self_pos={self_pos}, goal_x={goal_x}, goal_y={goal_y}")

        if self_pos is not None:
            angle_rad = math.atan2(self_pos[0] - goal_x, goal_y - self_pos[1])
            return angle_rad
        else:
            return 0.0
        
    def get_distance_to_our_goal(self) -> float:
        """Calculate distance to our goal based on self position."""
        self_pos = self.get_self_pos()
        goal_x = 0.0
        goal_y = - self._config["field_size"].get(self.league, [9, 6])[0] / 2

        if self_pos is not None:
            distance = math.sqrt((goal_x - self_pos[0]) ** 2 + (goal_y - self_pos[1]) ** 2)
            return distance
        else:
            return float('inf')

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
    parser.add_argument('--debuglog', action='store_true',
                        help='Enable debug logging')
    parser.add_argument('--rate', type=float, default=10.0,
                        help='Loop rate in Hz (default: 10.0)')
    cmd_args = parser.parse_args()

    # 获取当前文件所在目录的上级目录
    current_dir = Path(__file__).resolve().parent
    parent_dir = current_dir.parent
    
    # 创建log目录（如果不存在）
    log_dir = parent_dir / 'log'
    os.makedirs(log_dir, exist_ok=True)
    
    # 生成带时间戳的日志文件名
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_filename = f"client_{timestamp}.log"
    log_filepath = log_dir / log_filename
    
    # 设置ROS2日志环境变量
    os.environ['RCUTILS_LOGGING_USE_STDOUT'] = '1'  # 同时输出到控制台
    os.environ['RCUTILS_LOGGING_AUTO_INIT'] = '0'   # 禁用自动初始化
    os.environ['RCUTILS_LOGGING_BUFFERED_STREAM'] = '0'  # 禁用缓冲，立即写入
    os.environ['ROS_LOG_DIR'] = str(log_filepath)  # 设置日志文件路径

    # 设置日志等级
    if cmd_args.debuglog:
        rclpy.logging.set_logger_level('decider', rclpy.logging.LoggingSeverity.DEBUG)
    
    # Initialize ROS 2
    rclpy.init(args=args)
    
    # Create and run agent
    agent = Agent(cmd_args)
    
    try:
        # Spin the node to execute callbacks
        rclpy.spin(agent)
    except KeyboardInterrupt:
        print("Program interrupted by the user")
        agent.stop()
    finally:
        # Cleanup
        agent.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
