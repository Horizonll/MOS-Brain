#!/usr/bin/env python3

import json
import threading
from transitions import Machine
from collections import defaultdict
import time
import numpy as np
import math
import logging
import asyncio
import os
import sys
from robot_server import RobotServer
from sub_statemachines import DefendBallStateMachine
from sub_statemachines import DribbleBallStateMachine
from sub_statemachines import ShootBallStateMachine
from sub_statemachines import AttackStateMachine

# Define command constants
COMMANDS = {
    "dribble": "dribble",
    "forward": "forward",
    "stop": "stop",
    "find_ball": "find_ball",
    "chase_ball": "chase_ball",
    "shoot": "kick",
    "go_back_to_field": "go_back_to_field",
}

# Define player state constants
PLAYER_STATES = {
    "close_to_ball": "close_to_ball",
}

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler("agent.log"),
        logging.StreamHandler()
    ]
)

class Agent:
    def __init__(self):
        # Initialize the agent
        # Initialize the logger if it doesn't exist
        if not hasattr(self, 'logger'):
            self.logger = logging.getLogger(self.__class__.__name__)

        # Get parameters TODO: Determine parameters
        self.read_config()

        self.read_params()

        # Create a mapping between roles and IDs
        self.roles_to_id = {
            "forward_1": 2,
            "forward_2": 4,
            "defender_1": 1,
            "defender_2": 3,
            "goalkeeper": 4,
        }

        if self.get_config().get("roles_to_id", None) is not None:
            self.roles_to_id = self.get_config().get("roles_to_id")
            self.logger.info(f"get roles_to_id from config: {self.roles_to_id}")

        # Initialize robot states
        self.robots_data = defaultdict(
            lambda: {
                "last_seen": None,
                "status": "disconnected",
                "data": {}
            }
        )

        for role, robot_id in self.roles_to_id.items():
            self.robots_data[robot_id] = {
                "last_seen": None,
                "status": "disconnected",
                "data": {}
            }

        # Start TCP listening
        self.robot_server = RobotServer(agent=self)

        def start_robot_server():
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            loop.run_until_complete(self.robot_server.run())

        self.server_thread = threading.Thread(
            target=start_robot_server,
            daemon=True
        )
        self.server_thread.start()

        self._ball_pos = 1
        self._state = None

        self.t_no_ball = 0
        self.exit_flag = False
        self.keyboard_thread = None

        self._init_state_machine()

    def _init_state_machine(self):
        """
        Initialize the state machine.
        """
        self.state_machine = StateMachine(self)
        self.defend_ball_state_machine = DefendBallStateMachine(self)
        self.dribble_ball_state_machine = DribbleBallStateMachine(self)
        self.shoot_ball_state_machine = ShootBallStateMachine(self)
        self.attack_state_machine = AttackStateMachine(self)

    def read_config(self):
        """
        Read the configuration file.
        """
        # Read the local JSON configuration file
        try:
            with open("/home/thmos/MOS-Brain/decider/server/config.json", "r") as f:
                self._config = json.load(f)

            # # Check if automatic player detection is enabled
            # self.auto_detect_players = config.get("auto_detect_players", False)

            # # Check if static IP is used
            # self.use_static_ip = config.get("use_static_ip", False)
        except FileNotFoundError:
            logging.error("Configuration file not found.")

    def get_config(self):
        """
        Get the configuration file.
        """
        return self._config
    
    def read_params(self):
        self.backfield_machine = self.get_config().get("backfield_machine", "run_attack")
        self.midfield_machine = self.get_config().get("midfield_machine", "run_attack")
        self.frontfield_machine = self.get_config().get("frontfield_machine", "run_attack")

    def update(self, robot_id, robots_data):
        """
        Update robot data.
        """
        self.robot_states[robot_id] = robots_data.get("data")

    # TODO: Implement this method
    def ball_pos_callback(self, msg):
        pass

    def get_players_positions_without_goalkeeper(self):
        """
        获取除守门员外所有球员的位置（x, y 坐标），位置不可用则返回 None
        
        Returns:
            dict: 角色到位置的映射，格式为 {role: (x, y) 或 None}
        """
        positions = {}
        goalkeeper_role = "goalkeeper"  # 守门员角色名
        
        # 遍历所有角色，排除守门员
        for role, robot_id in self.roles_to_id.items():
            if role == goalkeeper_role:
                continue
            
            robot_data = self.robots_data.get(robot_id, {})
            data = robot_data.get("data", {})  # 获取机器人状态数据
            
            # 提取 x 和 y 坐标（注意可能为 None）
            x = data.get("x")
            y = data.get("y")
            
            # 如果坐标存在则存入元组，否则存入 None
            positions[role] = (x, y) if (x is not None and y is not None) else (0, -5000)

            # 如果未连接，位置为(0, 0)
            if robot_data.get("status") != "connected":
                positions[role] = (0, -5000)
        
        return positions

    def get_players_distance_to_ball(self):
        """
        Get the distance between players and the ball. If the position information cannot be obtained, the distance is considered a large number.

        Returns:
            dict: The distance between players and the ball
        """
        players_distance = {}
        BIG_NUMBER = 1e6  # A large value used when the position is unavailable

        for robot_id, data in self.robots_data.items():

            player_pos = [data.get("data", {}).get('x'), data.get("data", {}).get('y')]
            ball_pos = [data.get("data", {}).get('ballx'), data.get("data", {}).get('bally')]

            if player_pos[0] is not None:
                distance = data.get("data", {}).get('ball_distance', BIG_NUMBER)

            # Check for NoneType
            if any(v is None for v in player_pos) or any(v is None for v in ball_pos):
                distance = BIG_NUMBER
            else:
                distance = np.linalg.norm(np.array(player_pos) - np.array(ball_pos))
                distance = data.get("data", {}).get('ball_distance', BIG_NUMBER)
            
            if data.get("status") != "connected":
                distance = BIG_NUMBER

            players_distance[robot_id] = distance

        return players_distance

    def get_players_distance_to_ball_without_goalkeeper(self):
        players_distance = {}
        BIG_NUMBER = 1e6  # A large value used when the position is unavailable

        for robot_id, robot_data in self.robots_data.items():
            if robot_id != self.roles_to_id["goalkeeper"]:
                data = robot_data.get('data')
                player_pos = [data.get('x'), data.get('y')]
                ball_pos = [data.get('ballx'), data.get('bally')]

                # Check for NoneType
                if any(v is None for v in player_pos) or any(v is None for v in ball_pos):
                    distance = BIG_NUMBER
                else:
                    distance = np.linalg.norm(np.array(player_pos) - np.array(ball_pos))
                    distance = data.get('ball_distance', BIG_NUMBER)

                if robot_data.get("status") != "connected":
                    distance = BIG_NUMBER

                players_distance[robot_id] = distance
            else:
                players_distance[robot_id] = BIG_NUMBER

        return players_distance

    def switch_players_role(self, player_role_1, player_role_2):
        """
        Switch the roles of two players.

        Args:
            player_role_1 (int): Role of player 1
            player_role_2 (int): Role of player 2
        """
        temp_roles_to_id = self.roles_to_id.copy()
        temp_roles_to_id[player_role_1], temp_roles_to_id[player_role_2] = self.roles_to_id[player_role_2], self.roles_to_id[player_role_1]
        self.roles_to_id = temp_roles_to_id

    def publish_command(self, player_id, cmd, data={}):
        """
        Publish a command (enhanced with logging).

        Args:
            player_id (int): Player ID
            cmd (str/dict): Command content
        """
        try:
            # Initialize the logger if it doesn't exist
            if not hasattr(self, 'logger'):
                self.logger = logging.getLogger(self.__class__.__name__)

            # Log the command sending details
            cmd = COMMANDS[cmd]

            cmd_str = cmd if isinstance(cmd, str) else cmd.get("name", str(cmd))
            self.logger.debug(f"[CMD] Sending command -> Player {player_id}: {cmd_str}")

            # Construct the command data
            start_time = time.time()
            cmd_data = {
                "command": cmd,
                "data": data,
                "send_time": start_time,
            }

            # Send the command asynchronously
            asyncio.run(self.robot_server.send_to_robot(player_id, cmd_data))

            # Log the performance metric
            latency = (time.time() - start_time) * 1000  # in milliseconds
            self.logger.debug(f"[CMD] Command sent successfully | Player {player_id} | Time taken: {latency:.2f}ms")

        except asyncio.TimeoutError:
            self.logger.error(f"[CMD] Command sending timed out | Player {player_id} | Command: {cmd_str}")
        except ConnectionError as e:
            self.logger.error(f"[CMD] Connection error | Player {player_id} | Error: {str(e)}", exc_info=True)
        except Exception as e:
            self.logger.critical(f"[CMD] Unknown error | Player {player_id} | Error type: {type(e).__name__}", exc_info=True)

    def initialize(self):
        """
        All robots enter the field.
        """
        # for role, id in self.roles_to_id.items():
        #     self.publish_command(id, COMMANDS.get("go_back_to_field"))
        pass

    def stop(self):
        """
        All robots stop running.
        """
        for role, id in self.roles_to_id.items():
            self.publish_command(id, "stop")

    def ball_in_backcourt(self):
        """
        Determine if the ball is in the backcourt.

        Returns:
            bool: True if the ball is in the backcourt, False otherwise

        TODO: Determine the condition for the ball to be in the backcourt
        """
        if self.ball_pos[1] < -1000:
            return True
        return False

    def ball_in_midcourt(self):
        """
        Determine if the ball is in the midcourt.

        Returns:
            bool: True if the ball is in the midcourt, False otherwise

        TODO: Determine the condition for the ball to be in the midcourt
        """
        if self.ball_pos[1] > -1000 and self.ball_pos[1] < 1000:
            return True
        return False

    def ball_in_frontcourt(self):
        """
        Determine if the ball is in the frontcourt.

        Returns:
            bool: True if the ball is in the frontcourt, False otherwise

        TODO: Determine the condition for the ball to be in the frontcourt
        """
        if self.ball_pos[1] > 1000:
            return True
        return False

    @property
    def ball_pos(self):
        """
        Get the ball position.

        Returns:
            tuple: The ball position (x, y)
        """
        # 计算所有有球机器人球的位置的平均值
        ball_x = 0
        ball_y = 0
        count = 0
        for robot_id, data in self.robots_data.items():
            if data["status"] == "connected" and data.get("data", {}).get("if_ball"):
                ball_x = data.get("data", {}).get("ballx", 0) + ball_x
                ball_y = data.get("data", {}).get("bally", 0) + ball_y
                count += 1
        if count > 0:
            ball_x /= count
            ball_y /= count
        else:
            ball_x = 0
            ball_y = 0
        return (ball_x, ball_y)

    @property
    def get_if_ball(self):
        """
        Determine if the ball is in control.
        只要有一个机器人有球，则认为有球，否则没有球

        Returns:
            bool: True if the ball is in control, False otherwise
        """
        for robot_id, data in self.robots_data.items():
            if data["status"] == "connected":
                ball_data = data.get("data", {}).get("if_ball")
                if ball_data:
                    return True
        return False

    def run_defend_ball(self):
        self.defend_ball_state_machine.run()
        self.logger.info("Defending the ball")

    def run_dribble_ball(self):
        self.dribble_ball_state_machine.run()
        self.logger.info("Dribbling the ball")

    def run_shoot_ball(self):
        self.shoot_ball_state_machine.run()
        self.logger.info("Shooting the ball")

    def run_attack(self):
        self.attack_state_machine.run()
        self.logger.info("Attacking")

    def run_old(self):
        while self.robots_data[1].get("status") == "disconnected":
            logging.info("Waiting for connection")
            time.sleep(1)
        # self.state_machine.thread.start()
        logging.info("Start running")
        self.state_machine.run_in_state1()

    def start_keyboard_listener(self):
        """启动跨平台键盘监听（不使用pynput）"""
        def keyboard_thread():
            while not self.exit_flag:
                # 跨平台按键读取
                if os.name == 'nt':  # Windows
                    key = self._win_get_key()
                else:               # Linux/macOS
                    key = self._unix_get_key()
                
                if key == ' ':
                    self.toggle_run()
                elif key == 'q':
                    self.stop_playing()
                    self.exit_flag = True

        self.keyboard_thread = threading.Thread(target=keyboard_thread, daemon=True)
        self.keyboard_thread.start()

    def _win_get_key(self):
        """Windows平台按键读取"""
        import msvcrt
        key = msvcrt.getch().decode('utf-8').lower()
        # 仅处理目标按键，其他忽略
        return key if key in (' ', 'q') else ''

    def _unix_get_key(self):
        """Unix平台按键读取"""
        import termios
        import tty
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)
            key = sys.stdin.read(1).lower()
            return key if key in (' ', 'q') else ''
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    def stop_keyboard_listener(self):
        """停止键盘监听（通过标志位自然终止）"""
        self.exit_flag = True
        if self.keyboard_thread and self.keyboard_thread.is_alive():
            self.keyboard_thread.join()

    def is_playing(self):
        """Check if the system is currently running"""
        return self.state == "defend" or self.state == "dribble" or self.state == "shoot"

    def toggle_run(self):
        """Toggle the running state (with robot online detection)"""
        if self.is_playing():
            self.stop_playing()
            logging.info("System paused (Press space to continue, Q to exit)")
        else:
            if self.has_connected_robots():
                if self.state == "stop":
                    self.set_initial()
                self.play()
                logging.info("System started (Press space to pause, Q to exit)")
            else:
                logging.error("Error: No available online robots, cannot start")

    def has_connected_robots(self):
        """Check if there is at least one robot online"""
        return any(
            data["status"] == "connected"
            for data in self.robots_data.values()
        )

    def run(self):
        """Main loop with keyboard control and state detection"""

        logging.info("System initialized (Press space to start, Q to exit)")
        try:
            self.start_keyboard_listener()
            while True:

                time.sleep(self.get_config().get("control_interval_s", 1))
                if self.exit_flag:
                    break

                if not self.is_playing():
                    continue

                if self.has_connected_robots():
                    self.play()  # Run the state machine logic

                elif not self.has_connected_robots():
                    self.stop_playing()
                    self.running = False
                    logging.warning("Warning: All robots are offline, system has been automatically stopped")

        except KeyboardInterrupt:
            logging.info("\nProgram interrupted by user")
        finally:
            self.stop_playing()
            self.stop_keyboard_listener()
            logging.info("System safely shut down")


class StateMachine:
    def __init__(self, model: Agent):
        self.model = model  # Store the model as an attribute
        self.states = [
            "initial",
            "stop",
            "defend",
            "dribble",
            "shoot",
        ]
        self.transitions = [
            # defend, dribble, shoot can only be transitioned from non-stop states
            {
                "trigger": "play",
                "source": ["dribble", "shoot", "initial", "defend"],
                "dest": "defend",
                "conditions": "ball_in_backcourt",
                "after": self.model.backfield_machine
            },
            {
                "trigger": "play",
                "source": ["defend", "shoot", "initial", "dribble"],
                "dest": "dribble",
                "conditions": "ball_in_midcourt",
                "after": self.model.midfield_machine
            },
            {
                "trigger": "play",
                "source": ["defend", "dribble", "initial", "shoot"],
                "dest": "shoot",
                "conditions": "ball_in_frontcourt",
                "after": self.model.frontfield_machine
            },
            # initial can only be transitioned from stop
            {
                "trigger": "set_initial",
                "source": "stop",
                "dest": "initial",
                "after": "initialize",
            },
            {
                "trigger": "stop_playing",
                "source": "*",
                "dest": "stop",
                "after": "stop",
            },
        ]
        self.machine = Machine(
            model=self.model,  # Use self.model here
            states=self.states,
            initial="stop",  # 修正初始状态为有效状态
            transitions=self.transitions,
        )
        # self.thread = threading.Thread(target=self.run_in_state1)


def main():
    agent = Agent()
    agent.run()


if __name__ == "__main__":
    main()
