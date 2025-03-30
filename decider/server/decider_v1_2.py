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

from robot_server import RobotServer

COMMANDS = {
    "dribble": "dribble",
    "forward": "forward",
    "stop": "stop",
    "find_ball": "find_ball",
    "chase_ball": "chase_ball",
    "shoot": "kick",
    "go_back_to_field": "go_back_to_field",
}

PLAYER_STATES = {
    "close_to_ball": "close_to_ball",
}


class Agent:
    def __init__(self):

        # 初始化ROS节点
        # rospy.init_node("decider")

        # 获取参数 TODO: 确定参数

        # 创建角色与ID的映射
        self.roles_to_id = {
            "forward_1": 1,
            "forward_2": 2,
            "defender_1": 3,
            "goalkeeper": 4,
        }

        # 初始化机器人状态
        self.robots_data = defaultdict(lambda: {"last_seen": None, "status": "disconnected", "data": {}})

        for role, robot_id in self.roles_to_id.items():
            self.robots_data[robot_id] = {"last_seen": None, "status": "disconnected", "data": {}}

        # # 开启TCP监听
        # self.robot_server = RobotServer(agent=self)

        # self.server_thread = threading.Thread(target=asyncio.run(self.robot_server.run()), daemon=True)
        # self.server_thread.start()

        # 开启TCP监听
        self.robot_server = RobotServer(agent=self)

        def start_robot_server():
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            loop.run_until_complete(self.robot_server.run())

        self.server_thread = threading.Thread(target=start_robot_server, daemon=True)
        self.server_thread.start()

        # 等待所有机器人开机
        logging.info("Waiting for all robots to start...")
        count = 0
        while not all(data["status"] == "connected" for data in self.robots_data.values()):
            # self.robot_server.broadcast({"message": "thmos_hello", "ip": self.robot_server.ip})

            if self.robots_data[1].get("status") == "connected":
                print("Robot 1 connected")
                break

            time.sleep(1)
            count += 1
            if count > 30:
                logging.warning("Some robots are not connected")
                exit(1)
        print(self.robots_data)
        logging.info("All robots are connected")

        # 开启UDP监听

        # 订阅裁判盒消息

        # 获取可用球员

        # 根据可用球员更改角色对应ID
        # self.update_players_id()

        self._ball_pos = 1

        self._state = None

        self.ifBall = False
        self.ready_to_kick = False
        self.t_no_ball = 0

        self._init_state_machine()

    def _init_command_publishers(self):
        """
        根据可用球员初始化球员指令发布器。

        """
        self.command_publishers = {}
        for player in self.available_players:
            if player.role == FORWARD_1:
                self.command_publishers[player.uuid] = rospy.Publisher(f"/player_{player.uuid}/cmd", Twist, queue_size=10)
            elif player.role == FORWARD_2:
                self.command_publishers[player.uuid] = rospy.Publisher(f"/player_{player.uuid}/cmd", Twist, queue_size=10)
            elif player.role == DEFENDER_1:
                self.command_publishers[player.uuid] = rospy.Publisher(f"/player_{player.uuid}/cmd", Twist, queue_size=10)
            elif player.role == GOALKEEPER:
                self.command_publishers[player.uuid] = rospy.Publisher(f"/player_{player.uuid}/cmd", Twist, queue_size=10)
            else:
                raise ValueError("Invalid player role")

    def _init_state_machine(self):
        """
        初始化状态机。
        """
        self.state_machine = StateMachine(self)
        self.defend_ball_state_machine = DefendBallStateMachine(self)
        self.dribble_ball_state_machine = DribbleBallStateMachine(self)
        self.shoot_ball_state_machine = ShootBallStateMachine(self)

    def read_config(self):
        """
        读取配置文件。
        """
        # 读取本地json格式的配置文件
        with open("config.json", "r") as f:
            config = json.load(f)

        # 是否自动检测球员
        self.auto_detect_players = config.get("auto_detect_players", False)

        # 是否使用静态IP
        self.use_static_ip = config.get("use_static_ip", False)

    def update(self, robot_id, robots_data):
        """
        更新机器人数据。
        """
        self.robot_states[robot_id] = robots_data.get("data")

    # TODO: Implement this method
    def ball_pos_callback(self, msg):
        pass

    def get_players_distance_to_ball(self):
        """
        获取球员与球的距离。如果无法获取位置信息，则将距离视为一个大数。

        Returns:
            dict: 球员与球的距离
        """
        players_distance = {}
        BIG_NUMBER = 1e6  # 当位置不可用时使用的较大数值
        
        for robot_id, data in self.robots_data.items():
            player_pos = [data.get('x'), data.get('y')]
            ball_pos = [data.get('ballx'), data.get('bally')]

            # 检查是否为 NoneType
            if any(v is None for v in player_pos) or any(v is None for v in ball_pos):
                distance = BIG_NUMBER
            else:
                distance = np.linalg.norm(np.array(player_pos) - np.array(ball_pos))
            
            players_distance[robot_id] = distance
            
        return players_distance

    def get_players_distance_to_ball_without_goalkeeper(self):
        players_distance = {}
        BIG_NUMBER = 1e6  # 当位置不可用时使用的较大数值
        
        for robot_id, robot_data in self.robots_data.items():
            if robot_id != self.roles_to_id["goalkeeper"]:
                data = robot_data.get('data')
                player_pos = [data.get('x'), data.get('y')]
                ball_pos = [data.get('ballx'), data.get('bally')]
                
                # 检查是否为 NoneType
                if any(v is None for v in player_pos) or any(v is None for v in ball_pos):
                    distance = BIG_NUMBER
                else:
                    distance = np.linalg.norm(np.array(player_pos) - np.array(ball_pos))
                    
                players_distance[robot_id] = distance
            else:
                players_distance[robot_id] = BIG_NUMBER
                
        return players_distance

    def switch_players_role(self, player_role_1, player_role_2):
        """
        交换两个球员的角色。

        Args:
            player_role_1 (int): 球员1角色
            player_role_2 (int): 球员2角色

        """
        temp_roles_to_id = self.roles_to_id.copy()
        temp_roles_to_id[player_role_1], temp_roles_to_id[player_role_2] = self.roles_to_id[player_role_2], self.roles_to_id[player_role_1]
        self.roles_to_id = temp_roles_to_id

    def publish_command(self, player_id, cmd):
        """
        发布指令（带日志增强版）

        Args:
            player_id (int): 球员id
            cmd (str/dict): 指令内容
        """
        try:
            # 初始化日志（如果类中已有 logger 可省略）
            if not hasattr(self, 'logger'):
                self.logger = logging.getLogger(self.__class__.__name__)

            # 记录指令发送详情
            cmd_str = cmd if isinstance(cmd, str) else cmd.get("name", str(cmd))
            self.logger.info(f"[CMD] 正在发送指令 -> 球员 {player_id}: {cmd_str}")
            
            # 构造指令数据
            start_time = time.time()
            cmd_data = {
                "command": cmd,
                "data": {},
                "send_time": start_time,
            }

            # 执行异步发送
            asyncio.run(self.robot_server.send(player_id, cmd_data))
            
            # 记录性能指标
            latency = (time.time() - start_time) * 1000  # 毫秒
            self.logger.debug(f"[CMD] 指令发送成功 | 球员 {player_id} | 耗时 {latency:.2f}ms")

        except asyncio.TimeoutError:
            self.logger.error(f"[CMD] 指令发送超时 | 球员 {player_id} | 指令 {cmd_str}")
        except ConnectionError as e:
            self.logger.error(f"[CMD] 连接异常 | 球员 {player_id} | 错误: {str(e)}", exc_info=True)
        except Exception as e:
            self.logger.critical(f"[CMD] 未知错误 | 球员 {player_id} | 错误类型: {type(e).__name__}", exc_info=True)

    def initialize(self):
        """
        所有机器人入场
        """
        for role, id in self.roles_to_id.items():
            self.publish_command(id, COMMANDS.get("go_back_to_field"))

    def stop(self):
        """
        所有机器人停止运行
        """
        for role, id in self.roles_to_id.items():
            self.publish_command(id, COMMANDS.get("stop"))

    def ball_in_backcourt(self):
        """
        判断球是否在后场。

        Returns:
            bool: 如果球在后场返回True 否则返回False

        TODO: 确定球在后场的判断条件
        """
        if self.ball_x > 10:
            return True
        return False

    def ball_in_midcourt(self):
        """
        判断球是否在中场。

        Returns:
            bool: 如果球在中场返回True 否则返回False

        TODO: 确定球在中场的判断条件
        """
        if 5 < self.ball_x <= 10:
            return True
        return False

    def ball_in_frontcourt(self):
        """
        判断球是否在前场。

        Returns:
            bool: 如果球在前场返回True 否则返回False

        TODO: 确定球在前场的判断条件
        """
        if self.ball_x <= 5:
            return True
        return False

    def stop_condition(self):
        """
        停止条件。

        Returns:
            bool: 如果满足停止条件返回True 否则返回False
        """
        # 如果有两个以上机器人disconnect，停止
        count = 0
        for robot_id, data in self.robots_data.items():
            if data["status"] == "disconnected":
                count += 1
        if count >= 2:
            return True

    @property
    def state(self):
        return self._state

    @state.setter
    def state(self, value):
        self._state = value

    @property
    def ball_pos(self):
        return self._ball_pos

    def is_ball_in_control(self):
        """
        判断球是否在控制中。

        Returns:
            bool: 如果球在控制中返回True 否则返回False
        """
        return self.ifBall

    def run_defend_ball(self):
        self.defend_ball_state_machine.run()

    def run_dribble_ball(self):
        self.dribble_ball_state_machine.run()

    def run_shoot_ball(self):
        self.shoot_ball_state_machine.run()

    def run(self):
        while self.robots_data[1].get("status") == "disconnected":
            print("Waiting for connection")
            time.sleep(1)
            pass
        # self.state_machine.thread.start()
        print("Start running")
        self.state_machine.run_in_state1()


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
            # defend, dribble, shoot 只能由非 stop 状态转移而来
            {
                "trigger": "play",
                "source": ["dribble", "shoot", "initial"],
                "dest": "defend",
                "conditions": "ball_in_backcourt",
            },
            {
                "trigger": "play",
                "source": ["defend", "shoot", "initial"],
                "dest": "dribble",
                "conditions": "ball_in_midcourt",
            },
            {
                "trigger": "play",
                "source": ["defend", "dribble", "initial"],
                "dest": "shoot",
                "conditions": "ball_in_frontcourt",
            },
            # initial 只能由 stop 转移而来
            {
                "trigger": "initialize",
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
            initial="initial",  # Change to an existing state
            transitions=self.transitions,
        )
        # self.thread = threading.Thread(target=self.run_in_state1)

    def run_in_state1(self):
        try:
            while True:

                if self.model.stop_condition():
                    # self.model.stop_playing()
                    # break
                    # print("stop_condition satisfied")
                    pass

                self.model.play()
                time.sleep(0.1)

                if self.model.state == "stop":
                    pass
                elif self.model.state == "defend":
                    print("defend")
                    # self.model.run_defend_ball()
                    self.model.run_shoot_ball()
                elif self.model.state == "dribble":
                    print("dribble")
                    # self.model.run_dribble_ball()
                    self.model.run_shoot_ball()
                elif self.model.state == "shoot":
                    # print("shoot")
                    self.model.run_shoot_ball()
                elif self.model.state == "initial":
                    print("initial")
                    pass
                else:
                    logging.error("Invalid state")
                    pass
        except KeyboardInterrupt:
            print("\nProgram interrupted by user")
        finally:

            pass


class DefendBallStateMachine:
    def __init__(self, agent: Agent):
        self.agent = agent
        self.states = ["have_no_ball", "close_to_ball", "have_ball"]
        self.transitions = [
            {
                "trigger": "run",
                "source": "close_to_ball",
                "dest": "have_ball",
                "conditions": "ball_in_control",
                "after": "dribble_forward",
            },
            {
                "trigger": "run",
                "source": ["have_no_ball", "have_ball"],
                "dest": "close_to_ball",
                "conditions": "close_to_ball",
                "after": "go_for_possession_avoid_collsion",  # 争夺控球, 把find和chase封装进去
            },
            {
                "trigger": "run",
                "source": ["close_to_ball", "have_ball"],
                "dest": "have_no_ball",
                "conditions": "ball_out_of_control",
                "after": "go_for_possession",  # 争夺控球, 把find和chase封装进去
            },
        ]
        self.machine = Machine(
            model=self,
            states=self.states,
            initial="have_no_ball",
            transitions=self.transitions,
        )

    def ball_in_control(self):
        return not (self.ball_out_of_control() or self.close_to_ball())

    def ball_out_of_control(self):
        players_distance_to_ball = self.agent.get_players_distance_to_ball_without_goalkeeper()
        forward_1_distance_to_ball = players_distance_to_ball[self.agent.roles_to_id["forward_1"]]
        forward_2_distance_to_ball = players_distance_to_ball[self.agent.roles_to_id["forward_2"]]

        if forward_1_distance_to_ball > 0.5 and forward_2_distance_to_ball > 0.5:
            return True
        return False

    def close_to_ball(self):
        players_distance_to_ball = self.agent.get_players_distance_to_ball_without_goalkeeper()
        forward_1_distance_to_ball = players_distance_to_ball[self.agent.roles_to_id["forward_1"]]
        forward_2_distance_to_ball = players_distance_to_ball[self.agent.roles_to_id["forward_2"]]

        if forward_1_distance_to_ball < 0.5 or forward_2_distance_to_ball < 0.5:
            return True
        return False

    def dribble_forward(self):
        """
        判断非守门员机器人中谁拿到球，然后进行带球
        其它两个机器人距离球门最近的进行防守补位，另一个跟随进攻
        """
        players_status = self.agent.get_players_status()
        players_distance = self.agent.get_players_distance_to_ball_without_goalkeeper()
        for role, id in self.agent.roles_to_id.items():
            if players_status[id] == "controlling_ball":
                if role == "defender_1":
                    # 将players中的defender_1的角色与前锋中距离球门最近的角色交换
                    # 比较两名前锋与球门的距离
                    forward_1_distance = players_distance[self.agent.roles_to_id["forward_1"]]
                    forward_2_distance = players_distance[self.agent.roles_to_id["forward_2"]]
                    # 将前锋中距离球门最近的角色与defender_1的角色交换
                    if forward_1_distance < forward_2_distance:
                        self.agent.switch_players_role("defender_1", "forward_1")
                        self.agent.publish_command(self.agent.roles_to_id["forward_1"], COMMANDS["dribble"])
                        self.agent.publish_command(self.agent.roles_to_id["forward_2"], COMMANDS["forward"])
                    else:
                        self.agent.switch_players_role("defender_1", "forward_2")
                        self.agent.publish_command(self.agent.roles_to_id["forward_2"], COMMANDS["dribble"])
                        self.agent.publish_command(self.agent.roles_to_id["forward_1"], COMMANDS["forward"])
                else:
                    if role == "forward_1":
                        self.agent.publish_command(self.agent.roles_to_id["forward_1"], COMMANDS["dribble"])
                        self.agent.publish_command(self.agent.roles_to_id["forward_2"], COMMANDS["forward"])
                    else:
                        self.agent.publish_command(self.agent.roles_to_id["forward_2"], COMMANDS["dribble"])
                        self.agent.publish_command(self.agent.roles_to_id["forward_1"], COMMANDS["forward"])
        # 防守补位
        self.agent.publish_command(self.agent.roles_to_id["defender_1"], COMMANDS["go_to_defend_position"])

    def go_for_possession(self):
        """
        争夺控球
        """
        # self.agent.t_no_ball = 0
        self.agent.publish_command(self.agent.roles_to_id["forward_1"], COMMANDS["chase_ball"])
        self.agent.publish_command(self.agent.roles_to_id["forward_2"], COMMANDS["chase_ball"])
        self.agent.publish_command(self.agent.roles_to_id["defender_1"], COMMANDS["chase_ball"])

    def go_for_possession_avoid_collsion(self):
        """
        防止距离球太近时机器人发生碰撞
        """
        # 获取各球员与球的距离
        players_distances = self.agent.get_players_distance_to_ball_without_goalkeeper()

        # 将球员与他们的距离组合成列表
        players_distances = [(id, distance) for id, distance in players_distances.items()]

        # 根据距离从大到小排序
        players_distances_sorted = sorted(players_distances, key=lambda x: x[1], reverse=True)

        # 向最远的两个球员发送停止指令
        for role, _ in players_distances_sorted[:2]:
            self.agent.publish_command(role, COMMANDS["stop_moving"])


class DribbleBallStateMachine:
    def __init__(self, agent: Agent):
        self.agent = agent
        self.states = ["have_no_ball", "close_to_ball", "have_ball"]
        self.transitions = [
            {
                "trigger": "run",
                "source": "close_to_ball",
                "dest": "have_ball",
                "conditions": "ball_in_control",
                "after": "dribble_forward",
            },
            {
                "trigger": "run",
                "source": ["have_no_ball", "have_ball"],
                "dest": "close_to_ball",
                "conditions": "close_to_ball",
                "after": "go_for_possession_avoid_collsion",  # 争夺控球, 把find和chase封装进去
            },
            {
                "trigger": "run",
                "source": ["close_to_ball", "have_ball"],
                "dest": "have_no_ball",
                "conditions": "ball_out_of_control",
                "after": "go_for_possession",  # 争夺控球, 把find和chase封装进去
            },
        ]
        self.machine = Machine(
            model=self,
            states=self.states,
            initial="have_no_ball",
            transitions=self.transitions,
        )

    def ball_in_control(self):
        return self.agent.is_ball_in_control()

    def ball_out_of_control(self):
        players_distance_to_ball = self.agent.get_players_distance_to_ball_without_goalkeeper()
        forward_1_distance_to_ball = players_distance_to_ball[self.agent.roles_to_id["forward_1"]]
        forward_2_distance_to_ball = players_distance_to_ball[self.agent.roles_to_id["forward_2"]]
        if forward_1_distance_to_ball > 0.5 and forward_2_distance_to_ball > 0.5:
            return True
        return False

    def close_to_ball(self):
        players_distance_to_ball = self.agent.get_players_distance_to_ball_without_goalkeeper()
        forward_1_distance_to_ball = players_distance_to_ball[self.agent.roles_to_id["forward_1"]]
        forward_2_distance_to_ball = players_distance_to_ball[self.agent.roles_to_id["forward_2"]]
        if forward_1_distance_to_ball < 0.5 or forward_2_distance_to_ball < 0.5:
            return True
        return False

    def dribble_forward(self):
        """
        判断非守门员机器人中谁拿到球，然后进行带球
        其它两个机器人距离球门最近的进行防守补位，另一个跟随进攻
        """
        players_status = self.agent.get_players_status()
        players_distance = self.agent.get_players_distance_to_ball_without_goalkeeper()
        for role, id in self.agent.roles_to_id.items():
            if players_status[id] == "controlling_ball":
                if role == "forward_1":
                    self.agent.publish_command(self.agent.roles_to_id["forward_1"], COMMANDS["dribble"])
                    self.agent.publish_command(self.agent.roles_to_id["forward_2"], COMMANDS["forward"])
                else:
                    self.agent.publish_command(self.agent.roles_to_id["forward_2"], COMMANDS["dribble"])
                    self.agent.publish_command(self.agent.roles_to_id["forward_1"], COMMANDS["forward"])
        # 防守补位
        self.agent.publish_command(self.agent.roles_to_id["defender_1"], COMMANDS["go_to_defend_position"])

    def go_for_possession(self):
        """
        争夺控球
        """
        self.agent.t_no_ball = 0
        self.agent.publish_command(self.agent.roles_to_id["forward_1"], COMMANDS["chase_ball"])
        self.agent.publish_command(self.agent.roles_to_id["forward_2"], COMMANDS["chase_ball"])

    def go_for_possession_avoid_collsion(self):
        """
        防止距离球太近时两机器人发生碰撞
        """
        players_distance_to_ball = self.agent.get_players_distance_to_ball_without_goalkeeper()
        forward_1_distance_to_ball = players_distance_to_ball[self.agent.roles_to_id["forward_1"]]
        forward_2_distance_to_ball = players_distance_to_ball[self.agent.roles_to_id["forward_2"]]
        if forward_1_distance_to_ball < forward_2_distance_to_ball:
            self.agent.publish_command(self.agent.roles_to_id["forward_2"], COMMANDS["stop"])
        else:
            self.agent.publish_command(self.agent.roles_to_id["forward_1"], COMMANDS["stop"])

class ShootBallStateMachine:
    def __init__(self, agent: Agent):
        self.agent = agent
        self.logger = logging.getLogger(self.__class__.__name__)
        self.logger.setLevel(logging.DEBUG)
        
        # 初始化状态机
        self.states = ["have_no_ball", "close_to_ball", "have_ball"]
        self.transitions = [
            {
                "trigger": "run",
                "source": "close_to_ball",
                "dest": "have_ball",
                "conditions": "ball_in_control",
                "after": ["shoot", "log_transition"],  # 添加日志方法
            },
            {
                "trigger": "run",
                "source": ["have_no_ball", "have_ball"],
                "dest": "close_to_ball",
                "conditions": "close_to_ball",
                "after": ["go_for_possession_avoid_collsion", "log_transition"],
            },
            {
                "trigger": "run",
                "source": ["close_to_ball", "have_ball", "have_no_ball"],
                "dest": "have_no_ball",
                "conditions": "ball_out_of_control",
                "after": ["go_for_possession", "log_transition"],
            },
        ]
        
        self.machine = Machine(
            model=self,
            states=self.states,
            initial="have_no_ball",
            transitions=self.transitions,
            after_state_change=self.on_state_change,  # 添加全局状态变更回调
        )
        self.logger.info("状态机初始化完成，初始状态: have_no_ball")

    def log_transition(self):
        """记录状态迁移"""
        self.logger.debug(f"执行迁移后的动作，当前状态: {self.state}")

    def on_state_change(self, event=None):
        """全局状态变更回调"""
        if event:
            self.logger.info(f"状态变更: {event.transition.source} -> {event.transition.dest}")

    def ball_in_control(self):
        result = self.agent.is_ball_in_control()
        self.logger.debug(f"检查球权控制状态: {'已控球' if result else '未控球'}")
        return result

    def ball_out_of_control(self):
        players_distance_to_ball = self.agent.get_players_distance_to_ball_without_goalkeeper()
        forward_1_distance = players_distance_to_ball[self.agent.roles_to_id["forward_1"]]
        forward_2_distance = players_distance_to_ball[self.agent.roles_to_id["forward_2"]]
        result = (forward_1_distance > 0.5 and forward_2_distance > 0.5)
        self.logger.debug(
            f"检查球权丢失状态: 前锋1距离={forward_1_distance:.2f}, "
            f"前锋2距离={forward_2_distance:.2f}, {'已丢失' if result else '仍保持'}"
        )
        return result

    def close_to_ball(self):
        players_distance_to_ball = self.agent.get_players_distance_to_ball_without_goalkeeper()
        forward_1_distance = players_distance_to_ball[self.agent.roles_to_id["forward_1"]]
        forward_2_distance = players_distance_to_ball[self.agent.roles_to_id["forward_2"]]
        result = (forward_1_distance < 0.5 or forward_2_distance < 0.5)
        self.logger.debug(
            f"检查近距离状态: 前锋1距离={forward_1_distance:.2f}, "
            f"前锋2距离={forward_2_distance:.2f}, {'在范围内' if result else '未接近'}"
        )
        return result

    def shoot(self):
        """射门动作（添加距离信息日志）"""
        players_distance = self.agent.get_players_distance_to_ball_without_goalkeeper()
        min_distance = float('inf')
        closest_player_id = None
        
        # 记录所有球员距离
        distance_log = ", ".join(
            [f"{role}: {players_distance[id]:.2f}" 
             for role, id in self.agent.roles_to_id.items()]
        )
        self.logger.debug(f"球员距离球门数据: {distance_log}")

        # 寻找最近球员
        for role, id in self.agent.roles_to_id.items():
            if players_distance[id] < min_distance:
                min_distance = players_distance[id]
                closest_player_id = id

        if closest_player_id is not None:
            self.logger.info(f"执行射门: 球员 {closest_player_id} (距离={min_distance:.2f})")
            self.agent.publish_command(closest_player_id, COMMANDS["shoot"])
        else:
            self.logger.warning("射门失败: 未找到可用球员")

    def go_for_possession(self):
        """争夺控球（添加策略说明）"""
        self.logger.info("执行双前锋追球策略")
        self.agent.publish_command(self.agent.roles_to_id["forward_1"], COMMANDS["chase_ball"])
        self.agent.publish_command(self.agent.roles_to_id["forward_2"], COMMANDS["chase_ball"])

    def go_for_possession_avoid_collsion(self):
        """防碰撞策略（添加选择逻辑说明）"""
        players_distance_to_ball = self.agent.get_players_distance_to_ball_without_goalkeeper()
        forward_1_distance = players_distance_to_ball[self.agent.roles_to_id["forward_1"]]
        forward_2_distance = players_distance_to_ball[self.agent.roles_to_id["forward_2"]]
        
        # 记录比较结果
        self.logger.debug(
            f"防撞策略: 前锋1距离={forward_1_distance:.2f} vs 前锋2距离={forward_2_distance:.2f}"
        )

        if forward_1_distance < forward_2_distance:
            self.logger.info("执行策略: 前锋1追球，前锋2待命")
            self.agent.publish_command(self.agent.roles_to_id["forward_2"], COMMANDS["stop"])
            self.agent.publish_command(self.agent.roles_to_id["forward_1"], COMMANDS["chase_ball"])
        else:
            self.logger.info("执行策略: 前锋2追球，前锋1待命")
            self.agent.publish_command(self.agent.roles_to_id["forward_1"], COMMANDS["stop_moving"])
            self.agent.publish_command(self.agent.roles_to_id["forward_2"], COMMANDS["chase_ball"])


def main():
    agent = Agent()
    agent.ifBall = False
    agent.run()


if __name__ == "__main__":
    main()
