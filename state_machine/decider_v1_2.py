#!/usr/bin/env python3

import json

import threading
from transitions import Machine
from collections import defaultdict
import time
import numpy as np
import math
import logging

from state_machine.robot_server import RobotServer



COMMANDS = {
    "dribble": 'dribble',
    "forward": 'forward',
    "stop": 'stop',
    "find_ball": 'find_ball',
    "chase_ball": 'chase_ball',
    "shoot": 'kick',
    "go_back_to_field": 'go_back_to_field',
}

PLAYER_STATES = {
    "close_to_ball": "close_to_ball",
}



class Agent():
    def __init__(self):

        # 初始化ROS节点
        # rospy.init_node("decider")

        # 获取参数 TODO: 确定参数

        # 机器人状态
        self.robot_data = defaultdict( lambda: {
            "last_seen": None, 
            "status": "disconnected", 
            "data": {}} 
        )

        # 开启TCP监听
        self.robot_server = RobotServer(agent=self)

        # 开启UDP监听

        # 订阅裁判盒消息

        # 获取可用球员

        # 创建角色与ID的映射
        self.roles_to_id = {
            "forward_1": 1,
            "forward_2": 2,
            "defender_1": 3,
            "goalkeeper": 4,
        }

        # 根据可用球员更改角色对应ID
        self.update_players_id()

        self._ball_pos = None

        self._state = None
        
        self.ifBall = False
        self.ready_to_kick = False
        self.t_no_ball = 0


    def _init_command_publishers(self):
        """
        根据可用球员初始化球员指令发布器。

        """
        self.command_publishers = {}
        for player in self.available_players:
            if player.role == FORWARD_1:
                self.command_publishers[player.uuid] = rospy.Publisher(
                    f"/player_{player.uuid}/cmd", Twist, queue_size=10
                )
            elif player.role == FORWARD_2:
                self.command_publishers[player.uuid] = rospy.Publisher(
                    f"/player_{player.uuid}/cmd", Twist, queue_size=10
                )
            elif player.role == DEFENDER_1:
                self.command_publishers[player.uuid] = rospy.Publisher(
                    f"/player_{player.uuid}/cmd", Twist, queue_size=10
                )
            elif player.role == GOALKEEPER:
                self.command_publishers[player.uuid] = rospy.Publisher(
                    f"/player_{player.uuid}/cmd", Twist, queue_size=10
                )
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

    def update(self, robot_id, robot_data):
        """
        更新机器人数据。
        """
        self.robot_states[robot_id] = robot_data[data]

    # TODO: Implement this method
    def ball_pos_callback(self, msg):
        pass

    def get_players_distance_to_ball(self):
        """
        获取球员与球的距离。

        Returns:
            dict: 球员与球的距离
        """
        players_distance = {}

        for robot_id, data in self.robot_data.items():
            player_pos = [data.get('x'), data.get('y')]
            ball_pos = [data.get('ballx'), data.get('bally')]
            distance = np.linalg.norm(np.array(player_pos) - np.array(ball_pos))
            players_distance[robot_id] = distance

        return players_distance
    
    def get_players_distance_to_ball_without_goalkeeper(self):

        players_distance = {}

        for robot_id, data in self.robot_data.items():
            if data["role"] != "goalkeeper":
                player_pos = [data.get('x'), data.get('y')]
                ball_pos = [data.get('ballx'), data.get('bally')]
                distance = np.linalg.norm(np.array(player_pos) - np.array(ball_pos))
                players_distance[robot_id] = distance

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

    def publish_command(self, player_role, cmd):
        """
        发布指令。

        Args:
            player_role (int): 球员角色
            cmd (): 指令
        """
        # 将command, data, player_role, 时间戳等信息打包成字典
        cmd_data = {
            "command": cmd,
            "data": {},
            "send_time": time.time(),
        }

        self.robot_server.send(self.roles_to_id[player_role], cmd_data)

    def initialize(self):
        """
        所有机器人入场
        """
        for role, id in self.roles_to_id.items():
            self.publish_command(role, COMMANDS.get("go_back_to_field"))

    def stop(self):
        """
        所有机器人停止运行
        """
        for role, id in self.roles_to_id.items():
            self.publish_command(role, COMMANDS.get("stop"))

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


class StateMachine:
    def __init__(self, model: Agent):
        self.states = [
            "stop",
            "defend",
            "dribble",
            "shoot",
        ]
        self.transitions = [
            {
                "trigger": "play",
                "source": "*",
                "dest": "defend",
                "conditions": "ball_in_backcourt",
                # "after": "defend_ball",
            },
            {
                "trigger": "play",
                "source": "*",
                "dest": "dribble",
                "conditions": "ball_in_midcourt",
                # "after": "dribble_ball",
            },
            {
                "trigger": "play",
                "source": "*",
                "dest": "shoot",
                "conditions": "ball_in_frontcourt",
                # "after": "shoot_ball",
            },
            {
                "trigger": "stop_playing",
                "source": "*",
                "dest": "stop",
                "after": "stop",
            },
            {
                "trigger": "initialize",
                "source": "*",
                "dest": "initial",
                "after": "initialize",
            },
        ]
        self.machine = Machine(
            model=model,
            states=self.states,
            initial="find_ball",
            transitions=self.transitions,
        )
        self.thread = threading.Thread(target=self.run_in_state1)

    def ball_in_backcourt(self):
        """
        判断球是否在后场。

        Returns:
            bool: 如果球在后场返回True 否则返回False

        TODO: 确定球在后场的判断条件
        """
        if self.model.ball_pos > 10:
            return True
        return False
        
    def ball_in_midcourt(self):
        """
        判断球是否在中场。

        Returns:
            bool: 如果球在中场返回True 否则返回False

        TODO: 确定球在中场的判断条件
        """
        if 5 < self.model.ball_pos <= 10:
            return True
        return False
    
    def ball_in_frontcourt(self):
        """
        判断球是否在前场。

        Returns:
            bool: 如果球在前场返回True 否则返回False

        TODO: 确定球在前场的判断条件
        """
        if self.model.ball_pos <= 5:
            return True
        return False
    
    def stop(self):
        self.model.stop()

    def initialize(self):
        self.model.initialize()

    def run_in_state1(self):
        while True:
            self.machine.play()
            time.sleep(0.1)

            if self.model.state == "stop":
                pass
            elif self.model.state == "defend":
                self.model.run_defend_ball()
            elif self.model.state == "dribble":
                self.model.run_dribble_ball()
            elif self.model.state == "shoot":
                self.model.run_shoot_ball()
            elif self.model.state == "initial":
                pass
            else:
                logging.error("Invalid state")
                pass

        


class DefendBallStateMachine:
    def __init__(self, agent: Agent):
        self.agent = agent
        self.states = [
            "have_no_ball", 
            "close_to_ball",
            "have_ball"
            ]
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
                "source": "*",
                "dest": "close_to_ball",
                "conditions": "close_to_ball",
                "after": "go_for_possession_avoid_collsion", # 争夺控球, 把find和chase封装进去
            },
            {
                "trigger": "run",
                "source": "*",
                "dest": "have_no_ball",
                "conditions": "ball_out_of_control",
                "after": "go_for_possession", # 争夺控球, 把find和chase封装进去
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
        players_distances = [
            (id, distance) for id, distance in players_distances.items()
        ]

        # 根据距离从大到小排序
        players_distances_sorted = sorted(players_distances, key=lambda x: x[1], reverse=True)

        # 向最远的两个球员发送停止指令
        for role, _ in players_distances_sorted[:2]:
            self.agent.publish_command(role, COMMANDS["stop_moving"])

class DribbleBallStateMachine:
    def __init__(self, agent: Agent):
        self.agent = agent
        self.states = [
            "have_no_ball", 
            "close_to_ball",
            "have_ball"
            ]
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
                "source": "*",
                "dest": "close_to_ball",
                "conditions": "close_to_ball",
                "after": "go_for_possession_avoid_collsion", # 争夺控球, 把find和chase封装进去
            },
            {
                "trigger": "run",
                "source": "*",
                "dest": "have_no_ball",
                "conditions": "ball_out_of_control",
                "after": "go_for_possession", # 争夺控球, 把find和chase封装进去
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
        self.states = [
            "have_no_ball", 
            "close_to_ball",
            "have_ball"
        ]
        self.transitions = [
            {
                "trigger": "run",
                "source": "close_to_ball",
                "dest": "have_ball",
                "conditions": "ball_in_control",
                "after": "shoot",
            },
            {
                "trigger": "run",
                "source": "*",
                "dest": "close_to_ball",
                "conditions": "close_to_ball",
                "after": "go_for_possession_avoid_collsion", # 争夺控球, 把find和chase封装进去
            },
            {
                "trigger": "run",
                "source": "*",
                "dest": "have_no_ball",
                "conditions": "ball_out_of_control",
                "after": "go_for_possession", # 争夺控球, 把find和chase封装进去
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
    
    def shoot(self):
        """
        判断非守门员机器人中谁拿到球，然后进行射门
        """
        players_status = self.agent.get_players_status()
        players_distance = self.agent.get_players_distance_to_ball_without_goalkeeper()
        for role, id in self.agent.roles_to_id.items:
            if players_status[id] == "controlling_ball":
                self.agent.publish_command(id, COMMANDS["shoot"])


    def go_for_possession(self):
        """
        争夺控球
        """
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
            self.agent.publish_command(self.agent.roles_to_id["forward_1"], COMMANDS["chase_ball"])
        else:
            self.agent.publish_command(self.agent.roles_to_id["forward_1"], COMMANDS["stop_moving"])
            self.agent.publish_command(self.agent.roles_to_id["forward_2"], COMMANDS["chase_ball"])

def main():
    agent = Agent()
    while True:
        agent.ifBall = int(input())
        agent.run()


if __name__ == "__main__":
    main()
