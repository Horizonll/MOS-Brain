#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool, Int32, String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from decision_subscriber import Decision_Pos, Decision_Vision, Decision_Motion
from receiver import Receiver
from utils import config

import threading
from transitions import Machine
import time
import numpy as np
import math

ROLES = {
    "forward_1": 1,
    "forward_2": 2,
    "defender_1": 3,
    "goalkeeper": 4,
}

COMMANDS = {
    "dribble": 'dribble',
    "forward": 'forward',
    "stop": 'stop',
}

PLAYER_STATES = {
    "close_to_ball": "close_to_ball",
}



class Agent(Decision_Pos, Decision_Motion, Decision_Vision):
    def __init__(self):
        # 初始化父类
        Decision_Pos.__init__(self)
        Decision_Motion.__init__(self)
        Decision_Vision.__init__(self)

        # 初始化ROS节点
        rospy.init_node("decider")

        # 获取参数 TODO: 确定参数
        # 是否自动检测球员
        self.auto_detect_players = rospy.get_param("/auto_detect_players")

        # 开启TCP监听、
        self.receiver = Receiver()

        # 开启UDP监听
        self.receiver.start_udp_server()

        # 订阅球员心跳
        self.player_heartbeat_sub = rospy.Subscriber(
            "/player_heartbeat", Bool, self.player_heartbeat_callback
        )

        # 订阅裁判盒消息

        # 获取可用球员
        self.players = self.get_available_players()

        # 根据可用球员初始化球员指令发布器 TODO: 确定指令格式
        self._init_command_publishers()

        # 初始化ROS球位置订阅器 TODO: 确定消息格式
        self.ball_sub = rospy.Subscriber(
            "/ball_pos", JointState, self.ball_pos_callback
        )

        self._ball_pos = None

        self._state = None
        self.state_machine = StateMachine(self)
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

    def loop(self):
        return not rospy.is_shutdown()
    
    # TODO: Implement this method
    def ball_pos_callback(self, msg):
        pass

    def get_available_players(self):
        """
        扫描ROS参数，获取可用球员。
        根据参数，有两种方式获取可用球员：
        1. 在配置文件中指定可用球员角色、数量及UIID
        2. 在运行时通过扫描心跳消息自动添加可用球员

        Returns:
            list: 可用球员列表
        """
        self.players = []
    
        if self.auto_detect_players:
            # 从心跳消息中获取可用球员
            pass
        else:
            # 从配置文件中获取可用球员
            pass

        self.players = [
            {"role": ROLES["forward_1"], "uuid": 1},
            {"role": ROLES["forward_2"], "uuid": 2},
            {"role": ROLES["defender_1"], "uuid": 3},
            {"role": ROLES["goalkeeper"], "uuid": 4},
        ]

        return self.players

    def get_player(self, player_role=None, player_uuid=None):
        """
        获取单个球员role和uuid。

        Args:
            player_role (int): 球员角色
            player_uuid (int): 球员UUID

        Returns:
            dict: 球员基本信息
        """
        player = {}

        for p in self.players:
            if p["role"] == player_role or p["uuid"] == player_uuid:
                player = p

        return player
    
    def get_player_info(self, player_role=None, player_uuid=None):
        """
        获取单个球员的详细信息。

        Args:
            player_role (int): 球员角色
            player_uuid (int): 球员UUID

        Returns:
            dict: 球员详细信息
        """
        player_info = {}

        return player_info

    def get_players_info(self):
        """
        获取球员信息。

        Returns:
            list: 球员信息列表
        """

        players_info = []
        pass

    def switch_players_role(self, player_role_1, player_role_2):
        """
        交换两个球员的角色。

        Args:
            player_role_1 (int): 球员1角色
            player_role_2 (int): 球员2角色

        Returns:
            bool: 交换成功返回True 否则返回False
        """

        player_1 = self.get_player(player_role=player_role_1)
        player_2 = self.get_player(player_role=player_role_2)
        players = self.players

        if player_1 and player_2 and player_1 != player_2:
            players.remove(player_1)
            players.remove(player_2)
            player_1["role"], player_2["role"] = player_2["role"], player_1["role"]
            self.players = players
            return True

        return False

    def publish_command(self, player_role, cmd):
        """
        发布指令。

        Args:
            player_role (int): 球员角色
            cmd (): 指令
        """
        self.command_publishers[player_role].publish(cmd)

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
    
    def defend_ball(self):
        self.defend_ball_state_machine = DefendBallStateMachine(self)
        self.defend_ball_state_machine.run()

    def play(self):
        self.state_machine.play()


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
                "after": "defend_ball",
            },
            {
                "trigger": "play",
                "source": "*",
                "dest": "dribble",
                "conditions": "ball_in_midcourt",
                "after": "dribble_ball",
            },
            {
                "trigger": "play",
                "source": "*",
                "dest": "shoot",
                "conditions": "ball_in_frontcourt",
                "after": "shoot_ball",
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
    
    def defend_ball(self):
        """
        进入防守状态。
        """
        self.model.defend_ball()

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
        forward_1_distance_to_ball = self.agent.get_player_info(player_role=ROLES["forward_1"])["distance_ball"]
        forward_2_distance_to_ball = self.agent.get_player_info(player_role=ROLES["forward_2"])["distance_ball"]
        defender_1_distance_to_ball = self.agent.get_player_info(player_role=ROLES["defender_1"])["distance_ball"]
        if forward_1_distance_to_ball > 0.5 and forward_2_distance_to_ball > 0.5:
            return True
        return False
    
    def close_to_ball(self):
        forward_1_distance_to_ball = self.agent.get_player_info(player_role=ROLES["forward_1"])["distance_ball"]
        forward_2_distance_to_ball = self.agent.get_player_info(player_role=ROLES["forward_2"])["distance_ball"]
        defender_1_distance_to_ball = self.agent.get_player_info(player_role=ROLES["defender_1"])["distance_ball"]
        if forward_1_distance_to_ball < 0.5 or forward_2_distance_to_ball < 0.5:
            return True
        return False
    
    def dribble_forward(self):
        """
        判断非守门员机器人中谁拿到球，然后进行带球
        其它两个机器人距离球门最近的进行防守补位，另一个跟随进攻
        """
        players =  self.agent.players
        players_info = self.get_players_info()
        for player_info in players_info:
            if player_info["state"] == "controlling_ball":
                if player_info["role"] == ROLES["defender_1"]:
                    # 将players中的defender_1的角色与前锋中距离球门最近的角色交换
                    # 比较两名前锋与球门的距离
                    forward_1 = self.agent.get_player(player_role=ROLES["forward_1"])
                    forward_1_distance = self.agent.get_player_info(player_role=ROLES["forward_1"])["distance1"]
                    forward_2 = self.agent.get_player(player_role=ROLES["forward_2"])
                    forward_2_distance = self.agent.get_player_info(player_role=ROLES["forward_2"])["distance1"]
                    # 将前锋中距离球门最近的角色与defender_1的角色交换
                    if forward_1_distance < forward_2_distance:
                        self.agent.switch_players_role(ROLES["defender_1"], ROLES["forward_1"])
                        self.agent.publish_command(ROLES["forward_1"], COMMANDS["dribble"])
                        self.agent.publish_command(ROLES["forward_2"], COMMANDS["forward"])
                    else:
                        self.agent.switch_players_role(ROLES["defender_1"], ROLES["forward_2"])
                        self.agent.publish_command(ROLES["forward_2"], COMMANDS["dribble"])
                        self.agent.publish_command(ROLES["forward_1"], COMMANDS["forward"])
                else:
                    if player_info["role"] == ROLES["forward_1"]:
                        self.agent.publish_command(ROLES["forward_1"], COMMANDS["dribble"])
                        self.agent.publish_command(ROLES["forward_2"], COMMANDS["forward"])
                    else:
                        self.agent.publish_command(ROLES["forward_2"], COMMANDS["dribble"])
                        self.agent.publish_command(ROLES["forward_1"], COMMANDS["forward"])
        # 防守补位
        self.agent.publish_command(ROLES["defender_1"], COMMANDS["go_to_defend_position"])

    def go_for_possession(self):
        """
        争夺控球
        """
        # self.agent.t_no_ball = 0
        self.agent.publish_command(ROLES["forward_1"], COMMANDS["go_for_possession"])
        self.agent.publish_command(ROLES["forward_2"], COMMANDS["go_for_possession"])
        self.agent.publish_command(ROLES["defender_1"], COMMANDS["go_for_possession"])
            

    def go_for_possession_avoid_collsion(self):
        """
        防止距离球太近时机器人发生碰撞
        """
        # 获取各球员与球的距离
        forward_1_distance_to_ball = self.agent.get_player_info(player_role=ROLES["forward_1"])["distance_ball"]
        forward_2_distance_to_ball = self.agent.get_player_info(player_role=ROLES["forward_2"])["distance_ball"]
        defender_1_distance_to_ball = self.agent.get_player_info(player_role=ROLES["defender_1"])["distance_ball"]

        # 将球员与他们的距离组合成列表
        players_distances = [
            (ROLES["forward_1"], forward_1_distance_to_ball),
            (ROLES["forward_2"], forward_2_distance_to_ball),
            (ROLES["defender_1"], defender_1_distance_to_ball)
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
        forward_1_distance_to_ball = self.agent.get_player_info(player_role=ROLES["forward_1"])["distance_ball"]
        forward_2_distance_to_ball = self.agent.get_player_info(player_role=ROLES["forward_2"])["distance_ball"]
        if forward_1_distance_to_ball > 0.5 and forward_2_distance_to_ball > 0.5:
            return True
        return False
    
    def close_to_ball(self):
        forward_1_distance_to_ball = self.agent.get_player_info(player_role=ROLES["forward_1"])["distance_ball"]
        forward_2_distance_to_ball = self.agent.get_player_info(player_role=ROLES["forward_2"])["distance_ball"]
        if forward_1_distance_to_ball < 0.5 or forward_2_distance_to_ball < 0.5:
            return True
        return False
    
    def dribble_forward(self):
        """
        判断非守门员机器人中谁拿到球，然后进行带球
        其它两个机器人距离球门最近的进行防守补位，另一个跟随进攻
        """
        players =  self.agent.players
        players_info = self.get_players_info()
        for player_info in players_info:
            if player_info["state"] == "controlling_ball":
                if player_info["role"] == ROLES["forward_1"]:
                    self.agent.publish_command(ROLES["forward_1"], COMMANDS["dribble"])
                    self.agent.publish_command(ROLES["forward_2"], COMMANDS["forward"])
                else:
                    self.agent.publish_command(ROLES["forward_2"], COMMANDS["dribble"])
                    self.agent.publish_command(ROLES["forward_1"], COMMANDS["forward"])
    
        

    def go_for_possession(self):
        """
        争夺控球
        """
        self.agent.t_no_ball = 0
        self.agent.publish_command(ROLES["forward_1"], COMMANDS["go_for_possession"])
        self.agent.publish_command(ROLES["forward_2"], COMMANDS["go_for_possession"])
            

    def go_for_possession_avoid_collsion(self):
        """
        防止距离球太近时两机器人发生碰撞
        """
        forward_1_distance_to_ball = self.agent.get_player_info(player_role=ROLES["forward_1"])["distance_ball"]
        forward_2_distance_to_ball = self.agent.get_player_info(player_role=ROLES["forward_2"])["distance_ball"]
        if forward_1_distance_to_ball < forward_2_distance_to_ball:
            self.agent.publish_command(ROLES["forward_2"], COMMANDS["stop_moving"])
        else:
            self.agent.publish_command(ROLES["forward_1"], COMMANDS["stop_moving"])

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
        forward_1_distance_to_ball = self.agent.get_player_info(player_role=ROLES["forward_1"])["distance_ball"]
        forward_2_distance_to_ball = self.agent.get_player_info(player_role=ROLES["forward_2"])["distance_ball"]
        if forward_1_distance_to_ball > 0.5 and forward_2_distance_to_ball > 0.5:
            return True
        return False
    
    def close_to_ball(self):
        forward_1_distance_to_ball = self.agent.get_player_info(player_role=ROLES["forward_1"])["distance_ball"]
        forward_2_distance_to_ball = self.agent.get_player_info(player_role=ROLES["forward_2"])["distance_ball"]
        if forward_1_distance_to_ball < 0.5 or forward_2_distance_to_ball < 0.5:
            return True
        return False
    
    def shoot(self):
        """
        判断非守门员机器人中谁拿到球，然后进行带球
        其它两个机器人距离球门最近的进行防守补位，另一个跟随进攻
        """
        players =  self.agent.players
        players_info = self.get_players_info()
        for player_info in players_info:
            if player_info["state"] == "controlling_ball":
                self.agent.publish_command(player_info["role"], COMMANDS["shoot"])

    def go_for_possession(self):
        """
        争夺控球
        """
        self.agent.t_no_ball = 0
        self.agent.publish_command(ROLES["forward_1"], COMMANDS["go_for_possession"])
        self.agent.publish_command(ROLES["forward_2"], COMMANDS["go_for_possession"])
            

    def go_for_possession_avoid_collsion(self):
        """
        防止距离球太近时两机器人发生碰撞
        """
        forward_1_distance_to_ball = self.agent.get_player_info(player_role=ROLES["forward_1"])["distance_ball"]
        forward_2_distance_to_ball = self.agent.get_player_info(player_role=ROLES["forward_2"])["distance_ball"]
        if forward_1_distance_to_ball < forward_2_distance_to_ball:
            self.agent.publish_command(ROLES["forward_2"], COMMANDS["stop_moving"])
            self.agent.publish_command(ROLES["forward_1"], COMMANDS["go_for_possession"])
        else:
            self.agent.publish_command(ROLES["forward_1"], COMMANDS["stop_moving"])
            self.agent.publish_command(ROLES["forward_2"], COMMANDS["go_for_possession"])

def main():
    agent = Agent()
    while True:
        agent.ifBall = int(input())
        agent.run()


if __name__ == "__main__":
    main()
