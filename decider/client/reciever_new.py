# receiver.py
#   @description:   Utilities to connect with the game controller
import socket
import logging
import threading
from construct import Container, ConstError
from construct import (
    Byte,
    Struct,
    Enum,
    Bytes,
    Const,
    Array,
    Int16ul,
    Int16sl,
)

# 导入上半部分定义的常量
GAMECONTROLLER_DATA_PORT = 3838
GAMECONTROLLER_RETURN_PORT = 3939
MAX_NUM_PLAYERS = 20

# SPL 团队颜色定义（与上半部分统一）
TEAM_BLUE = 0
TEAM_RED = 1
TEAM_YELLOW = 2
TEAM_BLACK = 3
TEAM_WHITE = 4
TEAM_GREEN = 5
TEAM_ORANGE = 6
TEAM_PURPLE = 7
TEAM_BROWN = 8
TEAM_GRAY = 9

# 颜色ID到名称的映射
TEAM_COLOR_NAMES = {
    TEAM_BLUE: "BLUE",
    TEAM_RED: "RED",
    TEAM_YELLOW: "YELLOW",
    TEAM_BLACK: "BLACK",
    TEAM_WHITE: "WHITE",
    TEAM_GREEN: "GREEN",
    TEAM_ORANGE: "ORANGE",
    TEAM_PURPLE: "PURPLE",
    TEAM_BROWN: "BROWN",
    TEAM_GRAY: "GRAY"
}

# 犯规类型定义（与上半部分统一）
PENALTY_NONE = 0
PENALTY_SPL_ILLEGAL_BALL_CONTACT = 1
PENALTY_SPL_PLAYER_PUSHING = 2
PENALTY_SPL_ILLEGAL_MOTION_IN_SET = 3
PENALTY_SPL_INACTIVE_PLAYER = 4
PENALTY_SPL_ILLEGAL_POSITION = 5
PENALTY_SPL_LEAVING_THE_FIELD = 6
PENALTY_SPL_REQUEST_FOR_PICKUP = 7
PENALTY_SPL_LOCAL_GAME_STUCK = 8
PENALTY_SPL_ILLEGAL_POSITION_IN_SET = 9
PENALTY_SPL_PLAYER_STANCE = 10
PENALTY_SPL_ILLEGAL_MOTION_IN_STANDBY = 11
PENALTY_SUBSTITUTE = 14
PENALTY_MANUAL = 15

# 犯规ID到名称的映射
PENALTY_NAMES = {
    PENALTY_NONE: "NONE",
    PENALTY_SPL_ILLEGAL_BALL_CONTACT: "ILLEGAL_BALL_CONTACT",
    PENALTY_SPL_PLAYER_PUSHING: "PLAYER_PUSHING",
    PENALTY_SPL_ILLEGAL_MOTION_IN_SET: "ILLEGAL_MOTION_IN_SET",
    PENALTY_SPL_INACTIVE_PLAYER: "INACTIVE_PLAYER",
    PENALTY_SPL_ILLEGAL_POSITION: "ILLEGAL_POSITION",
    PENALTY_SPL_LEAVING_THE_FIELD: "LEAVING_THE_FIELD",
    PENALTY_SPL_REQUEST_FOR_PICKUP: "REQUEST_FOR_PICKUP",
    PENALTY_SPL_LOCAL_GAME_STUCK: "LOCAL_GAME_STUCK",
    PENALTY_SPL_ILLEGAL_POSITION_IN_SET: "ILLEGAL_POSITION_IN_SET",
    PENALTY_SPL_PLAYER_STANCE: "PLAYER_STANCE",
    PENALTY_SPL_ILLEGAL_MOTION_IN_STANDBY: "ILLEGAL_MOTION_IN_STANDBY",
    PENALTY_SUBSTITUTE: "SUBSTITUTE",
    PENALTY_MANUAL: "MANUAL"
}

# 比赛状态定义（与上半部分统一）
STATE_INITIAL = 0
STATE_READY = 1
STATE_SET = 2
STATE_PLAYING = 3
STATE_FINISHED = 4
STATE_STANDBY = 5

# 适配新协议的数据结构定义
RobotInfo = "robot_info" / Struct(
    "penalty" / Byte,                # 球员犯规状态
    "secs_till_unpenalized" / Byte   # 预计犯规结束时间
)

TeamInfo = "team" / Struct(
    "team_number" / Byte,            # 队伍编号
    "field_player_colour" / Byte,    # 场上球员颜色
    "goalkeeper_colour" / Byte,      # 守门员颜色
    "goalkeeper" / Byte,             # 守门员编号(1-MAX_NUM_PLAYERS)
    "score" / Byte,                  # 队伍得分
    "penalty_shot" / Byte,           # 点球计数
    "single_shots" / Int16ul,        # 点球成功位标识
    "message_budget" / Int16ul,      # 剩余可发送消息数
    "players" / Array(MAX_NUM_PLAYERS, RobotInfo)  # 球员数组
)

GameState = "gamedata" / Struct(
    "header" / Const(b"RGme"),                # 协议头标识
    "version" / Const(18, Byte),              # 协议版本号
    "packet_number" / Byte,                   # 数据包序号
    "players_per_team" / Byte,                # 每队球员数
    "competition_phase" / Byte,               # 比赛阶段
    "competition_type" / Byte,                # 比赛类型
    "game_phase" / Byte,                      # 比赛阶段
    "state" / Enum(Byte,                       # 比赛状态
        STATE_INITIAL=0,
        STATE_READY=1,
        STATE_SET=2,
        STATE_PLAYING=3,
        STATE_FINISHED=4,
        STATE_STANDBY=5
    ),
    "set_play" / Byte,                        # 定位球类型
    "first_half" / Byte,                      # 是否上半场(1=是)
    "kicking_team" / Byte,                    # 开球/发球方队伍
    "secs_remaining" / Int16sl,               # 半场剩余时间
    "secondary_time" / Int16sl,               # 次级计时
    "teams" / Array(2, TeamInfo)              # 两队信息
)

GAME_CONTROLLER_RESPONSE_VERSION = 2

ReturnData = Struct(
    "header" / Const(b"RGrt"),
    "version" / Const(2, Byte),
    "team" / Byte,
    "player" / Byte,
    "message" / Byte,
)


# 裁判盒接收器类
class Receiver:
    def __init__(self, team, player, goal_keeper=False, debug=True):
        self.ip = "0.0.0.0"  # 本地ip
        self.listen_port = 3838  # 本地端口
        self.answer_port = 3939  # 服务器端口

        self.debug = debug
        self.team = team  # 队伍序号
        self.player = player  # 球员序号(0-19，根据MAX_NUM_PLAYERS调整)
        self.game_state = None  # 比赛状态
        self.kicking_team = None  # 开球方
        self.data = None  # 接收数据
        self.player_info = None  # 球员信息
        self.penalized_time = 0  # 罚时倒计时
        self.team_color = None  # 队伍颜色
        self.team_color_name = None  # 队伍颜色名称
        self.player_penalty_name = None  # 球员犯规名称

        self.man_penalize = True  # 手动犯规状态
        self.is_goalkeeper = goal_keeper  # 是否守门员
        self.peer = None  # 服务器地址

        self.socket1 = socket.socket(
            socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP
        )
        self.socket1.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.addr = (self.ip, self.listen_port)
        self.socket1.bind(self.addr)
        self.socket1.settimeout(2)

        self.t = threading.Thread(target=self.receive, daemon=True)
        self.t.start()

    def receive_once(self):
        try:
            # 使用新结构体的大小作为接收缓冲区长度
            data, self.peer = self.socket1.recvfrom(GameState.sizeof())
            self.data = GameState.parse(data)
            
            # 解析比赛状态
            self.game_state = self.data.state
            self.kicking_team = self.data.kicking_team
            
            # 查找当前队伍索引
            team_index = 0
            if self.data.teams[1].team_number == self.team:
                team_index = 1
                
            # 获取球员信息
            self.player_info = self.data.teams[team_index].players[self.player]
            self.penalized_time = self.player_info.secs_till_unpenalized
            self.team_color = self.data.teams[team_index].field_player_colour
            
            # 将颜色ID转换为名称
            self.team_color_name = TEAM_COLOR_NAMES.get(self.team_color, "UNKNOWN")
            
            # 将犯规ID转换为名称
            self.player_penalty_name = PENALTY_NAMES.get(self.player_info.penalty, "UNKNOWN")
            
        except AssertionError as ae:
            logging.error(ae.message)
        except socket.timeout:
            pass
        except ConstError:
            pass
        except Exception as e:
            logging.exception(e)

    def receive(self):
        self.initialize()
        while True:
            self.receive_once()
            if self.debug:
                self.debug_print()

    def debug_print(self):
        print("-----------message-----------")
        print(f"Game State: {self.game_state}")
        print(f"Kicking Team: {self.kicking_team}")
        print(f"Penalized Time: {self.penalized_time}s")
        print(f"Team Color: {self.team_color_name}")
        print(f"Player Penalty: {self.player_penalty_name}")

    def initialize(self):
        while True:
            self.receive_once()
            if self.peer:
                for i in range(5):
                    self.answer_to_gamecontroller()
                print("initialized, break")
                break

    def answer_to_gamecontroller(self):
        return_message = 0 if self.man_penalize else 2
        if self.is_goalkeeper:
            return_message = 3
            
        data = Container(
            header=b"RGrt",
            version=GAME_CONTROLLER_RESPONSE_VERSION,
            team=self.team,
            player=self.player,
            message=return_message,
        )
        destination = (self.peer[0], self.answer_port)
        self.socket1.sendto(ReturnData.build(data), destination)


if __name__ == "__main__":
    # 注意：球员序号范围应与MAX_NUM_PLAYERS(20)匹配
    receiver = Receiver(team=70, player=3, goal_keeper=False, debug=True)
    receiver.receive()