import socket
import threading
import struct
import logging
import time
from construct import ConstError, Byte, Struct, Enum, Bytes, Const, Array, Int16ul

Short = Int16ul

FieldPlayerColour = Enum(
    Byte,
    CYAN=0,
    MAGENTA=1,
    BLUE=0,
    RED=1,
    YELLOW=2,
    BLACK=3,
    WHITE=4,
    GREEN=5,
    ORANGE=6,
    PURPLE=7,
    BROWN=8,
    GRAY=9,
)

GameStateEnum = Enum(
    Byte, STATE_INITIAL=0, STATE_READY=1, STATE_SET=2, STATE_PLAYING=3, STATE_FINISHED=4
)

SecondaryStateEnum = Enum(
    Byte,
    STATE2_NORMAL=0,
    STATE2_PENALTYSHOOT=1,
    STATE2_OVERTIME=2,
    STATE2_TIMEOUT=3,
    STATE2_DIRECT_FREEKICK=4,
    STATE2_INDIRECT_FREEKICK=5,
    STATE2_PENALTYKICK=6,
    STATE2_CORNER_KICK=7,
    STATE2_GOAL_KICK=8,
    STATE2_THROW_IN=9,
    DROPBALL=128,
    UNKNOWN=255,
)

RobotInfo = Struct(
    "penalty" / Byte,
    "secs_till_unpenalized" / Byte,
    "number_of_warnings" / Byte,
    "number_of_yellow_cards" / Byte,
    "number_of_red_cards" / Byte,
    "goalkeeper" / Byte,
)

TeamInfo = Struct(
    "team_number" / Byte,
    "field_player_colour" / FieldPlayerColour,
    "score" / Byte,
    "penalty_shot" / Byte,
    "single_shots" / Short,
    "coach_sequence" / Byte,
    "coach_message" / Bytes(253),
    "coach" / RobotInfo,
    "players" / Array(11, RobotInfo),
)

GameState = Struct(
    "header" / Const(b"RGme"),
    "version" / Const(12, Short),
    "packet_number" / Byte,
    "players_per_team" / Byte,
    "game_type" / Byte,
    "state" / GameStateEnum,
    "first_half" / Byte,
    "kick_off_team" / Byte,
    "secondary_state" / SecondaryStateEnum,
    "secondary_state_info" / Bytes(4),
    "drop_in_team" / Byte,
    "drop_in_time" / Short,
    "seconds_remaining" / Short,
    "secondary_seconds_remaining" / Short,
    "teams" / Array(2, TeamInfo),
)


class Receiver:
    def __init__(
        self, team=12, player=0, debug=False, logger=logging.getLogger(__name__)
    ):
        # 基本设置
        self.logger = logger
        self.team = team  # 队伍序号（0或1）
        self.player = player  # 球员序号（0-10，上场只有4个）
        self.debug = debug

        # 网络设置
        self.ip = "0.0.0.0"
        self.listen_port = 3838
        self.answer_port = 3939
        self.peer = None

        # 比赛状态
        self.game_state = None
        self.kick_off = None
        self.data = None
        self.player_info = None
        self.penalty = 0
        self.team_id = None
        self.corner_kick = None
        self.throw_in = None
        self.goal_kick = None
        self.can_kick = True
        self.t1 = 0

        # 创建socket
        self.socket1 = socket.socket(
            socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP
        )
        self.socket1.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket1.bind((self.ip, self.listen_port))
        self.socket1.settimeout(1)

        # 启动接收线程
        threading.Thread(target=self.receive, daemon=not self.debug).start()

    def receive_once(self):
        """接收一次消息并解析"""
        try:
            data, self.peer = self.socket1.recvfrom(GameState.sizeof())
            self.data = GameState.parse(data)
            # 确定队伍ID
            if self.data.teams[0].team_number == self.team:
                self.team_id = 0
            elif self.data.teams[1].team_number == self.team:
                self.team_id = 1
            else:
                raise AssertionError("Team number does not match!")
                return
            self.game_state = self.data.state
            self.kick_off = self.data.kick_off_team == self.team

            self.player_info = self.data.teams[self.team_id].players[self.player]
            self.penalty = self.player_info.penalty

            self.corner_kick = (
                self.data.secondary_state == SecondaryStateEnum.STATE2_CORNER_KICK
            )
            self.throw_in = (
                self.data.secondary_state == SecondaryStateEnum.STATE2_THROW_IN
            )
            self.goal_kick = (
                self.data.secondary_state == SecondaryStateEnum.STATE2_GOAL_KICK
            )
            self.secondary_state = self.data.secondary_state
            self.secondary_state_info = self.data.secondary_state_info
            self.secondary_seconds_remaining = self.data.secondary_seconds_remaining
            if self.corner_kick or self.throw_in or self.goal_kick:
                self.can_kick = self.data.secondary_state_info[0] == self.team
                print("secondary_state_info[0]:", self.data.secondary_state_info[0])
                self.t1 = time.time()
            elif time.time() - self.t1 > 10:
                self.can_kick = True

        except socket.timeout:
            logging.debug("Socket timeout")
        except ConstError:
            logging.error("ConstError")
        except Exception as e:
            logging.error("Exception: %s", e)

    def receive(self):
        while True:
            self.receive_once()
            self.send_status_to_gamecontroller()
            if self.debug:
                self.debug_print()

    def debug_print(self):
        print("-----------message-----------")
        print("Game State:", self.game_state)
        print("Kick Off:", self.kick_off)
        print("Penalty:", self.penalty)
        print("Player Info:", self.player_info)
        print("secondary_state:", self.secondary_state)
        print("secondary_state_info:", self.data.secondary_state_info)

    def send_status_to_gamecontroller(self):
        header = b"RGrt"
        version = 2
        player_num = self.player + 1
        message = 2
        packed = struct.pack("<4sBBBB", header, version, self.team, player_num, message)
        dest_ip = self.peer[0] if self.peer else "127.0.0.1"
        dest = (dest_ip, self.answer_port)
        self.socket1.sendto(packed, dest)


if __name__ == "__main__":
    receive = Receiver(team=12, player=0, debug=True)
