import socket
import threading
import struct
import time
from construct import ConstError, Byte, Struct, Enum, Bytes, Const, Array, Int16ul
import logging
from typing import Optional, Dict

# 设置日志格式
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)

Short = Int16ul

# 颜色枚举
FieldPlayerColour = Enum(
    Byte,
    # SPL定义
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
    # HL定义
    CYAN=0,
    MAGENTA=1,
    UNKNOWN=255,
)

# 比赛状态枚举
GameStateEnum = Enum(
    Byte, 
    STATE_INITIAL=0, 
    STATE_READY=1, 
    STATE_SET=2, 
    STATE_PLAYING=3, 
    STATE_FINISHED=4
)

# 次要状态枚举
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
    DROPBALL=255,  # 头文件中定义为255
    UNKNOWN=255,
)

# 机器人信息结构（匹配HlRobotInfo）
RobotInfo = Struct(
    "penalty" / Byte,               # 球员惩罚状态
    "secs_till_unpenalized" / Byte,  # 预计解除惩罚时间
    "number_of_warnings" / Byte,    # 警告次数
    "yellow_card_count" / Byte,     # 黄牌数量
    "red_card_count" / Byte,        # 红牌数量
    "goalkeeper" / Byte,            # 是否为守门员
)

# 队伍信息结构（匹配HlTeamInfo）
TeamInfo = Struct(
    "team_number" / Byte,            # 队伍编号
    "field_player_colour" / FieldPlayerColour,  # 队伍颜色
    "score" / Byte,                  # 队伍得分
    "penalty_shot" / Byte,           # 点球计数
    "single_shots" / Short,          # 点球成功位掩码
    "coach_sequence" / Byte,         # 教练消息序列号
    "coach_message" / Bytes(253),    # 教练消息内容
    "coach" / RobotInfo,             # 教练机器人信息
    "players" / Array(11, RobotInfo), # 球员信息数组
)

# 比赛状态结构（匹配RoboCupGameControlData）
GameState = Struct(
    "header" / Const(b"RGme"),       # 结构头
    "version" / Const(12, Short),    # 结构版本
    "packet_number" / Byte,          # 数据包编号
    "players_per_team" / Byte,       # 每队球员数
    "game_type" / Byte,              # 比赛类型
    "state" / GameStateEnum,         # 比赛状态
    "first_half" / Byte,             # 是否为上半场
    "kick_off_team" / Byte,          # 开球队伍
    "secondary_state" / SecondaryStateEnum,  # 次要状态
    "secondary_state_info" / Bytes(4), # 次要状态额外信息
    "drop_in_team" / Byte,           # 导致最后一次坠球的队伍
    "drop_in_time" / Short,          # 自上次坠球以来的秒数
    "seconds_remaining" / Short,     # 半场剩余秒数
    "secondary_seconds_remaining" / Short,  # 次要时间剩余秒数
    "teams" / Array(2, TeamInfo),    # 两队信息
)

# 修改后的比赛控制返回数据结构（新增is_goalkeeper字段）
ReturnData = Struct(
    "header" / Const(b"RGrt"),
    "version" / Const(3, Byte),      # 升级版本号以支持新字段
    "team" / Byte,
    "player" / Byte,
    "message" / Byte,
    "is_goalkeeper" / Byte,          # 新增：是否为守门员（1=是，0=否）
)

class Receiver:
    """比赛状态接收器类，负责接收和处理比赛控制器发送的信息"""
    
    def __init__(self, team=12, player=0, is_goalkeeper=False, debug=False, logger=None):
        # 基本设置
        self.logger = logger or logging.getLogger(__name__)
        self.team = team  # 队伍序号
        self.player = player  # 球员序号(0-10)
        self.is_goalkeeper = is_goalkeeper  # 是否为守门员
        self.debug = debug
        self.running = False  # 控制线程运行的标志
        
        # 网络设置（使用头文件定义的端口）
        self.ip = "0.0.0.0"
        self.listen_port = 3838
        self.answer_port = 3939
        self.peer = None
        
        # 比赛状态
        self._game_state = None
        self._kick_off = None
        self._data = None
        self._player_info = None
        self._penalty = 0
        self._team_id = None
        
        # 创建socket
        self.socket = socket.socket(
            socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP
        )
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind((self.ip, self.listen_port))
        self.socket.settimeout(1)
        
        # 启动接收线程
        self.receive_thread = threading.Thread(target=self._receive_loop, daemon=True)
        
    def start(self):
        """启动接收器线程"""
        if not self.receive_thread.is_alive():
            self.running = True
            self.receive_thread.start()
            self.logger.info(f"接收器已启动，监听端口: {self.listen_port}")
        else:
            self.logger.warning("接收器线程已经在运行")
            
    def stop(self):
        """停止接收器线程并清理资源"""
        self.running = False
        # 关闭socket以中断可能的阻塞调用
        try:
            self.socket.close()
            self.logger.info("接收器已停止")
        except Exception as e:
            self.logger.error(f"关闭socket时出错: {e}")
            
    def _receive_once(self):
        """接收一次消息并解析"""
        try:
            data, self.peer = self.socket.recvfrom(GameState.sizeof())
            parsed_data = GameState.parse(data)
            
            # 确定队伍ID
            team_ids = [parsed_data.teams[0].team_number, parsed_data.teams[1].team_number]
            if self.team in team_ids:
                self._team_id = team_ids.index(self.team)
            else:
                raise ValueError(f"未找到队伍号为 {self.team} 的队伍，可用队伍: {team_ids}")
            
            # 更新比赛状态
            self._data = parsed_data
            self._game_state = parsed_data.state
            self._kick_off = parsed_data.kick_off_team == self.team
            
            if self._team_id is not None:
                self._player_info = parsed_data.teams[self._team_id].players[self.player]
                self._penalty = self._player_info.penalty
                
            self.logger.debug(f"收到比赛状态包: {self._game_state}, 队伍: {self.team}")
                
        except socket.timeout:
            # 超时是正常现象，降低日志级别
            if self.debug:
                self.logger.debug("Socket超时")
        except ConstError as e:
            self.logger.error(f"解析数据包失败(ConstError): {e}，数据长度: {len(data)}")
        except ValueError as e:
            self.logger.error(f"数据处理错误: {e}")
        except Exception as e:
            self.logger.error(f"接收或解析数据时发生未知错误: {e}", exc_info=self.debug)
    
    def _receive_loop(self):
        """接收循环，持续接收和处理数据"""
        while self.running:
            try:
                self._receive_once()
                if self.peer:  # 只有收到数据后才发送状态
                    self._send_status_to_gamecontroller()
                if self.debug:
                    self._debug_print()
                # 添加短暂休眠，降低CPU使用率
                time.sleep(0.01)
            except Exception as e:
                self.logger.error(f"接收循环异常: {e}", exc_info=True)
                # 发生严重错误时短暂休眠，避免CPU占用过高
                time.sleep(1)
    
    def _debug_print(self):
        """调试信息打印"""
        if self._game_state:
            print(f"[DEBUG] 游戏状态: {self._game_state}")
            print(f"[DEBUG] 队伍ID: {self._team_id}, 开球权: {self._kick_off}")
            print(f"[DEBUG] 球员惩罚: {self._penalty}")
            print(f"[DEBUG] 球员信息: {self.player_info}")
            print(f"[DEBUG] 二级状态: {self._data.secondary_state}")
            print(f"[DEBUG] 是否为守门员: {self.is_goalkeeper}")
    
    def _send_status_to_gamecontroller(self):
        """向比赛控制器发送状态信息（包含守门员状态）"""
        try:
            # 构建返回数据结构
            return_data = {
                "header": b"RGrt",
                "version": 3,  # 使用新版本以支持is_goalkeeper字段
                "team": self.team,
                "player": self.player + 1,  # 球员编号从1开始
                "message": 2,  # 状态消息类型（ALIVE）
                "is_goalkeeper": 1 if self.is_goalkeeper else 0,  # 发送守门员状态
            }
            
            # 使用construct打包数据
            packed = ReturnData.build(return_data)
            
            dest_ip = self.peer[0] if self.peer else "127.0.0.1"
            dest = (dest_ip, self.answer_port)
            
            self.socket.sendto(packed, dest)
            self.logger.debug(f"已发送状态到 {dest}，消息类型: {return_data['message']}，守门员: {return_data['is_goalkeeper']}")
        except Exception as e:
            self.logger.error(f"发送状态消息失败: {e}")
    
    @property
    def game_state(self) -> Optional[str]:
        """获取当前比赛状态"""
        return self._game_state
    
    @property
    def kick_off(self) -> Optional[bool]:
        """获取当前队伍是否拥有开球权"""
        return self._kick_off
    
    @property
    def penalty(self) -> int:
        """获取当前球员的惩罚状态"""
        return self._penalty
    
    @property
    def player_info(self) -> Optional[Dict]:
        """获取当前球员的详细信息（转换为字典格式）"""
        if self._player_info:
            return {
                "penalty": self._player_info.penalty,
                "secs_till_unpenalized": self._player_info.secs_till_unpenalized,
                "warnings": self._player_info.number_of_warnings,
                "yellow_cards": self._player_info.yellow_card_count,
                "red_cards": self._player_info.red_card_count,
                "is_goalkeeper": self._player_info.goalkeeper == 1
            }
        return None

    def set_goalkeeper_status(self, is_goalkeeper: bool):
        """动态设置守门员状态"""
        self.is_goalkeeper = is_goalkeeper
        self.logger.info(f"已设置守门员状态为: {is_goalkeeper}")


if __name__ == "__main__":
    try:
        # 初始化时指定是否为守门员
        receiver = Receiver(team=12, player=0, is_goalkeeper=False, debug=True)
        receiver.start()
        
        print("接收器已启动，按Ctrl+C停止...")
        # 保持程序运行
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n程序被用户中断")
    finally:
        if 'receiver' in locals() and receiver.running:
            receiver.stop()