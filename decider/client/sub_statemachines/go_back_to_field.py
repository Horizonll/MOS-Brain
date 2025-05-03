import time
from transitions import Machine
import numpy as np


class GoBackToFieldStateMachine:
    def __init__(self, agent, aim_x=3300, aim_y=500):
        """
        初始化返回场地状态机

        :param agent: 机器人代理对象，包含机器人的各种状态和控制方法
        :param aim_x: 目标位置的 x 坐标
        :param aim_y: 目标位置的 y 坐标
        """
        self.agent = agent
        self._config = self.agent.get_config()
        self.read_params()  # 读取配置参数
        
        self.last_rotate = 1  # 上次旋转的方向, -1为右，1为左
        self.aim_yaw_last_rotate = 1  # 到达目标位置后，调整yaw的方向避免180度的情况抽搐
        self.last_arrive_time = time.time() - 5 * 60  # 初始化为5分钟前
        
        # 定义状态机的状态
        self.states = [
            "moving_to_target",       # 向目标位置移动
            "coarse_yaw_adjusting",   # 粗略偏航角调整
            "fine_yaw_adjusting",     # 精细偏航角调整
            "yaw_adjusting",          # 到达目标位置后的最终操作
            "arrived_at_target",      # 到达目标位置
        ]
        
        # 定义状态机的状态转移规则
        self.transitions = [
            {
                "trigger": "update_status",
                "source": "yaw_adjusting",
                "dest": "arrived_at_target",
                "conditions": "good_yaw",
                "after": "arrived_stop_moving",
            },
            {
                "trigger": "update_status",
                "source": ["coarse_yaw_adjusting", "fine_yaw_adjusting", "moving_to_target", "yaw_adjusting"],
                "dest": "yaw_adjusting",
                "conditions": "good_position",
                "after": "adjust_yaw",
            },
            {
                "trigger": "update_status",
                "source": ["arrived_at_target"],
                "dest": "coarse_yaw_adjusting",
                "conditions": "not_arrived",
            },
            {
                "trigger": "update_status",
                "source": ["moving_to_target", "coarse_yaw_adjusting"],
                "dest": "coarse_yaw_adjusting",
                "conditions": "need_coarse_yaw_adjustment",
                "after": "coarse_yaw_adjust",
            },
            {
                "trigger": "update_status",
                "source": ["coarse_yaw_adjusting", "fine_yaw_adjusting", "moving_to_target"],
                "dest": "moving_to_target",
                "conditions": "dont_need_coarse_yaw_adjustment",
                "after": "move_forward",
            },
            {
                "trigger": "update_status",
                "source": ["coarse_yaw_adjusting", "fine_yaw_adjusting"],
                "dest": "fine_yaw_adjusting",
                "conditions": "need_fine_yaw_adjustment",
                "after": "fine_yaw_adjust",
            },
        ]
        
        # 初始化状态机
        self.machine = Machine(
            model=self,
            states=self.states,
            initial="moving_to_target",
            transitions=self.transitions,
        )
        print(f"[Go Back to Field FSM] Initialized. Starting state: {self.state}")

    def read_params(self):
        """从配置中读取所有参数"""
        self.min_dist = self._config.get("go_back_to_field").get("min_dist_mm", 200)  # 到达目标位置的最小距离阈值（mm）
        self.coarse_yaw_threshold_degree = self._config.get("go_back_to_field").get("coarse_yaw_threshold_degree", 30)  # 粗略偏航调整阈值（度）
        self.fine_yaw_start_threshold_degree = self._config.get("go_back_to_field").get("fine_yaw_start_threshold_degree", 20)  # 精细偏航开始阈值（度）
        self.fine_yaw_end_threshold_degree = self._config.get("go_back_to_field").get("fine_yaw_end_threshold_degree", 5)  # 精细偏航结束阈值（度）
        self.good_yaw_threshold_degree = self._config.get("go_back_to_field").get("good_yaw_threshold_degree", 10)  # 良好偏航阈值（度）
        self.walk_vel_x = self._config.get("go_back_to_field").get("walk_vel_x", 0.3)  # 前进速度（m/s）
        self.walk_vel_theta = self._config.get("go_back_to_field").get("walk_vel_theta", 0.3)  # 旋转速度（rad/s）
        self.arrive_trust_time = self._config.get("go_back_to_field").get("arrive_trust_time", 5)  # 到达目标后信任时间（秒）
        self.aim_x = self._config.get("go_back_to_field").get("aim_x", 0)  # 目标位置的 x 坐标（mm）
        self.aim_y = self._config.get("go_back_to_field").get("aim_y", 1000)  # 目标位置的 y 坐标（mm）
        self.aim_yaw = self._config.get("go_back_to_field").get("aim_yaw", 0)  # 目标位置的偏航角（度）

    def need_coarse_yaw_adjustment(self):
        """检查是否需要进行粗略偏航角调整"""
        result = (self.go_back_to_field_dist > self.min_dist and 
                  abs(self.go_back_to_field_yaw_diff) > self.coarse_yaw_threshold_degree)
        print(f"[Go Back to Field] Need coarse yaw adjustment? {'Yes' if result else 'No'}")
        return result

    def need_fine_yaw_adjustment(self):
        """检查是否需要进行精细偏航角调整"""
        result = (self.min_dist <= self.go_back_to_field_dist < 3 * self.min_dist and 
                  abs(self.go_back_to_field_yaw_diff) > self.fine_yaw_end_threshold_degree and 
                  abs(self.go_back_to_field_yaw_diff) <= self.fine_yaw_start_threshold_degree)
        print(f"[Go Back to Field] Need fine yaw adjustment? {'Yes' if result else 'No'}")
        return result
    
    def dont_need_coarse_yaw_adjustment(self):
        """检查是否不需要进行粗略偏航角调整"""
        result = not self.need_coarse_yaw_adjustment()
        print(f"[Go Back to Field] Don't need coarse yaw adjustment? {'Yes' if result else 'No'}")
        return result

    def good_position(self):
        """检查是否到达目标位置"""
        print(f"[Go Back to Field] go_back_to_field_dist: {self.go_back_to_field_dist}, min_dist: {self.min_dist}")
        result = self.go_back_to_field_dist < self.min_dist
        print(f"[Go Back to Field] Arrived at target? {'Yes' if result else 'No'}")
        return result

    def run(self, aim_x=0, aim_y=0, aim_yaw=0):
        """状态机的主运行函数，控制机器人返回场地的整个流程"""
        # if self.agent.receiver.game_state != 'STATE_READY':
        #    self.agent.stop(0.5)
        #    return
        if self.state != "arrived_at_target":
            self.agent.look_at([0.0, 0.0])
        else:
            self.agent.look_at([None, None])
        
        self.agent.is_going_back_to_field = True
        print("[Go Back to Field FSM] Starting to go back to field...")
        self.aim_x = self.agent.get_command().get('data').get('aim_x', self.aim_x)
        self.aim_y = self.agent.get_command().get('data').get('aim_y', self.aim_y)
        self.aim_yaw = self.agent.get_command().get('data').get('aim_yaw', self.aim_yaw)
        self.update_go_back_to_field_status()
        print(f"\n[Go Back to Field FSM] Current state: {self.state}")
        print("[Go Back to Field FSM] Triggering 'update_status' transition")
        self.machine.model.trigger("update_status")

    def update_go_back_to_field_status(self):
        """更新返回场地的状态参数"""
        self.pos_x = self.agent.get_self_pos()[0]
        self.pos_y = self.agent.get_self_pos()[1]
        pos_yaw = self.agent.get_self_yaw()
        self.go_back_to_field_dist = np.sqrt((self.pos_x - self.aim_x) ** 2 + (self.pos_y - self.aim_y) ** 2)
        self.go_back_to_field_dir = np.arctan2(self.pos_x - self.aim_x, self.aim_y - self.pos_y)  # 目标方向（弧度）
        self.go_back_to_field_yaw_diff = np.degrees(
            np.arctan2(
                np.sin(self.go_back_to_field_dir - np.radians(pos_yaw)),
                np.cos(self.go_back_to_field_dir - np.radians(pos_yaw)),
            )
        )
        print(f"[Go Back to Field] aim_x: {self.aim_x}, aim_y: {self.aim_y}, aim_yaw: {self.aim_yaw}")
        print(f"[Go Back to Field] pos_x: {self.pos_x}, pos_y: {self.pos_y}")
        print(f"[Go Back to Field] Updated status: dist: {self.go_back_to_field_dist:.1f}, yaw_diff: {self.go_back_to_field_yaw_diff:.1f}°")

    def move_forward(self):
        """控制机器人向前移动"""
        print("[Go Back to Field] Moving forward...")
        self.agent.cmd_vel(self.walk_vel_x, 0, 0)

    def coarse_yaw_adjust(self):
        """进行粗略偏航角调整"""
        print("[Go Back to Field] Starting coarse yaw adjustment...")
        sgn = 1 if self.go_back_to_field_yaw_diff > 0 else -1
        
        if self.go_back_to_field_dist < self.min_dist:
            return
        
        # 大角度调整（超过粗略阈值）
        if abs(self.go_back_to_field_yaw_diff) > self.coarse_yaw_threshold_degree:
            print(f"[Go Back to Field] Large yaw error ({self.go_back_to_field_yaw_diff:.1f}°), rotating {'' if sgn>0 else 'right'}...")
            self.agent.cmd_vel(0, 0, sgn * self.walk_vel_theta)
            self.last_rotate = sgn
            time.sleep(0.2)

    def good_yaw(self):
        """检查是否朝向正确（目标yaw在允许范围内）"""
        aim_yaw_diff = self.aim_yaw - self.agent.get_self_yaw()
        result = abs(aim_yaw_diff) < self.good_yaw_threshold_degree
        print(f"[Go Back to Field] Good yaw? {'Yes' if result else 'No'} (diff: {aim_yaw_diff:.1f}°)")
        return result

    def fine_yaw_adjust(self):
        """进行精细偏航角调整（中低角度范围）"""
        print("[Go Back to Field] Starting fine yaw adjustment...")
        sgn = 1 if self.go_back_to_field_yaw_diff > 0 else -1
        
        if self.go_back_to_field_dist < self.min_dist:
            return
        
        # 中等角度调整（精细阈值范围内）
        if self.fine_yaw_end_threshold_degree < abs(self.go_back_to_field_yaw_diff) <= self.fine_yaw_start_threshold_degree:
            print(f"[Go Back to Field] Medium yaw error ({self.go_back_to_field_yaw_diff:.1f}°), rotating {'' if sgn>0 else 'right'} slowly...")
            self.agent.cmd_vel(0, 0, sgn * (self.walk_vel_theta))  # 较慢的旋转速度
            self.last_rotate = sgn
            time.sleep(0.2)

    def adjust_yaw(self):
        """到达目标位置后执行的操作，包括调整朝向和准备开始游戏"""
        print("[Go Back to Field] Arrived at target. Performing yaw adjust...")
        self.agent.cmd_vel(0, 0, 0)
        
        aim_yaw_diff = self.aim_yaw - self.agent.get_self_yaw()
        if abs(aim_yaw_diff) > 160:  # 处理180度附近的环绕问题
            print("[Go Back to Field] Correcting large yaw wrap-around...")
            self.agent.cmd_vel(0, 0, self.aim_yaw_last_rotate * self.walk_vel_theta)
        elif abs(aim_yaw_diff) > self.good_yaw_threshold_degree:  # 小角度精细调整
            sgn = 1 if aim_yaw_diff > 0 else -1
            print(f"[Go Back to Field] Final yaw adjustment ({aim_yaw_diff:.1f}°)...")
            self.agent.cmd_vel(0, 0, sgn * (self.walk_vel_theta))  # 最慢旋转速度
            self.aim_yaw_last_rotate = sgn
        else:  # 达到良好状态
            self.agent.cmd_vel(0, 0, 0)
            print("[Go Back to Field] Finished going back to field. Ready to play.")
            time.sleep(0.5)
            self.agent.is_going_back_to_field = False
            self.last_arrive_time = time.time()
            print("[Go Back to Field FSM] Arrived at target!")

    def not_arrived(self):
        """检查是否未到达目标位置（考虑短暂信任时间避免抖动）"""
        if time.time() - self.last_arrive_time < self.arrive_trust_time:
            print("[Go Back to Field] Trusting recent arrival, not rechecking.")
            return False
        
        result = not (self.go_back_to_field_dist < self.min_dist * 1.5 and self.good_yaw())
        print(f"[Go Back to Field] Not arrived? {'Yes' if result else 'No'} (dist: {self.go_back_to_field_dist:.1f})")
        return result

    def arrived_stop_moving(self):
        """到达目标位置后停止移动"""
        print("[Go Back to Field] Stopping moving...")
        self.agent.stop(0.5)
        self.update_go_back_to_field_status()
        self.last_arrive_time = time.time()
        self.agent.look_at([None, None])
