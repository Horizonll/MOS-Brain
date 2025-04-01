import time
from transitions import Machine
import numpy as np


class GoBackToFieldStateMachine:
    def __init__(self, agent, aim_x, aim_y, min_dist=1000):
        """
        初始化返回场地状态机

        :param agent: 机器人代理对象，包含机器人的各种状态和控制方法
        :param aim_x: 目标位置的 x 坐标
        :param aim_y: 目标位置的 y 坐标
        :param min_dist: 到达目标位置的最小距离阈值，默认为 300
        """
        self.agent = agent
        self._config = self.agent.get_config()
        self.aim_x = aim_x
        self.aim_y = aim_y
        self.min_dist = min_dist
        self.last_rotate = 1 # 上次旋转的方向,-1为右，1为左
        # 定义状态机的状态
        self.states = [
            "moving_to_target",  # 向目标位置移动
            "coarse_yaw_adjusting",  # 粗略偏航角调整
            "fine_yaw_adjusting",  # 精细偏航角调整
            "yaw_adjusting",  # 到达目标位置后的最终操作
            "arrived_at_target",  # 到达目标位置
        ]
        # 定义状态机的状态转移规则
        self.transitions = [
            {
                "trigger": "update_status",
                "source": ["coarse_yaw_adjusting", "fine_yaw_adjusting", "moving_to_target", "arrived_at_target", "yaw_adjusting"],
                "dest": "yaw_adjusting",
                "conditions": "good_position",
                "after": "adjust_yaw",
            },
            {
                "trigger": "update_status",
                "source": "yaw_adjusting",
                "dest": "arrived_at_target",
                "conditions": "good_yaw",
            },
            {
                "trigger": "update_status",
                "source": ["moving_to_target", "coarse_yaw_adjusting", "arrived_at_target"],
                "dest": "coarse_yaw_adjusting",
                "conditions": "need_coarse_yaw_adjustment",
                "before": "coarse_yaw_adjust",
            },
            {
                "trigger": "update_status",
                "source": ["coarse_yaw_adjusting", "fine_yaw_adjusting", "moving_to_target", "arrived_at_target"],
                "dest": "moving_to_target",
                "conditions": "dont_need_coarse_yaw_adjustment",
                "before": "move_forward",
            },
            {
                "trigger": "update_status",
                "source": ["coarse_yaw_adjusting", "fine_yaw_adjusting"],
                "dest": "fine_yaw_adjusting",
                "conditions": "need_fine_yaw_adjustment",
                "before": "fine_yaw_adjust",
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

    def need_coarse_yaw_adjustment(self):
        """
        检查是否需要进行粗略偏航角调整

        :return: 如果需要调整返回 True，否则返回 False
        """
        result = self.agent.go_back_to_field_dist > self.min_dist and (
            abs(self.agent.go_back_to_field_yaw_diff) > 30
        )
        print(f"[Go Back to Field] Need coarse yaw adjustment? {'Yes' if result else 'No'}")
        return result

    def need_fine_yaw_adjustment(self):
        """
        检查是否需要进行精细偏航角调整

        :return: 如果需要调整返回 True，否则返回 False
        """
        result = self.agent.go_back_to_field_dist < 5 * self.min_dist and -20 < self.agent.go_back_to_field_yaw_diff < 10 and not -10 < self.agent.go_back_to_field_yaw_diff < 5 and self.agent.go_back_to_field_dist >= self.min_dist
        print(f"[Go Back to Field] Need fine yaw adjustment? {'Yes' if result else 'No'}")
        return result
    
    def dont_need_fine_yaw_adjustment(self):
        """
        检查是否不需要进行精细偏航角调整

        :return: 如果不需要调整返回 True，否则返回 False
        """
        result = not self.need_fine_yaw_adjustment()
        print(f"[Go Back to Field] Don't need fine yaw adjustment? {'Yes' if result else 'No'}")
        return result
    
    def dont_need_coarse_yaw_adjustment(self):
        """
        检查是否不需要进行粗略偏航角调整

        :return: 如果不需要调整返回 True，否则返回 False
        """
        result = not self.need_coarse_yaw_adjustment()
        print(f"[Go Back to Field] Don't need coarse yaw adjustment? {'Yes' if result else 'No'}")
        return result

    def good_position(self):
        """
        检查是否到达目标位置

        :return: 如果到达返回 True，否则返回 False
        """
        print(f"[Go Back to Field] go_back_to_field_dist: {self.agent.go_back_to_field_dist}, min_dist: {self.min_dist}")
        result = self.agent.go_back_to_field_dist < self.min_dist
        print(f"[Go Back to Field] Arrived at target? {'Yes' if result else 'No'}")
        return result

    def run(self):
        """
        状态机的主运行函数，控制机器人返回场地的整个流程
        """
        self.agent.is_going_back_to_field = True
        print("[Go Back to Field FSM] Starting to go back to field...")
        self.agent.aim_x = self.agent.get_command().get('data').get('x')
        self.agent.aim_y = self.agent.get_command().get('data').get('y')
        self.agent.aim_yaw = self.agent.get_command().get('data').get('yaw')
        self.agent.update_go_back_to_field_status()
        print(f"\n[Go Back to Field FSM] Current state: {self.state}")
        print("[Go Back to Field FSM] Triggering 'update_status' transition")
        self.machine.model.trigger("update_status")
        time.sleep(0.1)

    def update_status(self):
        """
        更新机器人的状态信息，并根据当前状态执行相应的操作
        """
        self.agent.update_go_back_to_field_status()
        print(f"[Go Back to Field] Updated status: dist={self.agent.go_back_to_field_dist}, yaw_bias={self.agent.go_back_to_field_yaw_diff}")
        if self.state == "moving_to_target":
            if self.need_coarse_yaw_adjustment():
                self.coarse_yaw_adjust()
            else:
                self.move_forward()
        elif self.state == "coarse_yaw_adjusting":
            if self.need_fine_yaw_adjustment():
                self.fine_yaw_adjust()
            elif self.arrived_at_target():
                self.arrived_at_target_operations()
            else:
                self.coarse_yaw_adjust()
        elif self.state == "fine_yaw_adjusting":
            if self.arrived_at_target():
                self.arrived_at_target_operations()
            else:
                self.fine_yaw_adjust()

    def move_forward(self):
        """
        控制机器人向前移动
        """
        print("[Go Back to Field] Moving forward...")
        print(f"[Go Back to Field] dist: {self.agent.go_back_to_field_dist}, yaw_diff: {self.agent.go_back_to_field_yaw_diff}, dir: {self.agent.go_back_to_field_dir}, pos_yaw: {self.agent.pos_yaw}")
        self.agent.update_go_back_to_field_status()
        self.agent.cmd_vel(self._config.get("walk_vel_x", 0.3), 0, 0)
        time.sleep(0.2)

    def coarse_yaw_adjust(self):
        """
        进行粗略偏航角调整
        """
        print("[Go Back to Field] Starting coarse yaw adjustment...")
        sgn_bias = 1 if self.agent.go_back_to_field_yaw_diff > 0 else -1

        print(f"[Go Back to Field] dist: {self.agent.go_back_to_field_dist}, yaw_diff: {self.agent.go_back_to_field_yaw_diff}, dir: {self.agent.go_back_to_field_dir}, pos_yaw: {self.agent.pos_yaw}")
        self.agent.update_go_back_to_field_status()
        if self.agent.go_back_to_field_dist < self.min_dist:
            pass
        elif -20 < self.agent.go_back_to_field_yaw_diff and self.agent.go_back_to_field_yaw_diff < 20:
            print("[Go Back to Field] edge case, turning")
            self.agent.cmd_vel(0, 0, self.last_rotate * self._config.get("walk_vel_theta", 0.3))
            time.sleep(0.2)
        elif -self.agent.go_back_to_field_yaw_diff > 20:
            print("[Go Back to Field] Turning right")
            self.agent.cmd_vel(0, 0, -self._config.get("walk_vel_theta", 0.3))
            self.last_rotate = -1
            time.sleep(0.2)
        else:
            print("[Go Back to Field] Turning left")
            self.agent.cmd_vel(0, 0, self._config.get("walk_vel_theta", 0.3))
            self.last_rotate = 1
            time.sleep(0.2)
        self.agent.go_back_to_field_dir = np.arctan2(-self.aim_x + self.agent.pos_x, self.aim_y - self.agent.pos_y)
        self.agent.go_back_to_field_yaw_diff = np.degrees(
            np.arctan2(
                np.sin(self.agent.go_back_to_field_dir - self.agent.pos_yaw * np.pi / 180),
                np.cos(self.agent.go_back_to_field_dir - self.agent.pos_yaw * np.pi / 180),
            )
        )
        # print("[Go Back to Field] Coarse yaw adjustment completed.")

    def good_yaw(self):
        """
        检查是否朝向正确

        :return: 如果朝向正确返回 True，否则返回 False
        """
        result = -10 < self.agent.go_back_to_field_yaw_diff < 10
        print(f"[Go Back to Field] Good yaw? {'Yes' if result else 'No'}")
        return result

    def fine_yaw_adjust(self):
        """
        进行精细偏航角调整
        """
        print("[Go Back to Field] Starting fine yaw adjustment...")
        sgn_bias = 1 if self.agent.go_back_to_field_yaw_diff > 0 else -1

        self.agent.update_go_back_to_field_status()
        if self.agent.go_back_to_field_dist < self.min_dist:
            pass
        elif -20 < self.agent.go_back_to_field_yaw_diff and self.agent.go_back_to_field_yaw_diff < 20:
            print("[Go Back to Field] edge case, turning")
            self.agent.cmd_vel(0, 0, self.last_rotate * self._config.get("walk_vel_theta", 0.3))
            time.sleep(0.2)
        elif -self.agent.go_back_to_field_yaw_diff > 20:
            print("[Go Back to Field] Turning right slowly")
            self.agent.cmd_vel(0, 0, 0.05)
            self.last_rotate = -1
            time.sleep(0.2)
        else:
            print("[Go Back to Field] Turning left slowly")
            self.agent.cmd_vel(0, 0, -0.05)
            self.last_rotate = 1
            time.sleep(0.2)

        # print("[Go Back to Field] Fine yaw adjustment completed.")

    def adjust_yaw(self):
        """
        到达目标位置后执行的操作，包括调整朝向和准备开始游戏
        """
        print("[Go Back to Field] Arrived at target. Performing yaw adjust...")
        self.agent.cmd_vel(0, 0, 0)
        time.sleep(1)
        if abs(self.agent.pos_yaw) > 160:
            print("[Go Back to Field] Correcting large yaw angle...")
            self.agent.cmd_vel(0, 0, -np.sign(self.agent.pos_yaw) * self._config.get("walk_vel_theta", 0.3))
        elif self.agent.pos_yaw > 30:
            print("[Go Back to Field] Arrived. Turning right")
            self.agent.cmd_vel(0, 0, -self._config.get("walk_vel_theta", 0.3))
            time.sleep(0.2)
        elif self.agent.pos_yaw < -30:
            print("[Go Back to Field] Arrived. Turning left")
            self.agent.cmd_vel(0, 0, self._config.get("walk_vel_theta", 0.3))
            time.sleep(0.2)
        else:
            # ready
            self.agent.cmd_vel(0, 0, 0)
            print("[Go Back to Field] Finished going back to field. Ready to play.")
            time.sleep(1)
            self.agent.is_going_back_to_field = False
            print("[Go Back to Field FSM] Arrived at target!")