import time
from transitions import Machine
import numpy as np


class GoBackToFieldStateMachine:
    def __init__(self, agent, aim_x, aim_y, min_dist=300):
        """
        初始化返回场地状态机

        :param agent: 机器人代理对象，包含机器人的各种状态和控制方法
        :param aim_x: 目标位置的 x 坐标
        :param aim_y: 目标位置的 y 坐标
        :param min_dist: 到达目标位置的最小距离阈值，默认为 300
        """
        self.agent = agent
        self.aim_x = aim_x
        self.aim_y = aim_y
        self.min_dist = min_dist
        self.last_rotate = 1 # 上次旋转的方向,-1为右，1为左
        # 定义状态机的状态
        self.states = [
            "moving_to_target",  # 向目标位置移动
            "coarse_yaw_adjusting",  # 粗略偏航角调整
            "fine_yaw_adjusting",  # 精细偏航角调整
            "arrived_at_target",  # 到达目标位置
        ]
        # 定义状态机的状态转移规则
        self.transitions = [
            {
                "trigger": "update_status",
                "source": "moving_to_target",
                "dest": "coarse_yaw_adjusting",
                "conditions": "need_coarse_yaw_adjustment",
                "before": "update_status",
            },
            {
                "trigger": "update_status",
                "source": ["coarse_yaw_adjusting", "fine_yaw_adjusting"],
                "dest": "moving_to_target",
                "conditions": "dont_need_fine_yaw_adjustment",
                "before": "update_status",
            },
            {
                "trigger": "update_status",
                "source": "coarse_yaw_adjusting",
                "dest": "fine_yaw_adjusting",
                "conditions": "need_fine_yaw_adjustment",
                "before": "update_status",
            },
            {
                "trigger": "update_status",
                "source": ["coarse_yaw_adjusting", "fine_yaw_adjusting", "moving_to_target"],
                "dest": "arrived_at_target",
                "conditions": "arrived_at_target",
                "before": "update_status",
                "after": "arrived_at_target_operations",
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
            abs(self.agent.go_back_to_field_yaw_diff) > 15
            or (self.agent.go_back_to_field_dist > 3 * self.min_dist and abs(self.agent.go_back_to_field_yaw_diff) > 10)
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

    def arrived_at_target(self):
        """
        检查是否到达目标位置

        :return: 如果到达返回 True，否则返回 False
        """
        result = self.agent.go_back_to_field_dist < self.min_dist
        print(f"[Go Back to Field] Arrived at target? {'Yes' if result else 'No'}")
        return result

    def run(self):
        """
        状态机的主运行函数，控制机器人返回场地的整个流程
        """
        self.agent.is_going_back_to_field = True
        print("[Go Back to Field FSM] Starting to go back to field...")
        self.agent.aim_x = self.agent.command.get('data').get('aim_x')
        self.agent.aim_y = self.agent.command.get('data').get('aim_y')
        self.agent.update_go_back_to_field_status()
        if self.state != "arrived_at_target":
            print(f"\n[Go Back to Field FSM] Current state: {self.state}")
            print("[Go Back to Field FSM] Triggering 'update_status' transition")
            self.machine.model.trigger("update_status")
            time.sleep(0.3)

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
        self.agent.debug_info("[Go Back to Field] Going forward")
        self.agent.speed_controller(self.agent.walk_x_vel, 0, 0)
        time.sleep(1)

    def coarse_yaw_adjust(self):
        """
        进行粗略偏航角调整
        """
        print("[Go Back to Field] Starting coarse yaw adjustment...")
        sgn_bias = 1 if self.agent.go_back_to_field_yaw_diff > 0 else -1
        while self.agent.loop() and not -40 < self.agent.go_back_to_field_yaw_diff / sgn_bias < 20:
            print(f"[Go Back to Field] dist: {self.agent.go_back_to_field_dist}, yaw_bias: {self.agent.go_back_to_field_yaw_diff}, dir: {self.agent.go_back_to_field_dir}, pos_yaw: {self.agent.pos_yaw}")
            self.agent.update_go_back_to_field_status()
            if self.agent.go_back_to_field_dist < self.min_dist:
                break
            if -20 < self.agent.go_back_to_field_yaw_diff and self.agent.go_back_to_field_yaw_diff < 20:
                print("[Go Back to Field] edge case, turning")
                self.agent.speed_controller(0, 0, self.last_rotate * self.agent.walk_theta_vel)
            elif -self.agent.go_back_to_field_yaw_diff > 20:
                print("[Go Back to Field] Turning right")
                self.agent.speed_controller(0, 0, -self.agent.walk_theta_vel)
                time.sleep(0.3)
            else:
                print("[Go Back to Field] Turning left")
                self.agent.speed_controller(0, 0, self.agent.walk_theta_vel)
                time.sleep(0.3)
            self.agent.go_back_to_field_dir = np.arctan2(-self.aim_x + self.agent.pos_x, self.aim_y - self.agent.pos_y)
            self.agent.go_back_to_field_yaw_diff = np.degrees(
                np.arctan2(
                    np.sin(self.agent.go_back_to_field_dir - self.agent.pos_yaw * np.pi / 180),
                    np.cos(self.agent.go_back_to_field_dir - self.agent.pos_yaw * np.pi / 180),
                )
            )
        print("[Go Back to Field] Coarse yaw adjustment completed.")

    def fine_yaw_adjust(self):
        """
        进行精细偏航角调整
        """
        print("[Go Back to Field] Starting fine yaw adjustment...")
        sgn_bias = 1 if self.agent.go_back_to_field_yaw_diff > 0 else -1
        while not self.agent.loop() and not (-20 < self.agent.go_back_to_field_yaw_diff / sgn_bias < 10) and self.agent.go_back_to_field_dist < 5 * self.min_dist:
            self.agent.update_go_back_to_field_status()
            if self.agent.go_back_to_field_dist < self.min_dist:
                break
            if -20 < self.agent.go_back_to_field_yaw_diff and self.agent.go_back_to_field_yaw_diff < 20:
                print("[Go Back to Field] edge case, turning")
                self.agent.speed_controller(0, 0, self.last_rotate * self.agent.walk_theta_vel)
            elif -self.agent.go_back_to_field_yaw_diff > 20:
                print("[Go Back to Field] Turning right slowly")
                self.agent.speed_controller(0, 0, 0.05)
                time.sleep(0.5)
                self.agent.debug_info("[Go Back to Field] Turning right slowly")
            else:
                print("[Go Back to Field] Turning left slowly")
                self.agent.speed_controller(0, 0, -0.05)
                time.sleep(0.5)
            self.agent.go_back_to_field_dir = np.arctan2(-self.aim_x + self.agent.pos_x, self.aim_y - self.agent.pos_y)
            self.agent.go_back_to_field_yaw_diff = np.degrees(
                np.arctan2(
                    np.sin(self.agent.go_back_to_field_dir - self.agent.pos_yaw * np.pi / 180),
                    np.cos(self.agent.go_back_to_field_dir - self.agent.pos_yaw * np.pi / 180),
                )
            )
        print("[Go Back to Field] Fine yaw adjustment completed.")

    def arrived_at_target_operations(self):
        """
        到达目标位置后执行的操作，包括调整朝向和准备开始游戏
        """
        print("[Go Back to Field] Arrived at target. Performing final operations...")
        self.agent.debug_info("[Go Back to Field] " + str(self.agent.role) + " has arrived. Turn ahead")
        self.agent.speed_controller(0, 0, 0)
        time.sleep(1)
        if abs(self.agent.pos_yaw) > 160:
            print("[Go Back to Field] Correcting large yaw angle...")
            self.agent.speed_controller(0, 0, -np.sign(self.agent.pos_yaw) * self.agent.walk_theta_vel)
            time.sleep(2)
        while not self.agent.loop() and self.agent.pos_yaw > 5:
            print("[Go Back to Field] Arrived. Turning right")
            self.agent.speed_controller(0, 0, -self.agent.walk_theta_vel)
            self.agent.debug_info("[Go Back to Field] Arrived. Turning right")
            time.sleep(0.5)
        while not self.agent.loop() and self.agent.pos_yaw < -5:
            print("[Go Back to Field] Arrived. Turning left")
            self.agent.speed_controller(0, 0, self.agent.walk_theta_vel)
            self.agent.debug_info("[Go Back to Field] Arrived. Turning left")
            time.sleep(0.5)
        # ready
        self.agent.speed_controller(0, 0, 0)
        print("[Go Back to Field] Finished going back to field. Ready to play.")
        self.agent.debug_info("[Go Back to Field] Finished going back to field. Ready to play.")
        time.sleep(1)
        self.agent.is_going_back_to_field = False
        print("[Go Back to Field FSM] Arrived at target!")