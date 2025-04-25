# find_ball.py
# State machine for robot finding ball behavior

from transitions import Machine
from sensor_msgs.msg import JointState
import time
import rospy
import numpy as np

class FindBallStateMachine:
    def __init__(self, agent):
        """Initialize the find ball state machine with an agent"""
        self.agent = agent
        self._config = self.agent.get_config()
        self.rotate_start_time = 0  # 记录旋转开始时间
        self.find_ball_angle_threshold_degrees = 10  # 球的角度阈值（度）

        # 定义状态和转换规则
        self.states = ["init", "protecting", "rotating", "found"]
        self.transitions = [
            # 统一使用step触发器
            {
                "trigger": "step", 
                "source": "init", 
                "dest": "protecting", 
                "after": "set_protect_pose"
            },
            {
                "trigger": "step",
                "source": ["protecting", "rotating"],
                "dest": "rotating",
                "conditions": ["protection_done", "no_ball"],
                "after": "start_rotation"
            },
            {
                "trigger": "step",
                "source": "rotating",
                "dest": "protecting",
                "conditions":
                [
                    "rotation_timeout",
                    "no_ball"
                ],
                "after": "stop_rotation"
            },
            {
                "trigger": "step",
                "source": "*",
                "dest": "found",
                "conditions": "ball_in_sight",
                "after": "face_to_ball"
            },
            {
                "trigger": "step",
                "source": "found",
                "dest": "protecting",
                "conditions": "no_ball",
                "after": "start_rotation"
            }
        ]

        # 初始化状态机
        self.machine = Machine(
            model=self,
            states=self.states,
            initial="init",
            transitions=self.transitions,
            send_event=True
        )
        print(f"[FIND BALL FSM] Initialized. Starting state: {self.state}")


    # 状态检查条件 --------------------------------------------------
    def ball_in_sight(self, event=None):  
        """检查是否看到球"""
        result = self.agent.get_if_ball()
        print(f"[FIND BALL FSM] Ball in sight: {'Yes' if result else 'No'}")
        return result

    def protection_done(self, event=None):  
        """检查保护姿势是否完成（0.5秒超时）"""
        # elapsed = time.time() - self.enter_time
        # result = self.state == "protecting" and elapsed > 0.5
        # print(f"[FIND BALL FSM] Protection done: {'Yes' if result else f'No ({elapsed:.1f}s)'}")
        # return result
        return True

    def rotation_timeout(self, event=None):
        """检查旋转是否超时（10秒）"""
        elapsed = time.time() - self.rotate_start_time
        result = elapsed >= 10
        print(f"[FIND BALL FSM] Rotation timeout: {'Yes' if result else f'No ({elapsed:.1f}s)'}")
        return result and self.state == "rotating"

    def no_ball(self, event=None):
        """检查是否丢失球"""
        result = not self.ball_in_sight()
        print(f"[FIND BALL FSM] Ball lost: {'Yes' if result else 'No'}")
        return result

    # 状态动作 ------------------------------------------------------
    def set_protect_pose(self, event=None): 
        """设置保护姿势（手臂位置）"""
        print("[FIND BALL FSM] Setting protect pose...")
        # protect_pose = JointState()
        # protect_pose.name = ["L_arm_1", "L_arm_2", "L_arm_3", "R_arm_1", "R_arm_2", "R_arm_3"]
        # protect_pose.position = [0, 1.2, -0.5, 0, -1.2, 0.5]
        # self.agent.joint_goal_publisher.publish(protect_pose)
        # self.enter_time = time.time()  # 记录进入保护姿势的时间
        print("[FIND BALL FSM] Protect pose set")

    def start_rotation(self, event=None):
        """开始旋转身体寻找球"""
        print("[FIND BALL FSM] Starting rotation...")
        ball_angle_from_other_robots = self.agent.get_ball_angle_from_other_robots()
        if ball_angle_from_other_robots is not None:
            print(f"[FIND BALL FSM] Other robots see the ball at angle: {ball_angle_from_other_robots}")
            # 如果有其他机器人看到球，则朝向该角度旋转
            target_angle_rad = ball_angle_from_other_robots
            print(f"[FIND BALL FSM] Rotating towards ball angle: {target_angle_rad}")
            self.agent.cmd_vel(0, 0, np.sign(target_angle_rad) * self._config.get("walk_vel_theta", 0.3))
        else:
            print("[FIND BALL FSM] No other robots see the ball, rotating randomly...")
            # 如果没有其他机器人看到球，则随机旋转
            self.agent.cmd_vel(0, 0, self._config.get("walk_vel_theta", 0.3))


        self.rotate_start_time = time.time()  # 记录旋转开始时间

    def stop_rotation(self, event=None):
        """停止旋转"""
        print("[FIND BALL FSM] Stopping rotation...")
        self.agent.stop(0.5)
        self.rotate_start_time = time.time()  # 重置旋转开始时间
        print("[FIND BALL FSM] Rotation stopped")

    def face_to_ball(self, event=None):
        """调整身体面向球"""
        print("[FIND BALL FSM] Facing to ball...")

        if not self.agent.get_if_ball():
            print("[FIND BALL FSM] No ball in sight, cannot face to ball")
            return

        # 获取球的角度
        target_angle_rad = self.agent.get_ball_angle()

        find_ball_angle_threshold_rad = self.find_ball_angle_threshold_degrees * np.pi / 180.0

        if abs(target_angle_rad) > find_ball_angle_threshold_rad:
            print(
                f"[CHASE BALL FSM] target_angle_rad ({target_angle_rad}) > {find_ball_angle_threshold_rad}. ball_distance: {self.agent.get_ball_distance()}. Rotating..."
            )
            self.agent.cmd_vel(
                0, 0, np.sign(target_angle_rad) * self._config.get("walk_vel_theta", 0.3)
            )
        else:
            print(
                f"[FIND BALL FSM] target_angle_rad ({target_angle_rad}) < {find_ball_angle_threshold_rad}. Stopping rotation..."
            )
            self.agent.stop()

        print("[FIND BALL FSM] Facing to ball completed")

    def run(self):
        """运行状态机"""
        self.agent.look_at([None, None])
        self.step()
        print("[FIND BALL FSM] Running...")






# # find ball
# #

# from transitions import Machine
# from sensor_msgs.msg import JointState
# import time
# from configuration import configuration


# class FindBallStateMachine:
#     def __init__(self, agent):
#         self.agent = agent
#         self.states = ["search", "found"]
#         self.transitions = [
#             {
#                 "trigger": "search_ball",
#                 "source": "search",
#                 "dest": "found",
#                 "conditions": "ball_in_sight",
#                 "prepare": "search_ball",
#             },
#             {
#                 "trigger": "search_ball",
#                 "source": "found",
#                 "dest": "search",
#                 "conditions": "not ball_in_sight",
#                 "after": "search_ball",
#             }
#         ]
#         self.machine = Machine(
#             model=self,
#             states=self.states,
#             initial="search",
#             transitions=self.transitions,
#         )

#     def ball_in_sight(self):
#         return self.agent.ball_in_sight()

#     def run(self):
#         self.machine.model.trigger("search_ball")

#     def search_ball(self):
#         self.agent.stop(0.5)
#         protect_pose = JointState()
#         protect_pose.name = [
#             "L_arm_1",
#             "L_arm_2",
#             "L_arm_3",
#             "R_arm_1",
#             "R_arm_2",
#             "R_arm_3",
#         ]
#         protect_pose.position = [0, 1.2, -0.5, 0, -1.2, 0.5]
#         self.agent.joint_goal_publisher.publish(protect_pose)

#         for pos in range(6):
#             t = time.time()
#             while time.time() - t < 1:
#                 if self.agent.get_if_ball():
#                     return
#                 self.agent.head_set(
#                     head=configuration.find_head_pos[pos],
#                     neck=configuration.find_neck_pos[pos],
#                 )

#         self.agent.cmd_vel(0, 0, self._config.get("walk_vel_theta", 0.3))
#         t = time.time()
#         while self.agent.loop() and time.time() - t < 10:
#             for pos in [0, 3]:
#                 t1 = time.time()
#                 while time.time() - t1 < 2:
#                     if self.agent.get_if_ball():
#                         self.agent.stop(0.5)
#                         return
#                     self.agent.head_set(
#                         head=configuration.find_head_pos[pos],
#                         neck=configuration.find_neck_pos[pos],
#                     )
