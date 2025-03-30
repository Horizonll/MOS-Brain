# find_ball.py
# State machine for robot finding ball behavior

from transitions import Machine
from sensor_msgs.msg import JointState
import time
import rospy
from configuration import configuration

class FindBallStateMachine:
    def __init__(self, agent):
        """Initialize the find ball state machine with an agent"""
        self.agent = agent
        self.rotate_start_time = 0  # 记录旋转开始时间

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
                "source": "protecting", 
                "dest": "rotating", 
                "conditions": "protection_done", 
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
                "after": "stop_rotation"
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
        result = self.agent.ball_in_sight()
        print(f"[FIND BALL FSM] Ball in sight: {'Yes' if result else 'No'}")
        return result

    def protection_done(self, event=None):  
        """检查保护姿势是否完成（0.5秒超时）"""
        elapsed = time.time() - self.enter_time
        result = self.state == "protecting" and elapsed > 0.5
        print(f"[FIND BALL FSM] Protection done: {'Yes' if result else f'No ({elapsed:.1f}s)'}")
        return result

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
        protect_pose = JointState()
        protect_pose.name = ["L_arm_1", "L_arm_2", "L_arm_3", "R_arm_1", "R_arm_2", "R_arm_3"]
        protect_pose.position = [0, 1.2, -0.5, 0, -1.2, 0.5]
        self.agent.joint_goal_publisher.publish(protect_pose)
        self.agent.head_set(head=0.1, neck=0)
        self.enter_time = time.time()  # 记录进入保护姿势的时间
        print("[FIND BALL FSM] Protect pose set")

    def start_rotation(self, event=None):
        """开始旋转身体寻找球"""
        print("[FIND BALL FSM] Starting rotation...")
        self.agent.speed_controller(0, 0, configuration.walk_theta_vel)
        self.agent.head_set(head=0.1, neck=0)
        self.rotate_start_time = time.time()  # 记录旋转开始时间
        print(f"[FIND BALL FSM] Rotating at {configuration.walk_theta_vel} rad/s")

    def stop_rotation(self, event=None):
        """停止旋转"""
        print("[FIND BALL FSM] Stopping rotation...")
        self.agent.stop(0.5)
        self.rotate_start_time = time.time()  # 重置旋转开始时间
        print("[FIND BALL FSM] Rotation stopped")

    def run(self):
        """运行状态机"""
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
#                 if self.agent.ifBall:
#                     return
#                 self.agent.head_set(
#                     head=configuration.find_head_pos[pos],
#                     neck=configuration.find_neck_pos[pos],
#                 )

#         self.agent.speed_controller(0, 0, configuration.walk_theta_vel)
#         t = time.time()
#         while self.agent.loop() and time.time() - t < 10:
#             for pos in [0, 3]:
#                 t1 = time.time()
#                 while time.time() - t1 < 2:
#                     if self.agent.ifBall:
#                         self.agent.stop(0.5)
#                         return
#                     self.agent.head_set(
#                         head=configuration.find_head_pos[pos],
#                         neck=configuration.find_neck_pos[pos],
#                     )
