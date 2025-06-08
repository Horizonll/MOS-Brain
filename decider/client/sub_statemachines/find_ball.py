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
        self.read_params()  # 读取配置参数

        self.last_rotaion = 1
        
        self.rotate_start_time = 0  # 记录旋转开始时间
        
        # 定义状态和转换规则
        self.states = ["init", "protecting", "rotating", "found"]
        self.transitions = [
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
                "conditions": ["rotation_timeout", "no_ball"],
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
        rospy.loginfo(f"[FIND BALL FSM] Initialized. Starting state: {self.state}")

    # ------------------------- 参数读取方法 -------------------------
    def read_params(self):
        """从配置中读取所有参数"""
        # 球角度阈值（度）
        self.find_ball_angle_threshold_degrees = self._config.get("find_ball", {}).get(
            "angle_threshold_degrees", 10
        )
        # 旋转速度（rad/s）
        self.rotation_vel_theta = self._config.get("find_ball", {}).get(
            "rotation_vel_theta", 0.3
        )
        # 旋转超时时间（秒）
        self.rotation_timeout_seconds = self._config.get("find_ball", {}).get(
            "rotation_timeout_seconds", 10
        )
        # 保护姿势保持时间（秒）（当前逻辑未使用，预留参数）
        self.protection_pose_duration = self._config.get("find_ball", {}).get(
            "protection_pose_duration", 0.5
        )


    # ------------------------- 状态检查条件 -------------------------
    def ball_in_sight(self, event=None):  
        """检查是否看到球"""
        result = self.agent.get_if_ball()
        rospy.loginfo(f"[FIND BALL FSM] Ball in sight: {'Yes' if result else 'No'}")
        return result

    def protection_done(self, event=None):  
        """检查保护姿势是否完成（当前逻辑简化为直接通过，可根据需要添加超时逻辑）"""
        return True  # 如需超时逻辑，可参考原始注释恢复并使用self.protection_pose_duration

    def rotation_timeout(self, event=None):
        """检查旋转是否超时"""
        elapsed = time.time() - self.rotate_start_time
        result = elapsed >= self.rotation_timeout_seconds and self.state == "rotating"
        rospy.loginfo(f"[FIND BALL FSM] Rotation timeout: {'Yes' if result else f'No ({elapsed:.1f}s)'}")
        return result

    def no_ball(self, event=None):
        """检查是否丢失球"""
        result = not self.ball_in_sight()
        rospy.loginfo(f"[FIND BALL FSM] Ball lost: {'Yes' if result else 'No'}")
        return result

    # ------------------------- 状态动作 -------------------------
    def set_protect_pose(self, event=None): 
        """设置保护姿势（手臂位置）"""
        rospy.loginfo("[FIND BALL FSM] Setting protect pose...")
        # 示例：从配置中读取保护姿势参数（假设配置中存在相关定义）
        # protect_pose = self._config.get("find_ball", {}).get("protect_pose", default_pose)
        # self.agent.joint_goal_publisher.publish(protect_pose)
        rospy.loginfo("[FIND BALL FSM] Protect pose set")

    def start_rotation(self, event=None):
        """开始旋转身体寻找球"""
        rospy.loginfo("[FIND BALL FSM] Starting rotation...")
        ball_angle_from_other_robots = self.agent.get_ball_angle_from_other_robots()
        
        if ball_angle_from_other_robots is not None:
            rospy.loginfo(f"[FIND BALL FSM] Other robots see the ball at angle: {ball_angle_from_other_robots}")
            target_angle_rad = ball_angle_from_other_robots
            if abs(target_angle_rad) < 0.7:
                rotate_vel = self.last_rotaion * self.rotation_vel_theta
            else:
                rotate_vel = np.sign(target_angle_rad) * self.rotation_vel_theta
                self.last_rotaion = np.sign(target_angle_rad)
        else:
            rospy.loginfo("[FIND BALL FSM] No other robots see the ball, rotating randomly...")
            rotate_vel = self.rotation_vel_theta  # 可配置为随机方向或固定方向
        
        self.agent.cmd_vel(0, 0, rotate_vel)
        self.rotate_start_time = time.time()  # 记录旋转开始时间

    def stop_rotation(self, event=None):
        """停止旋转"""
        rospy.loginfo("[FIND BALL FSM] Stopping rotation...")
        self.agent.stop(0.5)
        self.rotate_start_time = time.time()  # 重置旋转开始时间
        rospy.loginfo("[FIND BALL FSM] Rotation stopped")

    def face_to_ball(self, event=None):
        """调整身体面向球"""
        rospy.loginfo("[FIND BALL FSM] Facing to ball...")
        if not self.agent.get_if_ball():
            rospy.logwarn("[FIND BALL FSM] No ball in sight, cannot face to ball")
            return

        target_angle_rad = self.agent.get_ball_angle()
        angle_threshold_rad = np.deg2rad(self.find_ball_angle_threshold_degrees)

        if abs(target_angle_rad) > angle_threshold_rad:
            rospy.loginfo(
                f"[FIND BALL FSM] Target angle ({np.rad2deg(target_angle_rad):.1f}°) > threshold ({self.find_ball_angle_threshold_degrees}°). Rotating..."
            )
            self.agent.cmd_vel(0, 0, np.sign(target_angle_rad) * self.rotation_vel_theta)
        else:
            rospy.loginfo(
                f"[FIND BALL FSM] Target angle within threshold. Stopping rotation..."
            )
            self.agent.stop()

        rospy.loginfo("[FIND BALL FSM] Facing to ball completed")

    def run(self):
        """运行状态机"""
        self.agent.look_at([None, None])
        self.step()
        rospy.loginfo("[FIND BALL FSM] Running...")