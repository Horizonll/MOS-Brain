import os
import math
import numpy as np
import rospy
import actionlib
from std_msgs.msg import Float32MultiArray, Header
from bitbots_msgs.msg import KickGoal, KickAction
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import Quaternion, Twist, Pose2D, Point


"""
机器人坐标
pos_x: 机器人在世界坐标系（球场）的x坐标
pos_y: 机器人在世界坐标系（球场）的y坐标
pos_yaw: 机器人在世界坐标系（球场）的偏航角，单位：度，[-180, 180]
"""


class Decision_Pos(object):
    def __init__(self):
        self.pos_x = 0
        self.pos_y = 0
        self.pos_yaw = 0
        self.pos_sub = rospy.Subscriber("/pos_in_map", Pose2D, self.pos_callback)

    def pos_callback(self, msg):
        try:
            self.pos_x = msg.x
            self.pos_y = msg.y
            self.pos_yaw = msg.theta
        except:
            pass


"""视觉的接收类
ifBall: 是否有球
ball_x: 球在图像中的x坐标
ball_y: 球在图像中的y坐标
ball_x_in_map: 球在世界坐标系（球场）的x坐标
ball_y_in_map: 球在世界坐标系（球场）的y坐标
ball_distance: 机器人到球的距离
"""


class Decision_Vision(object):
    def __init__(self):
        self.reset()
        # ROS接收视觉组信息
        self.vision_sub = rospy.Subscriber("/vision/obj_pos", Float32MultiArray, self.vision_callback)
        self.soccer_real_sub = rospy.Subscriber("soccer_real_pos_in_map", Point, self.soccer_real_callback)

    # Using ball_x ball_y distance if_Ball ball_x_in_map ball_y_in_map
    def reset(self):
        self.ifBall = False
        self.ball_x = 0  # this is contrast to before
        self.ball_y = 0  # PIX
        self.ball_x_in_map = 0  # real_pose_in_map
        self.ball_y_in_map = 0
        self.ball_distance = 0  # real pose in robot coordinate
        self.angle = 0

    def vision_callback(self, target_matrix):
        layout = target_matrix.layout
        h = layout.dim[0].size
        w = layout.dim[1].size

        # target_matrix shape： N * [class, x_center, y_center, confidence, distance, x, y, z]
        # target_matrix的第一位表示类别，其中0-5对应['ball', 'goalpost', 'robot', 'L-Intersection', 'T-Intersection', 'X-Intersection']
        # x_center, y_center 表示物体中心像素坐标
        # confidence置信度，表示物体识别准确度，越高越好
        # distance是通过相机内参矩阵算出来的结果，只有球有效
        # x, y, z 表示目标相对于机器人身体坐标系的坐标，机器人身体坐标系z朝上，y朝前，右手坐标系，单位mm
        self.target_metrix = np.array(target_matrix.data, dtype=np.float32).reshape(h, w)

        # self.ifBall = (self.target_metrix[:, 0] == 0).any()  # 第 0 列有为 0 的值

        # TODO: 如果没有消息就不发布
        try:
            if self.ifBall:
                # 如果有球，则只有一个球场上置信度最高的球。获取这一行的第6个值为x, 第7个值为y
                target_ball_msg = self.target_metrix[self.target_metrix[:, 0] == 0].squeeze()  # 直接切片是2维
                self.ball_distance = math.sqrt(target_ball_msg[-3] ** 2 + target_ball_msg[-2] ** 2) / 1000  # 球距离：m
                self.angle = math.degrees(math.atan2(target_ball_msg[-2], target_ball_msg[-3]))  # 球角度：degree

                self.ball_x = target_ball_msg[1]  # 像素中心坐标
                self.ball_y = target_ball_msg[2]

            else:
                self.ball_x = 335
                self.ball_y = 170
        except:
            pass

    def soccer_real_callback(self, real_msg):
        self.ifBall = real_msg.z == 1
        if self.ifBall:
            self.ball_x_in_map = real_msg.x
            self.ball_y_in_map = real_msg.y


"""动作的接收类
doKick: 踢球
head_set: 设置头部角度
"""


class Decision_Motion(object):
    def __init__(self):
        self.cam_head = 0  # [0, 1], 1 在最下, 0 在最上
        self.cam_neck = 0.5  # [-1, 1], -1 往右, +1 往左
        # ROS发布步态组指令
        self.head_pub = rospy.Publisher("/head_goals", JointState, queue_size=1)
        self.speed_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        # 创建客户端kick_client，向名为thoms_animation的服务端发送请求，请求类型为KickAction
        self.kick_client = actionlib.SimpleActionClient("thmos_animation", KickAction)

    def doKick(self):
        """
        @brief 踢球
        """
        kick_goal = KickGoal()
        kick_goal.header.seq = 1
        kick_goal.header.stamp = rospy.Time.now()
        frame_prefix = "" if os.environ.get("ROS_NAMESPACE") is None else os.environ.get("ROS_NAMESPACE") + "/"
        kick_goal.header.frame_id = frame_prefix + "base_footprint"
        kick_goal.ball_position.x = 0
        kick_goal.ball_position.y = 0
        kick_goal.ball_position.z = 0
        kick_goal.kick_direction = Quaternion(0, 0, 0, 0)
        kick_goal.kick_speed = 3
        self.kick_client.send_goal(kick_goal)
        rospy.loginfo("send kick")
        rospy.loginfo("kick goal init")
        self.kick_client.done_cb = self.done_cb
        self.kick_client.feedback_cb = self.feedback_cb
        self.kick_client.active_cb = self.active_cb
        rospy.loginfo("kick done")

    def done_cb(self):
        pass

    def feedback_cb(self):
        pass

    def active_cb(self):
        pass

    def head_set(self, head=0, neck=0):
        """
        @brief 设置头的角度，并记录角度信息并发布
        @param head: 上下角度，[0,1.5]，1.5下，0上
        @param neck: 左右角度，[-1.1,1.1]，-1.1右，1.1左
        """
        self.cam_head = head
        self.cam_neck = neck
        head = np.clip(head, 0, 1.5)
        neck = np.clip(neck, -1.1, 1.1)
        head_goal = JointState()
        head_goal.name = ["head", "neck"]
        head_goal.header = Header()
        head_goal.position = [head, neck]
        self.head_pub.publish(head_goal)

    def save_l(self):
        kick_goal = KickGoal()

        # kick_goal消息戳
        kick_goal.header.seq = 1
        kick_goal.header.stamp = rospy.Time.now()
        frame_prefix = "" if os.environ.get("ROS_NAMESPACE") is None else os.environ.get("ROS_NAMESPACE") + "/"
        kick_goal.header.frame_id = frame_prefix + "base_footprint"

        # 踢球速度
        kick_goal.kick_speed = 4  # 踢球

        # kick_client客户端发送kick_goal的踢球请求
        self.kick_client.send_goal(kick_goal)
        rospy.loginfo("send kick")

        rospy.loginfo("kick goal init")
        # 反馈函数定义，等待服务端反馈结果
        self.kick_client.done_cb = self.done_cb
        self.kick_client.feedback_cb = self.feedback_cb
        self.kick_client.active_cb = self.active_cb
        # self.kick_client.wait_for_result()
        rospy.loginfo("kick done")

    def save_r(self):
        kick_goal = KickGoal()

        # kick_goal消息戳
        kick_goal.header.seq = 1
        kick_goal.header.stamp = rospy.Time.now()
        frame_prefix = "" if os.environ.get("ROS_NAMESPACE") is None else os.environ.get("ROS_NAMESPACE") + "/"
        kick_goal.header.frame_id = frame_prefix + "base_footprint"

        # 踢球速度
        kick_goal.kick_speed = 5  # 踢球

        # kick_client客户端发送kick_goal的踢球请求
        self.kick_client.send_goal(kick_goal)
        rospy.loginfo("send kick")

        rospy.loginfo("kick goal init")
        # 反馈函数定义，等待服务端反馈结果
        self.kick_client.done_cb = self.done_cb
        self.kick_client.feedback_cb = self.feedback_cb
        self.kick_client.active_cb = self.active_cb
        # self.kick_client.wait_for_result()
        rospy.loginfo("kick done")
