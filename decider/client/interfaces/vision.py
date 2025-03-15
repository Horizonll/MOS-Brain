# interfaces/vision.py
#   @description:   Subscribe position
#

import math
import time
import angle
import numpy as np

import rospy
from std_msgs.msg import Float32MultiArray, Header
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import Quaternion, Twist, Pose2D, Point

class Vision:
    # @public variants:
    #   VARIANTS    TYPE        DESCTIPTION
    #   self_pos    np.array    self position in map
    #   pos_yaw     angle       self orientation in degree, [-180, 180)
    #
    # @public methods:
    #   None   
    #
    # @private variants
    #   _ball_pos_in_vis        the pixel coordinate of ball
    #   _ball_pos               the ball position from robot
    #   _ball_pos_in_map        the ball position in the whole map
    #   _ball_pos_accuracy      the 'accuracy' of ball_pos
    #   _self_pos_accuracy      the 'accuracy' of self_pos, the algorithm
    #                           to measure how 'inaccurary' has to be
    #                           improve. 
    #   @@@@ TODO IMPROVE THE ALGORITHM TO MEASURE INACCURACY @@@@
    #   _vision_last_frame_time     the timestamp of last frame of vision
    #   _head                   the angle of head
    #   _neck                   the angle of neck
    #   _config                 a dictionary, the config
    #   _pos_sub                the handler of /pos_in_map
    #   _vision_sub             the handler of /vision/obj_pos
    #   _soccer_real_sub        the handler of /soccer_real_pos_in_map
    #   _head_pub               the handler of /head_goals
    #
    # @private methods
    #   _head_set(head: float, neck: float)     set head and neck angle
    #   _position_callback(msg)                 callback of /pos_in_map
    #   _soccer_real_callback(msg)      callback of /soccer_real_pos_in_map
    #   _vision_callback(target_matrix)     callback of /vision/obj_pos

    def __init__(self, config): 
        self._ball_x = 0
        self._ball_y = 0
        self._ball_last_seen_time = 0
        self._neck = 0
        self._head = 0

        self._config = config
        self._pos_sub = rospy.Subscriber("/pos_in_map",  \
                                        Pose2D,  \
                                        self._position_callback)
        self._vision_sub = rospy.Subscriber("/vision/obj_pos", \
                                        Float32MultiArray, \
                                        self._vision_callback)
        self._soccer_real_sub = rospy.Subscriber("/soccer_real_pos_in_map",  \
                                        Point, \
                                        self._soccer_real_callback)
        self._head_pub = rospy.Publisher("/head_goals",  \
                                        JointState,  \
                                        queue_size = 1)
    

    # _head_set(head: float, neck: float):
    #   设置头的角度，并记录角度信息并发布
    #    @param head: 上下角度，[0,1.5]，1.5下，0上
    #    @param neck: 左右角度，[-1.1,1.1]，-1.1右，1.1左
    @classmethod
    def _head_set(self, head = 0, neck = 0):
        head = np.clip(head, 0, 1.5)
        neck = np.clip(neck, -1.1, 1.1)
        self._head, self._neck = head, neck
        head_goal = JointState()
        head_goal.name = ["head", "neck"]
        head_goal.header = Header()
        head_goal.position = [head, neck]
        self._head_pub.publish(head_goal) 

    @classmethod
    def _position_callback(self, msg):
        self.self_pos = np.array([msg.x, msg.y])
        self.pos_yaw  = msg.theta
    
    @classmethod
    def _soccer_real_callback(self, msg):
        self._ball_in_map = np.array([real_msg.x, real_msg.y])
    
    @classmethod
    def _vision_callback(self, msg):
        layout = msg.layout
        h = layout.dim[0].size
        w = layout.dim[1].size

        diff_time = time.time() - self._vision_last_frame_time;
        self._vision_last_frame_time = time.time()
        # accuracy *= exp^ -dt
        self._self_pos_accuracy *= math.exp(-diff_time)
        self._ball_pos_accuracy *= math.exp(-diff_time)

        # target_matrix shape： 
        #   N * [class, x_center, y_center, confidence, distance, x, y, z]
        # target_matrix的第一位表示类别，其中0-5对应
        #   ['ball', 'goalpost', 'robot', 'L-Intersection', 
        #       'T-Intersection', 'X-Intersection']
        # x_center, y_center 表示物体中心像素坐标
        # confidence置信度，表示物体识别准确度，越高越好
        # distance是通过相机内参矩阵算出来的结果，只有球有效
        # x, y, z 表示目标相对于机器人身体坐标系的坐标，机器
        # 人身体坐标系z朝上，y朝前，右手坐标系，单位mm
        obj_mat = np.array(msg.data, dtype=np.float32).reshape(h, w)
        ball_row, ball_confidence = None, 0

        for i, row in enumerate(obj_mat):
            if(row[0] == 0):  # ball
                if(row[3] > ball_confidence):
                    ball_confidence = row[3]
                    ball_row = row
            else:
                self._self_pos_accuracy += \
                        self._config["pos_accuracy_add"][row[0]]

        self._ball_last_seen_time = time.time()
        self._ball_pos_in_vis   = row[1:3]
        self._ball_pos          = row[5:8]
        self._ball_pos_accuracy += ball_confidence

        TODO: FIND BALL OR RELOCATE
    
