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
    #   VARIANTS        TYPE        DESCTIPTION
    #   self_pos        np.array    self position in map; already filtered
    #   pos_yaw         angle       self orientation in degree, [-180, 180)
    #   head            float       the angle of head
    #   neck            float       the angle of neck
    #   ball_distance   float   the distance from robot to tht ball
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
    #   _config                 a dictionary, the config
    #   _pos_sub                the handler of /pos_in_map
    #   _vision_sub             the handler of /vision/obj_pos
    #   _soccer_real_sub        the handler of /soccer_real_pos_in_map
    #   _head_pub               the handler of /head_goals
    #   _last_move_head_time    last timestamp running _move_head()
    #   _move_head_stage        the stage ( FSMID ) of _move_head()
    #   _last_move_head_stage_time      timestamp for changing stage periodicly
    #
    # @private methods
    #   _head_set(head: float, neck: float)     set head and neck angle
    #   _position_callback(msg)                 callback of /pos_in_map
    #   _soccer_real_callback(msg)      callback of /soccer_real_pos_in_map
    #   _vision_callback(target_matrix)     callback of /vision/obj_pos
    #   _move_head()                        the main algorithm to move head
    #   _move_head_stage_looking_at_ball()  looing at ball algorithm

    def __init__(self, config): 

        self._ball_pos_in_vis = np.array([0, 0])
        self._ball_pos_in_vis_D = np.array([0, 0])
        self._ball_pos_in_vis_I = np.array([0, 0])
        self._ball_pos = np.array([0, 0])
        self._ball_pos_in_map = np.array([0, 0])
        self._vision_last_frame_time = 0
        self.self_pos = np.array([0,0])
        self.pos_yaw = 0
        self._self_pos_accuracy = 0
        self._ball_pos_accuracy = 0
        self._last_move_head_time = -99999999
        self._last_move_head_stage_time = 0
        self._move_head_stage = 0

        self.head = 0.75
        self.neck = 0

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
        
        self._head_set(self.head, self.neck)


    def _move_head_stage_looking_at_ball(self):
        args = self._config["looking_at_ball_arguments"]
        addn =  (self._ball_pos_in_vis[0] - 640) / 1280 * args[1][0] + \
                (self._ball_pos_in_vis_I[0] - 640) / 1280 * args[1][1] + \
                (self._ball_pos_in_vis_D[0] - 640) / 1280 * args[1][2];
        addh =  (self._ball_pos_in_vis[1] - 368) / 736 * args[0][0] + \
                (self._ball_pos_in_vis_I[1] - 368) / 736 * args[0][1] + \
                (self._ball_pos_in_vis_D[1] - 368) / 736 * args[0][2];
        self.head += addh
        self.neck -= addn
        # print("x = " + str(self._ball_pos_in_vis[0]) + " y = " + str(self._ball_pos_in_vis[1]))
        # print("add head = " + str(addh) + " add neck = " + str(addn))
        # print("head = " + str(self.head) + " neck = " + str(self.neck))
        self._head_set(self.head, self.neck)


    def _move_head_stage_head_up(self):
        self._head_set(0.70, self.pos_yaw)


    def _move_head(self):
        if(time.time() - self._last_move_head_time < \
                self._config["move_head_time_gap"]):
            return
        self._last_move_head_time = time.time()

        # change move head stage periodicly
        if(time.time() - self._last_move_head_stage_time > 
           self._config["move_head_stage_time_gap"]):
            self._move_head_stage = (self._move_head_stage + 1) % 2
            self._last_move_head_stage_time = time.time()
        
        # if(self._move_head_stage % 2 == 0):
        self._move_head_stage_looking_at_ball()
        # else:
        #    self._move_head_stage_head_up()
    

    # _head_set(head: float, neck: float):
    #   设置头的角度，并记录角度信息并发布
    #    @param head: 上下角度，[0,1.5]，1.5下，0上
    #    @param neck: 左右角度，[-1.1,1.1]，-1.1右，1.1左
    def _head_set(self, head = 0, neck = 0):
        head = np.clip(head, 0, 1.5)
        neck = np.clip(neck, -1.1, 1.1)
        self.head, self.neck = head, neck
        head_goal = JointState()
        head_goal.name = ["head", "neck"]
        head_goal.header = Header()
        head_goal.position = [head, neck]
        self._head_pub.publish(head_goal) 


    def _position_callback(self, msg):
        self.self_pos = np.array([msg.x, msg.y])
        self.pos_yaw  = msg.theta


    def _soccer_real_callback(self, msg):
        self._ball_in_map = np.array([msg.x, msg.y])


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
            elif(int(row[0]) <= 5):
                self._self_pos_accuracy += \
                        self._config["pos_accuracy_add"][str(int(row[0]))]

        if(ball_row is not None):
            self._ball_pos_in_vis_D = \
                    (self._ball_pos_in_vis - ball_row[1:3]) * \
                    0.0001 / (time.time() - self._vision_last_frame_time)
            self._ball_pos_in_vis_I = self._ball_pos_in_vis_I * \
                    self._config["looking_at_ball_integrated"] +  \
                    self._ball_pos_in_vis; 

            self._ball_pos_in_vis           = ball_row[1:3]
            self.ball_distance              = ball_row[4]
            self._ball_pos                  = ball_row[5:8]
            self._ball_pos_accuracy        += ball_confidence
        
        self._vision_last_frame_time        = time.time()
        
        self._move_head(); 
