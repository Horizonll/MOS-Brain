# interfaces/vision.py
#   @description:   Subscribe position
#

import math
import time
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Header
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import Quaternion, Twist, Pose2D, Point
from nav_msgs.msg import Odometry
from thmos_msg.msg import VisionDetections, VisionObj, HeadPose
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class Vision(Node):
    # @public variants:
    #   VARIANTS        TYPE        DESCTIPTION
    #   self_pos        np.array    self position in map; already filtered
    #   self_yaw         angle       self orientation in degree, [-180, 180)
    #   head            float       the angle of head
    #   neck            float       the angle of neck
    #   ball_distance   float       the distance from robot to tht ball
    #
    # @public methods:
    #   look_at(args)
    #       disable automatically tracking and force to look_at
    #       use (NaN, NaN) to enable tracking
    #                                       
    #
    # @private variants
    #   _ball_pos_in_vis        the pixel coordinate of ball
    #   _ball_pos               the ball position from robot
    #   _ball_pos_in_map        the ball position in the whole map
    #   _ball_pos_accuracy      the 'accuracy' of ball_pos
    #   _self_pos_accuracy      the 'accuracy' of self_pos, the algorithm
    #                           to measure how 'inaccurary' has to be
    #                           improve. 
    #   _force_look_at          a list (head, neck) to force camera orientation
    #   @@@@ TODO IMPROVE THE ALGORITHM TO MEASURE INACCURACY @@@@
    #   _vision_last_frame_time     the timestamp of last frame of vision
    #   _config                 a dictionary, the config
    #   _pos_sub                the handler of /pos_in_map
    #   _vision_sub             the handler of /vision/obj_pos
    #   _soccer_real_sub        the handler of /soccer_real_pos_in_map
    #   _head_pub               the handler of /head_goals
    #   _last_track_ball_time    last timestamp running _track_ball()
    #   _track_ball_stage        the stage ( FSMID ) of _track_ball()
    #   _last_track_ball_phase_time      timestamp for changing phase periodicly
    #
    # @private methods
    #   _head_set(args: float[2])           set head and neck angle
    #   _position_callback(msg)             callback of /pos_in_map
    #   _soccer_real_callback(msg)          callback of /soccer_real_pos_in_map
    #   _vision_callback(target_matrix)     callback of /vision/obj_pos
    #   _track_ball()                        the main algorithm to move head
    #   _track_ball_stage_looking_at_ball()  looing at ball algorithm

    def __init__(self, agent):
        self.agent = agent
        self.logger = agent.get_logger().get_child("vision_node")
        
        self._ball_pos_in_vis = np.array([0, 0]) # 
        self._ball_pos_in_vis_D = np.array([0, 0])
        self._ball_pos_in_vis_I = np.array([0, 0])
        self._ball_pos = np.array([0, 0]) # mm
        self._ball_pos_in_map = np.array([0, 0]) # mm
        self._vision_last_frame_time = 0
        self.self_pos = np.array([0,0])
        self.self_yaw = 0
        self._self_pos_accuracy = 0
        self._ball_pos_accuracy = 0
        self.ball_distance = 6000
        self._search_ball_phase = 0

        self._config = self.agent._config
        
        # Configure QoS profile for subscriptions
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Create subscribers
        self._location_sub = self.agent.create_subscription(
            Pose2D,
            "/THMOS/location",
            self._position_callback,
            qos_profile
        )
        
        self._vision_sub = self.agent.create_subscription(
            VisionDetections,
            "/THMOS/vision/detections",
            self._vision_callback,
            qos_profile
        )


    def _position_callback(self, msg):
        self.self_pos = np.array([msg.x, msg.y])
        self.self_yaw = msg.theta


    def _soccer_real_callback(self, msg):
        self._ball_pos_in_map = np.array([msg.x, msg.y])

    def _vision_callback(self, msg: VisionDetections):
        """
        处理机器人位置信息并结合视觉检测计算球的绝对坐标
        
        Args:
            msg (RobotPosition): 包含机器人位置和朝向的消息
        """
        
        # 输出调试信息
        self.logger.debug(f"Robot position: ({self.self_pos[0]:.2f}, {self.self_pos[1]:.2f})")
        self.logger.debug(f"Robot yaw: {self.self_yaw:.2f} radians")

        ball_objects = [obj for obj in msg.detected_objects if obj.label == "Ball"]
        if not ball_objects:
            self.logger.info("No ball objects detected")
            return  
        
        # Find the best ball with highest confidence
        best_ball = max(ball_objects, key=lambda obj: obj.confidence)

        # Validate message
        if best_ball.xmin >= best_ball.xmax or best_ball.ymin >= best_ball.ymax:
            self.logger.warning("Invalid bounding box received for best ball")
            self._last_coord = None  # Invalid bounding box
            return
            
        # Calculate target center coordinates
        curr_coord = (np.array([best_ball.xmin, best_ball.ymin]) + 
                      np.array([best_ball.xmax, best_ball.ymax])) * 0.5
        
        # 获取position_projection (x, y) 坐标
        # 假设_position_projection格式为 [x, y]，其中y朝前，z朝上
        position_projection = np.array(best_ball.position_projection) * 1000 # 转换为毫米
        # 裁剪前两个值
        position_projection = position_projection[:2]
        if position_projection.shape != (2,):
            self.logger.error("Invalid position_projection format, expected 2D coordinates")
            return
        # 保存相对坐标
        self._ball_pos = position_projection
        
        # 计算到球的距离
        distance = np.linalg.norm(position_projection) / 1000  # 转换为米
        
        # 计算球在球场中的绝对坐标
        # 创建旋转矩阵 (假设机器人坐标系与全局坐标系的转换)
        # 注意: 这里假设self_yaw是绕z轴的旋转角（符合ROS惯例）
        rotation_matrix = np.array([
            [np.cos(self.self_yaw), -np.sin(self.self_yaw)],
            [np.sin(self.self_yaw), np.cos(self.self_yaw)]
        ])
        
        # 将相对坐标旋转到全局坐标系
        rotated_relative = rotation_matrix @ position_projection
        
        # 计算绝对坐标（全局坐标系）
        absolute_coord = self.self_pos + rotated_relative
        
        # 保存计算结果
        self._ball_distance = distance
        self._ball_pos_in_map = absolute_coord
        self._ball_pos_in_vis = curr_coord
        
        # 输出信息
        self.logger.info(f"Ball relative coordinates: ({position_projection[0]:.2f}, {position_projection[1]:.2f})")
        self.logger.info(f"Estimated distance to ball: {distance:.2f} meters")
        self.logger.info(f"Ball absolute coordinates on field: ({absolute_coord[0]:.2f}, {absolute_coord[1]:.2f})")

    def get_ball_pos(self):
        return self._ball_pos

    def get_ball_pos_in_vis(self):
        return self._ball_pos_in_vis

    def get_ball_pos_in_map(self):
        return self._ball_pos_in_map

    def get_if_ball(self):
        return self._ball_pos_accuracy > self._config["ball_pos_accuracy_critical"]


def main(args=None):
    rclpy.init(args=args)
    config = {
    }
    
    vision_node = Vision(config)
    rclpy.spin(vision_node)
    
    vision_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()    