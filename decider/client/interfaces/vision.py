import math
import time
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Header
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import Quaternion, Twist, Pose2D, Point
from nav_msgs.msg import Odometry
from thmos_msgs.msg import VisionDetections, VisionObj, HeadPose
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from queue import Queue

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
        self._last_ball_time = 0
        self.self_pos = np.array([0,0])
        self.self_yaw = 0
        self._self_pos_accuracy = 0
        self._ball_pos_accuracy = 0
        self.ball_distance = 6000
        self._search_ball_phase = 0
        self._ball_history = Queue(maxsize=20)

        self._config = self.agent._config
        
        # 设置默认配置参数
        self._config.setdefault("ball_pos_accuracy_critical", 0.5)
        self._config.setdefault("max_valid_ball_distance", 6000)  # 默认6米
        self._config.setdefault("max_ball_data_age", 0.5)  # 默认0.5秒
        
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
            "/THMOS/vision/obj_pos",
            self._vision_callback,
            qos_profile
        )

        self._relocal_pub = self.agent.create_publisher(
            Pose2D,
            "/THMOS/relocalization",
            1
        )


    def _position_callback(self, msg):
        self.self_pos = np.array([msg.x, msg.y])
        self.self_yaw = self.agent.angle_normalize(msg.theta) * 180 / np.pi


    def _soccer_real_callback(self, msg):
        self._ball_pos_in_map = np.array([msg.x, msg.y])

    def _vision_callback(self, msg: VisionDetections):
        """
        处理机器人位置信息并结合视觉检测计算各种物体的绝对坐标
        
        Args:
            msg (VisionDetections): 包含检测到的物体信息的消息
        """
        # 更新视觉数据时间戳
        self._vision_last_frame_time = time.time()
        
        # 输出调试信息
        self.logger.debug(f"Robot position: ({self.self_pos[0]:.2f}, {self.self_pos[1]:.2f})")
        self.logger.debug(f"Robot yaw: {self.self_yaw:.2f} radians")
        
        # 用于存储所有检测到的物体信息
        self._detected_objects = []
        self._ball_pos_accuracy = 0  # 默认无球
        
        # 处理所有检测到的物体
        for obj in msg.detected_objects:
            label = obj.label
            
            # 验证边界框是否有效
            if obj.xmin >= obj.xmax or obj.ymin >= obj.ymax:
                self.logger.warning(f"Invalid bounding box received for {label}")
                continue
                
            # 计算目标中心坐标
            curr_coord = (np.array([obj.xmin, obj.ymin]) + 
                        np.array([obj.xmax, obj.ymax])) * 0.5
            
            # 获取position_projection (x, y) 坐标
            position_projection = np.array(obj.position_projection)
            # 裁剪前两个值
            position_projection = position_projection[:2]
            if position_projection.shape != (2,):
                self.logger.error(f"Invalid position_projection format for {label}, expected 2D coordinates")
                continue

            # 如果有nan值，记录错误并跳过
            if np.isnan(position_projection).any():
                self.logger.error(f"NaN values found in position_projection for {label}")
                continue

            # 保存相对坐标（单位：米）
            relative_pos = position_projection / 1000
            
            # 计算到物体的距离
            distance = np.linalg.norm(position_projection) / 1000  # 转换为米
            
            # 计算物体在球场中的绝对坐标
            # 创建旋转矩阵 (假设机器人坐标系与全局坐标系的转换)
            # 注意: 这里假设self_yaw是绕z轴的旋转角（符合ROS惯例）
            rotation_matrix = np.array([
                [np.cos(self.self_yaw/180*math.pi), -np.sin(self.self_yaw/180*math.pi)],
                [np.sin(self.self_yaw/180*math.pi), np.cos(self.self_yaw/180*math.pi)]
            ])
            
            # 将相对坐标旋转到全局坐标系
            rotated_relative = rotation_matrix @ relative_pos
            
            # 计算绝对坐标（全局坐标系，单位：米）
            absolute_coord = self.self_pos + rotated_relative
            
            # 保存该物体的计算结果
            object_info = {
                'label': label,
                'relative_pos': relative_pos,
                'absolute_pos': absolute_coord,
                'distance': distance,
                'confidence': obj.confidence,
                'bounding_box_center': curr_coord / 1000,
                'bound_left_low': np.array(obj.bound_left_low[:2]) if obj.bound_left_low else None,
                'bound_right_low': np.array(obj.bound_right_low[:2]) if obj.bound_right_low else None,
                'timestamp': time.time()
            }
            
            self._detected_objects.append(object_info)
            
            # 输出该物体信息
            self.logger.debug(f"Detected {label}:")
            self.logger.debug(f"  Relative coordinates: ({relative_pos[0]:.2f}, {relative_pos[1]:.2f})")
            self.logger.debug(f"  Absolute coordinates on field: ({absolute_coord[0]:.2f}, {absolute_coord[1]:.2f})")
            self.logger.debug(f"  Distance: {distance:.2f} meters")
            self.logger.debug(f"  Confidence: {obj.confidence:.2f}")
        
        # 特别处理球（如果存在）
        ball_objects = [obj for obj in self._detected_objects if obj['label'] == 'ball']
        if ball_objects:
            # 选择置信度最高的球
            best_ball = max(ball_objects, key=lambda b: b['confidence'])
            
            # 更新球相关的变量
            self._ball_pos = best_ball['relative_pos']
            self._ball_pos_in_map = best_ball['absolute_pos']
            self._ball_pos_in_vis = best_ball['bounding_box_center']
            self._ball_pos_accuracy = best_ball['confidence']
            self.ball_distance = best_ball['distance']
            self._last_ball_time = best_ball['timestamp']
            self._ball_history.put({
                'pos': best_ball['relative_pos'],
                'pos_in_map': best_ball['absolute_pos'],
                'pos_in_vis': best_ball['bounding_box_center'],
                'distance': best_ball['distance'],
                'timestamp': best_ball['timestamp']
            })

    def get_objects(self):
        """
        获取检测到的所有物体信息
        
        Returns:
            list: 包含所有检测到物体的字典列表
        """
        return self._detected_objects

    def get_ball_pos(self):
        return self._ball_pos

    def get_ball_pos_in_vis(self):
        return self._ball_pos_in_vis

    def get_ball_pos_in_map(self):
        return self._ball_pos_in_map

    def get_if_ball(self):
        """
        判断是否成功检测到球
        
        Returns:
            bool: True表示成功检测到球，False表示未检测到
        """
        
        # 检查是否有有效的视觉数据
        if np.all(self._ball_pos_in_vis == 0):
            return False
        
        # 检查视觉数据的时间戳，确保数据是最近的
        current_time = time.time()
        if current_time - self._last_ball_time > 0.5:
            return False
        
        # 如果所有检查都通过，返回True表示检测到球
        return True

    def get_ball_history(self):
        """
        获取球的历史位置数据
        
        Returns:
            list: 包含球的历史位置和时间戳的字典列表
        """
        return list(self._ball_history.queue)

    def relocal(self, x=0, y=0, theta=0):
        """
        重新定位，重置视觉数据
        """
        msg = Pose2D()
        msg.x = float(x)
        msg.y = float(y) # m
        msg.theta = float(theta) # degree
        
        self._relocal_pub.publish(msg)

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
