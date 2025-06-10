# interfaces/action.py
#
#   @description :   The py files to call actions
#
#   @interfaces :
#       1. class Action
#

import os
import rclpy
import actionlib
from bitbots_msgs.msg import KickGoal, KickAction
from geometry_msgs.msg import Quaternion, Twist, Pose2D, Point
from thmos_msgs.msg import Location, VisionDetections, VisionObj, HeadPose
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32MultiArray, Header

class Action:
    # @public variants:
    #   None
    #
    # @public methods;
    #   cmd_vel(vel_x, vel_y, vel_theta) : publish a velocity command
    #   do_kick()                        : publish a kick command 
    #   
    # @private variants:
    #   _config             dictionary of configurations 
    #   _cmd_vel_pub        The publisher for /cmd_vel
    # 
    # @private methods
    #   _done_cb()
    #   _feedback_cb()
    #   _action_cb()        Do nothing
    
    def __init__(self, agent): 
        super().__init__('action_node')

        self.logger = agent.get_logger().get_child("action_node")
        self._config = self.agent._config

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self._cmd_vel_pub = self.create_publisher(Twist, "/THMOS/walk/move", 1)
        self._head_pose_pub = self.create_publisher(
            HeadPose,
            "hardware/set_head_pose",
            qos_profile
        )
            
    def _move_head(self, pitch, yaw):

        head_pose_msg = HeadPose()
        head_pose_msg.header = Header()
        head_pose_msg.header.stamp = self.get_clock().now().to_msg()
        head_pose_msg.pitch = pitch
        head_pose_msg.yaw = yaw

        self._head_pose_pub.publish(head_pose_msg)

    # cmd_vel(vel_x, vel_y, vel_theta)
    #   publish a velocity command
    def cmd_vel(self, vel_x, vel_y, vel_theta):
        cmd = Twist()
        cmd.linear.x = vel_x
        cmd.linear.y = vel_y
        cmd.angular.z = vel_theta
        self._cmd_vel_pub.publish(cmd)
