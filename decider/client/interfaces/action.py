# interfaces/action.py
#
#   @description :   The py files to call actions
#
#   @interfaces :
#       1. class Action
#

import os
import rclpy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Quaternion, Twist, Pose2D, Point
from thmos_msgs.msg import VisionDetections, VisionObj
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32MultiArray, Header, Int32


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
        self.agent = agent
        self.logger = agent.get_logger().get_child("action_node")
        self._config = self.agent._config

        self._cmd_vel_pub = self.agent.create_publisher(Twist, "/THMOS/walk/move", 1)
        self._head_pose_pub = self.agent.create_publisher(
            JointState, "/THMOS/head_control/manual_command", 1
        )
        self._kick_pub = self.agent.create_publisher(Int32, "/THMOS/motion/kick", 1)
        self._save_pub = self.agent.create_publisher(Int32, "/THMOS/motion/save", 1)

    def _move_head(self, pitch, yaw):

        head_pose_msg = JointState()
        head_pose_msg.header = Header()
        head_pose_msg.header.stamp = self.agent.get_clock().now().to_msg()
        head_pose_msg.position = [0.0, 0.0]
        head_pose_msg.position[0] = float(yaw)  # Set yaw
        head_pose_msg.position[1] = float(pitch)

        self._head_pose_pub.publish(head_pose_msg)

    # cmd_vel(vel_x, vel_y, vel_theta)
    #   publish a velocity command
    def cmd_vel(self, vel_x, vel_y, vel_theta):
        # convert to float
        vel_x = float(vel_x)
        vel_y = float(vel_y)
        vel_theta = float(vel_theta)

        cmd = Twist()
        cmd.linear.x = vel_x
        cmd.linear.y = vel_y
        cmd.angular.z = vel_theta
        self._cmd_vel_pub.publish(cmd)

    # kick()
    #   publish a kick command
    def kick(self):
        kick_msg = Int32()
        kick_msg.data = 1
        self._kick_pub.publish(kick_msg)
        self.logger.info("Kick command published")

    def save_l(self):
        save_msg = Int32()
        save_msg.data = 1
        self._kick_pub.publish(save_msg)
        self.logger.info("Save command published")

    def save_r(self):
        save_msg = Int32()
        save_msg.data = 2
        self._kick_pub.publish(save_msg)
        self.logger.info("Save command published")
