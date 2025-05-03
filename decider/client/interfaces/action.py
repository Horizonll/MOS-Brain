# interfaces/action.py
#
#   @description :   The py files to call actions
#
#   @interfaces :
#       1. class Action
#

import os
import rospy
import actionlib
from bitbots_msgs.msg import KickGoal, KickAction
from geometry_msgs.msg import Quaternion, Twist, Pose2D, Point

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
    
    def __init__(self, config): 
        self._config = config
        self._cmd_vel_pub = rospy.Publisher("/cmd_vel", \
                                        Twist,  \
                                        queue_size = 1)
        self._kick_client = actionlib.SimpleActionClient("thmos_animation", \
                                        KickAction)

    # cmd_vel(vel_x, vel_y, vel_theta)
    #   publish a velocity command
    def cmd_vel(self, vel_x, vel_y, vel_theta):
        cmd = Twist()
        cmd.linear.x = vel_x # * self._config["walk_vel_x"]
        cmd.linear.y = vel_y # * self._config["walk_vel_y"]
        cmd.angular.z = vel_theta # * self._config["walk_vel_theta"]
        self._cmd_vel_pub.publish(cmd)

    # do_kick()
    #   kick the ball
    def do_kick(self):
        kick_goal = KickGoal()
        kick_goal.header.seq = 1
        kick_goal.header.stamp = rospy.Time.now()
        frame_prefix = "" if os.environ.get("ROS_NAMESPACE") is None \
                else os.environ.get("ROS_NAMESPACE") + "/"
        kick_goal.header.frame_id = frame_prefix + "base_footprint"
        if self.config["kick_key_is_y"]:
            kick_goal.ball_position.x = 0.2
            kick_goal.ball_position.y = 0.1
            kick_goal.ball_position.z = 0
            kick_goal.kick_direction = Quaternion(1, 0, 0, 0)
            kick_goal.kick_speed = 3
        self._kick_client.send_goal(kick_goal)
        rospy.loginfo("send kick")
        rospy.loginfo("kick goal init")
        self._kick_client.done_cb = self._done_cb
        self._kick_client.feedback_cb = self._feedback_cb
        self._kick_client.active_cb = self._action_cb
        rospy.loginfo("kick done")

    # _action_cb(), etc. : private
    #   Do nothing
    def _done_cb(self):
        pass
    def _feedback_cb(self):
        pass
    def _action_cb(self):
        pass
