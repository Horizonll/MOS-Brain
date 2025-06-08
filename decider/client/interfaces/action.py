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

    # cmd_vel(vel_x, vel_y, vel_theta)
    #   publish a velocity command
    def cmd_vel(self, vel_x, vel_y, vel_theta):
        cmd = Twist()
        cmd.linear.x = vel_x # * self._config["walk_vel_x"]
        cmd.linear.y = vel_y # * self._config["walk_vel_y"]
        cmd.angular.z = vel_theta # * self._config["walk_vel_theta"]
        self._cmd_vel_pub.publish(cmd)
