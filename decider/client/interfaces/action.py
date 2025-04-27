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
        self._cmd_vel_pub = rospy.Publisher("/user_command", \
                                        Twist,  \
                                        queue_size = 1)

    # cmd_vel(vel_x, vel_y, vel_theta)
    #   publish a velocity command
    def cmd_vel(self, vel_x, vel_y, vel_theta):
        if(abs(vel_x) > 1 or abs(vel_y) > 1 or abs(vel_theta) > 1):
            rospy.logwarn("The velocity should be in [-1, 1].")
        cmd = Twist()
        cmd.linear.x = vel_x * self._config["walk_vel_x"]
        cmd.linear.y = vel_y * self._config["walk_vel_y"]
        cmd.angular.z = vel_theta * self._config["walk_vel_theta"]
        self._cmd_vel_pub.publish(cmd)

    # do_kick()
    #   kick the ball
    def do_kick(self):
        rospy.logerr("KICK NOT SUPPORT ON PI YET!")
        return;

    # _action_cb(), etc. : private
    #   Do nothing
    def _done_cb(self):
        pass
    def _feedback_cb(self):
        pass
    def _action_cb(self):
        pass
