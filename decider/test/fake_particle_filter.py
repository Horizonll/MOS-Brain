import os, sys, time, random, math, threading
from tf_transformations import euler_from_quaternion, quaternion_from_euler
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from booster_interface.msg import LowState
from geometry_msgs.msg import Pose, Vector3, Twist, Pose2D, Quaternion
from sensor_msgs.msg import Imu

class Interfaces(Node):
    def __init__(self, config={}):
        super().__init__("interface_booster", namespace="THMOS")
        self.logger = self.get_logger()

        # receiveing from booster topics
        self._imu_sub = self.create_subscription(
            LowState,
            "/low_state",
            self._imu_cb,
            1
        )

        self._location_pub = self.create_publisher(Pose2D, \
                                                "/THMOS/location_fake", 1)

        self.logger.info("Registered topics")
        
        # Initialize position offset
        self._position_offset = Pose2D()
        self._position_offset.x = 0.0
        self._position_offset.y = 0.0
        self._position_offset.theta = 0.0

        self.logger.info("Node running ...")


    def _imu_cb(self, msg: LowState):
        msg_pub = Pose2D()
        imu_data = msg.imu_state
        
        yaw = float(imu_data.rpy[2])
        
        msg_pub.x = 0.0
        msg_pub.y = 0.0
        msg_pub.theta = yaw + self._position_offset.theta
        self._location_pub.publish(msg_pub)



if __name__ == "__main__":
    rclpy.init(args = sys.argv)
    node = Interfaces()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
