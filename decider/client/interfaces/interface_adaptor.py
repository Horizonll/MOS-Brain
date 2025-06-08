# interfaces.py
#   @description: Interfaces with other components
#   @interfaces:
#       1. void cmd_vel(vel_x: float, vel_y: float, vel_theta: float)
#
#
#       2. void do_kick()  (Deprecated)
#
#       3. float get_imu()
#
#       4. list get_odometer():
#           @return: a list of odometer data, [x, y, theta]
#       
#       5. list[list] get_vision_detetion():
#           list[0]: [label = 'header', [sec, nanosec], frame_id]
#           list[i]: [label, confidence, [[xmin, ymin], [xmax, ymax]], position]
#               label: ["L", "T", "X", "P"]
#       

import os, sys, time, random, math, threading
from tf_transformations import euler_from_quaternion

try:
    from booster_robotics_sdk_python import B1LocoClient, ChannelFactory
except Exception as e:
    print("Fatal: Can not load booster SDK!")
    exit(-1)

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Header, Float32
    from geometry_msgs.msg import Pose, Vector3
    from sensor_msgs.msg import CameraInfo
    from rclpy.executors import SingleThreadedExecutor
except Exception as e:
    print("Fatal: Can not import ros2 standard library: " )
    print(e)
    print("Have you source the ros2 setup.bash ?")
    exit(-1)

try:
    from booster_interface.msg import LowState, ImuState
    # from booster_interface.msg import Odometer
    from nav_msgs.msg import Odometry
except Exception as e:
    print("Fatal: Can not import ros2 messages: " )
    print(e)
    print("Have you source the booster_ros2_interface packages ?")
    exit(-1)

class Interfaces(Node):
    def __init__(self):
        super().__init__("interface_booster", namespace="THMOS")
        self.logger = self.get_logger()

        # parameters
        self.declare_parameter('network_interface', '192.168.10.102')

        # action interfaces
        ChannelFactory.Instance().Init(0, self.get_parameter('network_interface').value)
        self._client = B1LocoClient()
        self._client.Init()
        time.sleep(0.4) # wait for the B1LocoClient to initialize

        # receiveing from booster topics
        self._imu_sub = self.create_subscription(LowState, \
                    "/low_state", self._imu_cb, 1)
        self._odometer_sub = self.create_subscription(Odometry, \
                    "/odom", self._odometer_cb, 1)
        # self._headpose_sub = self.create_subscription(Odometer, \
        #             "/head_pose", self._headpose_cb, 1)

        # redirecting to THMOS topics 
        self._head_pose_pub = self.create_publisher(Pose, \
                                                    "/THMOS/hardware/head_pose", 1)
        self._facing_pub = self.create_publisher(Float32, \
                                              "/THMOS/hardware/facing", 1)
        self._odometer_pub = self.create_publisher(Vector3, \
                                              "/THMOS/hardware/odometer", 1)
        self._move_sub = self.create_subscription(Vector3, \
                                                  "/THMOS/walk/move", \
                                                  self._move_cb, 1)
        self._head_sub = self.create_subscription(Vector3, \
                                                        "/THMOS/hardware/move_head", \
                                                        self._move_head_cb, 1)


    def _move_cb(self, msg: Vector3):
        msg.x *= self.get_parameter("walk_velocity_x").value
        msg.y *= self.get_parameter("walk_velocity_y").value
        msg.z *= self.get_parameter("walk_velocity_theta").value
        self._client.Move(msg.x, msg.y, msg.z)


    def _move_head_cb(self, msg: Vector3):
        self._client.RotateHead(msg.y, msg.z)


    def _imu_cb(self, msg: LowState):
        msg_pub = Float32()
        msg_pub.data = float(msg.imu_state.rpy[2])
        self._facing_pub.publish(msg_pub)


    def _odometer_cb(self, msg: Odometer):
        msg_pub = Vector3()
        msg_pub.x = msg.x
        msg_pub.y = msg.y
        msg_pub.z = msg.theta
        self._odometer_pub.publish(msg_pub)


    def _headpose_cb(self, msg: Pose):
        quaternion = (
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        )
        roll_rad, pitch_rad, yaw_rad = euler_from_quaternion(quaternion)
        self._head_pose_pub.publish(Vector3(roll_rad, pitch_rad, yaw_rad))
    

    def spin(self) -> None:
        self.logger.info("interface-booster node started")
        rclpy.spin(self._node)



if __name__ == "__main__":
    rclpy.init(args = sys.argv)
    node = Interfaces()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

