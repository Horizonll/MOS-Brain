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
    from std_msgs.msg import Header
    from geometry_msgs.msg import Pose
    from sensor_msgs.msg import CameraInfo
    from rclpy.executors import SingleThreadedExecutor
except Exception as e:
    print("Fatal: Can not import ros2 standard library: " )
    print(e)
    print("Have you source the ros2 setup.bash ?")
    exit(-1)

try:
    from booster_interface.msg import LowState, ImuState
    from booster_interface.msg import Odometer
except Exception as e:
    print("Fatal: Can not import ros2 messages: " )
    print(e)
    print("Have you source the booster_ros2_interface packages ?")
    exit(-1)

try:
    from vision_interface.msg import Detections, DetectedObject
except Exception as e:
    print("Fatal: Can not import ros2 messages: " )
    print(e)
    print("Have you source the booster_ros2_interface packages ?")
    exit(-1)


class Interfaces:
    def __init__(self, config = {}):
        self._config = config
        rclpy.init(args = sys.argv)
        self._node = rclpy.create_node('MOS_Brain')
        self._label_map = { "LCross": "L", \
                            "XCross": "X", \
                            "TCross": "T", \
                            "PenaltyPoint": "P", \
                            "Goalpost": "G", \
                            "Ball": "B", \
                            "Opponent": "R" }


        # action interfaces
        ChannelFactory.Instance().Init(0, \
                self._config.get("network_interface", "192.168.10.102"))
        self._client = B1LocoClient()
        self._client.Init()
        time.sleep(0.4) # wait for the B1LocoClient to initialize

        # ros2 ropic subscription
        self._imu_sub = self._node.create_subscription(LowState, \
                    "/low_state", self._imu_callback, 1)
        self._detection_sub = self._node.create_subscription(Detections, \
                    "/booster_vision/detection", self._detection_callback, 1)
        self._odometer_sub = self._node.create_subscription(Odometer, \
                    "/odometer_state", self._odometer_callback, 1)
        self._headpose_sub = self._node.create_subscription(Odometer, \
                    "/head_pose", self._headpose_callback, 1)
        self._camera_size_sub = self._node.create_subscription(Odometer, \
                    "/camera/camera/aligned_depth_to_color/camera_info", \
                    self._camera_size_callback, 1)
        self._imu_data = math.nan
        self._detection_data = [['header', [0, 0], 0]]
        self._ball_data = [['header', [0, 0], 0]]
        self._odometer_data = [0, 0, 0]
        self._headpose_data = [0, 0]
        self._camera_size_data = [0, 0]

        # multiple threading
        if self._config.get("multithread", True) == True:
            self._executor = SingleThreadedExecutor();
            self._executor.add_node(self._node)
            self._executor_thread = threading.Thread(target = self.spin, \
                                                     args = ())
            self._executor_thread.start()
        else:
            print("Warn: run in single thread mode; you have to call" + \
                  "Interfaces.spin_once() continueously")


    def cmd_vel(self, vel_x: float, vel_y: float, vel_theta: float):
        vel_x *= self._config.get("walk_vel_x", 1)
        vel_y *= self._config.get("walk_vel_y", 1)
        vel_theta *= self._config.get("walk_vel_theta", 1)
        self._client.Move(vel_x, vel_y, vel_theta)


    def head_goal(self, args : list):
        self._client.RotateHead(args[0], args[1])


    def do_kick(self):
        print("Warning: kick is not supported")
        return

    
    def get_imu(self) -> float:
        return self._imu_data

    
    def get_detection(self) -> list:
        return self._detection_data
    
   
    def get_headpose(self) -> list:
        return self._headpose_data

    def get_camera_size(self) -> list:
        return self._camera_size_data

    def get_ball(self) -> list:
        return self._ball_data


    def get_odometer(self) -> list:
        return self._odometer_data


    def spin_once(self) -> None:
        rclpy.spin_once(self._node)


    def spin(self) -> None:
        rclpy.spin(self._node)


    def _imu_callback(self, msg: LowState):
        self._imu_data = msg.imu_state.rpy[2]


    def _detection_callback(self, msg: Detections):
        header = msg.header
        data = [['header', \
                 [header.stamp.sec, header.stamp.nanosec], \
                header.frame_id]]
        self._ball_data = data
        for obj in msg.detected_objects:
            obj_data = [self._label_map.get(obj.label, obj.label), \
                        obj.confidence, \
                        [[obj.xmin, obj.ymin], [obj.xmax, obj.ymax]], \
                        obj.position_projection.tolist()]
            if(obj_data[0] == 'B'):
                self._ball_data.append(obj_data)
            data.append(obj_data)
        self._detection_data = data

    
    def _odometer_callback(self, msg: Odometer):
        self._odometer_data = [msg.x, msg.y, msg.theta]


    def _headpose_callback(self, msg: Pose):
        quaternion = (
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        )
        roll_rad, pitch_rad, yaw_rad = euler_from_quaternion(quaternion)
        self._headpose_data = [pitch_rad, yaw_rad]

    
    def _camera_size_callback(self, msg: CameraInfo):
        self._camera_size_data = [msg.width, msg.height]



if __name__ == "__main__":
    A = Interfaces(config = {})
    A.head_goal([0.4, 0])
    try:
        while rclpy.ok():
            time.sleep(0.01)
            imu_data = A.get_imu()
            det_data = A.get_detection()
            odo_data = A.get_odometer()
            print(f"imu: {imu_data}")
            print(f"odo: {odo_data}")
            print("detection:")
            for line in det_data:
                print(line)
            print("")
    except KeyboardInterrupt:
        pass
    pass


