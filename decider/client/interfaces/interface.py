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

import os, sys, time, random, math

try:
    from booster_robotics_sdk_python import B1LocoClient, ChannelFactory
except Exception as e:
    print("Fatal: Can not load booster SDK!")
    exit(-1)

try:
    import rclpy
    from std_msgs.msg import Header
    from geometry_msgs.msg import Pose
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
                            "Goalpost": "P", \
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
        self._imu_data = math.nan
        self._detection_data = [['header', [0, 0], 0]]
        self._odometer_data = [0, 0, 0]


    def cmd_vel(self, vel_x: float, vel_y: float, vel_theta: float):
        vel_x *= self._config.get("walk_vel_x", 1)
        vel_y *= self._config.get("walk_vel_y", 1)
        vel_theta *= self._config.get("walk_vel_theta", 1)
        self._client.Move(vel_x, vel_y, vel_theta)


    def do_kick(self):
        print("Warning: kick is not supported")
        return

    
    def get_imu(self) -> float:
        return self._imu_data

    
    def get_detection(self) -> list:
        return self._detection_data
   

    def get_odometer(self) -> list:
        return self._odometer_data


    def spin_once(self) -> None:
        rclpy.spin_once(self._node)


    def _imu_callback(self, msg: LowState):
        self._imu_data = msg.imu_state.rpy[2]


    def _detection_callback(self, msg: Detections):
        header = msg.header
        data = [['header', \
                 [header.stamp.sec, header.stamp.nanosec], \
                header.frame_id]]
        for obj in msg.detected_objects:
            obj_data = [self._label_map.get(obj.label, obj.label), \
                        [[obj.xmin, obj.ymin], [obj.xmax, obj.ymax]], \
                        obj.position_projection.tolist()]
            data.append(obj_data)
        self._detection_data = data

    
    def _odometer_callback(self, msg: Odometer):
        self._odometer_data = [msg.x, msg.y, msg.theta]



if __name__ == "__main__":
    A = Interfaces(config = {})
    try:
        while rclpy.ok():
            A.spin_once()
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


