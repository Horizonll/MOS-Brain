import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
import threading
from transitions import Machine
import time
import numpy as np
import math
from subscriber import *
from receiver import Receiver
from config import *
import socket
import json
import asyncio
import websockets
import logging


class Agent(Decision_Pos, Decision_Motion, Decision_Vision, config):
    def __init__(self, role="RF", team=1, player=0, goal_keeper=False, rec_debug=False):
        print("Initializing Agent instance...")

        # 初始化父类
        Decision_Pos.__init__(self)
        print("Initialized Decision_Pos.")
        Decision_Motion.__init__(self)
        print("Initialized Decision_Motion.")
        Decision_Vision.__init__(self)
        print("Initialized Decision_Vision.")

        # 初始化ROS节点
        rospy.init_node("decider")
        print("Initialized ROS node 'decider'.")

        # 设置ID
        self.id = 1
        print(f"Set ID to: {self.id}")

        # 初始化发布者
        self.speed_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        print("Initialized speed publisher on '/cmd_vel'.")
        self.joint_goal_publisher = rospy.Publisher("motor_goals", JointState, queue_size=1)
        print("Initialized joint goal publisher on 'motor_goals'.")

        # 初始化接收器
        # self.receiver = Receiver(team=team, player=player, goal_keeper=goal_keeper, debug=rec_debug)
        # print("Initialized Receiver with team={}, player={}, goal_keeper={}, debug={}".format(team, player, goal_keeper, rec_debug))

        # 初始化命令和状态
        self.info = "stop"
        print("Set initial info to 'stop'.")
        self.command = {
            "command": "stop",
            "data": {},
            "send_time": time.time(),
        }
        print("Set initial command to 'stop':", self.command)

        self.ready_to_kick = False
        print("Set ready_to_kick to False.")
        self.t_no_ball = 0
        print("Set t_no_ball to 0.")
        self.is_going_back_to_field = False
        print("Set is_going_back_to_field to False.")
        self.go_back_to_field_dist = None
        print("Set go_back_to_field_dist to None.")
        self.go_back_to_field_dir = None
        print("Set go_back_to_field_dir to None.")
        self.go_back_to_field_yaw_bias = None
        print("Set go_back_to_field_yaw_bias to None.")

        # detect local ip
        self.ip = self.detect_ip()

        # 监听主机IP
        print("Starting to listen for host IP...")
        # self.listen_host_ip()
        self.HOST_IP = '192.168.98.114'
        print("Finished listening for host IP.")

        # 启动发送线程
        send_thread = threading.Thread(target=self.send_loop)
        send_thread.daemon = True
        send_thread.start()
        print("Started send loop thread.")

        # 启动TCP客户端线程
        tcp_thread = threading.Thread(target=self.tcp_client)
        tcp_thread.daemon = True
        tcp_thread.start()
        print("Started TCP client thread.")

        # 注释掉的WebSocket客户端线程（如果需要可以取消注释）
        # websocket_thread = threading.Thread(
        #     target=lambda: asyncio.run(self.websocket_client())
        # )
        # websocket_thread.daemon = True
        # websocket_thread.start()
        # print("Started WebSocket client thread.")

        print("Agent instance initialization complete.")

    def send_loop(self):
        while True:
            try:
                robot_data = {
                    "id": self.id,
                    "data": {
                        "x": self.pos_x,
                        "y": self.pos_y,
                        "ballx": self.ball_x_in_map,
                        "bally": self.ball_y_in_map,
                        "yaw": self.pos_yaw,
                    },
                    "info": self.info,
                    "timestamp": time.time(),
                    "ip": self.ip
                }
                if not self.ifBall:
                    robot_data["ballx"] = robot_data["bally"] = None
                self.send_robot_data(robot_data)
                time.sleep(0.5)
            except Exception as e:
                print(f"Error in send: {e}")

    def send_robot_data(self, robot_data):
        try:
            client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            client_socket.connect((self.HOST_IP, 8001))
            client_socket.sendall(json.dumps(robot_data).encode("utf-8"))
        except Exception as e:
            print(f"Error in send_robot_data: {e}")
        finally:
            client_socket.close()

    def detect_ip(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            s.connect(("8.8.8.8", 80))
            ip = s.getsockname()[0]
        except Exception as e:
            logging.error(f"Failed to detect IP address: {e}")
            ip = '127.0.0.1'
        finally:
            s.close()
        return ip

    # async def websocket_client(self):
    #     uri = f"ws://{self.HOST_IP}:8001"
    #     while True:
    #         try:
    #             async with websockets.connect(uri) as websocket:
    #                 while True:
    #                     data = await websocket.recv()
    #                     try:
    #                         received_data = json.loads(data)
    #                         robot = next(
    #                             (
    #                                 r
    #                                 for r in received_data["robots"]
    #                                 if r["id"] == self.id
    #                             ),
    #                             None,
    #                         )
    #                         if robot:
    #                             self.command = robot["info"]
    #                         if not self.ifBall:
    #                             self.ball_x_in_map, self.ball_y_in_map = (
    #                                 received_data["ball"]["x"],
    #                                 received_data["ball"]["y"],
    #                             )
    #                     except json.JSONDecodeError as e:
    #                         print(f"JSON decode error: {e}")
    #         except websockets.ConnectionClosed as e:
    #             print(f"Connection closed: {e}")
    #         except ConnectionRefusedError as e:
    #             print(f"Connection refused: {e}. Retrying")
    #         except Exception as e:
    #             print(f"Error: {e}")

    async def tcp_client(self):
        """同步监听UDP广播消息并获取主机IP"""
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(("0.0.0.0", 8003))  # 监听指定端口上的UDP广播

        print("Listening for UDP broadcast messages on port 8003...")

        try:
            while True:
                data, addr = sock.recvfrom(1024)
                print(f"Received data from {addr}: {data}")

                try:
                    received_data = json.loads(data.decode("utf-8"))
                    print(f"Parsed JSON data: {received_data}")

                    if "message" in received_data and received_data["message"] == "thmos_hello":
                        # 假设广播消息中包含主机IP信息
                        if "ip" in received_data:
                            host_ip = received_data["ip"]
                            self.HOST_IP = host_ip  # 更新Agent实例中的IP地址
                            print(f"Updated IP to: {self.HOST_IP}")
                            break  # 退出监听循环
                        else:
                            print("Received 'thmos_hello' message but no 'ip' field found.")
                    else:
                        print("Received message does not match expected format.")
                except json.JSONDecodeError as e:
                    print(f"JSON decode error: {e}")
        except KeyboardInterrupt:
            print("\nInterrupted by user. Exiting...")
        finally:
            sock.close()
            print("Socket closed.")

    def listen_host_ip(self):
        """同步监听UDP广播消息并获取主机IP"""
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(('0.0.0.0', 8003))  # 监听指定端口上的UDP广播

        # 加入广播接收
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

        while True:
            data, addr = sock.recvfrom(1024)
            try:
                received_data = json.loads(data.decode('utf-8'))
                if 'message' in received_data and received_data['message'] == 'thmos_hello':
                    # 假设广播消息中包含主机IP信息
                    if 'ip' in received_data:
                        host_ip = received_data['ip']
                        self.HOST_IP = host_ip  # 更新Agent实例中的IP地址
                        print(f"Updated IP to: {self.HOST_IP}")
                        break
                    else:
                        print("Received 'thmos_hello' message but no 'host_ip' field found.")
                else:
                    print("Received message does not match expected format.")
            except json.JSONDecodeError as e:
                print(f"JSON decode error: {e}")
            except KeyboardInterrupt:
                print("\nInterrupted by user. Exiting...")

    def loop(self):
        return (
            self.receiver.game_state != "STATE_SET"
            and self.receiver.game_state != "STATE_READY"
        )

    def speed_controller(self, x, y, theta):
        move_cmd = Twist()
        move_cmd.linear.x = x
        move_cmd.linear.y = y
        move_cmd.angular.z = theta
        self.speed_pub.publish(move_cmd)

    def update_go_back_to_field_status(self, aim_x, aim_y):
        self.go_back_to_field_dist = np.sqrt(
            (self.pos_x - aim_x) ** 2 + (self.pos_y - aim_y) ** 2
        )
        self.go_back_to_field_dir = np.arctan2(-aim_x + self.pos_x, aim_y - self.pos_y)
        self.go_back_to_field_yaw_bias = np.degrees(
            np.arctan2(
                np.sin(self.go_back_to_field_dir - self.pos_yaw * np.pi / 180),
                np.cos(self.go_back_to_field_dir - self.pos_yaw * np.pi / 180),
            )
        )

    def stop(self, sleep_time):
        self.speed_controller(0, 0, 0)
        time.sleep(sleep_time)

    def kick(self):
        print("kick")
        self.kick_state_machine = KickStateMachine(self)
        self.kick_state_machine.run()

    def go_back_to_field(self, aim_x, aim_y, min_dist=300):
        print("go back to field")
        self.head_set(0.05, 0)
        go_back_to_field_machine = GoBackToFieldStateMachine(
            self, aim_x, aim_y, min_dist
        )
        go_back_to_field_machine.run()

    def find_ball(self):
        print("find ball")
        self.find_ball_state_machine = FindBallStateMachine(self)
        self.find_ball_state_machine.run()

    def chase_ball(self):
        print("chase ball")
        self.chase_ball_state_machine = ChaseBallStateMachine(self)
        self.chase_ball_state_machine.run()

    def dribble(self):
        print("dribble")
        self.dribble_state_machine = DribbleStateMachine(self)
        self.dribble_state_machine.run()

    def ball_in_sight(self):
        if self.ifBall:
            self.t_no_ball = time.time()
        return self.ifBall

    def close_to_ball(self):
        if self.ifBall:
            return 0.1 <= self.ball_distance <= 0.35
        return (
            math.sqrt(self.pos_x - self.ball_x_in_map) ** 2
            + (self.pos_y - self.ball_y_in_map) ** 2
            <= 100
        )

    def ready_to_kick(self):
        return self.ready_to_kick

    def run(self):
        self.info = self.command['command']
        if self.command['command'] == "find_ball":
            self.find_ball()
        elif self.command['command'] == "chase_ball":
            self.chase_ball()
        elif self.command['command'] == "dribble":
            self.dribble()
        elif self.command['command'] == "stop":
            self.stop(1)
        elif self.command['command'] == "kick":
            self.kick()
        elif self.command['command'] == "go_back_to_field":
            self.go_back_to_field(self.field_aim_x, self.field_aim_y)
        else:
            print("Unknown command: ", self.command)
            self.stop(1)


class GoBackToFieldStateMachine:
    def __init__(self, agent: Agent, aim_x, aim_y, min_dist=300):
        self.agent = agent
        self.aim_x = aim_x
        self.aim_y = aim_y
        self.min_dist = min_dist
        self.states = [
            "moving_to_target",
            "coarse_yaw_adjusting",
            "fine_yaw_adjusting",
            "arrived_at_target",
        ]
        self.transitions = [
            {
                "trigger": "update_status",
                "source": "moving_to_target",
                "dest": "coarse_yaw_adjusting",
                "conditions": "need_coarse_yaw_adjustment",
                "before": "update_status",
            },
            {
                "trigger": "update_status",
                "source": "coarse_yaw_adjusting",
                "dest": "fine_yaw_adjusting",
                "conditions": "need_fine_yaw_adjustment",
                "before": "update_status",
            },
            {
                "trigger": "update_status",
                "source": "moving_to_target",
                "dest": "arrived_at_target",
                "conditions": "arrived_at_target",
                "before": "update_status",
            },
            {
                "trigger": "update_status",
                "source": "coarse_yaw_adjusting",
                "dest": "arrived_at_target",
                "conditions": "arrived_at_target",
                "before": "update_status",
            },
            {
                "trigger": "update_status",
                "source": "fine_yaw_adjusting",
                "dest": "arrived_at_target",
                "conditions": "arrived_at_target",
                "before": "update_status",
            },
        ]
        self.machine = Machine(
            model=self,
            states=self.states,
            initial="moving_to_target",
            transitions=self.transitions,
        )
        self.lock = threading.Lock()

    def need_coarse_yaw_adjustment(self):
        return self.agent.go_back_to_field_dist > self.min_dist and (
            abs(self.agent.go_back_to_field_yaw_bias) > 15
            or (
                self.agent.go_back_to_field_dist > 3 * self.min_dist
                and abs(self.agent.go_back_to_field_yaw_bias) > 10
            )
        )

    def need_fine_yaw_adjustment(self):
        return (
            self.agent.go_back_to_field_dist < 5 * self.min_dist
            and -20 < self.agent.go_back_to_field_yaw_bias < 10
            and not -10 < self.agent.go_back_to_field_yaw_bias < 5
            and self.agent.go_back_to_field_dist >= self.min_dist
        )

    def arrived_at_target(self):
        return self.agent.go_back_to_field_dist < self.min_dist

    def run(self):
        self.agent.is_going_back_to_field = True
        while self.state != "arrived_at_target":
            print("Going back to field...")
            print(f"Current state: {self.state}")
            with self.lock:
                self.machine.model.trigger("update_status")

    def update_status(self):
        with self.lock:
            self.agent.update_go_back_to_field_status(self.aim_x, self.aim_y)
            if self.state == "moving_to_target":
                if self.need_coarse_yaw_adjustment():
                    self.coarse_yaw_adjust()
                else:
                    self.move_forward()
            elif self.state == "coarse_yaw_adjusting":
                if self.need_fine_yaw_adjustment():
                    self.fine_yaw_adjust()
                elif self.arrived_at_target():
                    self.arrived_at_target_operations()
                else:
                    self.coarse_yaw_adjust()
            elif self.state == "fine_yaw_adjusting":
                if self.arrived_at_target():
                    self.arrived_at_target_operations()
                else:
                    self.fine_yaw_adjust()

    def move_forward(self):
        with self.lock:
            self.agent.debug_info("[Go Back to Field] Going forward")
            self.agent.speed_controller(self.agent.walk_x_vel, 0, 0)
            time.sleep(1)

    def coarse_yaw_adjust(self):
        with self.lock:
            sgn_bias = 1 if self.agent.go_back_to_field_yaw_bias > 0 else -1
            while (
                not self.agent.loop()
                and not -20 < self.agent.go_back_to_field_yaw_bias / sgn_bias < 10
            ):
                print(
                    f"dist: {self.agent.go_back_to_field_dist}, yaw_bias: {self.agent.go_back_to_field_yaw_bias}, dir: {self.agent.go_back_to_field_dir}, pos_yaw: {self.agent.pos_yaw}"
                )
                self.agent.update_go_back_to_field_status(self.aim_x, self.aim_y)
                if self.agent.go_back_to_field_dist < self.min_dist:
                    break
                if -self.agent.go_back_to_field_yaw_bias > 0:
                    self.agent.speed_controller(0, 0, -self.agent.walk_theta_vel)
                    self.agent.debug_info("[Go Back to Field] Turning right")
                    time.sleep(0.3)
                else:
                    self.agent.speed_controller(0, 0, self.agent.walk_theta_vel)
                    self.agent.debug_info("[Go Back to Field] Turning left")
                    time.sleep(0.3)
                self.agent.go_back_to_field_dir = np.arctan2(
                    -self.aim_x + self.agent.pos_x, self.aim_y - self.agent.pos_y
                )
                self.agent.go_back_to_field_yaw_bias = np.degrees(
                    np.arctan2(
                        np.sin(
                            self.agent.go_back_to_field_dir
                            - self.agent.pos_yaw * np.pi / 180
                        ),
                        np.cos(
                            self.agent.go_back_to_field_dir
                            - self.agent.pos_yaw * np.pi / 180
                        ),
                    )
                )

    def fine_yaw_adjust(self):
        with self.lock:
            sgn_bias = 1 if self.agent.go_back_to_field_yaw_bias > 0 else -1
            while (
                not self.agent.loop()
                and -20 < self.agent.go_back_to_field_yaw_bias / sgn_bias < 10
                and (not -10 < self.agent.go_back_to_field_yaw_bias / sgn_bias < 5)
                and self.agent.go_back_to_field_dist < 5 * self.min_dist
            ):
                self.agent.update_go_back_to_field_status(self.aim_x, self.aim_y)
                if self.agent.go_back_to_field_dist < self.min_dist:
                    break
                if -self.agent.go_back_to_field_yaw_bias > 0:
                    self.agent.speed_controller(0, 0, -0.1)
                    time.sleep(0.5)
                    self.agent.debug_info("[Go Back to Field] Turning right slowly")
                else:
                    self.agent.speed_controller(0, 0, 0.1)
                    time.sleep(0.5)
                self.agent.go_back_to_field_dir = np.arctan2(
                    -self.aim_x + self.agent.pos_x, self.aim_y - self.agent.pos_y
                )
                self.agent.go_back_to_field_yaw_bias = np.degrees(
                    np.arctan2(
                        np.sin(
                            self.agent.go_back_to_field_dir
                            - self.agent.pos_yaw * np.pi / 180
                        ),
                        np.cos(
                            self.agent.go_back_to_field_dir
                            - self.agent.pos_yaw * np.pi / 180
                        ),
                    )
                )

    def arrived_at_target_operations(self):
        with self.lock:
            self.agent.debug_info(
                "[Go Back to Field] "
                + str(self.agent.role)
                + " has arrived. Turn ahead"
            )
            self.agent.speed_controller(0, 0, 0)
            time.sleep(1)
            if abs(self.agent.pos_yaw) > 160:
                self.agent.speed_controller(
                    0, 0, -np.sign(self.agent.pos_yaw) * self.agent.walk_theta_vel
                )
                time.sleep(2)
            while not self.agent.loop() and self.agent.pos_yaw > 5:
                self.agent.speed_controller(0, 0, -self.agent.walk_theta_vel)
                self.agent.debug_info("[Go Back to Field] Arrived. Turning right")
                time.sleep(0.5)
            while not self.agent.loop() and self.agent.pos_yaw < -5:
                self.agent.speed_controller(0, 0, self.agent.walk_theta_vel)
                self.agent.debug_info("[Go Back to Field] Arrived. Turning left")
                time.sleep(0.5)
            # ready
            self.agent.speed_controller(0, 0, 0)
            self.agent.debug_info(
                "[Go Back to Field] Finished going back to field. Ready to play."
            )
            time.sleep(1)
            self.agent.is_going_back_to_field = False


class FindBallStateMachine:
    def __init__(self, agent: Agent):
        self.agent = agent
        self.states = ["search", "found"]
        self.transitions = [
            {
                "trigger": "search_ball",
                "source": "search",
                "dest": "found",
                "conditions": "ball_in_sight",
                "prepare": "search_ball",
            }
        ]
        self.machine = Machine(
            model=self,
            states=self.states,
            initial="search",
            transitions=self.transitions,
        )

    def ball_in_sight(self):
        return self.agent.ball_in_sight()

    def run(self):
        while self.state != "found":
            print("Searching for the ball...")
            self.machine.model.trigger("search_ball")

    def search_ball(self):
        with self.lock:
            self.agent.stop(0.5)
            protect_pose = JointState()
            protect_pose.name = [
                "L_arm_1",
                "L_arm_2",
                "L_arm_3",
                "R_arm_1",
                "R_arm_2",
                "R_arm_3",
            ]
            protect_pose.position = [0, 1.2, -0.5, 0, -1.2, 0.5]
            self.agent.joint_goal_publisher.publish(protect_pose)

            for pos in range(6):
                t = time.time()
                while time.time() - t < 1:
                    if self.agent.ifBall:
                        return
                    self.agent.head_set(
                        head=config.find_head_pos[pos], neck=config.find_neck_pos[pos]
                    )

            self.agent.speed_controller(0, 0, config.walk_theta_vel)
            t = time.time()
            while self.agent.loop() and time.time() - t < 10:
                for pos in [0, 3]:
                    t1 = time.time()
                    while time.time() - t1 < 2:
                        if self.agent.ifBall:
                            self.agent.stop(0.5)
                            return
                        self.agent.head_set(
                            head=config.find_head_pos[pos],
                            neck=config.find_neck_pos[pos],
                        )


class ChaseBallStateMachine:
    def __init__(self, agent: Agent):
        self.agent = agent
        self.states = ["chase", "arrived"]
        self.transitions = [
            {
                "trigger": "run",
                "source": "chase",
                "dest": "arrived",
                "conditions": "close_to_ball",
                "prepare": "move_to_ball",
            }
        ]
        self.machine = Machine(
            model=self,
            states=self.states,
            initial="chase",
            transitions=self.transitions,
        )

    def close_to_ball(self):
        return self.agent.close_to_ball()

    def run(self):
        while self.state != "arrived" and self.agent.command == self.agent.info:
            self.machine.model.trigger("chase_ball")

    def move_to_ball(self, ang=0.25):
        if abs(self.cam_neck) > ang:
            self.speed_controller(0, 0, np.sign(self.cam_neck) * config.walk_theta_vel)
        elif abs(self.cam_neck) <= ang:
            self.speed_controller(
                config.walk_x_vel, 0, 2.5 * self.cam_neck * config.walk_theta_vel
            )
            time.sleep(0.1)


class KickStateMachine:
    def __init__(self, agent: Agent):
        self.agent = agent
        self.states = ["angel", "lr", "fb", "finished"]
        self.transitions = [
            {
                "trigger": "adjust_position",
                "source": "angel",
                "dest": "lr",
                "conditions": "good_angel",
                "prepare": "adjust_angel",
            },
            {
                "trigger": "adjust_position",
                "source": "lr",
                "dest": "fb",
                "conditions": "good_lr",
                "prepare": "adjust_lr",
            },
            {
                "trigger": "adjust_position",
                "source": "fb",
                "dest": "finished",
                "conditions": "good_fb",
                "prepare": "adjust_fb",
            },
        ]
        self.machine = Machine(
            model=self,
            states=self.states,
            initial="angel",
            transitions=self.transitions,
        )

    def run(self):
        while (
            self.state != "finished"
            and self.agent.ifBall
            and self.agent.command == self.agent.info
        ):
            print("Adjusting position...")
            self.machine.model.trigger("adjust_position")
        if self.state == "finished" and self.agent.info == self.agent.command:
            self.agent.speed_controller(0, 0, 0)
            self.agent.head_set(head=0.1, neck=0)
            time.sleep(1)
            self.agent.doKick()
            time.sleep(2)

    def adjust_angel(self):
        self.agent.head_set(0.05, 0)
        target_angle_rad = math.atan((self.agent.pos_x - 0) / (4500 - self.agent.pos_y))
        ang_tar = target_angle_rad * 180 / math.pi
        ang_delta = ang_tar - self.agent.pos_yaw
        if ang_delta > 10:
            self.agent.speed_controller(0, -0.05, 0.3)
            ang_delta = ang_tar - self.agent.pos_yaw
            print(f"ang_delta={ang_delta}")
        elif ang_delta < -10:
            self.agent.speed_controller(0, 0.05, 0.3)
            ang_delta = ang_tar - self.agent.pos_yaw
            print(f"ang_delta={ang_delta}")

    def good_angel(self):
        target_angle_rad = math.atan((self.pos_x - 0) / (4500 - self.agent.pos_y))
        ang_tar = target_angle_rad * 180 / math.pi
        ang_delta = ang_tar - self.agent.pos_yaw
        return abs(ang_delta) < 10

    def adjust_lr(self):
        self.agent.head_set(head=0.9, neck=0)
        self.agent.stop(1)
        no_ball_count = 0
        t0 = time.time()
        while (
            self.agent.loop() and (self.agent.ball_x < 600) or (self.agent.ball_x == 0)
        ):
            if time.time() - t0 > 10 or no_ball_count > 5:
                return
            if not self.agent.ifBall:
                no_ball_count += 1
                time.sleep(0.7)
                continue
            self.agent.speed_controller(0, 0.6 * config.walk_y_vel, 0)
        while (
            self.agent.loop() and (self.agent.ball_x > 660) or (self.agent.ball_x == 0)
        ):
            if time.time() - t0 > 10 or no_ball_count > 5:
                return
            if not self.agent.ifBall:
                no_ball_count += 1
                time.sleep(0.7)
                continue
            self.agent.speed_controller(0, -0.6 * config.walk_y_vel, 0)
        self.agent.stop(0.5)

    def good_lr(self):
        return 600 <= self.agent.ball_x <= 660

    def adjust_fb(self):
        self.agent.stop(0.5)
        t0 = time.time()
        no_ball_count = 0
        while (
            self.agent.loop() and (self.agent.ball_y < 420) or (self.agent.ball_y == 0)
        ):
            if time.time() - t0 > 10 or no_ball_count > 5:
                return
            if not self.agent.ifBall:
                no_ball_count += 1
                time.sleep(0.7)
                continue
            self.agent.speed_controller(0.5 * config.walk_x_vel, 0, 0)
        self.agent.stop(0.5)

    def good_fb(self):
        self.agent.ready_to_kick = 420 <= self.agent.ball_y
        return 420 <= self.agent.ball_y


class CanNotFindBallStateMachine:
    def __init__(self, agent: Agent):
        self.agent = agent
        self.states = ["cannot_find_ball", "going_back_to_field", "ball_in_sight"]
        self.transitions = [
            {
                "trigger": "going_back_to_field",
                "source": "cannot_find_ball",
                "dest": "ball_in_sight",
            },
        ]
        self.machine = Machine(
            model=self,
            states=self.states,
            initial="cannot_find_ball",
            transitions=self.transitions,
        )
        self.lock = threading.Lock()

    def cannot_find_ball(self):
        return not self.agent.ball_in_sight()

    def go_back_to_field(self):
        self.agent.go_back_to_field(self.agent.field_aim_x, self.agent.field_aim_y)

    def run(self):
        while self.state != "ball_in_sight":
            print("Cannot find the ball. Going back to the field...")
            self.machine.model.trigger("going_back_to_field")


class DribbleStateMachine:
    """
    朝球门带球
    TODO: 处理丢球情况，检查方向
    """

    def __init__(self, agent: Agent):
        self.agent = agent
        self.states = ["forward", "adjust", "finished"]
        self.transitions = [
            {
                "trigger": "dribble",
                "source": "forward",
                "dest": "adjust",
                "conditions": "bad_position",
                "after": "adjust",
            },
            {
                "trigger": "dribble",
                "source": "adjust",
                "dest": "forward",
                "conditions": "good_position",
                "after": "forward",
            },
            {
                "trigger": "dribble",
                "source": "*",
                "dest": "finished",
                "conditions": "finished",
            },
        ]
        self.machine = Machine(
            model=self,
            states=self.states,
            initial="forward",
            transitions=self.transitions,
        )
        self.direction = True  # FIXME: True: right, False: left

    def run(self):
        while self.state != "finished":
            print("Dribbling...")
            self.machine.model.trigger("dribble")

    def forward(self):
        self.agent.speed_controller(0.5, 0, 0)
        time.sleep(0.5)

    def adjust(self):
        while not self.good_angel():  # 调整角度
            if self.direction:
                self.agent.speed_controller(0, 0, 0.1)
            else:
                self.agent.speed_controller(0, 0, -0.1)
            time.sleep(0.5)
        while self.agent.ball_y > 600:  # 后退
            self.agent.speed_controller(0, -0.1, 0)
            time.sleep(0.5)
        while abs(self.agent.ball_x - 640) > 10:  # 左右调整
            if self.agent.ball_x < 640:
                self.agent.speed_controller(0, 0.1, 0)
            else:
                self.agent.speed_controller(0, -0.1, 0)
            time.sleep(0.5)

    def good_position(self):
        return (
            self.good_angel()
            and self.agent.ball_y < 600
            and abs(self.agent.ball_x - 640) < 10
        )

    def bad_position(self):
        return not self.good_position()

    def good_angel(self):
        rad1 = math.atan(
            (self.agent.pos_x - 1300) / (4500 - self.agent.pos_y)
        )  # FIXME:球门左侧
        ang_tar1 = rad1 * 180 / math.pi
        rad2 = math.atan(
            (self.agent.pos_x + 1300) / (4500 - self.agent.pos_y)
        )  # FIXME:球门右侧
        ang_tar2 = rad2 * 180 / math.pi
        if ang_tar1 < self.agent.pos_yaw < ang_tar2:
            return True
        else:
            self.direction = ang_tar1 > self.agent.pos_yaw
            return False

    def finished(self):
        return not self.agent.ifBall


def main():
    agent = Agent()
    try:
        while True:
                agent.run()
    except KeyboardInterrupt:
        print("\nProgram interrupted by user")
        break
    finally:
        pass



if __name__ == "__main__":
    main()
