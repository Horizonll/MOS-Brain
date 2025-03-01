# system packages
import asyncio
import json
import logging
import math
import signal
import time
import threading
import socket
import websockets
import numpy as np
from transitions import Machine

# ROS
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState

# Submodules
from subscriber import *
from receiver import Receiver
from config import *

# Sub StateMachines
from subStateMachines.can_not_find_ball import CanNotFindBallStateMachine
from subStateMachines.chase_ball import ChaseBallStateMachine
from subStateMachines.go_back_to_field import GoBackToFieldStateMachine
from subStateMachines.kick import KickStateMachine
from subStateMachines.find_ball import FindBallStateMachine
from subStateMachines.dribble import DribbleStateMachine


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

        self.kick_state_machine = KickStateMachine(self)
        self.go_back_to_field_machine = GoBackToFieldStateMachine(self, 0, 4500, 300)
        self.find_ball_state_machine = FindBallStateMachine(self)
        self.chase_ball_state_machine = ChaseBallStateMachine(self)
        self.dribble_state_machine = DribbleStateMachine(self)

        # detect local ip
        self.ip = self.detect_ip()

        # 监听主机IP
        print("Starting to listen for host IP...")
        # self.listen_host_ip()
        self.HOST_IP = "192.168.0.40"
        print("Finished listening for host IP.")

        # 启动发送线程
        send_thread = threading.Thread(target=self.send_loop)
        send_thread.daemon = True
        send_thread.start()
        print("Started send loop thread.")

        # 启动TCP客户端线程
        tcp_thread = threading.Thread(target=self.start_tcp_listener_loop)
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
                    "ip": self.ip,
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
            ip = "127.0.0.1"
        finally:
            s.close()
        return ip

    def start_tcp_listener_loop(self):
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        try:
            loop.run_until_complete(self.tcp_listener())
        except Exception as e:
            print(f"Error in start_tcp_listener_loop: {e}")
        finally:
            loop.close()

    async def tcp_listener(self):
        """异步监听TCP命令消息"""
        self.ip = self.detect_ip()
        print(f"Detected IP address: {self.ip}")

        # 创建一个异步服务器
        server = await asyncio.start_server(self.handle_connection, self.ip, 8002)

        addr = server.sockets[0].getsockname()
        print(f"Listening for TCP command messages on {addr}...")

        try:
            async with server:
                await server.serve_forever()
        except KeyboardInterrupt:
            print("\nInterrupted by user. Exiting...")
        finally:
            # 关闭服务器
            server.close()
            await server.wait_closed()
            print("Server closed.")

    async def handle_connection(self, reader, writer):
        try:
            while True:
                data = await reader.read(1024)
                if not data:
                    break
                addr = writer.get_extra_info("peername")
                print(f"Received data from {addr}: {data}")

                try:
                    received_data = json.loads(data.decode("utf-8"))
                    print(f"Parsed JSON data: {received_data}")

                    # 这里可以根据具体的命令进行不同的处理
                    if "command" in received_data:
                        self.command = received_data
                    else:
                        print("Received message does not contain 'command' field.")
                except json.JSONDecodeError as e:
                    print(f"JSON decode error: {e}")
        except Exception as e:
            print(f"Error handling connection: {e}")
        finally:
            writer.close()
            await writer.wait_closed()

    def listen_host_ip(self):
        """同步监听UDP广播消息并获取主机IP"""
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(("0.0.0.0", 8003))  # 监听指定端口上的UDP广播

        # 加入广播接收
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

        while True:
            data, addr = sock.recvfrom(1024)
            try:
                received_data = json.loads(data.decode("utf-8"))
                if "message" in received_data and received_data["message"] == "thmos_hello":
                    # 假设广播消息中包含主机IP信息
                    if "ip" in received_data:
                        host_ip = received_data["ip"]
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
        return self.receiver.game_state != "STATE_SET" and self.receiver.game_state != "STATE_READY"

    def speed_controller(self, x, y, theta):
        move_cmd = Twist()
        move_cmd.linear.x = x
        move_cmd.linear.y = y
        move_cmd.angular.z = theta
        self.speed_pub.publish(move_cmd)

    def update_go_back_to_field_status(self, aim_x, aim_y):
        self.go_back_to_field_dist = np.sqrt((self.pos_x - aim_x) ** 2 + (self.pos_y - aim_y) ** 2)
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
        self.kick_state_machine.run()

    def go_back_to_field(self, aim_x, aim_y, min_dist=300):
        print("go back to field")
        self.head_set(0.05, 0)
        # go_back_to_field_machine = GoBackToFieldStateMachine(self, aim_x, aim_y, min_dist)
        self.go_back_to_field_machine.run()

    def find_ball(self):
        print("find ball")
        self.find_ball_state_machine.run()

    def chase_ball(self):
        print("chase ball")
        self.chase_ball_state_machine.run()

    def dribble(self):
        print("dribble")
        self.dribble_state_machine.run()

    def ball_in_sight(self):
        if self.ifBall:
            self.t_no_ball = time.time()
        return self.ifBall

    def close_to_ball(self):
        if self.ifBall:
            return 0.1 <= self.ball_distance <= 0.35
        return math.sqrt(self.pos_x - self.ball_x_in_map) ** 2 + (self.pos_y - self.ball_y_in_map) ** 2 <= 100

    def ready_to_kick(self):
        return self.ready_to_kick

    def run(self):
        self.info = self.command["command"]
        if self.command["command"] == "find_ball":
            self.find_ball()
        elif self.command["command"] == "chase_ball":
            self.chase_ball()
        elif self.command["command"] == "dribble":
            self.dribble()
        elif self.command["command"] == "stop":
            self.stop(1)
        elif self.command["command"] == "kick":
            self.kick()
        elif self.command["command"] == "go_back_to_field":
            self.go_back_to_field(self.field_aim_x, self.field_aim_y)
        else:
            print("Unknown command: ", self.command)
            self.stop(1)


def main():
    agent = Agent()
    try:
        # 注册信号处理器
        def sigint_handler(sig, frame):
            print("\nExiting gracefully...")
            # 这里可以添加清理代码（如关闭网络连接）
            exit(0)

        signal.signal(signal.SIGINT, sigint_handler)
        while True:
            agent.run()
            time.sleep(0.2)
    except KeyboardInterrupt:
        print("\nProgram interrupted by user")
        exit()
    finally:
        pass


if __name__ == "__main__":
    main()
