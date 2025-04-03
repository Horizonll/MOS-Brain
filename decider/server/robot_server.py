import asyncio
import json
import logging
from collections import defaultdict
from datetime import datetime
import time
import socket
import traceback


logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')


class RobotServer:
    def __init__(self, agent, port=8001):
        self.port = port
        self.agent = agent
        self.robot_ips = {}
        # 存储机器人数据
        self.robots_data = self.agent.robots_data
        # 创建 UDP 套接字用于广播
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        # 自动探测 IP 地址
        self.ip = self.detect_ip()
        # 发送端口
        self.send_port = 8002
        # 广播端口
        self.udp_port = 8003
        # 启动服务器
        logging.info(f"Robot server initialized")

        # 广播'thmos_hello'消息两次
        self.broadcast('')

        

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

    async def handle_robot(self, reader, writer):
        addr = writer.get_extra_info('peername')
        robot_ip = addr[0]  # 提取 IP 地址部分
        robot_id = None
        try:
            while True:
                data_received = await reader.read(1024)
                if not data_received:
                    break
                receive_time = time.time()
                try:
                    robot_data = json.loads(data_received.decode("utf-8"))
                    print(robot_data)
                    send_time_str = robot_data.get('send_time')
                    if send_time_str:
                        send_time = float(send_time_str)
                    robot_id = robot_data["id"]
                    # logging.info(f"Received data from robot {robot_id}: {robot_data}")
                    # 更新机器人 IP 地址
                    # if robot_id not in self.robot_ips:
                    #     self.robot_ips[robot_id] = addr[0]
                    # elif self.robot_ips[robot_id]!= addr[0]:
                    #     logging.warning(f"Robot {robot_id} IP address changed from {self.robot_ips[robot_id]} to {addr[0]}")
                    #     self.robot_ips[robot_id] = addr[0]
                    # 更新或添加机器人数据
                    self.agent.robots_data[robot_id]['last_seen'] = datetime.now().isoformat()
                    self.agent.robots_data[robot_id]['status'] = 'connected'
                    self.agent.robots_data[robot_id]['data'].update(robot_data.get('data'))
                    # 更新机器人的 IP 地址
                    self.robot_ips[robot_id] = robot_ip

                    self.agent.ball_x = robot_data.get('data').get('ballx')

                    if robot_data.get('data').get('ballx') is not None:
                        self.agent.ifBall = True
                    else:
                        self.agent.ifBall = False
                    
                    # # 构建响应消息
                    # response = {
                    #     "status": "received",
                    #     "server_receive_time": receive_time,
                    #     "server_send_time": time.time()  # 记录发送时间
                    # }
                    # writer.write(json.dumps(response).encode("utf-8"))
                    await writer.drain()
                except json.JSONDecodeError as e:
                    logging.error(f"JSON decode error: {e}")
                except Exception as e:
                    logging.error(f"Error processing message from robot {robot_id}: {e}")
                    traceback.print_exc()
        except Exception as e:
            logging.error(f"Connection error with {addr}: {e}")
        finally:
            # if robot_id is not None:
            #     self.agent.robots_data[robot_id]['status'] = 'disconnected'
            writer.close()
            await writer.wait_closed()
            # logging.info(f"Disconnected from {addr}")

    async def start_server(self):
        server = await asyncio.start_server(self.handle_robot, '0.0.0.0', self.port)
        addr = server.sockets[0].getsockname()
        logging.info(f"Serving on {addr}")
        async with server:
            await server.serve_forever()

    async def monitor_robots(self):
        while True:
            logging.info("\nRobot Status:")
            current_time = time.time()  # 获取当前时间

            for robot_id, data in self.agent.robots_data.items():
                if data['status'] != 'connected':
                    pass
                last_seen = data['last_seen']
                if last_seen is None:
                    # 如果 last_seen 为 None，说明还没有收到过该机器人的数据
                    logging.info(f"Robot ID: {robot_id}, Last Seen: None, Status: {data['status']}")
                    continue
                # 将上次更新时间转换为时间戳
                last_seen = datetime.fromisoformat(last_seen).timestamp()
                time_diff = current_time - last_seen  # 计算时间差

                # 如果超过5秒没有更新，将状态改为disconnected
                if time_diff > 5:
                    data['status'] = 'disconnected'

                logging.info(f"Robot ID: {robot_id}, Last Seen: {data['last_seen']}, "
                            f"Status: {data['status']}")

            await asyncio.sleep(10)  # 每5秒检查一次

    async def send(self, robot_id, data):
        # 获取机器人的 IP 地址
        robot_ip = self.robot_ips.get(robot_id)
        if robot_ip:
            try:
                reader, writer = await asyncio.open_connection(robot_ip, self.send_port)
                writer.write(json.dumps(data).encode("utf-8"))
                await writer.drain()
                logging.info(f"Sent data to robot {robot_id} at {robot_ip}: {data}")
                writer.close()
                await writer.wait_closed()
            except Exception as e:
                logging.error(f"Failed to send data to robot {robot_id}: {e}")
        else:
            logging.error(f"No IP address found for robot {robot_id}")
            print(self.robot_ips)

    def broadcast(self, data):
        broadcast_addr = ('192.168.99.255', self.udp_port)
        self.udp_socket.sendto(json.dumps(data).encode("utf-8"), broadcast_addr)
        logging.info(f"Broadcast data: {data}")

    async def run(self):
        try:
            tasks = [self.start_server(), self.monitor_robots()]
            await asyncio.gather(*tasks)
        except Exception as e:
            logging.error(f"Error in main loop: {e}")


# Example usage:
if __name__ == "__main__":
    class MyAgent:
        def update(self, robot_id, data):
            # 实现自己的逻辑来处理传入的数据
            pass

    robot_server = RobotServer(MyAgent)
    try:
        asyncio.run(robot_server.run())
    except KeyboardInterrupt:
        logging.info("Server stopped by user")