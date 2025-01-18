import asyncio
import json
import logging
from collections import defaultdict
from datetime import datetime
import time
import socket


logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')


class RobotServer:
    def __init__(self, agent, port=8001):
        self.port = port
        self.agent = agent
        self.robot_ips = defaultdict(str)
        # 存储机器人数据
        self.robots_data = self.agent.robots_data
        # 创建 UDP 套接字用于广播
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        # 自动探测 IP 地址
        self.ip = self.detect_ip()

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
        robot_id = None
        try:
            while True:
                data_received = await reader.read(1024)
                if not data_received:
                    break
                receive_time = time.time()
                try:
                    robot_data = json.loads(data_received.decode("utf-8"))
                    send_time_str = robot_data.get('send_time')
                    if send_time_str:
                        send_time = float(send_time_str)
                    robot_id = robot_data["id"]
                    logging.info(f"Received data from robot {robot_id}: {robot_data}")
                    # 更新机器人 IP 地址
                    if robot_id not in self.robot_ips:
                        self.robot_ips[robot_id] = addr[0]
                    elif self.robot_ips[robot_id]!= addr[0]:
                        logging.warning(f"Robot {robot_id} IP address changed from {self.robot_ips[robot_id]} to {addr[0]}")
                        self.robot_ips[robot_id] = addr[0]
                    # 更新或添加机器人数据
                    self.agent.robots_data[robot_id]['last_seen'] = datetime.now().isoformat()
                    self.agent.robots_data[robot_id]['status'] = 'connected'
                    self.agent.robots_data[robot_id]['data'].update(robot_data)
                    # 构建响应消息
                    response = {
                        "status": "received",
                        "server_receive_time": receive_time,
                        "server_send_time": time.time()  # 记录发送时间
                    }
                    writer.write(json.dumps(response).encode("utf-8"))
                    await writer.drain()
                except json.JSONDecodeError as e:
                    logging.error(f"JSON decode error: {e}")
                except Exception as e:
                    logging.error(f"Error processing message from robot {robot_id}: {e}")
        except Exception as e:
            logging.error(f"Connection error with {addr}: {e}")
        finally:
            if robot_id is not None:
                self.agent.robots_data[robot_id]['status'] = 'disconnected'
            writer.close()
            await writer.wait_closed()
            logging.info(f"Disconnected from {addr}")

    async def start_server(self):
        server = await asyncio.start_server(self.handle_robot, self.ip, self.port)
        addr = server.sockets[0].getsockname()
        logging.info(f"Serving on {addr}")
        async with server:
            await server.serve_forever()

    async def monitor_robots(self):
        while True:
            logging.info("\nRobot Status:")
            for robot_id, info in self.agent.robots_data.items():
                logging.info(f"Robot ID: {robot_id}, Last Seen: {info['last_seen']}, "
                             f"Status: {info['status']}")
            await asyncio.sleep(10)

    async def send(self, robot_id, data):
        # 获取机器人的 IP 地址
        robot_ip = self.robot_ips.get(robot_id)
        if robot_ip:
            try:
                reader, writer = await asyncio.open_connection(robot_ip, self.port)
                writer.write(json.dumps(data).encode("utf-8"))
                await writer.drain()
                logging.info(f"Sent data to robot {robot_id} at {robot_ip}: {data}")
                writer.close()
                await writer.wait_closed()
            except Exception as e:
                logging.error(f"Failed to send data to robot {robot_id}: {e}")
        else:
            logging.error(f"No IP address found for robot {robot_id}")

    def broadcast(self, data):
        broadcast_addr = ('<broadcast>', self.port)
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