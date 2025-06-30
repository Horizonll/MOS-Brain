import asyncio
import hashlib
import json
import logging
from collections import defaultdict
from datetime import datetime
import time
import socket
import traceback
import netifaces
import re

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')


class RobotServer:
    def __init__(self, agent, port=8001):
        self.port = port
        self.agent = agent
        self.robot_ips = {}
        self.robots_data = self.agent.robots_data
        self.send_robot_data_port = self.config.get("send_robot_data", {}).get("port", 8001)
        self.send_robot_data_secret = self.config.get("send_robot_data", {}).get("secret", "YXdpb3BqeHo7bHas")
        self.receive_from_server_port = self.config.get("receive_from_server", {}).get("port", 8002)
        self.receive_from_server_secret = self.config.get("receive_from_server", {}).get("secret", "PIDcvpasdvfpIFES")
        self.find_server_ip_port = self.config.get("find_server_ip", {}).get("port", 8003)
        self.find_server_ip_secret = self.config.get("find_server_ip", {}).get("secret", "a2xzYXZoO29hd2pp")
        self.client_udp_port = 8003  # UDP端口用于与机器人通信
        self.update_interval = 0.5  # 默认更新间隔
        self.discovery_interval = 1  # 发现广播间隔
        self.config = self.agent.get_config().get("robot_server", {})
        self.broadcast_address = self.get_wireless_broadcast()

        # 网络初始化
        self._init_udp_socket()
        self._init_udp_server()
        # self.ip = self._detect_ip()

        logging.info(f"Robot server initialized on UDP port {self.port}")

    def _init_udp_socket(self):
        """初始化用于广播的UDP socket"""
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

    def _init_udp_server(self):
        """初始化UDP服务器socket"""
        self.udp_server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_server_socket.bind(('0.0.0.0', self.receive_from_server_port))
        self.udp_server_socket.setblocking(False)  # 设置为非阻塞模式

    def _detect_self_ip(self):
        """自动检测服务器IP地址"""
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
                s.connect(("8.8.8.8", 80))
                return s.getsockname()[0]
        except Exception as e:
            logging.error(f"IP detection failed: {e}")
            return '127.0.0.1'

    def get_wireless_broadcast(self):
        """获取无线网络的广播地址"""
        try:
            # 无线接口名称的常见模式
            wireless_patterns = [
                r'^wlan\d+$',      # Windows: wlan0, wlan1
                r'^wl.*$',         # Linux: wlp2s0, wlan0
                r'^en.*wireless$', # macOS: en0 (Wi-Fi)
                r'^Wi-Fi$',        # Windows 10+: Wi-Fi
                r'^无线网络连接$'   # Windows中文: 无线网络连接
            ]
            
            for iface in netifaces.interfaces():
                # 检查是否为无线接口
                is_wireless = False
                for pattern in wireless_patterns:
                    if re.match(pattern, iface):
                        is_wireless = True
                        break
                
                if not is_wireless:
                    continue
                
                # 获取该接口的IPv4地址信息
                if netifaces.AF_INET in netifaces.ifaddresses(iface):
                    for addr in netifaces.ifaddresses(iface)[netifaces.AF_INET]:
                        if 'broadcast' in addr:
                            logging.info(f"Found wireless broadcast address: {addr['broadcast']} on interface {iface}")
                            return addr['broadcast']
            
            # 如果没找到无线接口，回退到自动检测
            logging.warning("No wireless interface found, falling back to auto-detection")
            return self._infer_broadcast()
        except Exception as e:
            logging.error(f"Wireless broadcast detection failed: {e}")
            return "255.255.255.255"

    def _infer_broadcast(self):
        """自动推断广播地址"""
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
                s.connect(("8.8.8.8", 80))
                ip = s.getsockname()[0]
                parts = list(map(int, ip.split('.')))
                return '.'.join(map(str, parts[:3] + [255]))
        except Exception as e:
            logging.error(f"Infer broadcast failed: {e}")
            return "255.255.255.255"

    async def _send_discovery(self):
        """发送初始发现广播"""
        while True:
            self.broadcast('NjM5NWVkMSAgLQo')
            await asyncio.sleep(self.discovery_interval)

    async def handle_udp_packets(self):
        """处理接收到的UDP数据包"""
        while True:
            try:
                # 使用asyncio.sleep(0)让出控制权，避免CPU占用过高
                await asyncio.sleep(0)
                
                try:
                    data, addr = self.udp_server_socket.recvfrom(1024)
                    robot_ip = addr[0]
                    self._process_incoming_data(data, robot_ip)
                except BlockingIOError:
                    # 没有数据可读，继续循环
                    continue
                except Exception as e:
                    logging.error(f"Error receiving UDP data: {e}")
                    
            except Exception as e:
                logging.error(f"UDP packet handler error: {e}")

    def _process_incoming_data(self, data, robot_ip):
        """处理接收到的数据并更新机器人状态"""
        try:
            robot_data = json.loads(data.decode("utf-8"))
            robot_data = self._verify_sign(robot_data, self.receive_from_server_secret)
            robot_id = robot_data["id"]

            # 更新机器人元数据
            self.agent.robots_data[robot_id].update({
                'last_seen': datetime.now().isoformat(),
                'status': 'connected',
                'data': {
                    **self.agent.robots_data[robot_id].get('data', {}),
                    **robot_data.get('data', {})
                }
            })

            # 更新球检测状态
            ball_data = robot_data.get('data', {}).get('ballx')
            self.agent.ball_x = ball_data
            logging.info(f"Robot {robot_id} data: {robot_data}")

            # 存储机器人IP
            self.robot_ips[robot_id] = robot_ip
            return robot_id

        except (json.JSONDecodeError, KeyError) as e:
            logging.error(f"Data processing error: {e}")
            return None

    def send_to_robot(self, robot_id, data):
        """向特定机器人发送数据"""
        if robot_ip := self.robot_ips.get(robot_id):
            try:
                # 将数据转换为JSON并发送
                message = json.dumps(data).encode("utf-8")
                message = self._sign_message(message, self.send_robot_data_secret)
                self.udp_server_socket.sendto(message, (robot_ip, self.send_robot_data_port))
                logging.debug(f"Data sent to {robot_id}")
            except Exception as e:
                logging.error(f"Failed to send to {robot_id}: {e}")
        else:
            logging.warning(f"No IP found for {robot_id}")

    def broadcast(self, data):
        """向所有机器人广播数据"""
        if isinstance(data, str):
            encoded_data = data.encode("utf-8")
        else:
            encoded_data = str(data).encode("utf-8")

        encoded_data = self._sign_message(encoded_data, self.find_server_ip_secret)
        self.udp_socket.sendto(
            encoded_data,
            (self.broadcast_address, self.find_server_ip_port)
        )
        logging.debug(f"Broadcasted: {data}")

    async def send_all_robots_data_loop(self):
        """定期向所有连接的机器人发送更新"""
        while True:
            await asyncio.sleep(self.update_interval)
            try:
                full_status = {
                    "robots_data": self.agent.robots_data.copy(),
                }

                # 向所有已知的机器人发送数据
                for robot_id in self.robot_ips:
                    if self.agent.robots_data.get(robot_id, {}).get('status') == 'connected':
                        self.send_to_robot(robot_id, full_status)
                
                logging.debug(f"Updates sent to {len(self.robot_ips)} robots")
            except Exception as e:
                logging.error(f"Update cycle error: {e}")

    async def monitor_connections(self):
        """监控和更新机器人连接状态"""
        while True:
            current_time = time.time()
            for robot_id, data in self.agent.robots_data.items():
                last_seen = data.get('last_seen')
                if not last_seen:
                    continue

                if (current_time - datetime.fromisoformat(last_seen).timestamp()) > 5:
                    data['status'] = 'disconnected'

            logging.debug("Connection status updated")
            # 列出所有机器人数据
            for robot_id, data in self.agent.robots_data.items():
                logging.debug(f"Robot {robot_id}: {data}")
            await asyncio.sleep(5)

    async def run(self):
        """主执行方法"""
        try:
            await asyncio.gather(
                self.handle_udp_packets(),
                self.monitor_connections(),
                self.send_all_robots_data_loop(),
                self._send_discovery()
            )
        except asyncio.CancelledError:
            logging.info("Server shutdown requested")
        except Exception as e:
            logging.error(f"Critical error: {e}")
        finally:
            self.udp_socket.close()
            self.udp_server_socket.close()

    def _sign_message(self, message: str, secret: str) -> str:
        hash_str = hashlib.sha3_256((message + secret).encode("utf-8")).hexdigest()
        ret = {"raw": message, "sign": hash_str}
        return json.dumps(ret)
    
    
    def _verify_sign(self, data: str, secret: str) -> str:
        try:
            js_data = json.loads(data)
            message = js_data.get("raw", None)
            target_hash = hashlib.sha3_256((message + secret).encode("utf-8")).hexdigest()
            if js_data.get("sign", None) == target_hash:
                return message
        except Exception as e:
            pass
        return None


if __name__ == "__main__":
    class BaseAgent:
        def __init__(self):
            self.robots_data = defaultdict(lambda: {
                'last_seen': None,
                'status': 'disconnected',
                'data': {}
            })
            self.ball_x = None
            
        def get_config(self):
            return {"broadcast_address": "192.168.9.255"}

    robot_server = RobotServer(BaseAgent())

    try:
        asyncio.run(robot_server.run())
    except KeyboardInterrupt:
        logging.info("Server stopped by user")
