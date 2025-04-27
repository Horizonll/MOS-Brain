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
        self.robots_data = self.agent.robots_data
        self.client_tcp_port = 8002
        self.client_udp_port = 8003
        self.update_interval = 0.5  # Default interval for periodic updates
        self.discovery_interval = 1  # Interval for sending discovery broadcasts

        # Network initialization
        self._init_udp_socket()
        # self.ip = self._detect_ip()

        logging.info(f"Robot server initialized on port {self.port}")

    def _init_udp_socket(self):
        """Initialize UDP socket for broadcasting"""
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

    def _detect_ip(self):
        """Detect server IP address automatically"""
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
                s.connect(("8.8.8.8", 80))
                return s.getsockname()[0]
        except Exception as e:
            logging.error(f"IP detection failed: {e}")
            return '127.0.0.1'

    async def _send_discovery(self):
        """Send initial discovery broadcasts"""
        while True:
            self.broadcast('NjM5NWVkMSAgLQo')
            await asyncio.sleep(self.discovery_interval)

    async def handle_robot_connection(self, reader, writer):
        """Handle incoming robot connections and data exchange"""
        addr = writer.get_extra_info('peername')
        robot_ip = addr[0]
        robot_id = None

        try:
            while True:
                data = await reader.read(1024)
                if not data:
                    break

                robot_id = self._process_incoming_data(data, robot_ip)
                # await self._send_robot_status(writer)

        except Exception as e:
            logging.error(f"Connection error with {addr}: {e}")
            traceback.print_exc()
        finally:
            self._cleanup_connection(writer, robot_id)

    def _process_incoming_data(self, data, robot_ip):
        """Process received data and update robot status"""
        try:
            robot_data = json.loads(data.decode("utf-8"))
            robot_id = robot_data["id"]

            # Update robot metadata
            self.agent.robots_data[robot_id].update({
                'last_seen': datetime.now().isoformat(),
                'status': 'connected',
                'data': {
                    **self.agent.robots_data[robot_id].get('data', {}),
                    **robot_data.get('data', {})
                }
            })

            # Update ball detection status
            ball_data = robot_data.get('data', {}).get('ballx')
            self.agent.ifBall = ball_data is not None
            self.agent.ball_x = ball_data

            # Store robot IP
            self.robot_ips[robot_id] = robot_ip
            return robot_id

        except (json.JSONDecodeError, KeyError) as e:
            logging.error(f"Data processing error: {e}")
            return None

    async def _send_robot_status(self, writer):
        """Send current robot status to connected client"""
        response = {"robots": self.agent.robots_data}
        writer.write(json.dumps(response).encode("utf-8"))
        await writer.drain()

    def _cleanup_connection(self, writer, robot_id):
        """Handle connection cleanup"""
        writer.close()

    async def start_server(self):
        """Start main TCP server"""
        server = await asyncio.start_server(
            self.handle_robot_connection, '0.0.0.0', self.port
        )
        logging.info(f"Server listening on {server.sockets[0].getsockname()}")
        await server.serve_forever()

    async def monitor_connections(self):
        """Monitor and update robot connection status"""
        while True:
            current_time = time.time()
            for robot_id, data in self.agent.robots_data.items():
                last_seen = data.get('last_seen')
                if not last_seen:
                    continue

                if (current_time - datetime.fromisoformat(last_seen).timestamp()) > 5:
                    data['status'] = 'disconnected'

            logging.debug("Connection status updated")
            # list all robots data
            for robot_id, data in self.agent.robots_data.items():
                logging.debug(f"Robot {robot_id}: {data}")
            await asyncio.sleep(5)

    async def send_to_robot(self, robot_id, data):
        """Send data to specific robot"""
        if robot_ip := self.robot_ips.get(robot_id):
            try:
                # 设置超时时间为 2 秒，你可以按需调整
                reader, writer = await asyncio.wait_for(
                    asyncio.open_connection(robot_ip, self.client_tcp_port),
                    timeout=0.5
                )
                writer.write(json.dumps(data).encode("utf-8"))
                await writer.drain()
                writer.close()
                await writer.wait_closed()
                logging.debug(f"Data sent to {robot_id}")
            except asyncio.TimeoutError:
                logging.error(f"Connection to {robot_id} timed out.")
            except Exception as e:
                logging.error(f"Failed to send to {robot_id}: {e}")
        else:
            logging.warning(f"No IP found for {robot_id}")

    def broadcast(self, data):
        """Broadcast data to all robots"""
        if isinstance(data, str):
            encoded_data = data.encode("utf-8")
        else:
            encoded_data = str(data).encode("utf-8")
        self.udp_socket.sendto(
            encoded_data,
            ('192.168.9.255', self.client_udp_port)
        )
        logging.debug(f"Broadcasted: {data}")

    async def send_all_robots_data_loop(self):
        """Periodically send updates to all connected robots with max 4 parallel"""
        semaphore = asyncio.Semaphore(4)

        async def send_with_semaphore(robot_id, data):
            async with semaphore:
                try:
                    await self.send_to_robot(robot_id, data)
                except Exception as e:
                    logging.error(f"Send to {robot_id} failed: {e}")

        while True:
            await asyncio.sleep(self.update_interval)
            try:
                full_status = {
                    "robots_data": self.agent.robots_data.copy(),
                }

                tasks = []
                for robot_id in self.robot_ips:
                    if self.agent.robots_data.get(robot_id, {}).get('status') == 'connected':
                        task = asyncio.create_task(send_with_semaphore(robot_id, full_status))
                        tasks.append(task)

                await asyncio.gather(*tasks, return_exceptions=True)
                logging.debug(f"Parallel updates sent to {len(tasks)} robots")
            except Exception as e:
                logging.error(f"Update cycle error: {e}")

    async def run(self):
        """Main execution method"""
        try:
            await asyncio.gather(
                self.start_server(),
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


if __name__ == "__main__":
    class BaseAgent:
        def __init__(self):
            self.robots_data = defaultdict(lambda: {
                'last_seen': None,
                'status': 'disconnected',
                'data': {}
            })
            self.ifBall = False
            self.ball_x = None

    robot_server = RobotServer(BaseAgent())

    try:
        asyncio.run(robot_server.run())
    except KeyboardInterrupt:
        logging.info("Server stopped by user")
    