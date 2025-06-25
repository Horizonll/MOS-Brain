import asyncio
import json
import socket
import threading
import time


class RobotClient:
    def __init__(self, agent):
        self.agent = agent
        self.logger = self.agent.get_logger().get_child("robot_client")
        self.config = self.agent._config
        # 设置默认的 HOST_IP
        self.HOST_IP = self.config.get('server_ip', '127.0.0.1')
        self.logger.info("Default Host ip = " + self.HOST_IP)
        self.logger.info("Default Host ip = " + self.HOST_IP)

        # Start network-related threads
        self.start_network_threads()

    def start_network_threads(self):
        # 启动监听服务器 IP 的线程
        listen_ip_thread = threading.Thread(target=self.listen_server_ip_loop)
        listen_ip_thread.daemon = True
        listen_ip_thread.start()
        self.logger.info("Started the server IP listening thread")

        # Start the sending thread
        send_thread = threading.Thread(target=self.send_loop)
        send_thread.daemon = True
        send_thread.start()
        self.logger.info("send_thread started")
        self.logger.info("Started the sending loop thread")

        # Start the TCP client thread
        tcp_thread = threading.Thread(target=self.start_tcp_listener_loop)
        tcp_thread.daemon = True
        tcp_thread.start()
        self.logger.info("Started the TCP client thread")

    def listen_server_ip_loop(self):
        while True:
            if self.config['auto_find_server_ip']:
                loop = asyncio.new_event_loop()
                asyncio.set_event_loop(loop)
                try:
                    new_ip = loop.run_until_complete(self.listen_host_ip())
                    if new_ip:
                        self.HOST_IP = new_ip
                        self.logger.info("Host ip updated to = " + self.HOST_IP)
                        self.logger.info("Host ip updated to = " + self.HOST_IP)
                except Exception as e:
                    self.logger.error(f"Error listening for server IP: {e}")
                finally:
                    loop.close()
            time.sleep(1)  # 每秒检查一次

    def send_loop(self):
        while True:
            try:
                robot_data = {
                    "id": self.agent.get_config()["id"],
                    "data": {
                        "x": float(self.agent.get_self_pos()[0]),
                        "y": float(self.agent.get_self_pos()[1]),
                        "ballx": float(self.agent.get_ball_pos_in_map()[0]),
                        "bally": float(self.agent.get_ball_pos_in_map()[1]),
                        "yaw": float(self.agent.get_self_yaw()),
                        "ball_distance": float(self.agent.get_ball_distance()),
                        "if_ball": bool(self.agent.get_if_ball()),
                    },
                    "info": self.agent._command["command"],
                    "timestamp": time.time(),
                }
                if not self.agent.get_if_ball():
                    robot_data["ballx"] = robot_data["bally"] = None
                self.send_robot_data(robot_data)
                time.sleep(0.5)
            except Exception as e:
                self.logger.error(f"Error in send_loop: {e}")

    def send_robot_data(self, robot_data):
        try:
            client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            if self.agent.if_local_test:
                client_socket.connect(('127.0.0.1', 8001))
            else:
                client_socket.connect((self.HOST_IP, 8001))
            client_socket.sendall(json.dumps(robot_data).encode("utf-8"))
        except Exception as e:
            self.logger.error(f"Error sending robot data: {e}")
        finally:
            client_socket.close()

    def start_tcp_listener_loop(self):
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        try:
            loop.run_until_complete(self.tcp_listener())
        except Exception as e:
            self.logger.error(f"Error starting the TCP listening loop: {e}")
        finally:
            loop.close()

    async def tcp_listener(self):
        server = await asyncio.start_server(self.handle_connection,
                                            "0.0.0.0", self.agent._config.get("server_port"))
        addr = server.sockets[0].getsockname()
        self.logger.info(f"Started listening for TCP command messages at address: {addr}")
        try:
            async with server:
                await server.serve_forever()
        except KeyboardInterrupt:
            self.logger.info("User interrupted the program, exiting...")
        finally:
            server.close()
            await server.wait_closed()
            self.logger.info("Server has been closed")

    async def handle_connection(self, reader, writer):
        try:
            while True:
                data = await reader.read(2048)
                if not data:
                    break
                addr = writer.get_extra_info("peername")
                try:
                    received_data = json.loads(data.decode("utf-8"))
                    if "command" in received_data:
                        self.agent._command = received_data
                        self.agent._last_command_time = time.time()
                    elif "robots_data" in received_data:
                        self.agent._robots_data = received_data["robots_data"]
                        self.agent._last_command_time = time.time()
                    else:
                        self.logger.warn("Received message does not contain 'command' or 'robots_data'")
                except json.JSONDecodeError as e:
                    self.logger.error(f"JSON decoding error: {e}")
                except KeyError as e:
                    self.logger.error(f"Key error: {e}")
        except Exception as e:
            self.logger.error(f"Error handling connection: {addr}:{e}")
        finally:
            writer.close()
            await writer.wait_closed()

    async def listen_host_ip(self):
        port = self.agent._config.get("auto_find_server_ip_listen_port")
        expected_token = self.config.get("auto_find_server_ip_token")
        loop = asyncio.get_running_loop()
        transport, protocol = await loop.create_datagram_endpoint(
            lambda: UDPProtocol(self, expected_token),
            local_addr=('0.0.0.0', port)
        )
        self.logger.info(f"Listening for UDP broadcast on port {port}...")
        try:
            while True:
                await asyncio.sleep(0.1)
        except asyncio.CancelledError:
            pass
        finally:
            transport.close()
        self.logger.info("UDP listener stopped.")
        return getattr(self, 'HOST_IP', None)


class UDPProtocol(asyncio.DatagramProtocol):
    def __init__(self, client, expected_token):
        self.client = client
        self.expected_token = expected_token

    def datagram_received(self, data, addr):
        received_message = data.decode("utf-8").strip()
        if received_message == self.expected_token:
            self.client.HOST_IP = addr[0]
            self.logger.info(f"Host IP found: {self.client.HOST_IP}")
        else:
            self.logger.warn(f"Received unexpected message: {received_message}")
            self.logger.warn(f"Expected token: {self.expected_token}")
    
