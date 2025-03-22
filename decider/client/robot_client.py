import asyncio
import json
import socket
import threading
import time
import logging

class RobotClient:
    def __init__(self, agent):
        self.agent = agent
        self.config = self.agent._config
        self.ip = self.get_ip()
        self.HOST_IP = self.get_host_ip()

        # Start network-related threads
        self.start_network_threads()

    def get_ip(self):
        if self.config.get('use_detected_ip', True):
            logging.info("Starting to automatically detect the local IP address")
            return self.detect_ip()
        else:
            logging.info("Using the local IP address from the configuration file")
            return self.config.get('local_ip', "127.0.0.1")

    def get_host_ip(self):
        if self.config.get('use_detected_host_ip', True):
            logging.info("Starting to listen for UDP broadcasts to get the host IP address")
            return self.listen_host_ip()
        else:
            logging.info("Using the host IP address from the configuration file")
            return self.config.get('host_ip', "192.168.9.103")

    def start_network_threads(self):
        # Start the sending thread
        send_thread = threading.Thread(target=self.send_loop)
        send_thread.daemon = True
        send_thread.start()
        logging.info("Started the sending loop thread")

        # Start the TCP client thread
        tcp_thread = threading.Thread(target=self.start_tcp_listener_loop)
        tcp_thread.daemon = True
        tcp_thread.start()
        logging.info("Started the TCP client thread")

    def send_loop(self):
        while True:
            try:
                robot_data = {
                    "id": self.agent.id if hasattr(self.agent, 'id') else 0,
                    "data": {
                        "x": self.agent.get_self_pos()[0] if hasattr(self.agent, 'get_self_pos') else 0,
                        "y": self.agent.get_self_pos()[1] if hasattr(self.agent, 'get_self_pos') else 0,
                        "ballx": self.agent.get_ball_pos_in_map()[0] if hasattr(self.agent, 'get_ball_pos_in_map') else 0,
                        "bally": self.agent.get_ball_pos_in_map()[1] if hasattr(self.agent, 'get_ball_pos_in_map') else 0,
                        "yaw": self.agent.get_self_yaw() if hasattr(self.agent, 'get_self_yaw') else 0,
                    },
                    "info": self.agent._command["command"],
                    "timestamp": time.time(),
                    "ip": self.ip,
                }
                if not self.agent.get_if_ball():
                    robot_data["ballx"] = robot_data["bally"] = None
                self.send_robot_data(robot_data)
                time.sleep(0.5)
            except Exception as e:
                logging.error(f"Error sending data: {e}")

    def send_robot_data(self, robot_data):
        try:
            client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            client_socket.connect((self.HOST_IP, 8001))
            client_socket.sendall(json.dumps(robot_data).encode("utf-8"))
            logging.info("Successfully sent robot data")
        except Exception as e:
            logging.error(f"Error sending robot data: {e}")
        finally:
            client_socket.close()

    def detect_ip(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            s.connect(("8.8.8.8", 80))
            ip = s.getsockname()[0]
            logging.info(f"Detected local IP address: {ip}")
        except Exception as e:
            logging.error(f"Failed to detect local IP address: {e}")
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
            logging.error(f"Error starting the TCP listening loop: {e}")
        finally:
            loop.close()

    async def tcp_listener(self):
        """Asynchronously listen for TCP command messages"""
        self.ip = self.get_ip()
        logging.info(f"Current local IP address: {self.ip}")

        # Create an asynchronous server
        server = await asyncio.start_server(self.handle_connection, self.ip, 8002)

        addr = server.sockets[0].getsockname()
        logging.info(f"Started listening for TCP command messages at address: {addr}")

        try:
            async with server:
                await server.serve_forever()
        except KeyboardInterrupt:
            logging.info("User interrupted the program, exiting...")
        finally:
            # Close the server
            server.close()
            await server.wait_closed()
            logging.info("Server has been closed")

    async def handle_connection(self, reader, writer):
        try:
            while True:
                data = await reader.read(1024)
                if not data:
                    break
                addr = writer.get_extra_info("peername")
                logging.info(f"Received data from {addr}: {data}")

                try:
                    received_data = json.loads(data.decode("utf-8"))
                    logging.info(f"Parsed JSON data: {received_data}")

                    # Here you can handle different commands specifically
                    if "command" in received_data:
                        self.agent._command = received_data
                    else:
                        logging.warning("Received message does not contain the 'command' field")
                except json.JSONDecodeError as e:
                    logging.error(f"JSON decoding error: {e}")
        except Exception as e:
            logging.error(f"Error handling connection: {e}")
        finally:
            writer.close()
            await writer.wait_closed()

    def listen_host_ip(self):
        """Synchronously listen for UDP broadcast messages and get the host IP"""
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(("0.0.0.0", 8003))  # Listen for UDP broadcasts on the specified port

        # Join the broadcast reception
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

        while True:
            data, addr = sock.recvfrom(1024)
            try:
                received_data = json.loads(data.decode("utf-8"))
                if "message" in received_data and received_data["message"] == "thmos_hello":
                    # Assume the broadcast message contains the host IP information
                    if "ip" in received_data:
                        host_ip = received_data["ip"]
                        self.HOST_IP = host_ip  # Update the IP address in the Agent instance
                        logging.info(f"Updated the host IP address to: {host_ip}")
                        return host_ip
                    else:
                        logging.warning("Received 'thmos_hello' message, but the 'ip' field was not found")
                else:
                    logging.warning("Received message does not match the expected format")
            except json.JSONDecodeError as e:
                logging.error(f"JSON decoding error: {e}")
            except KeyboardInterrupt:
                logging.info("User interrupted the program, exiting...")