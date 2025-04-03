import asyncio
import json
import socket
import threading
import time
import rospy


class RobotClient:
    def __init__(self, agent):

        self.agent = agent
        self.config = self.agent._config
        self.HOST_IP = self.get_host_ip()
        print("Host ip = " + self.HOST_IP)
        rospy.loginfo("Host ip = " + self.HOST_IP)

        # Start network-related threads
        self.start_network_threads()

    def get_host_ip(self):
        if self.config['auto_find_server_ip'] is True:
            rospy.loginfo("Starting to listen for UDP broadcasts to get the host IP address")
            return self.listen_host_ip()
        else:
            rospy.loginfo("Using the host IP address from the configuration file")
            return self.config.get('server_ip')

    def start_network_threads(self):
        # Start the sending thread
        send_thread = threading.Thread(target=self.send_loop)
        send_thread.daemon = True
        send_thread.start()
        print("send_thread started")
        rospy.loginfo("Started the sending loop thread")

        # Start the TCP client thread
        tcp_thread = threading.Thread(target=self.start_tcp_listener_loop)
        tcp_thread.daemon = True
        tcp_thread.start()
        rospy.loginfo("Started the TCP client thread")

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
                    },
                    "info": self.agent._command["command"],
                    "timestamp": time.time(),
                }
                if not self.agent.get_if_ball():
                    robot_data["ballx"] = robot_data["bally"] = None
                self.send_robot_data(robot_data)
                time.sleep(0.5)
            except Exception as e:
                print(f"Error in send_loop: {e}")
                rospy.logerr(f"Error sending data: {e}")

    def send_robot_data(self, robot_data):
        try:
            client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            if self.agent.if_local_test == True:
                client_socket.connect(('127.0.0.1', 8001))
            else:
                client_socket.connect((self.HOST_IP, 8001))
            client_socket.sendall(json.dumps(robot_data).encode("utf-8"))
            # rospy.loginfo("Successfully sent robot data")
        except Exception as e:
            rospy.logerr(f"Error sending robot data: {e}")
        finally:
            client_socket.close()

    def start_tcp_listener_loop(self):
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        try:
            loop.run_until_complete(self.tcp_listener())
        except Exception as e:
            rospy.logerr(f"Error starting the TCP listening loop: {e}")
        finally:
            loop.close()

    async def tcp_listener(self):
        """Asynchronously listen for TCP command messages"""
        # Create an asynchronous server
        server = await asyncio.start_server(self.handle_connection, 
                                "0.0.0.0", self.agent._config.get("server_port"))

        addr = server.sockets[0].getsockname()
        rospy.loginfo(f"Started listening for TCP command messages at address: {addr}")

        try:
            async with server:
                await server.serve_forever()
        except KeyboardInterrupt:
            rospy.loginfo("User interrupted the program, exiting...")
        finally:
            # Close the server
            server.close()
            await server.wait_closed()
            rospy.loginfo("Server has been closed")

    async def handle_connection(self, reader, writer):
        try:
            while True:
                data = await reader.read(1024)
                if not data:
                    break
                addr = writer.get_extra_info("peername")
                rospy.loginfo(f"Received data from {addr}: {data}")

                try:
                    received_data = json.loads(data.decode("utf-8"))
                    rospy.loginfo(f"Parsed JSON data: {received_data}")

                    # Here you can handle different commands specifically
                    if "command" in received_data:
                        self.agent._command = received_data
                    else:
                        rospy.logwarn("Received message does not contain the 'command' field")
                except json.JSONDecodeError as e:
                    rospy.logerr(f"JSON decoding error: {e}")
        except Exception as e:
            rospy.logerr(f"Error handling connection: {addr}:{e}")
        finally:
            writer.close()
            await writer.wait_closed()

    # Synchronously listen for UDP broadcast messages and get the host IP
    def listen_host_ip(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.BROADCAST, 1)
        sock.bind(("", self.agent._config.get("auto_find_server_ip_listen_port")))

        while True:
            data, addr = sock.recvfrom(1024)
            try:
                if(data.decode("utf-8") == \
                        self.config["auto_find_server_ip_token"]):
                    self.HOST_IP = addr[0]
                    rospy.loginfo(f"Updated the host IP address to: {self.HOST_IP}")
                    return self.HOST_IP
                else:
                    rospy.logwarn("Received message does not match the expected format")
            except json.JSONDecodeError as e:
                rospy.logerr(f"JSON decoding error: {e}")
            except KeyboardInterrupt:
                rospy.loginfo("User interrupted the program, exiting...")
                break
