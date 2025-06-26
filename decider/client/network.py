# py
#
#   @description: Commucate with server using UDP
#
#   @change logs:
#       Jun 23th, 2025: First code
#       Jun 24th, 2025: Test passed

import json, socket, time, threading, hashlib

class Network:
    def __init__(self, agent):
        self.agent = agent
        self.logger = self.agent.get_logger().get_child("network")
        self.logger.info("Submodule started")

        # TODO: read config from file or transfer in from agent
        self.config = {
            "send_robot_data": {
                "port": 8001, 
                "frequency": 10,     # in Hz
                "secret": "YXdpb3BqeHo7bHas"
            },
            "receive_from_server": {
                "port": 8002, 
                "timeout": 10,       # re-qeury server ip after timeout, in second
                "secret": "PIDcvpasdvfpIFES"
            }, 
            "find_server_ip": {
                "constant_server_ip": "auto-find", 
                "port": 8003, 
                "secret": "a2xzYXZoO29hd2pp"
            }
        }

        self.server_ip = None
        self.start_find_srv_ip()

    def start_send_loop(self):
        try:
            self._send_loop_thread.stop()
        except Exception as e:
            pass
        self.logger.info("Starting the send_loop thread")
        self._send_loop_thread = threading.Thread(target=self._send_loop)
        self._send_loop_thread.daemon = True
        self._send_loop_thread.start()
    
    def start_receive_loop(self):
        try:
            self._recv_loop_thread.stop()
        except Exception as e:
            pass
        self.logger.info("Starting the recv_loop thread")
        self._recv_loop_thread = threading.Thread(target=self._receive_loop)
        self._recv_loop_thread.daemon = True
        self._recv_loop_thread.start()
    
    def start_find_srv_ip(self):
        try:
            self._find_srv_ip_thread.stop()
        except Exception as e:
            pass
        self.logger.info("Starting the find_srv_ip thread")
        self._find_srv_ip_thread = threading.Thread(target=self._find_server_ip)
        self._find_srv_ip_thread.daemon = True
        self._find_srv_ip_thread.start()


    def _send_loop(self):
        send_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        send_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            send_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        except Exception as e:
            self.logger.warn("SO_REUSEPORT are not supported!")
        
        config = self.config.get("send_robot_data")
        logger = self.logger.get_child("send_loop")
        logger.info("Started loop")
        while True:
            time.sleep(1.0 / config.get("frequency"))
            if self.server_ip == None:
                continue
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
                "timestamp": time.time()
            }
            if not self.agent.get_if_ball():
                robot_data["ballx"] = robot_data["bally"] = None
            signed_msg = self._sign_message(json.dumps(robot_data), \
                                            config.get("secret"))
            try:
                address = (self.server_ip, config.get("port"))
                send_socket.sendto(signed_msg.encode("utf-8"), address)
            except Exception as e:
                logger.error(f"Error sending robot data: {e}")
        send_socket.close()


    def _receive_loop(self):
        config = self.config.get("receive_from_server")
        logger = self.logger.get_child("receive_loop")
        logger.info("Started loop")
        recv_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        recv_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            recv_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        except Exception as e:
            logger.warn("SO_REUSEPORT are not supported!")
        recv_socket.settimeout(config.get("timeout"))
        recv_socket.bind(("", config.get("port")))
        while True:
            try:
                data, addr = recv_socket.recvfrom(4096)
                # if addr[0] != self.server_ip:
                #     continue
                # data = self._verify_sign(data.decode("utf-8"), config.get("secret"))
                # if data == None:
                #     logger.warn("Signature mismatch!")
                #     continue
                logger.info(f"Received data from {addr[0]}:{addr[1]}")
                js_data = json.loads(data)
                if "command" in js_data:
                    self.agent._command = js_data
                    self.agent._last_command_time = time.time()
                elif "robots_data" in js_data:
                    self.agent._robots_data = js_data["robots_data"]
                    self.agent._last_command_time = time.time()
                else:
                    logger.warn("Received message does not contain 'command' or 'robots_data'")
            except socket.timeout:
                logger.error(f"Server lost")
                self.server_ip = None
                self.start_find_srv_ip()
            except json.JSONDecodeError as e:
                logger.error(f"JSON decoding error: {e}")
            except KeyError as e:
                self.logger.error(f"[receive_loop] Key error: {e}")
            except KeyboardInterrupt:
                logger.info("Loop break due to KeyboradInterrupt")
                break
        recv_socket.close()
        

    def _find_server_ip(self) -> str:
        config = self.config.get("find_server_ip")
        logger = self.logger.get_child("find_server_ip")
        port = config.get("port")
        server_ip = config.get("constant_server_ip")
        if server_ip != "auto-find":
            logger.info(f"Constant server ip: {server_ip}")
            self.server_ip = server_ip
            return
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        client_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            client_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        except Exception as e:
            self.logger.warn("SO_REUSEPORT are not supported!")
        client_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        logger.info(f"Listening for server ip on {port}")
        logger.debug("Secret = {broadcast_secret}")
        client_socket.bind(('', port))
        while True:
            try:
                data, addr = client_socket.recvfrom(4096)
                data = self._verify_sign(data.decode("utf-8"), config.get("secret"))
                if data == addr[0]:
                    server_ip = addr[0]
                    logger.info(f"Verified server ip: {server_ip}")
                    break
            except socket.timeout:
                logger.debug("No broadcast message received within timeout period.")
                continue
            except KeyboardInterrupt:
                logger.info("Client stopped due to KeyboradInterrupt")
                break
            except Exception as e:
                logger.info(f"Client stopped: {e}")
                break
        client_socket.close()
        self.server_ip = server_ip


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
