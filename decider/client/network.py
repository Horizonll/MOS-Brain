# Network.py
#
#   @description: Commucate with server using UDP
#
#   @change logs:
#       Jun 23th, 2025: First code

import json, socket, time, threading, hashlib

class Network:
    def __init__(self, agent):
        self.agent = agent
        self.logger = self.agent.get_logger()
        self.logger.info("[Network] Submodule started")

        # TODO: read config from file or transfer in from agent
        self.config = {
            "send_robot_data": {
                "port": 8001, 
                "frequency": 20,     # in Hz
                "secret": "YXdpb3BqeHo7bHas"
            },
            "receive_from_server": {
                "port": 8002, 
                "timeout": 10       # re-qeury server ip after timeout, in second
                "secret": "PIDcvpasdvfpIFES"
            }, 
            "find_server_ip": {
                "constant_server_ip": "auto-find", 
                "broadcast_port": 8003, 
                "secret": "a2xzYXZoO29hd2pp"
            }
        }


    def _send_loop(self):
        send_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        config = self.config.get("send_robot_data")
        self.logger.info("[Network.send_loop] Started")
        while True:
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
            # TODO: comment here if signature is not required
            robot_data_str = json.dumps(robot_data)
            hash_str = hashlib.sha3_256(robot_data_str.encode("utf-8") + \
                config.get("secret")).hexdigest()
            data = {"raw": robot_data_str, "signature": hash_str};
            try:
                send_socket.connect((self.server_ip, config.port))
                send_socket.sendall(json.dumps(data).encode("utf-8"))
            except Exception as e:
                self.logger.error(f"[Network.send_loop] Error sending robot data: {e}")
            time.sleep(1.0 / config.get("frequency"))


    def _receive_loop(self):
        config = self.config.get("receive_robot_data")
        self.logger.info("[Network.receive_loop] Started")
        recv_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        recv_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        recv_socket.settimeout(config.get("timeout"))
        while True:
            try:
                recv.bind((self.server_ip, config.get("port")))
                data_with_signature_raw, addr = recv_sock.recvfrom(4096)
                data_with_signature = json.loads(data_with_signature_raw.decode("utf-8").strip())

                # verify signature
                data = data_with_signature["raw"]
                signature = data_with_signature["signature"]
                target_signature = hashlib.sha3_256(data.encode("utf-8") + \
                    config.get("secret")).hexdigest()
                if target_signature != signature:
                    self.logger.warn("[Network.receive_loop] Received but signature mismatch")
                    continue

                js_data = json.loads(data.strip())
                if "command" in js_data:
                    self.agent._command = js_data
                    self.agent._last_command_time = time.time()
                elif "robots_data" in js_data:
                    self.agent._robots_data = js_data["robots_data"]
                    self.agent._last_command_time = time.time()
                else:
                    self.logger.warn("[Network.receive_loop]" + \
                        "Received message does not contain 'command' or 'robots_data'")
            except socket.timeout:
                self.logger.error(f"[Network.receive_loop] Server lost")
                self.server_ip = self._find_server_ip()
            except json.JSONDecodeError as e:
                self.logger.error(f"[Network.receive_loop] JSON decoding error: {e}")
            except KeyError as e:
                self.logger.error(f"[Network.receive_loop] Key error: {e}")
            except KeyboardInterrupt:
                self.logger.info("[Network.receive_loop] Loop break" + \
                    " due to KeyboradInterrupt")
                break
        

    def _find_server_ip(self):
        config = self.config.get("find_server_ip")
        broadcast_port = config.get("broadcast_port")
        broadcast_token = config.get("broadcast_secret")
        server_ip = config.get("constant_server_ip")

        if server_ip != "auto-find":
            self.logger.info(f"[Network.listen_server_ip] Constant servre ip: {server_ip}")
            return server_ip
        
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        client_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        client_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

        self.logger.info(f"[Network.listen_server_ip] Listening for server ip on {broadcast_port}")
        self.logger.debug("[Network.listen_server_ip] Secret = {broadcast_secret}")

        client_socket.bind(('', broadcast_port))
        while True:
            try:
                data, addr = client_socket.recvfrom(4096)
                # TODO: change here if you do not want encryption
                server_ip = addr[0]
                target = hashlib.sha3_256((server_ip + config.get("secret")).encode("utf-8")).hexdigest()
                if message == target:
                    break;
            except socket.timeout:
                self.logger.debug("[Network.listen_server_ip] No broadcast" + \
                    " message received within timeout period.")
                continue
            except KeyboardInterrupt:
                self.logger.info("[Network.listen_server_ip] Client stopped" + \
                    " due to KeyboradInterrupt")
                break
            except Exception as e:
                self.logger.info(f"[Network.listen_server_ip] Client stopped: {e}")
                break
        client_socket.close()
        return server_ip

    
