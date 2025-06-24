# fake_server.py
# 
#   @description:
#       Run this code on your computer, without any dependency 
#   

YOUR_IP_ADDRESS = "192.168.9.103" # The IP address in the same subnet with robot
import time, hashlib, socket, json, threading

class FakeServer:
    def start_broadcast_ip(self):
        self.broadcast_thread = threading.Thread(target = self.broadcast_ip)
        self.broadcast_thread.daemon = True
        self.broadcast_thread.start()

    
    def start_listen_loop(self):
        self.listen_thread = threading.Thread(target = self.listen_loop)
        self.listen_thread.daemon = True
        self.listen_thread.start()


    def broadcast_ip(self):
        print("Start broadcast thread")
        
        self_ip = YOUR_IP_ADDRESS
        address = ("255.255.255.255", 8003) # NOT RECOMMAND, BUT GOOD FOR DEBUG
        secret = "a2xzYXZoO29hd2pp"

        server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

        while True:
            try:
                signed_message = self._sign_message(self_ip, secret)
                server_socket.sendto(signed_message.encode("utf-8"), address)
                print(f"[Broadcast] Sent: {signed_message}")
                time.sleep(1)
            except KeyboardInterrupt:
                print("[Broadcast] Loop break due to KeyboradInterrupt")
                break

        server_socket.close()

    
    def listen_loop(self):
        print("Start listen loop")
        secret = "YXdpb3BqeHo7bHas"
        port = 8001
        
        listen_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        listen_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        listen_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        listen_socket.bind(("", port))

        while True:
            try:
                data_with_sign, addr = listen_socket.recvfrom(4096)
                data = self._verify_sign(data_with_sign, secret)
                print("[Listen] Received: " + data)
            except KeyboardInterrupt:
                print("[Listen] Loop break due to KeyboradInterrupt")
                break

        listen_socket.close()

    
    def send_loop(self):
        print("Start send loop")
        secret = "PIDcvpasdvfpIFES"
        address = ("192.168.9.42", 8002)
        
        send_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        send_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        send_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

        num = 1
        while True:
            try:
                data = json.dumps({"command": "test_udp", "data": str(num)})
                send_socket.sendto(self._sign_message(data, secret).encode("utf-8"), address)
                # print(f"[Send] Sent command: {data}")
                time.sleep(1)
            except KeyboardInterrupt:
                print("[Send] Loop break due to KeyboradInterrupt")
                break
            num += 1
        send_socket.close()


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
    srv = FakeServer()
    srv.start_broadcast_ip()
    srv.start_listen_loop()
    srv.send_loop()
