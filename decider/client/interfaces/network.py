# interfaces/network.py
#   
#   @description: The class of network utilities
#

import time
import socket

# interfaces/network.py
#   @description:   network utilities

class Network:
    # class Network
    #
    #   @public variants
    #
    #   @public methods
    #       receive() -> string     receive message from server async 
    #
    #   @private variants
    #       _config     a dict of configuration
    #       _server     a tuple of 2, (ip, port)
    #       _reader
    #       _write
    #
    #   @private methods
    #       _find_server_ip():  listening on udp broadcase to find server

    def __init__(self, config):
        self._config = config
        if(self._config["auto_find_server_ip"]):
            self._server_addr = self._find_server_ip()
        else:
            self._server_addr = (self._config["server_ip"], \
                                 self._config["server_port"])
        self._server_socket = socket.socket(family=socket.AF_INET, \
                                    type=socket.SOCK_STREAM)
        self._server_socket.setblocking(False)
        print("[+] interfaces/network.py: Network.__init__(): " +\
                "listening tcp on " + str(self._server_addr))
        self._server_socket.connect(self._server_addr)

    def receive(self):
        try:
            data = self._server_socket.recv(self._config["max_buffer_size"])
            return data.decode("utf-8")
        except Exception as e:
            pass
        return None
    
    def _find_server_ip():
        udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        udp_socket.bind(('', self._config["auto_find_server_ip_listen_port"]))
        print("[+] interfaces/network.py: Network._find_server_ip(): " + \
                "listening for server on port " + \
                str(self._config["auto_find_server_ip_listen_port"] + \
                " token = " = self._config["auto_find_server_ip_token"]
        while True:
                    message, address = udp_socket.recvfrom(self._config["max_buffer_size"])
                    if(message == self._config["auto_find_server_ip_token"]):
                        return (address[0], self_config["server_port"])
                    time.sleep(0.1)


