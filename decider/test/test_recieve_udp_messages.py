import socket

# 创建一个UDP套接字
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# 允许地址重用
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

# 绑定到所有可用的接口上的指定端口（例如：9999）
port = 8003
sock.bind(('', port))

# 加入广播接收
sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

print(f"Listening for broadcast messages on UDP port {port}...")

while True:
    # 接收数据，缓冲区大小为1024字节
    data, addr = sock.recvfrom(1024)
    print(f"Received message: {data.decode('utf-8')} from {addr}")
