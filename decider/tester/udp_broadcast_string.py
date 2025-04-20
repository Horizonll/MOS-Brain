import socket

def broadcast_message(message, port):
    # 创建一个UDP套接字
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    # 设置套接字选项，允许广播
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    
    # 广播地址
    broadcast_address = ('192.168.9.255', port)
    
    try:
        # 发送广播消息
        sock.sendto(message.encode(), broadcast_address)
        print(f"Broadcasted message: {message}")
    finally:
        # 关闭套接字
        sock.close()

if __name__ == "__main__":
    message = "NjM5NWVkMSAgLQo="
    port = 8004  # 选择一个端口号
    broadcast_message(message, port)