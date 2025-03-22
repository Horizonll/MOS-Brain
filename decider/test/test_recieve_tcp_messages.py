import socket

def start_server(host='192.168.98.112', port=8002):
    # 创建socket对象
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    
    # 绑定socket到指定的host和port
    server_socket.bind((host, port))
    
    # 开始监听请求
    server_socket.listen(5)  # 参数表示最多可以挂起的连接数
    print(f"Listening on {host}:{port}...")

    try:
        while True:
            # 接受客户端连接
            client_socket, addr = server_socket.accept()
            print(f"Received connection from {addr}")
            
            while True:
                # 接收数据
                data = client_socket.recv(1024)  # 缓冲区大小为1024字节
                if not data:
                    break
                print(f"Received data: {data.decode()}")  # 假设数据是文本格式
                
            # 关闭客户端连接
            client_socket.close()
            print(f"Connection with {addr} closed.")
            
    except KeyboardInterrupt:
        print("Server is shutting down.")
    finally:
        server_socket.close()

if __name__ == '__main__':
    start_server()