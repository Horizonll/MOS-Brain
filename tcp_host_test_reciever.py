import socket
import signal
import sys


def get_local_ip():
    try:
        # 创建一个 UDP 套接字
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # 连接到一个公共的 DNS 服务器
        s.connect(("8.8.8.8", 80))
        # 获取本机 IP 地址
        local_ip = s.getsockname()[0]
        s.close()
        return local_ip
    except Exception as e:
        print(f"获取本机 IP 地址时出错: {e}")
        return None


# 定义信号处理函数
def signal_handler(sig, frame):
    print('你按下了 Ctrl+C，程序即将退出...')
    sys.exit(0)


def listen_tcp_messages(ip, port):
    try:
        # 创建 TCP 套接字
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # 绑定 IP 地址和端口
        server_socket.bind((ip, port))
        # 开始监听，最大连接数为 1
        server_socket.listen(1)
        print(f"正在监听 {ip}:{port}...")

        while True:
            try:
                # 接受客户端连接
                client_socket, client_address = server_socket.accept()
                print(f"已连接到 {client_address}")
                try:
                    while True:
                        # 接收客户端发送的数据
                        data = client_socket.recv(1024)
                        if not data:
                            break
                        # 解码并打印接收到的数据
                        message = data.decode('utf-8')
                        print(f"收到消息: {message}")
                except KeyboardInterrupt:
                    print("接收到中断信号，程序即将退出...")
                    break
                except Exception as e:
                    print(f"接收消息时出错: {e}")
                finally:
                    # 关闭客户端套接字
                    client_socket.close()
            except Exception as e:
                print(f"监听时出错: {e}")
    except KeyboardInterrupt:
        print("服务器正在关闭...")
    except Exception as e:
        print(f"发生错误: {e}")
    finally:
        # 关闭服务器套接字
        if 'server_socket' in locals():
            server_socket.close()


if __name__ == "__main__":
    # 注册信号处理函数
    signal.signal(signal.SIGINT, signal_handler)
    # 获取本机 IP 地址
    local_ip = get_local_ip()
    if local_ip:
        # 监听的端口号
        port = 8001
        # 开始监听 TCP 消息
        listen_tcp_messages(local_ip, port)