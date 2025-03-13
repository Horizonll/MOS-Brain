import socket
import time
import json

# 定义命令字典
COMMANDS = {
    "dribble": 'dribble',
    "forward": 'forward',
    "stop": 'stop',
    "find_ball": 'find_ball',
    "chase_ball": 'chase_ball',
    "shoot": 'kick',
    "go_back_to_field": 'go_back_to_field',
    "exit": 'exit'  # 添加一个退出选项
}

COMMANDS_DATA = {
    "dribble": {},
    "forward": {},
    "stop": {},
    "find_ball": {},
    "chase_ball": {},
    "shoot": {},
    "go_back_to_field": {'aim_x': 1000, 'aim_y': 2000},
    "exit": {},
}

def send_command(cmd, server_ip, server_port):
    """
    发送指定的命令到指定的IP和端口。
    
    :param cmd: 要发送的命令（必须在COMMANDS中）
    :param server_ip: 服务器IP地址
    :param server_port: 服务器端口号
    """
    if cmd == 'exit':
        print("Exiting program.")
        return False
    
    # 创建socket对象
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    
    try:
        # 连接服务器
        client_socket.connect((server_ip, server_port))
        
        # 准备要发送的数据
        cmd_data = {
            "command": cmd,
            "data": COMMANDS_DATA[cmd],
            "send_time": time.time(),
        }
        
        # 将字典转换为JSON字符串并编码为字节
        message = json.dumps(cmd_data).encode('utf-8')
        
        # 发送数据
        client_socket.sendall(message)
        print(f"Command '{cmd}' sent successfully.")
        
    except Exception as e:
        print(f"Error: {e}")
    finally:
        # 关闭socket连接
        client_socket.close()
    return True

def main():
    server_ip = "192.168.9.103"  # 默认服务器IP地址
    server_port = 8002       # 默认服务器端口号
    
    print("Available commands:")
    for key in COMMANDS.keys():
        if key != 'exit':
            print(f"- {key}")
    print("Enter 'exit' to quit the program.")

    while True:
        # 获取用户输入
        user_input = input("Enter command: ").strip().lower()
        
        if user_input not in COMMANDS:
            print("Invalid command. Please try again.")
            continue
        
        # 发送命令
        if not send_command(COMMANDS[user_input], server_ip, server_port):
            break

if __name__ == "__main__":
    main()