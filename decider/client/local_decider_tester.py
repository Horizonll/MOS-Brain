import subprocess
import time


try:
    subprocess.Popen(['xterm', '-e', 'roscore'])

    time.sleep(3)

    subprocess.Popen(['xterm', '-e', 'roslaunch thmos_bringup mos_run.launch'])

    command = 'cd ~/thmos_ws/src/thmos_code/vision/scripts && python3 vision_with_local.py'
    subprocess.Popen(['xterm', '-e', command])

    command2 = 'python3 /home/thmos/MOS-Brain/decider/client/test_decider.py'
    subprocess.Popen(['xterm', '-e', command2])
except Exception as e:
    print(f"执行过程中出现错误: {e}")  

import socket
import time
import json
import threading
import signal
import sys
from contextlib import contextmanager

# 全局配置
HEARTBEAT_PORT = 8001       # 心跳监听端口
COMMAND_PORT = 8002         # 指令发送端口
ROBOT_IP = "127.0.0.1"   # 机器人IP地址
HEARTBEAT_TIMEOUT = 5       # 心跳超时时间（秒）

# 共享状态
last_heartbeat = None
heartbeat_lock = threading.Lock()
connection_active = False
alert_triggered = False

# 命令配置
COMMANDS = {
    "stop": 'stop',
    "find_ball": 'find_ball',
    "chase_ball": 'chase_ball',
    "shoot": 'kick',
    "go_back_to_field": 'go_back_to_field',
    "goalkeeper": 'goalkeeper',
}

COMMANDS_DATA = {
    "dribble": {},
    "forward": {},
    "stop": {},
    "find_ball": {},
    "chase_ball": {},
    "kick": {},
    "go_back_to_field": {'aim_x': 0, 'aim_y': 2000, 'aim_yaw': 0},
    "goalkeeper": {},
    "exit": {},
}

@contextmanager
def socket_context(*args,**kwargs):
    """Socket上下文管理器"""
    sock = socket.socket(*args, **kwargs)
    try:
        yield sock
    finally:
        sock.close()

def signal_handler(sig, frame):
    print("\n程序已终止")
    sys.exit(0)

def heartbeat_server():
    """心跳监听服务器"""
    global last_heartbeat, connection_active
    local_ip = '127.0.0.1'
    
    with socket_context(socket.AF_INET, socket.SOCK_STREAM) as server:
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            server.bind((local_ip, HEARTBEAT_PORT))
            server.listen(1)
            print(f"心跳监听已启动 @ {local_ip}:{HEARTBEAT_PORT}")
            
            while True:
                client, addr = server.accept()
                with client:
                    connection_active = True
                    # print(f"设备已连接: {addr}")
                    while True:
                        data = client.recv(32)
                        if not data:
                            break
                        with heartbeat_lock:
                            last_heartbeat = time.time()
        except Exception as e:
            print(f"心跳服务器异常: {e}")
        finally:
            connection_active = False

def monitor_heartbeat():
    """心跳监控线程"""
    global alert_triggered
    while True:
        current_time = time.time()
        with heartbeat_lock:
            expired = last_heartbeat and (current_time - last_heartbeat > HEARTBEAT_TIMEOUT)
            connected = connection_active and last_heartbeat

        if connected and expired and not alert_triggered:
            print(f"⚠️  超过{HEARTBEAT_TIMEOUT}秒未收到心跳！")
            alert_triggered = True
        elif connected and not expired and alert_triggered:
            print("✅ 心跳恢复")
            alert_triggered = False
        
        time.sleep(1)

def send_command(command, retries=3):
    """发送控制指令"""
    if command == "exit":
        return False

    for attempt in range(retries):
        try:
            with socket_context(socket.AF_INET, socket.SOCK_STREAM) as sock:
                sock.connect((ROBOT_IP, COMMAND_PORT))
                payload = json.dumps({
                    "command": command,
                    "data": COMMANDS_DATA.get(command, {}),
                    "timestamp": time.time()
                }).encode()
                sock.sendall(payload)
                print(f"指令发送成功: {command}")
                return True
        except Exception as e:
            if attempt == retries-1:
                print(f"指令发送失败: {command} ({e})")
            else:
                time.sleep(0.5)
    return True

def main():
    signal.signal(signal.SIGINT, signal_handler)
    
    # 启动心跳服务
    threading.Thread(target=heartbeat_server, daemon=True).start()
    
    # 启动心跳监控
    threading.Thread(target=monitor_heartbeat, daemon=True).start()
    
    # 用户指令界面
    print("可用指令:")
    print("\n".join([f"• {cmd}" for cmd in COMMANDS if cmd != "exit"]))
    
    while True:
        try:
            cmd = input("输入指令 > ").strip().lower()
            if cmd not in COMMANDS:
                print("无效指令")
                continue
            
            if not send_command(COMMANDS[cmd]):
                break
            if cmd == "exit":
                break
                
        except KeyboardInterrupt:
            print("\n操作已取消")
            break

if __name__ == "__main__":
    main()