#!/usr/bin/env python3
import socket
import fcntl
import struct
import subprocess
import re
import time

def get_wireless_ip():
    """获取当前无线网卡的IP地址"""
    # 获取所有网络接口
    interfaces = subprocess.check_output("ls /sys/class/net/", shell=True).decode().strip().split('\n')
    
    # 检查每个接口是否为无线接口并获取IP
    for interface in interfaces:
        # 检查是否为无线接口
        try:
            subprocess.check_output(f"iwconfig {interface} 2>/dev/null | grep -i ESSID", shell=True)
            is_wireless = True
        except subprocess.CalledProcessError:
            is_wireless = False
        
        if is_wireless:
            try:
                # 使用socket获取IP地址
                s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                ip = socket.inet_ntoa(fcntl.ioctl(
                    s.fileno(),
                    0x8915,  # SIOCGIFADDR
                    struct.pack('256s', interface[:15].encode())
                )[20:24])
                return interface, ip
            except Exception as e:
                print(f"获取接口 {interface} 的IP地址时出错: {e}")
    
    return None, None

if __name__ == "__main__":
    interface, ip = get_wireless_ip()
    if interface and ip:
        print(f"无线接口: {interface}, IP地址: {ip}")
    else:
        print("未找到无线接口或IP地址")