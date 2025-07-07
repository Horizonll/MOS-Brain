import hashlib
import socket
import time
import json
import argparse
import traceback

# Command dictionary (using first letters as shortcuts)
COMMANDS = {
    'd': {'name': 'dribble', 'shortcut': 'd', 'description': 'Dribble'},
    's': {'name': 'stop', 'shortcut': 's', 'description': 'Stop'},
    'f': {'name': 'find_ball', 'shortcut': 'f', 'description': 'Find Ball'},
    'c': {'name': 'chase_ball', 'shortcut': 'c', 'description': 'Chase Ball'},
    'k': {'name': 'kick', 'shortcut': 'k', 'description': 'Kick Ball'},
    'g': {'name': 'go_back_to_field', 'shortcut': 'g', 'description': 'Return to Field'},
    'q': {'name': 'exit', 'shortcut': 'q', 'description': 'Exit Program'},
    'gk': {'name': 'goalkeeper', 'shortcut': 'gk', 'description': 'Goalkeeper Mode'}
}

COMMANDS_DATA = {
    "dribble": {},
    "forward": {},
    "stop": {},
    "find_ball": {},
    "chase_ball": {},
    "kick": {},
    "go_back_to_field": {'aim_x': 0.0, 'aim_y': 2.0, 'aim_yaw': 0},
    "exit": {},
    "goalkeeper": {}
}

def sign_message(message: str, secret: str) -> str:
    hash_str = hashlib.sha3_256((message + secret).encode("utf-8")).hexdigest()
    ret = {"raw": message, "sign": hash_str}
    return json.dumps(ret)


def verify_sign(data: str, secret: str) -> str:
    try:
        js_data = json.loads(data)
        message = js_data.get("raw", None)
        target_hash = hashlib.sha3_256((message + secret).encode("utf-8")).hexdigest()
        if js_data.get("sign", None) == target_hash:
            return message
    except Exception as e:
        pass
    return None 

def send_command(cmd, server_ip, server_port=8002, protocol='udp'):
    """
    Send the specified command to the given IP and port using the specified protocol.
    
    :param cmd: Command to send (must exist in COMMANDS)
    :param server_ip: Server IP address
    :param server_port: Server port number
    :param protocol: Protocol to use ('tcp' or 'udp'), default is 'udp'
    """
    if cmd == 'exit':
        print("Program exited.")
        return False
    
    try:
        # Prepare data
        cmd_data = {
            "command": cmd,
            "data": COMMANDS_DATA[cmd],
            "send_time": time.time()
        }
        message = json.dumps(cmd_data)
        
        if protocol.lower() == 'tcp':
            # Create and connect TCP socket
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client_socket:
                client_socket.settimeout(2.0)  # Set timeout
                client_socket.connect((server_ip, server_port))
                
                # Send data
                client_socket.sendall(message)
                
                # Receive server response
                try:
                    response = client_socket.recv(1024).decode('utf-8')
                    print(f"Server response: {response}")
                except socket.timeout:
                    print("Sent successfully, but no response from server")
                    
        elif protocol.lower() == 'udp':
            # Create UDP socket
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as client_socket:
                client_socket.settimeout(2.0)  # Set timeout
                message = sign_message(message=message, secret="PIDcvpasdvfpIFES")
                client_socket.sendto(message.encode(), (server_ip, server_port))

        else:
            print(f"Unsupported protocol: {protocol}. Please use 'tcp' or 'udp'.")
            return True
            
        print(f"Command '{cmd}' sent successfully using {protocol.upper()}.")
        
    except Exception as e:
        print(f"Failed to send command using {protocol.upper()}: {e}")
        traceback.print_exc()
        
    return True


def main():
    # Parse command line arguments to get server IP and protocol
    parser = argparse.ArgumentParser(description='Send control commands to specified server')
    parser.add_argument('--ip', type=str, default="192.168.9.51",
                        help='Server IP address (default: 192.168.9.51)')
    parser.add_argument('--protocol', type=str, default="udp", choices=['tcp', 'udp'],
                        help='Protocol to use (default: udp)')
    args = parser.parse_args()
    
    server_ip = args.ip
    server_port = 8002  # Fixed port number
    protocol = args.protocol.lower()
    
    print(f"Server IP: {server_ip}, Port: {server_port}, Protocol: {protocol.upper()}")
    print("\nAvailable commands and shortcuts:")
    
    # Display command menu (sorted alphabetically)
    for key in sorted(COMMANDS.keys()):
        cmd = COMMANDS[key]
        print(f"[{cmd['shortcut']}] {cmd['description']}")
    
    print("\nEnter corresponding letter to execute command, 'q' to quit.")
    
    while True:
        user_input = input("\nEnter command: ").strip().lower()
        
        if user_input in COMMANDS:
            cmd_name = COMMANDS[user_input]['name']
            if not send_command(cmd_name, server_ip, server_port, protocol):
                break
        else:
            print("Invalid command, please try again.")

if __name__ == "__main__":
    main()
