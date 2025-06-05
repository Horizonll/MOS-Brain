import socket
import time
import json
import argparse

# Command dictionary (using first letters as shortcuts)
COMMANDS = {
    'd': {'name': 'dribble', 'shortcut': 'd', 'description': 'Dribble'},
    's': {'name': 'stop', 'shortcut': 's', 'description': 'Stop'},
    'f': {'name': 'find_ball', 'shortcut': 'f', 'description': 'Find Ball'},
    'c': {'name': 'chase_ball', 'shortcut': 'c', 'description': 'Chase Ball'},
    'k': {'name': 'kick', 'shortcut': 'k', 'description': 'Kick Ball'},
    'g': {'name': 'go_back_to_field', 'shortcut': 'g', 'description': 'Return to Field'},
    'q': {'name': 'exit', 'shortcut': 'q', 'description': 'Exit Program'}
}

COMMANDS_DATA = {
    "dribble": {},
    "forward": {},
    "stop": {},
    "find_ball": {},
    "chase_ball": {},
    "kick": {},
    "go_back_to_field": {'aim_x': 1000, 'aim_y': 2000, 'aim_yaw': 0},
    "exit": {},
}

def send_command(cmd, server_ip, server_port=8002):
    """
    Send the specified command to the given IP and port.
    
    :param cmd: Command to send (must exist in COMMANDS)
    :param server_ip: Server IP address
    :param server_port: Server port number
    """
    if cmd == 'exit':
        print("Program exited.")
        return False
    
    try:
        # Create and connect socket
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client_socket:
            client_socket.settimeout(2.0)  # Set timeout
            client_socket.connect((server_ip, server_port))
            
            # Prepare and send data
            cmd_data = {
                "command": cmd,
                "data": COMMANDS_DATA[cmd],
                "send_time": time.time()
            }
            message = json.dumps(cmd_data).encode('utf-8')
            client_socket.sendall(message)
            
            # Optional: Receive server response
            try:
                response = client_socket.recv(1024).decode('utf-8')
                print(f"Server response: {response}")
            except socket.timeout:
                print("Sent successfully, but no response from server")
                
            print(f"Command '{cmd}' sent successfully.")
            
    except Exception as e:
        print(f"Failed to send command: {e}")
        
    return True

def main():
    # Parse command line arguments to get server IP
    parser = argparse.ArgumentParser(description='Send control commands to specified server')
    parser.add_argument('--ip', type=str, default="192.168.9.51",
                        help='Server IP address (default: 192.168.9.51)')
    args = parser.parse_args()
    
    server_ip = args.ip
    server_port = 8002  # Fixed port number
    
    print(f"Server IP: {server_ip}, Port: {server_port}")
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
            if not send_command(cmd_name, server_ip, server_port):
                break
        else:
            print("Invalid command, please try again.")

if __name__ == "__main__":
    main()