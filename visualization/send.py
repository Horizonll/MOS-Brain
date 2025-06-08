import socket
import json
import time
import random
import argparse
import logging
import numpy as np
from config import *

logging.basicConfig(level=logging.INFO)


def send_robot_data(robot_data):
    try:
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client_socket.connect((IP, 8002))
        client_socket.sendall(json.dumps(robot_data).encode("utf-8"))
    except Exception as e:
        logging.error(f"Error in send_robot_data: {e}")
    finally:
        client_socket.close()


# parser = argparse.ArgumentParser(description="Send robot data to server.")
# parser.add_argument("--id", type=int, required=True, help="Robot ID")
# args = parser.parse_args()

robot_data = {
    "id": 1,
    "x": 0,
    "y": 0,
    "ballx": 0,
    "bally": 0,
    "yaw": 0,
    "info": "moving to target",
}

while True:
    robot_data["id"] += 1
    if robot_data["id"] > 6:
        robot_data["id"] = 1
    robot_data["x"] += random.randint(-100, 100)
    robot_data["y"] += random.randint(-100, 100)
    robot_data["ballx"] += random.randint(-100, 100)
    robot_data["bally"] += random.randint(-100, 100)
    robot_data["yaw"] += random.randint(-100, 100)
    robot_data["info"] = random.choice(
        ["moving to target", "searching for ball", "kicking ball"]
    )
    robot_data["x"] = int(np.clip(robot_data["x"], -250, 250))
    robot_data["y"] = int(np.clip(robot_data["y"], -400, 400))
    robot_data["ballx"] = int(np.clip(robot_data["ballx"], -250, 250))
    robot_data["bally"] = int(np.clip(robot_data["bally"], -400, 400))
    robot_data["yaw"] = int(np.clip(robot_data["yaw"], -180, 180))
    send_robot_data(robot_data)
    # print(robot_data)
    time.sleep(0.5)
