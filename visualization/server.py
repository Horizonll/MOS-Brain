import asyncio
import json
import websockets
import time
import logging
from config import *

logging.basicConfig(level=logging.INFO)
data = {
    "robots": [
        {"id": 1, "timestamp": 0},
        {"id": 2, "timestamp": 0},
        {"id": 3, "timestamp": 0},
        {"id": 4, "timestamp": 0},
        {"id": 5, "timestamp": 0},
        {"id": 6, "timestamp": 0},
    ]
}


async def reset_robot_data():
    while True:
        current_time = time.time()
        for robot in data["robots"]:
            if current_time - robot["timestamp"] > 3:
                robot["timestamp"] = 0
                robot_keys = list(robot.keys())
                for key in robot_keys:
                    if key not in ["id", "timestamp"]:
                        del robot[key]
        await asyncio.sleep(0.1)


async def handle_robot(reader, writer):
    try:
        while True:
            data_received = await reader.read(1000)
            if not data_received:
                break
            try:
                robot_data = json.loads(data_received.decode("utf-8"))
                for robot in data["robots"]:
                    if robot["id"] == robot_data["id"]:
                        robot.update(robot_data)
                        robot["timestamp"] = time.time()
                        break
                writer.write(json.dumps(data).encode("utf-8"))
                await writer.drain()
            except json.JSONDecodeError as e:
                logging.error(f"JSON decode error: {e}")
    except Exception as e:
        logging.error(f"Error in handle_robot: {e}")
    finally:
        writer.close()
        await writer.wait_closed()


async def start_tcp_server():
    server = await asyncio.start_server(handle_robot, IP, 8002)
    try:
        async with server:
            await server.serve_forever()
    except Exception as e:
        logging.error(f"Error in start_tcp_server: {e}")


async def websocket_server(websocket, path):
    try:
        while True:
            await websocket.send(json.dumps(data))
            await asyncio.sleep(0.1)
    except websockets.ConnectionClosed as e:
        logging.info(f"WebSocket connection closed: {e}")
    except Exception as e:
        logging.error(f"Error in websocket_server: {e}")


async def websocket_read_data(websocket, path):
    try:
        while True:
            await websocket.send(json.dumps(data))
            await asyncio.sleep(0.1)
    except websockets.ConnectionClosed as e:
        logging.info(f"WebSocket connection closed: {e}")
    except Exception as e:
        logging.error(f"Error in websocket_read_data: {e}")


start_server = websockets.serve(websocket_server, IP, 8000)
read_data_server = websockets.serve(websocket_read_data, IP, 8001)

loop = asyncio.get_event_loop()
try:
    loop.run_until_complete(
        asyncio.gather(
            start_server, read_data_server, start_tcp_server(), reset_robot_data()
        )
    )
    loop.run_forever()
except Exception as e:
    logging.error(f"Error in main loop: {e}")
finally:
    loop.close()
