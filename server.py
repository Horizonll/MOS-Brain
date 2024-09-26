import asyncio
import json
import websockets
import time
import logging

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
            except json.JSONDecodeError as e:
                logging.error(f"JSON decode error: {e}")
    except Exception as e:
        logging.error(f"Error in handle_robot: {e}")
    finally:
        writer.close()
        await writer.wait_closed()


async def start_tcp_server():
    server = await asyncio.start_server(handle_robot, "localhost", 8002)
    try:
        async with server:
            await server.serve_forever()
    except Exception as e:
        logging.error(f"Error in start_tcp_server: {e}")


async def websocket_server(websocket, path):
    try:
        while True:
            await websocket.send(json.dumps(data))
            await asyncio.sleep(0.5)
    except websockets.ConnectionClosed as e:
        logging.info(f"WebSocket connection closed: {e}")
    except Exception as e:
        logging.error(f"Error in websocket_server: {e}")


start_server = websockets.serve(websocket_server, "localhost", 8000)

loop = asyncio.get_event_loop()
try:
    loop.run_until_complete(asyncio.gather(start_server, start_tcp_server()))
    loop.run_forever()
except Exception as e:
    logging.error(f"Error in main loop: {e}")
finally:
    loop.close()
