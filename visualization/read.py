import asyncio
import websockets
import json
from concurrent.futures import ThreadPoolExecutor
from config import *


def process_data(received_data):
    for robot in received_data["robots"]:
        print(robot["timestamp"])


async def websocket_client():
    uri = f"ws://{IP}:8001"
    with ThreadPoolExecutor() as executor:
        while True:
            try:
                async with websockets.connect(uri) as websocket:
                    while True:
                        data = await websocket.recv()
                        try:
                            received_data = json.loads(data)
                            print(f"Received: {received_data}")
                            executor.submit(process_data, received_data)
                        except json.JSONDecodeError as e:
                            print(f"JSON decode error: {e}")
            except websockets.ConnectionClosed as e:
                print(f"Connection closed: {e}")
            except ConnectionRefusedError as e:
                print(f"Connection refused: {e}. Retrying")
            except Exception as e:
                print(f"Error: {e}")


asyncio.run(websocket_client())
