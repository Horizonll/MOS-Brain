import asyncio
import json
import websockets
import time
import logging
from config import *

# 初始化日志配置
logging.basicConfig(level=logging.INFO)

# 初始化数据字典，包含机器人信息
data = {
    "robots": [
        {"id": 1, "timestamp": 0},
        {"id": 2, "timestamp": 0},
        {"id": 3, "timestamp": 0},
        {"id": 4, "timestamp": 0},
        {"id": 5, "timestamp": 0},
        {"id": 6, "timestamp": 0},
    ],
    "ball": {"x": 0, "y": 0},
}


# 定义一个异步函数，用于重置机器人的数据
async def reset_robot_data():
    """
    定期检查并重置机器人的数据。
    遍历机器人列表，如果机器人的空闲时间超过3秒，则将其数据重置。
    """
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


# 定义一个异步函数，用于处理来自机器人的TCP连接
async def handle_robot(reader, writer):
    """
    处理来自机器人的TCP连接。
    读取数据，解析JSON，并更新机器人的数据。
    """
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


# 定义一个异步函数，用于启动TCP服务器
async def start_tcp_server():
    """
    启动一个TCP服务器，监听指定的IP和端口。
    """
    server = await asyncio.start_server(handle_robot, IP, 8002)
    try:
        async with server:
            await server.serve_forever()
    except Exception as e:
        logging.error(f"Error in start_tcp_server: {e}")


# 定义一个异步函数，用于处理WebSocket服务器
async def websocket_server(websocket, path):
    """
    处理WebSocket连接，定期发送机器人的数据。
    """
    try:
        while True:
            await websocket.send(json.dumps(data))
            await asyncio.sleep(0.1)
    except websockets.ConnectionClosed as e:
        logging.info(f"WebSocket connection closed: {e}")
    except Exception as e:
        logging.error(f"Error in websocket_server: {e}")


# 定义一个异步函数，用于读取WebSocket数据
async def websocket_read_data(websocket, path):
    """
    通过WebSocket读取数据。
    """
    try:
        while True:
            await websocket.send(json.dumps(data))
            await asyncio.sleep(0.1)
    except websockets.ConnectionClosed as e:
        logging.info(f"WebSocket connection closed: {e}")
    except Exception as e:
        logging.error(f"Error in websocket_read_data: {e}")


# 启动WebSocket服务器
start_server = websockets.serve(websocket_server, IP, 8000)
read_data_server = websockets.serve(websocket_read_data, IP, 8001)

# 获取事件循环并运行服务器和数据重置任务
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
