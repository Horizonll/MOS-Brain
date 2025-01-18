import asyncio
import json
import websockets
import time
import logging
from config import *

# 初始化日志配置
logging.basicConfig(level=logging.INFO)

IP = '192.168.63.69'
# 初始化数据字典，包含机器人信息
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


async def websocket_server(websocket, path):
    """
    处理WebSocket连接，接收并响应客户端消息，定期发送机器人的数据。
    """
    try:
        # 定期发送机器人的数据给客户端
        send_task = asyncio.create_task(send_data_periodically(websocket))
        # 接收并处理来自客户端的数据
        async for message in websocket:
            try:
                robot_data = json.loads(message)
                await handle_websocket_message(robot_data)
            except json.JSONDecodeError as e:
                logging.error(f"Failed to decode JSON: {e}")
        
        # 如果连接关闭，取消定时发送任务
        send_task.cancel()
        try:
            await send_task
        except asyncio.CancelledError:
            pass

    except websockets.ConnectionClosed as e:
        logging.info(f"WebSocket connection closed: {e}")
    except Exception as e:
        logging.error(f"Error in websocket_server: {e}")

async def send_data_periodically(websocket):
    """
    定期向已连接的WebSocket客户端发送机器人的数据。
    """
    try:
        while True:
            await websocket.send(json.dumps(data))
            await asyncio.sleep(0.1)  # 调整这个值以改变发送频率
    except asyncio.CancelledError:
        # 正常处理任务取消的情况
        raise
    except Exception as e:
        logging.error(f"Error in send_data_periodically: {e}")

async def handle_websocket_message(robot_data):
    """
    根据从WebSocket接收到的消息更新机器人数据。
    """
    try:
        for robot in data["robots"]:
            if robot["id"] == robot_data["id"]:
                robot.update(robot_data)
                robot["timestamp"] = time.time()
                break
    except Exception as e:
        logging.error(f"Error handling WebSocket message: {e}")


# 定义一个异步主函数来管理所有任务
async def main():
    # 创建WebSocket服务器的任务
    start_server_task = websockets.serve(websocket_server, IP, 8000)
    
    # 使用 asyncio.gather 来同时运行多个协程
    await asyncio.gather(
        start_server_task,  # WebSocket服务器
        start_tcp_server(), # TCP服务器
        reset_robot_data()  # 数据重置任务
    )

if __name__ == "__main__":
    try:
        # 使用 asyncio.run() 来运行顶层的异步函数，并自动管理事件循环
        asyncio.run(main())
    except Exception as e:
        logging.error(f"Error in main loop: {e}")
