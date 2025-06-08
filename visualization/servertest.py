import asyncio
import json
import time
import logging
from collections import defaultdict
from datetime import datetime

# 初始化日志配置
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

IP = "192.168.63.157"
PORT = 8001

# 使用字典存储机器人数据，以便快速查找和更新
robots_data = defaultdict(lambda: {
    'last_seen': None,
    'latency': [],
    'status': 'disconnected',
    'data': {}
})

async def handle_robot(reader, writer):
    """
    异步处理来自机器人的TCP连接。
    读取数据，解析JSON，并更新机器人的数据。
    """
    addr = writer.get_extra_info('peername')
    robot_id = None
    try:
        while True:
            data_received = await reader.read(1024)
            if not data_received:
                break
            
            # 记录接收到数据的时间
            receive_time = time.time()
            try:
                robot_data = json.loads(data_received.decode("utf-8"))
                send_time_str = robot_data.get('send_time')
                if send_time_str:
                    send_time = float(send_time_str)
                
                robot_id = robot_data["id"]
                logging.info(f"Received data from robot {robot_id}: {robot_data}")
                
                # 更新或添加机器人数据
                robots_data[robot_id]['last_seen'] = datetime.now().isoformat()
                robots_data[robot_id]['status'] = 'connected'
                robots_data[robot_id]['data'].update(robot_data)
                

                # 构建响应消息
                response = {
                    "status": "received",
                    "server_receive_time": receive_time,
                    "server_send_time": time.time()  # 记录发送时间
                }
                writer.write(json.dumps(response).encode("utf-8"))
                await writer.drain()

                # 记录处理结束时间
                process_end_time = time.time()
                # processing_latency = process_end_time - receive_time
                # logging.info(f"Processing latency for robot {robot_id}: {processing_latency:.3f} seconds")
                # robots_data[robot_id]['latency'].append(processing_latency)

            except json.JSONDecodeError as e:
                logging.error(f"JSON decode error: {e}")
            except Exception as e:
                logging.error(f"Error processing message from robot {robot_id}: {e}")
    except Exception as e:
        logging.error(f"Connection error with {addr}: {e}")
    finally:
        if robot_id is not None:
            robots_data[robot_id]['status'] = 'disconnected'
        writer.close()
        await writer.wait_closed()
        # logging.info(f"Disconnected from {addr}")

async def start_tcp_server():
    """
    启动一个TCP服务器，监听指定的IP和端口。
    """
    server = await asyncio.start_server(handle_robot, IP, PORT)
    addr = server.sockets[0].getsockname()
    logging.info(f"Serving on {addr}")

    async with server:
        await server.serve_forever()

async def monitor_robots():
    """
    定期打印所有机器人的状态信息。
    """
    while True:
        logging.info("\nRobot Status:")
        for robot_id, info in robots_data.items():
            # logging.info(f"Robot ID: {robot_id}, Last Seen: {info['last_seen']}, "
            #             f"Status: {info['status']}")
            pass
        await asyncio.sleep(5)  # 每隔5秒打印一次

if __name__ == "__main__":
    try:
        loop = asyncio.get_event_loop()
        # Schedule both tasks to run concurrently
        tasks = [start_tcp_server(), monitor_robots()]
        loop.run_until_complete(asyncio.gather(*tasks))
    except KeyboardInterrupt:
        logging.info("Server stopped by user")
    except Exception as e:
        logging.error(f"Error in main loop: {e}")