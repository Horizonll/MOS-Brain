import socket
import json
import time
import random

# 配置服务器地址和端口
SERVER_HOST = '192.168.98.112'
SERVER_PORT = 8001  # 需要与RobotServer端口一致

# 角色到ID的映射
ROBOT_IDS = {
    "forward_1": 1,
    "forward_2": 2,
    "defender_1": 3,
    "goalkeeper": 4
}


def generate_robot_data(robot_id, scenario):
    """生成不同测试场景的模拟数据"""
    data = {
        "id": robot_id,
        "data": {
            "x": 0.0,
            "y": 0.0,
            "ballx": 0.0,
            "bally": 0.0,
            "status": 0  # 修改为 status 字段，0 表示没有球，1 表示有球
        },
        "timestamp": time.time()
    }

    # 场景1：球在后场（x > 10）
    if scenario == "backcourt_defense":
        data["data"].update({
            "x": random.uniform(8, 10),
            "y": random.uniform(2, 8),
            "ballx": 10.5,
            "bally": 5.0,
            # "status": 1 if robot_id == ROBOT_IDS["defender_1"] else 0  # 防守机器人持球
            "status": 0
        })

    # 场景2：球在中场（5 < x <= 10）
    elif scenario == "midfield_dribble":
        data["data"].update({
            "x": random.uniform(5, 10),
            "y": random.uniform(2, 8),
            "ballx": 7.5,
            "bally": 5.0,
            "status": 1 if robot_id == ROBOT_IDS["forward_1"] else 0  # 前锋1持球
        })

    # 场景3：球在前场（x <= 5）
    elif scenario == "frontcourt_shoot":
        data["data"].update({
            "x": random.uniform(2, 5),
            "y": random.uniform(2, 8),
            "ballx": 3.0,
            "bally": 5.0,
            "status": 1 if robot_id == ROBOT_IDS["forward_2"] else 0  # 前锋2持球
        })

    return json.dumps(data).encode('utf-8')


def simulate_scenario(scenario, duration=10):
    """模拟指定场景持续一段时间"""
    start_time = time.time()
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((SERVER_HOST, SERVER_PORT))
        print(f"\n===== 开始模拟场景：{scenario} =====")

        while time.time() - start_time < duration:
            # 逐个机器人生成并发送数据
            for role, rid in ROBOT_IDS.items():
                data = generate_robot_data(rid, scenario)
                # 发送数据
                s.sendall(data)

                print(f"已发送机器人 {rid} 的数据")
                time.sleep(0.1)  # 100ms 发送间隔

        print(f"===== 结束场景：{scenario} =====")


if __name__ == '__main__':
    # 依次测试三个主要场景
    simulate_scenario("backcourt_defense", duration=5)
    time.sleep(1)
    simulate_scenario("midfield_dribble", duration=5)
    time.sleep(1)
    simulate_scenario("frontcourt_shoot", duration=5)