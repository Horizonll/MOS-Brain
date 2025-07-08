import unittest
import numpy as np

def calculate_velocities(ball_history, current_time, time_window=0.3):
    """
    提取速度计算核心逻辑，独立于类实例
    
    参数:
        ball_history: 球的历史位置数据列表
        current_time: 当前时间戳
        time_window: 时间窗口（秒）
    返回:
        (vx, vy): x和y方向的速度分量，计算失败返回(None, None)
    """
    # 筛选时间窗口内的历史数据
    recent_positions = [
        item for item in ball_history 
        if current_time - item["timestamp"] <= time_window
    ]
    
    # 至少需要3个点进行拟合
    if len(recent_positions) < 3:
        return (None, None)
    
    # 按时间排序
    recent_positions.sort(key=lambda x: x["timestamp"])
    
    # 提取时间和位置数据
    times = np.array([item["timestamp"] for item in recent_positions])
    positions = np.array([item["pos"][:2] for item in recent_positions])  # 只取x、y
    
    # 计算相对时间（相对于第一个点）
    t_rel = times - times[0]
    
    # 线性拟合计算速度 (s = v*t + b)
    A = np.vstack([t_rel, np.ones_like(t_rel)]).T  # 设计矩阵 [t, 1]
    
    try:
        # 拟合x方向速度
        vx, _ = np.linalg.lstsq(A, positions[:, 0], rcond=None)[0]
        # 拟合y方向速度
        vy, _ = np.linalg.lstsq(A, positions[:, 1], rcond=None)[0]
        return (vx, vy)
    except np.linalg.LinAlgError:
        return (None, None)


class TestVelocityCalculation(unittest.TestCase):
    def setUp(self):
        """准备日志中的球历史数据"""
        self.ball_history = [
            {"timestamp": 1751990456.2289412, "pos": [0.5665414, 1.5434854]},
            {"timestamp": 1751990456.2302444, "pos": [0.6075206, 1.530562]},
            {"timestamp": 1751990456.3097534, "pos": [0.67052203, 1.5007927]},
            {"timestamp": 1751990456.3242688, "pos": [0.7548452, 1.4828427]},
            {"timestamp": 1751990456.4096608, "pos": [0.60532385, 1.49697]},
            {"timestamp": 1751990456.430112, "pos": [0.56568927, 1.510653]},
            {"timestamp": 1751990456.5135608, "pos": [0.5603736, 1.5244484]},
            {"timestamp": 1751990456.982347, "pos": [0.6256609, 1.4816303]},
            {"timestamp": 1751990457.139644, "pos": [0.6258822, 1.481567]},
            {"timestamp": 1751990457.364897, "pos": [0.6056662, 1.4817841]},
            {"timestamp": 1751990457.4161732, "pos": [0.60711026, 1.4817278]},
            {"timestamp": 1751990457.4172833, "pos": [0.6117038, 1.483594]},
            {"timestamp": 1751990457.4182196, "pos": [0.6038387, 1.483494]},
            {"timestamp": 1751990457.4192612, "pos": [0.60938656, 1.4800274]},
            {"timestamp": 1751990457.4217296, "pos": [0.6085306, 1.4859834]},
            {"timestamp": 1751990457.4227185, "pos": [0.6159285, 1.5067725]},
            {"timestamp": 1751990457.4235685, "pos": [0.61254406, 1.5117701]},
            {"timestamp": 1751990457.4243956, "pos": [0.6070253, 1.5133129]},
            {"timestamp": 1751990457.4375424, "pos": [0.6114724, 1.5143818]},
            {"timestamp": 1751990457.528807, "pos": [0.6116738, 1.5169407]},
        ]
        # 从日志提取当前时间（第一条日志的current_time）
        self.current_time = 1751990457.585736397

    def test_velocity_calculation(self):
        """验证速度计算逻辑"""
        # 1. 调用速度计算函数
        vx, vy = calculate_velocities(
            ball_history=self.ball_history,
            current_time=self.current_time,
            time_window=0.3  # 与原代码保持一致的时间窗口
        )

        # 2. 验证返回值有效性
        self.assertIsNotNone(vx, "x方向速度计算失败")
        self.assertIsNotNone(vy, "y方向速度计算失败")

        # 3. 手动筛选有效数据（0.3秒内）
        valid_data = [
            item for item in self.ball_history
            if self.current_time - item["timestamp"] <= 0.3
        ]
        self.assertEqual(len(valid_data), 11, "有效数据量不符合预期")  # 验证筛选逻辑

        # 4. 手动计算预期速度（线性拟合）
        times = np.array([item["timestamp"] for item in valid_data])
        t_rel = times - times[0]  # 相对时间
        x_pos = np.array([item["pos"][0] for item in valid_data])
        y_pos = np.array([item["pos"][1] for item in valid_data])

        # 构建拟合矩阵并计算预期速度
        A = np.vstack([t_rel, np.ones_like(t_rel)]).T
        expected_vx, _ = np.linalg.lstsq(A, x_pos, rcond=None)[0]
        expected_vy, _ = np.linalg.lstsq(A, y_pos, rcond=None)[0]

        # 5. 对比计算结果与预期值（允许±0.05的误差）
        self.assertAlmostEqual(
            vx, expected_vx, delta=0.05,
            msg=f"x方向速度误差过大: 计算值={vx:.4f}, 预期值={expected_vx:.4f}"
        )
        self.assertAlmostEqual(
            vy, expected_vy, delta=0.05,
            msg=f"y方向速度误差过大: 计算值={vy:.4f}, 预期值={expected_vy:.4f}"
        )

        # 打印结果供参考
        print(f"\n速度计算验证结果:")
        print(f"x方向速度: 计算值={vx:.6f} m/s, 预期值={expected_vx:.6f} m/s")
        print(f"y方向速度: 计算值={vy:.6f} m/s, 预期值={expected_vy:.6f} m/s")


if __name__ == "__main__":
    unittest.main(verbosity=2)