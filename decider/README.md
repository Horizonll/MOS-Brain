# subscribe.py

- pos_x: 机器人在世界坐标系（球场）的 x 坐标
- pos_y: 机器人在世界坐标系（球场）的 y 坐标
- pos_yaw: 机器人在世界坐标系（球场）的偏航角，单位：度，[-180, 180]
- ifBall: 是否有球
- ball_x: 球在图像中的 x 坐标
- ball_y: 球在图像中的 y 坐标
- ball_x_in_map: 球在世界坐标系（球场）的 x 坐标
- ball_y_in_map: 球在世界坐标系（球场）的 y 坐标
- ball_distance: 机器人到球的距离
- doKick: 踢球
- head_set: 设置头部角度

# config.py

- walk_x_vel 行走前向速度
- walk_y_vel 行走横向速度
- walk_theta_vel 转向速度
- 定位点,x,y

# 机器人->主机

```python
robot_data = {
    "id": self.id,
    "x": self.pos_x,
    "y": self.pos_y,
    "ballx": self.ball_x_in_map,
    "bally": self.ball_y_in_map,
    "yaw": self.pos_yaw,
    "info": self.info,
    "kick_off": self.receiver.kick_off,
}
```

# 主机->机器人

```python
data = {
    "robots": [
        {"id": 1, "commamd": "find_ball", "timestamp": 0},
        {"id": 2, "commamd": "find_ball", "timestamp": 0},
        {"id": 3, "commamd": "find_ball", "timestamp": 0},
        {"id": 4, "commamd": "find_ball", "timestamp": 0},
    ],
    "ball": {"x": 0, "y": 0},
}
```

command: find_ball, chase_ball, dribble, stop, kick, go_back_to_field
