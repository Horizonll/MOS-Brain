# MOS-Brain

K1 机器人决策代码库

### 调试及启动方法

#### 硬件准备

1. 按电源键启动时，注意不要移动机器人，头最好看向前方，使ZED视觉里程计正常初始化

#### 各模块文件说明

| 文件名称                | 所属类型       | systemd service名称       | 位置                                                         |
| ----------------------- | -------------- | ------------------------- | ------------------------------------------------------------ |
| interface.py            | 混合（转接层） | booster_interface.service | /home/booster/Workspace/THMOS/interface-booster/interface.py |
| test_zed_ros2_debug.py  | 视觉           | vision.service            | /home/booster/Workspace/THMOS/vision/scripts/test_zed_ros2_debug.py |
| particle_filter_ros2.py | 视觉（定位）   | 暂未实现                  | /home/booster/Workspace/THMOS/vision/scripts/particle_filter_ros2.py |
| head_control.py         | 决策           | head_control.service      | /home/booster/Workspace/THMOS/head-control/head_control.py   |
| decider.py              | 决策主模块     | 暂未实现                  | /home/booster/Workspace/THMOS/MOS-Brain/decider/client/decider.py |
| decider_tester.py       | 决策测试程序   | 不需要                 | /home/booster/Workspace/THMOS/MOS-Brain/decider/test/decider_tester.py |


#### 启动流程（调试）

1. systemd自动启动模块说明

目前，大部分模块已经实现开机自启，正常情况下无需额外操作，如果需要调试参照下面的说明

    - interface.py：用于将ROS消息转换为Booster接口消息
    - vision.service：用于启动视觉模块，包含ZED相机的ROS2节点
    - head_control.service：用于控制机器人的头部动作

有用的命令：
```sh
# 查看服务状态
systemctl --user status vision.service
# 停止服务
systemctl --user stop vision.service
# 重启服务
systemctl --user restart vision.service
# 禁用开机自启
systemctl --user disable vision.service
# 查看日志
journalctl --user-unit vision.service -f
```

2. 启动粒子滤波

根据机器人位置更改particle_filter_ros2.py中20行的坐标（单位为m，rad），然后启动粒子滤波
```sh
python3 /home/booster/Workspace/THMOS/vision/scripts/particle_filter_ros2.py
```

3. 启动decider

- debug模式（测试单独动作）
脚本使用tmux分屏，ctrl+b 然后按方向键切换终端，左边是决策输出，右边发送指令
```sh
bash /home/booster/Workspace/THMOS/MOS-Brain/decider/scripts/decider_local_test.sh
```

- 运行模式（测试比赛逻辑）
```sh
bash /home/booster/Workspace/THMOS/MOS-Brain/decider/client/decider.py
```


#### 启动流程（比赛）


#### 目录结构

目前项目用到的所有文件都在decider文件夹下。

- decider
  - client # 存放机器人（从机）决策代码
    - subStateMachines # 所有动作的子状态机类
    - \_\_init\_\_.py
    - config.yaml # 机器人配置文件
    - decider.py # **机器人决策主程序**
    - receiver.py # 裁判盒接收器
    - network.py # 网络通信相关
  - server # 多机决策相关
  - statemachine_readme # 决策主机状态机说明文档
  - test # 决策主机测试代码
    - decider_tester.py # 机器人控制指令发送程序
  - README.md # 参数和接口说明
  - requirements.txt
  - transitions.md
