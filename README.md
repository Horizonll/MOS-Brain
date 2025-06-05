# MOS-Brain

The decision-rev of MOS-8.5.

### 调试及启动方法

#### 硬件准备

1. 确认电池有电，电压大于等于16.5V，否则需要充电，充电器操作方法问李老师

2. 电池紧贴泡沫板安装牢固，拧紧舱盖，电池线从机器人底部走

3. 电池接口用魔术贴固定在背箱侧面，防止运动时脱落，切勿放在箱内导致短路

4. 启动时确认肩部关节位置正常没有过度旋转

#### 调试启动流程

1. 启动roscore，保证所有topic注册在一个core中

```sh
roscore
```

2. 检查电机开关，启动步态程序

```sh
roslaunch thmos_bringup mos_run.launch
```

- 如果带有电机序号的错误提示，先检查电机线是否松动，如果没有问题，尝试重启步态程序，再不行联系本体同学。
- 不带电机序号的错误提示，大概率IMU没接好，检查IMU线是否牢固，是否亮灯

3. 启动视觉程序

插拔ZED一次，将机器人放置到入场点，根据入场位置调整三个参数
  
```sh
cd ~/thmos_ws/src/thmos_code/vision/scripts && python3 vision_with_local.py 3300 1000 90
```

启动后，一定检查IMU是否发生明显漂移（0.1degrees/s以上），如果有漂移就插拔IMU重启步态

- 正常情况下帧率为10，如果大于10问题大概率是ZED没连上，先尝试插拔，插拔解决不了换线
- 帧率小于10可能是debug没关或内存泄漏
- 检查当前定位命令：`rostopic echo /pos_in_map`，如果没有数据，检查IMU和ZED是否正常工作

1. 启动决策程序

- 情况一：单独测试每个状态机

以debug模式启动单机决策

```sh
cd ~/MOS-Brain/decider/client && python3 decider.py --debug
```

拿到本机ip后，在同一子网内的设备上（如笔记本）运行测试程序

```sh
cd ~/MOS-Brain/decider/test && python3 decider_tester.py --ip your-ip
```

- 情况二：测试比赛策略

启动子机决策，注意在`client/config_override.json`中配置team和id

```sh
cd ~/MOS-Brain/decider/client && python3 decider.py
```

如果在该机器人上运行多机决策，运行

```sh
cd ~/MOS-Brain/decider/server && python3 decider_server.py
```

### Sync

command:

```sh
rsync -av <path-on-your-computer>/MOS-Brain/ thmos@david:<path-on-robot>/MOS-Brain/ --delete
```

### 快速开始

1. 将机器人放置在场上(2600,500,90)处
2. 启动nomachine连接机器人
3. 为防止识别不到ZED相机，插拔相机一次
4. 打开终端，运行`python3 ~/MOS-brain/decider/client/local_decider_tester.py`，该操作将启动roscore、视觉、步态、决策程序，需要将机器人平放或提起来防止受损。
5. 等待步态和视觉程序启动
6. 在终端中输入指令，机器人将执行相应动作。

- chase_ball：追球，离得近时会停下
- shoot：射门，离得近时再执行
- go_back_to_field：回场，离得近时会停下，默认去(0,2000,0)，可以在local_decider_tester.py中修改
- find_ball：逆时针转身找球
- stop：停止，停止所有动作

#### 现存问题

1. 杂物太多会导致不面朝球门时定位不准，进而导致go_back_to_field动作失败。
2. 踢球不够精准高效

### 逐模块启动

1. 将机器人放置在场上(2600,500,90)处
2. 启动nomachine连接机器人
3. 为防止识别不到ZED相机，插拔相机一次
4. 打开终端
5. 运行步态程序

```
roslaunch thmos_bringup mos_run.launch
```

6. 运行视觉程序

```
cd ~/thmos_ws/src/thmos_code/vision/scripts && python3 vision_with_local.py
```

7. 在`client/config.json`中配置`"server_ip"`地址为你想要运行主机决策设备的ip
8. 运行从机决策程序

```
python3 /home/thmos/MOS-Brain/decider/client/decider.py
```

9. 运行主机决策程序

- 测试时（手动发送指令）用`client/tester/tcp_host_test_reciever.py`接收心跳包，用`client/tester/decider_tester.py`发送指令。
- 主机决策，运行`server_v1_2.py`

### 目录结构

目前项目用到的所有文件都在decider文件夹下。

- decider
  - client # 存放机器人（从机）决策代码
    - subStateMachines # 所有动作的子状态机类
    - \_\_init\_\_.py
    - config.json # 机器人配置文件，暂未使用
    - configuration.py # 机器人配置类，暂未使用
    - decider.py # **机器人决策主程序**
    - receiver.py # 接收ROS消息的类
    - subscriber.py # ROS订阅者类
  - server # 存放决策主机代码，开发中
  - statemachine_readme # 决策主机状态机说明文档
  - test # 决策主机测试代码
    - decider_tester.py # 机器人控制指令发送程序
    - tcp_host_test_reciever.py # 机器人心跳包接收程序
  - README.md # 参数和接口说明
  - requirements.txt
  - transitions.md

### Players / Clients

```client/decider.py``` is run on the robot. It sent its locations and the ball position to the server, and execute the commands from server.

##### Arch

- subscriber.py / publisher.py
    The interface of ROS. ```subscriber.py``` reads ```/pos_in_map``` and ```/obj_pos```, ```publisher.py``` publish topic ```/kick```, ```/head_goal``` and ```/cmd_vel```.

- decider.py
    The main file, where the project entry.
    Object Agent has attribute ```(pos_x, pos_y), (ball_x_in_map, ball_y_in_map), self_yaw, self_info, self.ip```

- subStateMachines/
    The directory containing sub-statemachines such as, chase_ball, go_back_to_field.

### Controller / Server

The code in dir server/ is run on server.  

### 关于端口

- server端tcp端口：8001
- client端tcp端口：8002
- client端udp端口：8003
