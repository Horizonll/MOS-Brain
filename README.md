# MOS-Brain

The decision-rev of MOS-8.5.

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

* subscriber.py / publisher.py
    The interface of ROS. ```subscriber.py``` reads ```/pos_in_map``` and ```/obj_pos```, ```publisher.py``` publish topic ```/kick```, ```/head_goal``` and ```/cmd_vel```.

* decider.py
    The main file, where the project entry. 
    Object Agent has attribute ```(pos_x, pos_y), (ball_x_in_map, ball_y_in_map), self_yaw, self_info, self.ip```

* subStateMachines/
    The directory containing sub-statemachines such as, chase_ball, go_back_to_field.


### Controller / Server

The code in dir server/ is run on server.  
