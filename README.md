# MOS-Brain

The decision-rev of MOS-8.5.

### 快速开始

#### 1. 机器人

##### 快速启动脚本

首先将`./decider/client/decider.py`中的100行`HOST_IP`改为自己电脑或决策主机的ip地址。

启动机器人，会打开多个screen，分别启动roscore、视觉、步态、决策程序。

```bash
chmod +x ./decider/scripts/run_robot_everything.sh
./decider/scripts/run_robot_everything.sh
```

可以用`screen -R [name]`进入screen，用`ctrl+A`然后`ctrl+D`退出screen，screen名称分别如下：

- vision：无报错，输出debug信息，表示视觉成功启动。

- walk：无报错，表示步态启动成功。

- decider：输出`serving on [ip]`表示机器人子机决策程序已经启动。

##### 手动启动

首先将`./decider/client/decider.py`中的100行`HOST_IP`改为自己电脑或决策主机的ip地址。

假设vision、walk已经启动，则运行

```bash
python3 ./decider/client/decider.py
```

#### 2. 决策主机

##### 快速启动脚本

在自己电脑上

windows运行  

```bash
./decider/scripts/run_decider_tester_on_windows.ps1
```

Linux运行  

```bash
chmod +x ./decider/scripts/run_decider_tester_on_linux.sh

./decider/scripts/run_decider_tester_on_linux.sh
```  

 启动决策主机，会打开2个终端窗口，分别启动接收从机实时状态、决策程序。

可以按照提示向机器人发送指令。

##### 手动启动

在自己电脑上开启两个终端，分别运行

```bash
python3 ./decider/test/decider_tester.py
```

(用于指令发送)

和

```bash
python3 ./decider/test/tcp_host_test_reciever.py
```

（用于接收从机实时状态）

### 目录结构

目前项目用到的所有文件都在decider文件夹下。

- decider
  - client # 存放机器人（从机）决策代码
    - subStateMachines # 所有动作的子状态机类
    - \_\_init\_\_.py
    - config.json # 机器人配置文件，暂未使用
    - configuration.py # 机器人配置类，暂未使用
    - decider.py # **机器人决策主程序**
    - receiver.py # 接收裁判盒
    - subscriber.py # ROS订阅者类
  - scripts # 存放启动脚本
    - run_robot_everything.sh # 机器人决策启动脚本
    - run_decider_tester_on_linux.sh # 决策主机启动脚本
    - run_decider_tester_on_windows.ps1 # 决策主机启动脚本
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
