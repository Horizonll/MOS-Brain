# MOS-Brain

The decision-rev of MOS-8.5.

### 快速开始

#### 1. 机器人

运行 `./decider/scripts/run_robot_everything.sh` 启动机器人，会打开多个screen，分别启动roscore、视觉、步态、决策程序。

可以用`screen -R [name]`进入screen，用`ctrl+A`然后`ctrl+D`退出screen，screen名称分别如下：

- vision：无报错，输出debug信息，表示视觉成功启动。

- walk：无报错，表示步态启动成功。

- decider：输出`serving on [ip]`表示机器人子机决策程序已经启动。

#### 2. 决策主机

运行 `./decider/scripts/run_decider_tester_on_windows.ps1` (windows) 或 `./decider/scripts/run_decider_tester_on_linux.sh` (Linux) 启动决策主机，会打开2个终端窗口，分别启动接收从机实时状态、决策程序。

可以按照提示向机器人发送指令。

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
