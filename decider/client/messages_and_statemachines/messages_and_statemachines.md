# MOS-Brain

The structure of messages and statemachines of MOS-8.5.

## 消息

### decider.py

#### 核心控制模块

- **启动与终止**  
  - Decider started  
  - User interrupted  
- **状态管理**  
  - Penalized_time  
  - Stopping  
  - Running  
  - Debug_mode  
- **指令执行**  
  - =======================> cmd = {cmd}  
  - Setting the robot's speed {vel_x} {vel_y} {vel_theta}  
  - Executing the kicking action  
- **球位置计算**  
  - Calculating ball position in map from other robots  
  - Ball position relative to self: {ball_pos_relative}  
  - Ball angle in radians: {angle_rad}  
  - Ball angle from other robots: {angle_relative}  
- **错误处理**  
  - State machine {cmd} not found  
  - Error in decider run  

### local_decider_tester.py

#### 测试工具模块

- **心跳监控**  
  - 心跳监听已启动 @ {local_ip}:{HEARTBEAT_PORT}  
  - 心跳服务器异常  
  - 超过{HEARTBEAT_TIMEOUT}秒未收到心跳！  
  - 心跳恢复  
- **指令传输**  
  - 指令发送成功: {command}  
  - 指令发送失败: {command} ({e})  
- **程序状态**  
  - 执行过程中出现错误  
  - 程序已终止  

### robot_client.py

#### 机器人通信模块

- **网络连接**  
  - Default Host ip + self.HOST_IP  
  - Host ip updated to + self.HOST_IP  
  - Started listening for TCP command messages at address: {addr}  
  - Listening for UDP broadcast on port {port}  
  - Host IP found: {self.client.HOST_IP}  
- **线程管理**  
  - Started the server IP listening thread  
  - Started the sending loop thread  
  - Started the TCP client thread  
- **中断处理**  
  - User interrupted  
  - Server has been closed  
  - UDP listener stopped  
- **消息异常**  
  - Not contain 'command' or 'robots_data' (警告)  
  - Unexpected message (警告)  
- **错误处理**  
  - Error listening for server IP  
  - Error in send_loop  
  - Error sending robot data  
  - JSON decoding error  
  - Key error  
  - Error handling connection: {addr}  

### receiver.py

#### 数据接收模块

- **初始化**  
  - Initialized, break  
- **网络异常**  
  - Socket timeout (警告)  
  - Parse Error: Probably using an old protocol! (错误)  

### network.py

#### 网络配置模块

- **网络监听**  
  - listening tcp on + str(self._server_addr)  
  - listening for server on port + str(self._config["auto_find_server_ip_listen_port"] + token = self._config["auto_find_server_ip_token"]  

### tcp_host_test_receiver.py

#### TCP测试模块

- **连接管理**  
  - 正在监听 {ip}:{port}  
  - 已连接到 {client_address}  
  - 收到消息: {message}  
- **中断处理**  
  - 按下了 Ctrl+C，程序即将退出  
  - 接收到中断信号，程序即将退出  
- **错误处理**  
  - 获取本机 IP 地址时出错  
  - 接收消息时出错  
  - 监听时出错  
  - 服务器正在关闭  
  - 发生错误  

### read.py

#### 数据读取模块

- **数据接收**  
  - Received: {received_data}  
  - Connection closed  
  - Connection refused  
- **解析错误**  
  - JSON decode error  

### decider_tester.py

#### 决策测试模块

- **程序状态**  
  - Program exited  
- **服务器通信**  
  - Server response  
  - No response  
  - Sent successfully  
  - Failed to send  
  - Server IP: {server_ip}, Port: {server_port}  

### action.py

#### 动作执行模块

- **踢球动作**  
  - Send kick  
  - Kick goal init  
  - Kick done  

### chase_ball.py

#### 追球行为模块

- **状态检测**  
  - Close to ball? Distance: {distance_close}, Angle: {angle_close}, Result: {result}  
  - Agent.command: {command}, state: {self.state}  
  - Current state: {self.state}  
- **动作执行**  
  - Starting to rotate towards the ball  
  - Starting to move forward towards the ball  
  - Movement stopped  
  - Setting head position  
- **异常检测**  
  - No ball in sight (警告)  

### dribble.py

#### 带球行为模块

- **位置校准**  
  - Adjusting position to ball (多步骤调整逻辑)  
  - Starting horizontal position adjustment  
  - Starting yaw angle adjustment  
- **状态检测**  
  - Calculated aim_yaw: {self.aim_yaw:.2f}°  
  - Good position to ball: {'Yes' if result else 'No'}  
  - Ball distance: {ball_distance:.2f}  
  - Lost ball: {'Yes' if result else 'No'}  
- **动作控制**  
  - Moving forward  
  - Stopping  
  - Dribbling forward  
- **异常检测**  
  - No ball in sight (警告)  

### find_ball.py

#### 寻球行为模块

- **寻球逻辑**  
  - Ball in sight: {'Yes' if result else 'No'}  
  - Starting rotation (含其他机器人角度判断)  
  - Facing to ball (角度阈值控制)  
- **状态检测**  
  - Rotation timeout: {'Yes' if result else f'No ({elapsed:.1f}s)'}  
  - Ball lost: {'Yes' if result else 'No'}  
- **异常检测**  
  - No ball in sight (警告)  

### go_back_to_field.py

#### 回场行为模块

- **位置校准**  
  - Starting coarse yaw adjustment (大角度纠偏)  
  - Starting fine yaw adjustment (小角度微调)  
- **状态检测**  
  - Need coarse yaw adjustment? {'Yes' if result else 'No'}  
  - Arrived at target? {'Yes' if result else 'No'}  
  - Updated status: dist: {self.go_back_to_field_dist:.1f}, yaw_diff: {self.go_back_to_field_yaw_diff:.1f}°  
- **动作控制**  
  - Moving forward  
  - Stopping moving  

### kick.py

#### 踢球行为模块

- **踢球流程**  
  - Starting kick sequence (状态机控制)  
  - Positioning complete! Executing kick  
- **位置校准**  
  - Starting angle adjustment (旋转控制)  
  - Starting lateral adjustment (横向移动)  
  - Starting forward adjustment (前后移动)  
- **条件判断**  
  - Ball is too far. Stopping robot  
  - Large angle delta: {abs(ang_delta):.2f}° (Bad? {'Yes' if result else 'No'})  
  - Ball Distance: {ball_distance:.2f}m (OK? {'Yes' if result else 'No'})  

### imu_test.py

#### IMU测试模块

- **数据输出**  
  - [1] Yaw/Pitch 弧度/角度值输出  
  - [2] Yaw: {yaw_deg:.2f} degrees, Pitch: {pitch_deg:.2f} degrees  
- **错误处理**  
  - Error processing IMU data: %s  

### test_decider.py

#### 决策器测试模块

- **程序状态**  
  - Decider started  
  - User interrupted  

## 状态机

### chase_ball

![chase_ball](chase_ball.drawio.png)

### dribble

![dribble](dribble.drawio.png)

### find_ball

![find_ball](find_ball.drawio.png)

### go_back_to_field

![go_back_to_field](go_back_to_field.drawio.png)

### goalkeeper

暂无

### kick

![kick](kick.drawio.png)
