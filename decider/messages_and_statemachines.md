# MOS-Brain

The structure of messages and statemachines of MOS-8.5.

### 消息

#### print

- decider.py
  - Decider started
  - User interrupted
- local_decider_tester.py
  - 执行过程中出现错误
  - 程序已终止
  - 心跳监听已启动 @ {local_ip}:{HEARTBEAT_PORT}
  - 心跳服务器异常
  - 超过{HEARTBEAT_TIMEOUT}秒未收到心跳！
  - 心跳恢复
  - 指令发送成功: {command}
  - 指令发送失败: {command} ({e})  
- receiver.py
  - Initialized, break  
- network.py
  - listening tcp on + str(self._server_addr)
  - listening for server on port + str(self._config["auto_find_server_ip_listen_port"] + token = self._config["auto_find_server_ip_token"] 
- decider_tester.py
  - Program exited
  - Server response
  - No response
  - Sent successfully
  - Failed to send
  - Server IP: {server_ip}, Port: {server_port}
- tcp_host_test_receiver.py
  - 获取本机 IP 地址时出错
  - 按下了 Ctrl+C，程序即将退出
  - 正在监听 {ip}:{port}
  - 已连接到 {client_address}
  - 收到消息: {message}
  - 接收到中断信号，程序即将退出
  - 接收消息时出错
  - 监听时出错
  - 服务器正在关闭
  - 发生错误
- read.py
  - Received: {received_data}
  - JSON decode error
  - Connection closed
  - Connection refused

#### loginfo

- decider.py
  - Initializing
  - Registering interfaces
  - Penalized_time
  - Stopping
  - Running
  - =======================> cmd = {cmd}
  - Debug_mode
  - Setting the robot's speed {vel_x} {vel_y} {vel_theta}
  - Executing the kicking action
  - Calculating ball position in map from other robots
  - Ball position relative to self: {ball_pos_relative}
  - Ball angle in radians: {angle_rad}
  - Ball angle from other robots: {angle_relative}
- robot_client.py
  - Default Host ip + self.HOST_IP
  - Started the server IP listening thread
  - Started the sending loop thread
  - Started the TCP client thread
  - Host ip updated to + self.HOST_IP
  - Started listening for TCP command messages at address: {addr}
  - User interrupted
  - Server has been closed
  - Listening for UDP broadcast on port {port}
  - UDP listener stopped
  - Host IP found: {self.client.HOST_IP}
- test_decider.py
  - Decider started
  - User interrupted
- action.py
  - Send kick
  - Kick goal init
  - Kick done  
- chase_ball.py
  - Initialized
  - Close to ball? Distance: {distance_close}, Angle: {angle_close}, Result: {result}
  - Agent.command: {command}, state: {self.state}
  - Current state: {self.state}
  - Starting to rotate towards the ball
  - Noball,cant rotate
  - Starting to move forward towards the ball  
  - Movement stopped
  - Setting head position
- dribble.py
  - Initialized
  - Calculated aim_yaw: {self.aim_yaw:.2f}°
  - Current state: {self.state}
  - Moving forward
  - Stopping
  - Adjusting position to ball
    - {target_angle_rad} > {self.angle_to_ball_adjust_threshold_rad} {ball_distance} Rotating
    - {ball_distance} > {self.max_ball_distance_m}. Moving backward
    - {ball_distance} < 0.0. Moving forward
    - Position adjustment step completed
  - Good position to ball: {'Yes' if result else 'No'}
  - Starting horizontal position adjustment
    - Moving left (Ball x: {ball_x})
    - Moving right (Ball x: {ball_x})
  - Adjusting angle... Current angle: {target_angle_rad}
    - Good angle to ball: {'Yes' if result else 'No'}
    - Angle delta: {abs(ang_delta):.2f}° (OK? {'Yes' if result else 'No'})
    - Bad yaw angle: {'Yes' if result else 'No'} angle delta: {abs(self.aim_yaw - self.agent.get_self_yaw()):.2f}°
  - Starting yaw angle adjustment
    - Target yaw: {target_angle_deg:.2f}°, Current yaw: {current_yaw:.2f}°, Delta: {yaw_delta:.2f}°
    - Rotating CCW (Δ={yaw_delta:.2f}°)
    - Rotating CW (Δ={yaw_delta:.2f}°)
    - Yaw angle is within acceptable range
  - Good position: {'Yes' if result else 'No'}
  - Ball distance: {ball_distance:.2f}
  - Lost ball: {'Yes' if result else 'No'}
    - Lost ball yaw: {'Yes' if result else 'No'}
    - Lost ball x: {'Yes' if result else 'No'}
  - Calculating angles
  - Dribbling forward
- find_ball.py
  - Initialized
  - Ball in sight: {'Yes' if result else 'No'}
  - Rotation timeout: {'Yes' if result else f'No ({elapsed:.1f}s)'}
  - Ball lost: {'Yes' if result else 'No'}
  - Setting protect pose
  - Starting rotation
    - Other robots see the ball at angle: {ball_angle_from_other_robots}
    - No other robots see the ball, rotating randomly
    - Stopping rotation
  - Facing to ball
    - No ball in sight, cannot face to ball
    - Target angle ({np.rad2deg(target_angle_rad):.1f}°) > threshold ({self.find_ball_angle_threshold_degrees}°). Rotating...
    - Target angle within threshold. Stopping rotation
    - Facing to ball completed
- go_back_to_field.py
  - Initialized
  - Need coarse yaw adjustment? {'Yes' if result else 'No'}
    - Need fine yaw adjustment? {'Yes' if result else 'No'}
    - Don't need coarse yaw adjustment? {'Yes' if result else 'No'}
  - {self.go_back_to_field_dist} {self.min_dist}
  - Arrived at target? {'Yes' if result else 'No'}
  - Starting to go back to field
    - Current state
    - Triggering 'update_status' transition
    - aim_x: {self.aim_x}, aim_y: {self.aim_y}, aim_yaw: {self.aim_yaw}
    - pos_x: {self.pos_x}, pos_y: {self.pos_y}
    - Updated status: dist: {self.go_back_to_field_dist:.1f}, yaw_diff: {self.go_back_to_field_yaw_diff:.1f}°
  - Moving forward
  - Starting coarse yaw adjustment
    - Large yaw error ({self.go_back_to_field_yaw_diff:.1f}°), rotating {'' if sgn>0 else 'right'}
    - Good yaw? {'Yes' if result else 'No'} (diff: {aim_yaw_diff:.1f}°)
    - Starting fine yaw adjustment
    - Medium yaw error ({self.go_back_to_field_yaw_diff:.1f}°), rotating {'' if sgn>0 else 'right'} slowly
  - Arrived at target. Performing yaw adjust
    - Correcting large yaw wrap-around
    - Final yaw adjustment ({aim_yaw_diff:.1f}°)
    - Finished going back to field. Ready to play
  - Trusting recent arrival, not rechecking
    - Not arrived? {'Yes' if result else 'No'} dist: {self.go_back_to_field_dist:.1f}
  - Stopping moving
- goalkeeper.py
  - 同dribble.py
- kick.py
  - Initialized
  - Starting kick sequence
    - if_ball: {self.agent.get_if_ball()} state: {self.state}
    - Ball is too far. Stopping robot
    - Current state: {self.state}
    - Triggering 'adjust_position' transition
    - Positioning complete! Executing kick
  - Stopping robot
  - Starting angle adjustment
    - Target angle: {ang_tar:.2f}°, Current yaw: {self.agent.get_self_yaw():.2f}°, Delta: {ang_delta:.2f}°
    - Rotating CCW (Δ={ang_delta:.2f}°)
    - [ANGLE ADJUST] Rotating CW (Δ={ang_delta:.2f}°)
    - Large angle delta: {abs(ang_delta):.2f}° (Bad? {'Yes' if result else 'No'})
  - Starting lateral adjustment
    - Moving left (Ball Angle: {math.degrees(ball_angle):.2f}°)
    - Moving right (Ball Angle: {math.degrees(ball_angle):.2f}°)
  - Starting forward adjustment
    - Moving forward Distance: {ball_distance:.2f}
    - Moving backward Distance: {ball_distance:.2f}
    - Ball Distance: {ball_distance:.2f}m (OK? {'Yes' if result else 'No'})
- imu_test.py
  - [1] Yaw: %.2f radians, %.2f degrees, Pitch: %.2f radians, %.2f degrees
  - [2] Yaw: {yaw_deg:.2f} degrees, Pitch: {pitch_deg:.2f} degrees
  - Node started. Waiting for IMU data

#### logwarn

- receiver.py
  - Socket timeout
- robot_client.py
  -Not contain 'command' or 'robots_data'  
  -Unexpected message  
- chase_ball.py
  - No ball in sight  
- dribble.py
  - No ball in sight
- find_ball.py
  - No ball in sight
- goalkeeper.py
  - No ball in sight

#### logerr

- decider.py
  - State machine {cmd} not found
  - Error in decider run
- receiver.py
  -Parse Error: Probably using an old protocol!  
- robot_client.py
  - Error listening for server IP
  - Error in send_loop
  - Error sending robot data
  - Error starting the TCP listening loop
  - JSON decoding error
  - Key error
  - Error handling connection: {addr}  
- imu_test.py
  - Error processing IMU data: %s

### 状态机

#### chase_ball

#### dribble

#### find_ball

#### go_back_to_field

#### goalkeeper

#### kick
