# MOS-Brain

The decision-rev of MOS-8.5.

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
