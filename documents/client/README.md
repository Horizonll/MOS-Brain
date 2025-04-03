# Client docuements

The robot playing on field is called 'client'

![decider svg](./decider.svg)


### Coding schema


##### decider.py

Class ```Agent``` in file **decider.py** is the main class. In method ```__init__```, 
an ```Agent``` instance loads configuration and initialize sub-instances of Action, 
Vision and Network. Then, the main() functions call method ```run()``` continously.


##### configuration.py

configuration.py just does one thing: loading configuration from ```config.json``` and 
```config_override.json```. C++ flavored annotations are supported in json parser.

You should place shared configurations into ```config.json``` which is tracked by git,
and private configurations into ```config_override.json``` which is not tracked by git.
Notice that config_override.json will override configurations in config.json.



##### directory: interfaces/

Sub-dir interfaces/ contains interfaces with other component. ```Action.py``` provide 
interfaces with walk-engine ( cmd_vel and do_kick ), and ```Vision.py``` receive data
from vision node. Some network related functions are in ```Network.py```.

>[!INFO]  
> Servo neck and head control are not exposed and are controlled by ```Vision.py``` but
> not ```Action.py```. The reason is that the camera have to look at both the ball and 
> the field to avoid mistracking, so external controlling of the neck and head will 
> disturb the algorithm in ```Vision.py```.
> But although not recommanded, there is a api to disable auto tracking. See apis below


##### directory: sub_statemachines/

sub-statemachines are placed under this directory. These files are **dinamicly load** by 
Agent. The implementation of the sub-state machine is arbitrary, but you have to provide
these functions: 

|       method            |                     arguments                |              description                |
| ----------------------- | -------------------------------------------- | --------------------------------------- |
| ```__init__(agent)```   |              the instance of Agent           | Initialize your code here               |
|    ```run()```          |                         None                 | Executing your statemachine             |


You can call public method and read public variants of the instance ```agent``` given to you in 
```__init__```. Check decider.py for further infomation. 

Please do not use methods started with a underline, cause their behaviour may change in
the further updates. You should not access private variants directly either. 

### APIs and Interfaces

Every sub-statemachine should be placed under directory sub_statemachine.
Add your class name to __init__.py.

Your class have to provide a run() method, which MUST NOT CONTAIN ANY LOOPS.
If there is, change it to a statemachine. 

##### APIs from Agent

The __init__ entry of your class should receive a Agent() object, which provides you
all the apis you need.   
  
**cmd_vel(vel_x : float, vel_y : float, vel_theta : float)**
Publish a velocity() command. Forward, left, anti-clockwise are the positives.


**kick()**  
Publish a kick() command. Notice that the action may not be complete in a slice of time,
so please handle it.


**look_at(head: float, neck: float)**  
Disable automatically tracking and force to look_at some specific direction
Use (NaN, NaN) to enable tracking  
@param head: 上下角度，[0,1.5]，1.5下，0上
@param neck: 左右角度，[-1.1,1.1]，-1.1右，1.1左


**public methods to get varants:**  
|        METHODS       |      TYPE        |        DESCRIPTION      |
|:--------------------:|:----------------:|:-----------------------:|
|get_self_pos()        |  numpy.array     |   self position in map  |
|get_self_yaw()        |  angle           |   self angle in deg, [-180, 180)  |
|get_ball_pos()        |  numpy.array     |   ball postiion from robot  |
|get_ball_pos_in_vis() |  numpy.array     |   ball position in **vision**  |
|get_ball_pos_in_map() |  numpy.array     |   ball position in global   |
|get_if_ball()         |  bool            |   whether can find ball  |
|get_ball_distance()   |  float           |   the distance to the ball  |
|get_neck()            |  angle           |   the neck angle, left and right  |
|get_head()            |  angle           |   the head angle, up and down  |
