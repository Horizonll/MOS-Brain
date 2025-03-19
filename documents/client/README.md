# Client docuements

The robot playing on field is called 'client'

![decider svg](./decider.svg)


### Coding schema


##### decider.py

Class ```Agent``` in file **decider.py** is the main class. In method ```__init__```, 
an ```Agent``` instance loads configuration and initialize sub-instances of Action, 
Vision and Network. Then, the main() functions call method ```run()``` continously.
When ```run()``` is call, it receive command from server **asynchronously**. If there 
is a new command, the Agent will calls the ```stop()``` method of the running statemachine 
( if exists ), then calls ```start()``` from the up-coming statemachine once ( to load 
arguments from server, if exists ). Otherwise, the Agent calls the ```run()``` method 
of the current statemachine.

If there's a new statemachine, Agent will try to load the python modules with the same name.
Agent searches directory sub_statemachines and import the .py file with the same name, 
then call its ```__init()__``` method.



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


##### directory: sub_statemachines/

sub-statemachines are placed under this directory. These files are **dinamicly load** by 
Agent. The implementation of the sub-state machine is arbitrary, but you have to provide
these functions: 

|       method            |                     arguments                |              description                |
| ----------------------- | -------------------------------------------- | --------------------------------------- |
| ```__init__(agent)```   |              the instance of Agent           | Initialize your code here               |
| ```start(args, prev)``` | a dictionary of args; previous statemachine  | Called when started your statemachine   |
|    ```run()```          |                         None                 | Executing your statemachine             |
|    ```stop(next)```     |          the name of next statemachine       | Called when your statemachine is stoped |


You can call public method and read public variants of the instance ```agent``` given to you in 
```__init__```. Check decider.py for further infomation. 

Please do not use methods started with a underline, cause their behaviour may change in
the further updates. You should not access private variants directly either. 
