#!/bin/bash

source $HOME/Workspace/unitree_ros2/setup.sh
source $HOME/Workspace/THMOS/thmos-msgs/install/setup.sh
export PATH=/usr/local/cuda-11.4/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda-11.4/lib64:$LD_LIBRARY_PATH
export LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libgomp.so.1:$LD_PRELOAD

/usr/bin/python3 ~/Workspace/THMOS/interface-booster/interface.py