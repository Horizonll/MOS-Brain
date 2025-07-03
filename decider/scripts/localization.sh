#!/bin/bash

export PATH=/usr/local/cuda/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/snap/bin
export LD_LIBRARY_PATH=/usr/local/cuda/lib64:
export CUDA_ROOT=/usr/local/cuda


export PATH=/usr/local/cuda/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/snap/bin
export LD_LIBRARY_PATH=/usr/local/cuda/lib64:
export CUDA_ROOT=/usr/local/cuda


export FASTRTPS_DEFAULT_PROFILES_FILE=/opt/booster/BoosterRos2/fastdds_profile.xml
export PATH=/home/booster/.local/bin:$PATH

# >>> fishros initialize >>>
source /opt/ros/humble/setup.bash
# <<< fishros initialize <<<

export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/cuda/lib64
export PATH=$PATH:/usr/local/cuda/bin
export CUDA_HOME=$CUDA_HOME:/usr/local/cuda

source /home/booster/Workspace/THMOS/thmos-msgs/install/setup.sh

source ~/Workspace/booster_ros2_interface/install/setup.bash

/usr/bin/python /home/booster/Workspace/THMOS/vision/scripts/localization.py
