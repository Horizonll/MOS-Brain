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

source ~/Workspace/THMOS/thmos-msgs/install/setup.sh

source ~/Workspace/booster_ros2_interface/install/setup.bash

# 定义要运行的Python文件路径
DECIDER_DEBUG="$HOME/Workspace/THMOS/MOS-Brain/decider/client/decider.py"
DECIDER_TESTER="$HOME/Workspace/THMOS/MOS-Brain/decider/test/decider_tester.py"

# 检查Python文件是否存在
if [ ! -f "$DECIDER_DEBUG" ]; then
    echo "错误: $DECIDER_DEBUG 不存在!"
    exit 1
fi

if [ ! -f "$DECIDER_TESTER" ]; then
    echo "错误: $DECIDER_TESTER 不存在!"
    exit 1
fi

# 检查tmux是否已安装
if ! command -v tmux &> /dev/null; then
    echo "错误: tmux未安装。请先安装tmux: sudo apt-get install tmux (Ubuntu/Debian)"
    exit 1
fi

# 创建一个新的tmux会话并水平分屏
tmux new-session -d -s python_session

# 在左侧面板运行第一个Python文件
tmux send-keys -t python_session:0.0 "python3 $DECIDER_DEBUG" Enter

# 水平分屏
tmux split-window -h -t python_session:0

# 在右侧面板运行第二个Python文件
tmux send-keys -t python_session:0.1 "python3 $DECIDER_TESTER" Enter

# 切换到这个会话
tmux attach-session -t python_session

echo "已在分屏终端中启动两个Python文件"
