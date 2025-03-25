#!/bin/bash

# 加载ROS环境（根据实际情况修改路径）
source /opt/ros/noetic/setup.bash
source /home/thmos/thmos_ws/devel/setup.bash

# 检测已有screen会话
existing_screens=$(screen -ls | grep -E '(core|walk|vision|decider)\s')
if [ -n "$existing_screens" ]; then
    echo -e "\033[31m发现冲突的screen会话：\033[0m"
    echo "$existing_screens" | awk '{print $1}'
    echo -e "\n请先执行以下命令关闭会话："
    echo "screen -XS <会话名> quit"
    exit 1
fi

# 视频设备检测
shopt -s nullglob
video_devices=(/dev/video*)
if [ ${#video_devices[@]} -eq 0 ]; then
    echo "错误：未检测到视频设备"
    ls -l /dev/video* 2>/dev/null || echo "  (无设备)"
    exit 1
fi
shopt -u nullglob

# 服务启动函数
start_service() {
    screen -dmS $1 bash -c "$2"
    echo "服务已启动：$1"
}

# 启动核心服务
start_service core "roscore"
sleep 5

# 启动其他服务
start_service walk "roslaunch thmos_bringup mos_run.launch"
start_service vision "cd /home/thmos/thmos_ws/src/thmos_code/vision/scripts/ && python3 vision_with_local.py"
start_service decider "python3 ~/MOS-Brain/decider/client/decider.py"

# 等待话题出现
echo -e "\n等待系统初始化..."
timeout_counter=0
max_wait=60  # 最大等待时间(秒)

while [ $timeout_counter -lt $max_wait ]; do
    rostopic echo /pos_in_map -n 1 --timeout 1 >/dev/null 2>&1
    if [ $? -eq 0 ]; then
        echo -e "\033[32m收到位置信息，系统准备就绪！\033[0m"
        exit 0
    fi
    
    # 显示进度条
    ((timeout_counter++))
    percentage=$((timeout_counter*100/max_wait))
    printf "\r等待中: [%-50s] %d%%" $(printf "%${percentage}s" | tr ' ' '#') $percentage
    sleep 1
done

# 超时处理
echo -e "\n\033[31m错误：未在指定时间内收到位置信息\033[0m"
echo "请检查以下服务状态："
screen -ls
exit 1