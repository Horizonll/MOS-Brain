#!/bin/bash

# Load ROS environment (modify paths as needed)
source /opt/ros/noetic/setup.bash
source /home/thmos/thmos_ws/devel/setup.bash

# Check for existing screen sessions
existing_screens=$(screen -ls | grep -E '(core|walk|vision|decider)\s')
if [ -n "$existing_screens" ]; then
    echo -e "\033[31mConflict detected in screen sessions:\033[0m"
    echo "$existing_screens" | awk '{print $1}'
    echo -e "\nPlease terminate existing sessions first using:"
    echo "screen -XS <session_name> quit"
    exit 1
fi

# Video device detection
shopt -s nullglob
video_devices=(/dev/video*)
if [ ${#video_devices[@]} -eq 0 ]; then
    echo "Error: No video devices detected"
    ls -l /dev/video* 2>/dev/null || echo "  (No devices found)"
    exit 1
fi
shopt -u nullglob

# Service starter function
start_service() {
    screen -dmS $1 bash -c "$2"
    echo "Service started: $1"
}

# Launch core services
start_service core "roscore"
sleep 5  # Allow roscore to initialize

# Launch other services
start_service walk "roslaunch thmos_bringup mos_run.launch"
start_service vision "cd /home/thmos/thmos_ws/src/thmos_code/vision/scripts/ && python3 vision_with_local.py"
start_service decider "python3 ~/MOS-Brain/decider/client/decider.py"

# Wait for topic initialization
echo -e "\nInitializing system..."
timeout_counter=0
max_wait=90  # Increased timeout to 90 seconds

while [ $timeout_counter -lt $max_wait ]; do
    # Improved detection using rostopic info
    if rostopic info /pos_in_map &>/dev/null; then
        # Verify actual message flow
        if rostopic hz /pos_in_map --window=1 --filter="mean > 0" &>/dev/null; then
            echo -e "\n\033[32mPosition data confirmed! System ready.\033[0m"
            exit 0
        fi
    fi
    
    # Visual progress indicator
    ((timeout_counter++))
    percentage=$((timeout_counter*100/max_wait))
    printf "\rInitializing: [%-50s] %d%%" $(printf "%${percentage}s" | tr ' ' '#') $percentage
    sleep 0.5  # Reduced polling interval
done

# Timeout handling
echo -e "\n\033[31mError: Position data not received within timeframe\033[0m"
echo "Current screen sessions:"
screen -ls
echo -e "\nTroubleshooting steps:"
echo "1. Check vision service logs: screen -r vision"
echo "2. Verify ROS topics: rostopic list"
echo "3. Monitor position data: rostopic echo /pos_in_map"

exit 1