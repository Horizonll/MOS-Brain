#!/bin/bash

# Check if ZED camera is connected
if ! lsusb | grep -q '2b03:'; then
    echo "Error: ZED camera not detected. Please check the connection."
    exit 1
fi

# Start core session with roscore
screen -dmS core roscore
sleep 2  # Wait for roscore initialization

# Start walk session with roslaunch
screen -dmS walk bash -c "roslaunch thmos_bringup mos_run.launch"

# Start vision session with Python script
screen -dmS vision bash -c "cd ~/thmos_ws/src/thmos_code/vision/scripts/ && python3 vision_with_local.py"

# Start decider session with decision script
screen -dmS decider bash -c "python3 ~/MOS-Brain/decider/client/decider.py"

echo "All services started in screen sessions."
echo -e "\nAttach to sessions using:"
echo "  core   : screen -r core"
echo "  walk   : screen -r walk"
echo "  vision : screen -r vision"
echo "  decider: screen -r decider"