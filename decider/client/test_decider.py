 


import os
import json
import math
from pathlib import Path
import time
import signal
import asyncio
import numpy as np
import threading
import logging
import sys

import rospy

# 将项目根目录（client和tester的上级目录）加入模块搜索路径
project_root = Path(__file__).parent.parent  # 获取当前文件父目录的父目录
sys.path.append(str(project_root))

from client.decider import Agent


rospy.loginfo("Decider started")
agent = Agent()
agent.if_local_test = True

try:
    def sigint_handler(sig, frame):
        rospy.loginfo("User interrupted the program, exiting gracefully...")
        exit(0)

    signal.signal(signal.SIGINT, sigint_handler)
    while True:
        agent.run()
        time.sleep(0.1)
except KeyboardInterrupt:
    rospy.loginfo("Program interrupted by the user")
    exit()
finally:
    pass

