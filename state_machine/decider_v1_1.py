#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from decision_subscriber import Decision_Pos, Decision_Vision, Decision_Motion
from receiver import Receiver
from utils import config

import threading
from transitions import Machine
import time
import numpy as np
import math



class Agent(Decision_Pos, Decision_Motion, Decision_Vision):
    def __init__(self):
        # 初始化父类
        Decision_Pos.__init__(self)
        Decision_Motion.__init__(self)
        Decision_Vision.__init__(self)

        # 初始化ROS节点
        rospy.init_node("decider")

        # 初始化ROS速度发布器，
        self.speed_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        # 初始化ROS关节目标发布器
        self.joint_goal_publisher = rospy.Publisher(
            "motor_goals", JointState, queue_size=1
        )
        # 初始化ROS是否可以移动发布器
        self.can_move_publisher = rospy.Publisher("can_move", Bool, queue_size=1)

        # 初始化ROS速度订阅器
        self._state = None
        self.state_machine = StateMachine(self)
        self.ifBall = False
        self.ready_to_kick = False
        self.t_no_ball = 0

    def loop(self):
        return not rospy.is_shutdown()

    def stop(self, sleep_time):
        self.speed_controller(0, 0, 0)
        time.sleep(sleep_time)

    def kick(self):
        self.speed_controller(0, 0, 0)
        time.sleep(1)
        self.head_set(head=0.1, neck=0)
        self.doKick()
        time.sleep(2)

    def speed_controller(self, x, y, theta):
        move_cmd = Twist()
        move_cmd.linear.x = x
        move_cmd.linear.y = y
        move_cmd.angular.z = theta
        self.speed_pub.publish(move_cmd)

    def find_ball(self):
        print("Looking for the ball")
        self.find_ball_state_machine = FindBallStateMachine(self)
        self.find_ball_state_machine.run()

    def chase_ball(self):
        print("Chasing the ball")
        self.chase_ball_state_machine = ChaseBallStateMachine(self)
        self.chase_ball_state_machine.run()

    def adjust_position(self):
        print("Adjusting position")
        self.adjust_position_state_machine = AdjustPositionStateMachine(self)
        self.adjust_position_state_machine.run()

    def no_ball(self):
        if self.ifBall:
            self.t_no_ball = time.time()
        return time.time() - self.t_no_ball > 5

    def ball_in_sight(self):
        if self.ifBall:
            self.t_no_ball = time.time()
        return self.ifBall

    def close_to_ball(self):
        return 0.1 <= self.ball_distance <= 0.35

    def ready_to_kick(self):
        return self.ready_to_kick

    @property
    def state(self):
        return self._state

    @state.setter
    def state(self, value):
        self._state = value

    def run(self):
        self.state_machine.run()


class StateMachine:
    def __init__(self, model: Agent):
        self.states = [
            "find_ball",
            "chase_ball",
            "adjust_position",
            "kick",
        ]
        self.transitions = [
            {
                "trigger": "run",
                "source": "*",
                "dest": "find_ball",
                "conditions": "no_ball",
                "after": "find_ball",
            },
            {
                "trigger": "run",
                "source": "find_ball",
                "dest": "chase_ball",
                "conditions": "ball_in_sight",
                "after": "chase_ball",
            },
            {
                "trigger": "run",
                "source": "chase_ball",
                "dest": "adjust_position",
                "conditions": "close_to_ball",
                "after": "adjust_position",
            },
            {
                "trigger": "run",
                "source": "adjust_position",
                "dest": "kick",
                "conditions": "ready_to_kick",
                "after": "kick",
            },
            {
                "trigger": "run",
                "source": "kick",
                "dest": "find_ball",
            },
        ]
        self.machine = Machine(
            model=model,
            states=self.states,
            initial="find_ball",
            transitions=self.transitions,
        )

    def find_ball(self):
        self.machine.model.find_ball()

    def chase_ball(self):
        self.machine.model.chase_ball()

    def adjust_position(self):
        self.machine.model.adjust_position()

    def kick(self):
        self.machine.model.kick()

    def run(self):
        self.machine.model.trigger("run")


class FindBallStateMachine:
    def __init__(self, agent: Agent):
        self.agent = agent
        self.states = ["search", "found"]
        self.transitions = [
            {
                "trigger": "search_ball",
                "source": "search",
                "dest": "found",
                "conditions": "ball_in_sight",
                "before": "search_ball",
            }
        ]
        self.machine = Machine(
            model=self,
            states=self.states,
            initial="search",
            transitions=self.transitions,
        )
        self.lock = threading.Lock()

    def ball_in_sight(self):
        return self.agent.ball_in_sight()

    def run(self):
        while self.state != "found":
            print("Searching for the ball...")
            with self.lock:
                self.machine.model.trigger("search_ball")

    def search_ball(self):
        with self.lock:
            self.agent.stop(0.5)
            protect_pose = JointState()
            protect_pose.name = [
                "L_arm_1",
                "L_arm_2",
                "L_arm_3",
                "R_arm_1",
                "R_arm_2",
                "R_arm_3",
            ]
            protect_pose.position = [0, 1.2, -0.5, 0, -1.2, 0.5]
            self.agent.joint_goal_publisher.publish(protect_pose)

            for pos in range(6):
                t = time.time()
                while time.time() - t < 1:
                    if self.agent.ifBall:
                        return
                    self.agent.head_set(
                        head=config.find_head_pos[pos], neck=config.find_neck_pos[pos]
                    )

            self.agent.speed_controller(0, 0, config.walk_theta_vel)
            t = time.time()
            while self.agent.loop() and time.time() - t < 10:
                for pos in [0, 3]:
                    t1 = time.time()
                    while time.time() - t1 < 2:
                        if self.agent.ifBall:
                            self.agent.stop(0.5)
                            return
                        self.agent.head_set(
                            head=config.find_head_pos[pos],
                            neck=config.find_neck_pos[pos],
                        )


class ChaseBallStateMachine:
    def __init__(self, agent: Agent):
        self.agent = agent
        self.states = ["move", "arrived"]
        self.transitions = [
            {
                "trigger": "move_to_ball",
                "source": "move",
                "dest": "arrived",
                "conditions": "close_to_ball",
                "before": "move_to_ball",
            }
        ]
        self.machine = Machine(
            model=self,
            states=self.states,
            initial="move",
            transitions=self.transitions,
        )
        self.lock = threading.Lock()

    def close_to_ball(self):
        return self.agent.close_to_ball()

    def run(self):
        while self.state != "arrived":
            print("Moving to the ball...")
            with self.lock:
                self.machine.model.trigger("move_to_ball")

    def move_to_ball(self, ang=0.25):
        with self.lock:
            if abs(self.cam_neck) > ang:
                self.speed_controller(
                    0, 0, np.sign(self.cam_neck) * config.walk_theta_vel
                )

            elif abs(self.cam_neck) <= ang:
                self.motionState = "Forward"
                self.speed_controller(
                    config.walk_x_vel, 0, 2.5 * self.cam_neck * config.walk_theta_vel
                )
                time.sleep(0.1)


class AdjustPositionStateMachine:
    def __init__(self, agent: Agent):
        self.agent = agent
        self.states = ["angel", "lr", "fb", "finished"]
        self.transitions = [
            {
                "trigger": "adjust_position",
                "source": "angel",
                "dest": "lr",
                "conditions": "good_angel",
                "before": "adjust_angel",
            },
            {
                "trigger": "adjust_position",
                "source": "lr",
                "dest": "fb",
                "conditions": "good_lr",
                "before": "adjust_lr",
            },
            {
                "trigger": "adjust_position",
                "source": "fb",
                "dest": "finished",
                "conditions": "good_fb",
                "before": "adjust_fb",
            },
        ]
        self.machine = Machine(
            model=self,
            states=self.states,
            initial="angel",
            transitions=self.transitions,
        )
        self.lock = threading.Lock()

    def run(self):
        while self.state != "finished" and self.agent.ifBall:
            print("Adjusting position...")
            with self.lock:
                self.machine.model.trigger("adjust_position")

    def adjust_angel(self):
        with self.lock:
            self.agent.head_set(0.05, 0)
            target_angle_rad = math.atan(
                (self.agent.pos_x - 0) / (4500 - self.agent.pos_y)
            )
            ang_tar = target_angle_rad * 180 / math.pi
            ang_delta = ang_tar - self.agent.pos_yaw
            if ang_delta > 10:
                self.agent.speed_controller(0, -0.05, 0.3)
                ang_delta = ang_tar - self.agent.pos_yaw
                print(f"ang_delta={ang_delta}")
            elif ang_delta < -10:
                self.agent.speed_controller(0, 0.05, 0.3)
                ang_delta = ang_tar - self.agent.pos_yaw
                print(f"ang_delta={ang_delta}")

    def good_angel(self):
        target_angle_rad = math.atan((self.pos_x - 0) / (4500 - self.agent.pos_y))
        ang_tar = target_angle_rad * 180 / math.pi
        ang_delta = ang_tar - self.agent.pos_yaw
        return abs(ang_delta) < 10

    def adjust_lr(self):
        with self.lock:
            self.agent.head_set(head=0.9, neck=0)
            self.agent.stop(1)
            no_ball_count = 0
            t0 = time.time()
            while (
                self.agent.loop()
                and (self.agent.ball_x < 600)
                or (self.agent.ball_x == 0)
            ):
                if time.time() - t0 > 10 or no_ball_count > 5:
                    return
                if not self.agent.ifBall:
                    no_ball_count += 1
                    time.sleep(0.7)
                    continue
                self.agent.speed_controller(0, 0.6 * config.walk_y_vel, 0)

            while (
                self.agent.loop()
                and (self.agent.ball_x > 660)
                or (self.agent.ball_x == 0)
            ):
                if time.time() - t0 > 10 or no_ball_count > 5:
                    return
                if not self.agent.ifBall:
                    no_ball_count += 1
                    time.sleep(0.7)
                    continue
                self.agent.speed_controller(0, -0.6 * config.walk_y_vel, 0)
            self.agent.stop(0.5)

    def good_lr(self):
        return 600 <= self.agent.ball_x <= 660

    def adjust_fb(self):
        with self.lock:
            self.agent.stop(0.5)
            t0 = time.time()
            no_ball_count = 0
            while (
                self.agent.loop()
                and (self.agent.ball_y < 420)
                or (self.agent.ball_y == 0)
            ):
                if time.time() - t0 > 10 or no_ball_count > 5:
                    return
                if not self.agent.ifBall:
                    no_ball_count += 1
                    time.sleep(0.7)
                    continue
                self.agent.speed_controller(0.5 * config.walk_x_vel, 0, 0)
            self.agent.stop(0.5)

    def good_fb(self):
        self.agent.ready_to_kick = 420 <= self.agent.ball_y
        return 420 <= self.agent.ball_y


def main():
    agent = Agent()
    while True:
        agent.ifBall = int(input())
        agent.run()


if __name__ == "__main__":
    main()
