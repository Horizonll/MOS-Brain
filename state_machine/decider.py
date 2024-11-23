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
        Decision_Pos.__init__(self)
        Decision_Motion.__init__(self)
        Decision_Vision.__init__(self)
        rospy.init_node("decider")
        self.speed_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.joint_goal_publisher = rospy.Publisher(
            "motor_goals", JointState, queue_size=1
        )
        self.can_move_publisher = rospy.Publisher("can_move", Bool, queue_size=1)

        self._state = None
        self.state_machine = StateMachine(self)
        self.ifBall = False
        self.ready_to_kick = False
        self.t_no_ball = 0
        self.is_going_back_to_field = False
        self.go_back_to_field_dist = None
        self.go_back_to_field_dir = None
        self.go_back_to_field_yaw_bias = None

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

    def update_go_back_to_field_status(self, aim_x, aim_y):
        self.go_back_to_field_dist = np.sqrt((self.pos_x - aim_x) ** 2 + (self.pos_y - aim_y) ** 2)
        self.go_back_to_field_dir = np.arctan2(-aim_x + self.pos_x, aim_y - self.pos_y)
        self.go_back_to_field_yaw_bias = np.degrees(
        np.arctan2(
            np.sin(self.go_back_to_field_dir - self.pos_yaw * np.pi / 180),
            np.cos(self.go_back_to_field_dir - self.pos_yaw * np.pi / 180),
        )
    ) 

    def go_back_to_field(self, aim_x, aim_y, min_dist=300):
        print("Going back to field")
        self.head_set(0.05, 0)
        go_back_to_field_machine = GoBackToFieldStateMachine(self, aim_x, aim_y, min_dist)
        go_back_to_field_machine.run()

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
            "go_back_to_field",
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

    def go_back_to_field(self):
        self.machine.model.go_back_to_field()

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


class GoBackToFieldStateMachine:
    def __init__(self, agent: Agent, aim_x, aim_y, min_dist=300):
        self.agent = agent
        self.aim_x = aim_x
        self.aim_y = aim_y
        self.min_dist = min_dist

        self.states = [
            "moving_to_target",
            "coarse_yaw_adjusting",
            "fine_yaw_adjusting",
            "arrived_at_target"
        ]
        self.transitions = [
            {
                "trigger": "update_status",
                "source": "moving_to_target",
                "dest": "coarse_yaw_adjusting",
                "conditions": "need_coarse_yaw_adjustment",
                "before": "update_status"
            },
            {
                "trigger": "update_status",
                "source": "coarse_yaw_adjusting",
                "dest": "fine_yaw_adjusting",
                "conditions": "need_fine_yaw_adjustment",
                "before": "update_status"
            },
            {
                "trigger": "update_status",
                "source": "moving_to_target",
                "dest": "arrived_at_target",
                "conditions": "arrived_at_target",
                "before": "update_status"
            },
            {
                "trigger": "update_status",
                "source": "coarse_yaw_adjusting",
                "dest": "arrived_at_target",
                "conditions": "arrived_at_target",
                "before": "update_status"
            },
            {
                "trigger": "update_status",
                "source": "fine_yaw_adjusting",
                "dest": "arrived_at_target",
                "conditions": "arrived_at_target",
                "before": "update_status"
            }
        ]
        self.machine = Machine(
            model=self,
            states=self.states,
            initial="moving_to_target",
            transitions=self.transitions
        )
        self.lock = threading.Lock()

    def need_coarse_yaw_adjustment(self):
        return (self.agent.go_back_to_field_dist > self.min_dist and
                (abs(self.agent.go_back_to_field_yaw_bias) > 15 or
                 (self.agent.go_back_to_field_dist > 3 * self.min_dist and
                  abs(self.agent.go_back_to_field_yaw_bias) > 10)))

    def need_fine_yaw_adjustment(self):
        return (self.agent.go_back_to_field_dist < 5 * self.min_dist and
                -20 < self.agent.go_back_to_field_yaw_bias < 10 and
                not -10 < self.agent.go_back_to_field_yaw_bias < 5 and
                self.agent.go_back_to_field_dist >= self.min_dist)

    def arrived_at_target(self):
        return self.agent.go_back_to_field_dist < self.min_dist

    def run(self):
        self.agent.is_going_back_to_field = True
        while self.state!= "arrived_at_target":
            print("Going back to field...")
            print(f"Current state: {self.state}")
            with self.lock:
                self.machine.model.trigger("update_status")

    def update_status(self):
        with self.lock:
            self.agent.update_go_back_to_field_status(self.aim_x, self.aim_y)

            if self.state == "moving_to_target":
                if self.need_coarse_yaw_adjustment():
                    self.coarse_yaw_adjust()
                else:
                    self.move_forward()

            elif self.state == "coarse_yaw_adjusting":
                if self.need_fine_yaw_adjustment():
                    self.fine_yaw_adjust()
                elif self.arrived_at_target():
                    self.arrived_at_target_operations()
                else:
                    self.coarse_yaw_adjust()

            elif self.state == "fine_yaw_adjusting":
                if self.arrived_at_target():
                    self.arrived_at_target_operations()
                else:
                    self.fine_yaw_adjust()

    def move_forward(self):
        with self.lock:
            self.agent.debug_info("[Go Back to Field] Going forward")
            self.agent.speed_controller(self.agent.cfg.walk_x_vel, 0, 0)
            time.sleep(1)

    def coarse_yaw_adjust(self):
        with self.lock:
            sgn_bias = 1 if self.agent.go_back_to_field_yaw_bias > 0 else -1
            while (
                not self.agent.loop() and not -20 < self.agent.go_back_to_field_yaw_bias / sgn_bias < 10
            ):
                print(
                    f"dist: {self.agent.go_back_to_field_dist}, yaw_bias: {self.agent.go_back_to_field_yaw_bias}, dir: {self.agent.go_back_to_field_dir}, pos_yaw: {self.agent.pos_yaw}"
                )
                self.agent.update_go_back_to_field_status(self.aim_x, self.aim_y)
                if self.agent.go_back_to_field_dist < self.min_dist:
                    break
                if -self.agent.go_back_to_field_yaw_bias > 0:
                    self.agent.speed_controller(0, 0, -self.agent.cfg.walk_theta_vel)
                    self.agent.debug_info("[Go Back to Field] Turning right")
                    time.sleep(0.3)
                else:
                    self.agent.speed_controller(0, 0, self.agent.cfg.walk_theta_vel)
                    self.agent.debug_info("[Go Back to Field] Turning left")
                    time.sleep(0.3)
                self.agent.go_back_to_field_dir = np.arctan2(-self.aim_x + self.agent.pos_x, self.aim_y - self.agent.pos_y)
                self.agent.go_back_to_field_yaw_bias = np.degrees(
                    np.arctan2(
                        np.sin(self.agent.go_back_to_field_dir - self.agent.pos_yaw * np.pi / 180),
                        np.cos(self.agent.go_back_to_field_dir - self.agent.pos_yaw * np.pi / 180),
                    )
                )

    def fine_yaw_adjust(self):
        with self.lock:
            sgn_bias = 1 if self.agent.go_back_to_field_yaw_bias > 0 else -1
            while (
                not self.agent.loop()
                and -20 < self.agent.go_back_to_field_yaw_bias / sgn_bias < 10
                and (not -10 < self.agent.go_back_to_field_yaw_bias / sgn_bias < 5)
                and self.agent.go_back_to_field_dist < 5 * self.min_dist
            ):
                self.agent.update_go_back_to_field_status(self.aim_x, self.aim_y)
                if self.agent.go_back_to_field_dist < self.min_dist:
                    break
                if -self.agent.go_back_to_field_yaw_bias > 0:
                    self.agent.speed_controller(0, 0, -0.1)
                    time.sleep(0.5)
                    self.agent.debug_info("[Go Back to Field] Turning right slowly")
                else:
                    self.agent.speed_controller(0, 0, 0.1)
                    time.sleep(0.5)
                self.agent.go_back_to_field_dir = np.arctan2(-self.aim_x + self.agent.pos_x, self.aim_y - self.agent.pos_y)
                self.agent.go_back_to_field_yaw_bias = np.degrees(
                    np.arctan2(
                        np.sin(self.agent.go_back_to_field_dir - self.agent.pos_yaw * np.pi / 180),
                        np.cos(self.agent.go_back_to_field_dir - self.agent.pos_yaw * np.pi / 180),
                    )
                )

    def arrived_at_target_operations(self):
        with self.lock:
            self.agent.debug_info(
                "[Go Back to Field] " + str(self.agent.role) + " has arrived. Turn ahead"
            )
            self.agent.speed_controller(0, 0, 0)
            time.sleep(1)
            if abs(self.agent.pos_yaw) > 160:
                self.agent.speed_controller(
                    0, 0, -np.sign(self.agent.pos_yaw) * self.agent.cfg.walk_theta_vel
                )
                time.sleep(2)
            while not self.agent.loop() and self.agent.pos_yaw > 5:
                self.agent.speed_controller(0, 0, -self.agent.cfg.walk_theta_vel)
                self.agent.debug_info("[Go Back to Field] Arrived. Turning right")
                time.sleep(0.5)
            while not self.agent.loop() and self.agent.pos_yaw < -5:
                self.agent.speed_controller(0, 0, self.agent.cfg.walk_theta_vel)
                self.agent.debug_info("[Go Back to Field] Arrived. Turning left")
                time.sleep(0.5)
            # ready
            self.agent.speed_controller(0, 0, 0)
            self.agent.debug_info(
                "[Go Back to Field] Finished going back to field. Ready to play."
            )
            time.sleep(1)
            self.agent.is_going_back_to_field = False


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


class DribbleStateMachine:
    """
    朝球门带球
    TODO: 处理丢球情况，检查方向
    """

    def __init__(self, agent: Agent):
        self.agent = agent
        self.states = ["forward", "adjust", "finished"]
        self.transitions = [
            {
                "trigger": "dribble",
                "source": "forward",
                "dest": "adjust",
                "conditions": "bad_position",
                "after": "adjust",
            },
            {
                "trigger": "dribble",
                "source": "adjust",
                "dest": "forward",
                "conditions": "good_position",
                "after": "forward",
            },
            {
                "trigger": "dribble",
                "source": "*",
                "dest": "finished",
                "conditions": "finished",
            },
        ]
        self.machine = Machine(
            model=self,
            states=self.states,
            initial="forward",
            transitions=self.transitions,
        )
        self.direction = True  # FIXME: True: right, False: left

    def run(self):
        while self.state != "finished":
            print("Dribbling...")
            self.machine.model.trigger("dribble")

    def forward(self):
        self.agent.speed_controller(0.5, 0, 0)
        time.sleep(0.5)

    def adjust(self):
        while not self.good_angel():  # 调整角度
            if self.direction:
                self.agent.speed_controller(0, 0, 0.1)
            else:
                self.agent.speed_controller(0, 0, -0.1)
            time.sleep(0.5)
        while self.agent.ball_y > 600:  # 后退
            self.agent.speed_controller(0, -0.1, 0)
            time.sleep(0.5)
        while abs(self.agent.ball_x - 640) > 10:  # 左右调整
            if self.agent.ball_x < 640:
                self.agent.speed_controller(0, 0.1, 0)
            else:
                self.agent.speed_controller(0, -0.1, 0)
            time.sleep(0.5)

    def good_position(self):
        return (
            self.good_angel()
            and self.agent.ball_y < 600
            and abs(self.agent.ball_x - 640) < 10
        )

    def bad_position(self):
        return not self.good_position()

    def good_angel(self):
        rad1 = math.atan(
            (self.agent.pos_x - 1300) / (4500 - self.agent.pos_y)
        )  # FIXME:球门左侧
        ang_tar1 = rad1 * 180 / math.pi
        rad2 = math.atan(
            (self.agent.pos_x + 1300) / (4500 - self.agent.pos_y)
        )  # FIXME:球门右侧
        ang_tar2 = rad2 * 180 / math.pi
        if ang_tar1 < self.agent.pos_yaw < ang_tar2:
            return True
        else:
            self.direction = ang_tar1 > self.agent.pos_yaw
            return False

    def finished(self):
        return not self.agent.ifBall


def main():
    agent = Agent()
    while True:
        agent.ifBall = int(input())
        agent.run()


if __name__ == "__main__":
    main()
