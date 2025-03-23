# find ball
#

from transitions import Machine
from sensor_msgs.msg import JointState
import time
from configuration import configuration


class find_ball:
    def __init__(self, agent):
        self.agent = agent
        self.states = ["search", "found"]
        self.transitions = [
            {
                "trigger": "search_ball",
                "source": "search",
                "dest": "found",
                "conditions": "ball_in_sight",
                "prepare": "search_ball",
            }
        ]
        self.machine = Machine(
            model=self,
            states=self.states,
            initial="search",
            transitions=self.transitions,
        )

    def ball_in_sight(self):
        return self.agent.ball_in_sight()

    def run(self):
        if self.state != "found":
            print("Searching for the ball...")
            self.machine.model.trigger("search_ball")

    def search_ball(self):
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
                    head=configuration.find_head_pos[pos],
                    neck=configuration.find_neck_pos[pos],
                )

        self.agent.speed_controller(0, 0, configuration.walk_theta_vel)
        t = time.time()
        while self.agent.loop() and time.time() - t < 10:
            for pos in [0, 3]:
                t1 = time.time()
                while time.time() - t1 < 2:
                    if self.agent.ifBall:
                        self.agent.stop(0.5)
                        return
                    self.agent.head_set(
                        head=configuration.find_head_pos[pos],
                        neck=configuration.find_neck_pos[pos],
                    )

    def start(self, args, last_statemachine):
        pass
    
    def stop(self, next_statemachine);
        pass


