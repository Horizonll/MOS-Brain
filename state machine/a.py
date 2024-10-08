from transitions import Machine
import random

states = [
    "find_ball",
    "chase_ball",
    "adjust_position",
    "kick",
]

transitions = [
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
]


class Agent:
    def __init__(self):
        self.machine = Machine(
            model=self, states=states, initial="find_ball", transitions=transitions
        )

    def find_ball(self):
        print("Looking for the ball")

    def chase_ball(self):
        print("Chasing the ball")

    def adjust_position(self):
        print("Adjusting position")

    def kick(self):
        print("Kicking the ball")

    def no_ball(self):
        return random.random() < 0.2

    def ball_in_sight(self):
        return random.random() < 0.8

    def close_to_ball(self):
        return random.random() < 0.8

    def ready_to_kick(self):
        return random.random() < 0.8

    def run(self):
        self.trigger("run")


agent = Agent()
while True:
    agent.run()
    if agent.state == "kick":
        break
