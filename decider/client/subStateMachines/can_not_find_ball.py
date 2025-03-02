# can not find ball
#
import threading
from transitions import Machine

class CanNotFindBallStateMachine:
    def __init__(self, agent):
        self.agent = agent
        self.states = ["cannot_find_ball", "going_back_to_field", "ball_in_sight"]
        self.transitions = [
            {
                "trigger": "going_back_to_field",
                "source": "cannot_find_ball",
                "dest": "ball_in_sight",
            },
        ]
        self.machine = Machine(
            model=self,
            states=self.states,
            initial="cannot_find_ball",
            transitions=self.transitions,
        )
        self.lock = threading.Lock()

    def cannot_find_ball(self):
        return not self.agent.ball_in_sight()

    def go_back_to_field(self):
        self.agent.go_back_to_field(self.agent.field_aim_x, self.agent.field_aim_y)

    def run(self):
        while self.state != "ball_in_sight":
            print("Cannot find the ball. Going back to the field...")
            self.machine.model.trigger("going_back_to_field")


