# chase ball
#

class ChaseBallStateMachine:
    def __init__(self, agent):
        self.agent = agent
        self.states = ["chase", "arrived"]
        self.transitions = [
            {
                "trigger": "run",
                "source": "chase",
                "dest": "arrived",
                "conditions": "close_to_ball",
                "prepare": "move_to_ball",
            }
        ]
        self.machine = Machine(
            model=self,
            states=self.states,
            initial="chase",
            transitions=self.transitions,
        )

    def close_to_ball(self):
        return self.agent.close_to_ball()

    def run(self):
        while self.state != "arrived" and self.agent.command == self.agent.info:
            self.machine.model.trigger("chase_ball")

    def move_to_ball(self, ang=0.25):
        if abs(self.cam_neck) > ang:
            self.speed_controller(0, 0, np.sign(self.cam_neck) * config.walk_theta_vel)
        elif abs(self.cam_neck) <= ang:
            self.speed_controller(
                config.walk_x_vel, 0, 2.5 * self.cam_neck * config.walk_theta_vel
            )
            time.sleep(0.1)



