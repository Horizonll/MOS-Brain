# kick
#

class KickStateMachine:
    def __init__(self, agent):
        self.agent = agent
        self.states = ["angel", "lr", "fb", "finished"]
        self.transitions = [
            {
                "trigger": "adjust_position",
                "source": "angel",
                "dest": "lr",
                "conditions": "good_angel",
                "prepare": "adjust_angel",
            },
            {
                "trigger": "adjust_position",
                "source": "lr",
                "dest": "fb",
                "conditions": "good_lr",
                "prepare": "adjust_lr",
            },
            {
                "trigger": "adjust_position",
                "source": "fb",
                "dest": "finished",
                "conditions": "good_fb",
                "prepare": "adjust_fb",
            },
        ]
        self.machine = Machine(
            model=self,
            states=self.states,
            initial="angel",
            transitions=self.transitions,
        )

    def run(self):
        while (
            self.state != "finished"
            and self.agent.ifBall
            and self.agent.command == self.agent.info
        ):
            print("Adjusting position...")
            self.machine.model.trigger("adjust_position")
        if self.state == "finished" and self.agent.info == self.agent.command:
            self.agent.speed_controller(0, 0, 0)
            self.agent.head_set(head=0.1, neck=0)
            time.sleep(1)
            self.agent.doKick()
            time.sleep(2)

    def adjust_angel(self):
        self.agent.head_set(0.05, 0)
        target_angle_rad = math.atan((self.agent.pos_x - 0) / (4500 - self.agent.pos_y))
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
        self.agent.head_set(head=0.9, neck=0)
        self.agent.stop(1)
        no_ball_count = 0
        t0 = time.time()
        while (
            self.agent.loop() and (self.agent.ball_x < 600) or (self.agent.ball_x == 0)
        ):
            if time.time() - t0 > 10 or no_ball_count > 5:
                return
            if not self.agent.ifBall:
                no_ball_count += 1
                time.sleep(0.7)
                continue
            self.agent.speed_controller(0, 0.6 * config.walk_y_vel, 0)
        while (
            self.agent.loop() and (self.agent.ball_x > 660) or (self.agent.ball_x == 0)
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
        self.agent.stop(0.5)
        t0 = time.time()
        no_ball_count = 0
        while (
            self.agent.loop() and (self.agent.ball_y < 420) or (self.agent.ball_y == 0)
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



