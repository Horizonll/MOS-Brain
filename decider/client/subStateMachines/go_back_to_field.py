# Go back to field
# 

class GoBackToFieldStateMachine:
    def __init__(self, agent, aim_x, aim_y, min_dist=300):
        self.agent = agent
        self.aim_x = aim_x
        self.aim_y = aim_y
        self.min_dist = min_dist
        self.states = [
            "moving_to_target",
            "coarse_yaw_adjusting",
            "fine_yaw_adjusting",
            "arrived_at_target",
        ]
        self.transitions = [
            {
                "trigger": "update_status",
                "source": "moving_to_target",
                "dest": "coarse_yaw_adjusting",
                "conditions": "need_coarse_yaw_adjustment",
                "before": "update_status",
            },
            {
                "trigger": "update_status",
                "source": "coarse_yaw_adjusting",
                "dest": "fine_yaw_adjusting",
                "conditions": "need_fine_yaw_adjustment",
                "before": "update_status",
            },
            {
                "trigger": "update_status",
                "source": "moving_to_target",
                "dest": "arrived_at_target",
                "conditions": "arrived_at_target",
                "before": "update_status",
            },
            {
                "trigger": "update_status",
                "source": "coarse_yaw_adjusting",
                "dest": "arrived_at_target",
                "conditions": "arrived_at_target",
                "before": "update_status",
            },
            {
                "trigger": "update_status",
                "source": "fine_yaw_adjusting",
                "dest": "arrived_at_target",
                "conditions": "arrived_at_target",
                "before": "update_status",
            },
        ]
        self.machine = Machine(
            model=self,
            states=self.states,
            initial="moving_to_target",
            transitions=self.transitions,
        )
        self.lock = threading.Lock()

    def need_coarse_yaw_adjustment(self):
        return self.agent.go_back_to_field_dist > self.min_dist and (
            abs(self.agent.go_back_to_field_yaw_bias) > 15
            or (
                self.agent.go_back_to_field_dist > 3 * self.min_dist
                and abs(self.agent.go_back_to_field_yaw_bias) > 10
            )
        )

    def need_fine_yaw_adjustment(self):
        return (
            self.agent.go_back_to_field_dist < 5 * self.min_dist
            and -20 < self.agent.go_back_to_field_yaw_bias < 10
            and not -10 < self.agent.go_back_to_field_yaw_bias < 5
            and self.agent.go_back_to_field_dist >= self.min_dist
        )

    def arrived_at_target(self):
        return self.agent.go_back_to_field_dist < self.min_dist

    def run(self):
        self.agent.is_going_back_to_field = True
        while self.state != "arrived_at_target":
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
            self.agent.speed_controller(self.agent.walk_x_vel, 0, 0)
            time.sleep(1)

    def coarse_yaw_adjust(self):
        with self.lock:
            sgn_bias = 1 if self.agent.go_back_to_field_yaw_bias > 0 else -1
            while (
                not self.agent.loop()
                and not -20 < self.agent.go_back_to_field_yaw_bias / sgn_bias < 10
            ):
                print(
                    f"dist: {self.agent.go_back_to_field_dist}, yaw_bias: {self.agent.go_back_to_field_yaw_bias}, dir: {self.agent.go_back_to_field_dir}, pos_yaw: {self.agent.pos_yaw}"
                )
                self.agent.update_go_back_to_field_status(self.aim_x, self.aim_y)
                if self.agent.go_back_to_field_dist < self.min_dist:
                    break
                if -self.agent.go_back_to_field_yaw_bias > 0:
                    self.agent.speed_controller(0, 0, -self.agent.walk_theta_vel)
                    self.agent.debug_info("[Go Back to Field] Turning right")
                    time.sleep(0.3)
                else:
                    self.agent.speed_controller(0, 0, self.agent.walk_theta_vel)
                    self.agent.debug_info("[Go Back to Field] Turning left")
                    time.sleep(0.3)
                self.agent.go_back_to_field_dir = np.arctan2(
                    -self.aim_x + self.agent.pos_x, self.aim_y - self.agent.pos_y
                )
                self.agent.go_back_to_field_yaw_bias = np.degrees(
                    np.arctan2(
                        np.sin(
                            self.agent.go_back_to_field_dir
                            - self.agent.pos_yaw * np.pi / 180
                        ),
                        np.cos(
                            self.agent.go_back_to_field_dir
                            - self.agent.pos_yaw * np.pi / 180
                        ),
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
                self.agent.go_back_to_field_dir = np.arctan2(
                    -self.aim_x + self.agent.pos_x, self.aim_y - self.agent.pos_y
                )
                self.agent.go_back_to_field_yaw_bias = np.degrees(
                    np.arctan2(
                        np.sin(
                            self.agent.go_back_to_field_dir
                            - self.agent.pos_yaw * np.pi / 180
                        ),
                        np.cos(
                            self.agent.go_back_to_field_dir
                            - self.agent.pos_yaw * np.pi / 180
                        ),
                    )
                )

    def arrived_at_target_operations(self):
        with self.lock:
            self.agent.debug_info(
                "[Go Back to Field] "
                + str(self.agent.role)
                + " has arrived. Turn ahead"
            )
            self.agent.speed_controller(0, 0, 0)
            time.sleep(1)
            if abs(self.agent.pos_yaw) > 160:
                self.agent.speed_controller(
                    0, 0, -np.sign(self.agent.pos_yaw) * self.agent.walk_theta_vel
                )
                time.sleep(2)
            while not self.agent.loop() and self.agent.pos_yaw > 5:
                self.agent.speed_controller(0, 0, -self.agent.walk_theta_vel)
                self.agent.debug_info("[Go Back to Field] Arrived. Turning right")
                time.sleep(0.5)
            while not self.agent.loop() and self.agent.pos_yaw < -5:
                self.agent.speed_controller(0, 0, self.agent.walk_theta_vel)
                self.agent.debug_info("[Go Back to Field] Arrived. Turning left")
                time.sleep(0.5)
            # ready
            self.agent.speed_controller(0, 0, 0)
            self.agent.debug_info(
                "[Go Back to Field] Finished going back to field. Ready to play."
            )
            time.sleep(1)
            self.agent.is_going_back_to_field = False

