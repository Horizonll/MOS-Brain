from transitions import Machine
import logging


class AttackStateMachine:
    def __init__(self, agent):
        self.logger = logging.getLogger(__name__)
        self.agent = agent
        self._config = self.agent.get_config()
        self.read_params()

        self.states = ["have_no_ball", "close_to_ball", "have_ball"]
        self.transitions = [
            {
                "trigger": "run",
                "source": ["close_to_ball", "have_ball"],
                "dest": "have_ball",
                "conditions": "ball_in_control",
                "after": ["shoot", "log_transition"],
            },
            {
                "trigger": "run",
                "source": "have_ball",
                "dest": "close_to_ball",
                "conditions": "dont_have_ball",
                "after": ["go_for_possession_avoid_collsion", "log_transition"],
            },
            {
                "trigger": "run",
                "source": ["have_no_ball", "close_to_ball"],
                "dest": "close_to_ball",
                "conditions": "close_to_ball",
                "after": ["go_for_possession_avoid_collsion", "log_transition"],
            },
            {
                "trigger": "run",
                "source": ["close_to_ball", "have_ball", "have_no_ball"],
                "dest": "have_no_ball",
                "conditions": "ball_out_of_control",
                "after": ["go_for_possession", "log_transition"],
            },
        ]

        self.machine = Machine(
            model=self,
            states=self.states,
            initial="have_no_ball",
            transitions=self.transitions,
            after_state_change=self.on_state_change,
        )
        self.logger.info("[AttackStateMachine] 状态机初始化完成，初始状态: have_no_ball")

    def read_params(self):
        shoot_config = self._config.get("attacking", {})
        self.ball_in_control_threshold_m = shoot_config.get("ball_in_control_threshold_m", 0.4)
        self.ball_out_of_control_threshold_m = shoot_config.get("ball_out_of_control_threshold_m", 0.8)
        self.close_to_ball_threshold_m = shoot_config.get("close_to_ball_threshold_m", 0.8)
        self.no_control_chase_distance = shoot_config.get("no_control_chase_distance", 1.0)

    def log_transition(self):
        pass

    def on_state_change(self, event=None):
        if event:
            self.logger.info(f"[AttackStateMachine] 状态变更: {event.transition.source} -> {event.transition.dest}")

    def ball_in_control(self):
        players_distance = self.agent.get_players_distance_to_ball()
        return any(dist < self.ball_in_control_threshold_m for dist in players_distance.values())

    def ball_out_of_control(self):
        players_distance = self.agent.get_players_distance_to_ball()
        return all(dist > self.ball_out_of_control_threshold_m for dist in players_distance.values())

    def dont_have_ball(self):
        return not self.ball_in_control() and not self.ball_out_of_control()

    def close_to_ball(self):
        players_distance = self.agent.get_players_distance_to_ball()
        return any(dist < self.close_to_ball_threshold_m for dist in players_distance.values())

    def shoot(self):
        players_distance = self.agent.get_players_distance_to_ball()
        closest_id = min(players_distance, key=players_distance.get)
        closest_dist = players_distance[closest_id]

        self.logger.info(f"[AttackStateMachine] 执行射门: 最近球员{closest_id}（距离{closest_dist:.2f}m）")
        if self.agent.robots_data[closest_id]["data"]["bally"] is not None:
            if self.agent.robots_data[closest_id]["data"]["bally"] > 0 or self.agent.robots_data[closest_id]["data"]["bally"] < 0:
                self.agent.publish_command(closest_id, "shoot")
            else:
                self.agent.publish_command(closest_id, "dribble")
        else:
            self.agent.publish_command(closest_id, "dribble")

        for id in range(1, 6):
            if id != closest_id and self.agent.roles_to_id["goalkeeper"] != id:
                self.agent.publish_command(id, "chase_ball", {"chase_distance": self.no_control_chase_distance})
                self.logger.info(f"[AttackStateMachine] 球员{id}执行支持任务")

    def go_for_possession(self):
        self.logger.info(f"[AttackStateMachine] 执行追球策略（阈值:{self.ball_out_of_control_threshold_m}m）")
        for id in range(1, 6):
            if self.agent.roles_to_id["goalkeeper"] != id:
                self.agent.publish_command(id, "chase_ball")

    def go_for_possession_avoid_collsion(self):
        players_distance = self.agent.get_players_distance_to_ball_without_goalkeeper()
        sorted_players = sorted(players_distance.items(), key=lambda item: item[1])
        closest_id = sorted_players[0][0]
        self.agent.publish_command(closest_id, "chase_ball")
        for id in range(1, 6):
            if id not in [closest_id] and self.agent.roles_to_id["goalkeeper"] != id:
                self.agent.publish_command(id, "chase_ball", {"chase_distance": self.no_control_chase_distance})
        self.logger.info("[AttackStateMachine] 执行防碰撞追球策略")
    