from transitions import Machine
import logging


class DribbleBallStateMachine:
    def __init__(self, agent):
        self.logger = logging.getLogger(__name__)  # 类级logger
        self.agent = agent
        self.states = ["have_no_ball", "close_to_ball", "have_ball"]
        self.transitions = [
            {
                "trigger": "run",
                "source": "close_to_ball",
                "dest": "have_ball",
                "conditions": "ball_in_control",
                "after": "dribble_forward",
            },
            {
                "trigger": "run",
                "source": ["have_no_ball", "have_ball"],
                "dest": "close_to_ball",
                "conditions": "close_to_ball",
                "after": "go_for_possession_avoid_collsion",  # 争夺控球, 把find和chase封装进去
            },
            {
                "trigger": "run",
                "source": ["close_to_ball", "have_ball"],
                "dest": "have_no_ball",
                "conditions": "ball_out_of_control",
                "after": "go_for_possession",  # 争夺控球, 把find和chase封装进去
            },
        ]
        self.machine = Machine(
            model=self,
            states=self.states,
            initial="have_no_ball",
            transitions=self.transitions,
        )

    def ball_in_control(self):
        return self.agent.if_ball_in_sight()

    def ball_out_of_control(self):
        players_distance_to_ball = self.agent.get_players_distance_to_ball_without_goalkeeper()
        forward_1_distance_to_ball = players_distance_to_ball[self.agent.roles_to_id["forward_1"]]
        forward_2_distance_to_ball = players_distance_to_ball[self.agent.roles_to_id["forward_2"]]
        if forward_1_distance_to_ball > 0.5 and forward_2_distance_to_ball > 0.5:
            return True
        return False

    def close_to_ball(self):
        players_distance_to_ball = self.agent.get_players_distance_to_ball_without_goalkeeper()
        forward_1_distance_to_ball = players_distance_to_ball[self.agent.roles_to_id["forward_1"]]
        forward_2_distance_to_ball = players_distance_to_ball[self.agent.roles_to_id["forward_2"]]
        if forward_1_distance_to_ball < 0.5 or forward_2_distance_to_ball < 0.5:
            return True
        return False

    def dribble_forward(self):
        """
        判断非守门员机器人中谁拿到球，然后进行带球
        其它两个机器人距离球门最近的进行防守补位，另一个跟随进攻
        """
        players_status = self.agent.get_players_status()
        players_distance = self.agent.get_players_distance_to_ball_without_goalkeeper()
        for role, id in self.agent.roles_to_id.items():
            if players_status[id] == "controlling_ball":
                if role == "forward_1":
                    self.agent.publish_command(self.agent.roles_to_id["forward_1"], "dribble")
                    self.agent.publish_command(self.agent.roles_to_id["forward_2"], "forward")
                else:
                    self.agent.publish_command(self.agent.roles_to_id["forward_2"], "dribble")
                    self.agent.publish_command(self.agent.roles_to_id["forward_1"], "forward")
        # 防守补位
        self.agent.publish_command(self.agent.roles_to_id["defender_1"], "go_to_defend_position")

    def go_for_possession(self):
        """
        争夺控球
        """
        self.agent.t_no_ball = 0
        self.agent.publish_command(self.agent.roles_to_id["forward_1"], "chase_ball")
        self.agent.publish_command(self.agent.roles_to_id["forward_2"], "chase_ball")

    def go_for_possession_avoid_collsion(self):
        """
        防止距离球太近时两机器人发生碰撞
        """
        players_distance_to_ball = self.agent.get_players_distance_to_ball_without_goalkeeper()
        forward_1_distance_to_ball = players_distance_to_ball[self.agent.roles_to_id["forward_1"]]
        forward_2_distance_to_ball = players_distance_to_ball[self.agent.roles_to_id["forward_2"]]
        if forward_1_distance_to_ball < forward_2_distance_to_ball:
            self.agent.publish_command(self.agent.roles_to_id["forward_2"], "stop")
        else:
            self.agent.publish_command(self.agent.roles_to_id["forward_1"], "stop")