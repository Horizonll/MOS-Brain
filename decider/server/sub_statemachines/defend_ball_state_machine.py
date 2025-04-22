from transitions import Machine
import logging


class DefendBallStateMachine:
    def __init__(self, agent):
        self.logger = logging.getLogger(__name__)  # 类级logger
        self.agent = agent
        self.states = ["have_no_ball", "close_to_ball", "have_ball"]
        self.transitions = [
            {
                "trigger": "run",
                "source": ["close_to_ball", "have_ball"],
                "dest": "have_ball",
                "conditions": "ball_in_control",
                "after": "dribble_forward",
            },
            {
                "trigger": "run",
                "source": ["have_no_ball", "have_ball", "close_to_ball"],
                "dest": "close_to_ball",
                "conditions": "close_to_ball",
                "after": "go_for_possession_avoid_collsion",  # 争夺控球, 把find和chase封装进去
            },
            {
                "trigger": "run",
                "source": ["close_to_ball", "have_ball", "have_no_ball"],
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
        self.state_machine_name = "DefendBallStateMachine"

    def get_current_state_info(self):
        return f"{self.state_machine_name} - Current state: {self.state}"

    def ball_in_control(self):
        result = not (self.ball_out_of_control() or self.close_to_ball())
        self.logger.info(f"{self.get_current_state_info()}, ball_in_control result: {result}")
        return result

    def ball_out_of_control(self):
        players_distance_to_ball = self.agent.get_players_distance_to_ball_without_goalkeeper()
        forward_1_distance_to_ball = players_distance_to_ball[self.agent.roles_to_id["forward_1"]]
        forward_2_distance_to_ball = players_distance_to_ball[self.agent.roles_to_id["forward_2"]]

        out_of_control = forward_1_distance_to_ball > 0.5 and forward_2_distance_to_ball > 0.5
        self.logger.info(f"{self.get_current_state_info()}, ball_out_of_control result: {out_of_control}, forward_1_distance: {forward_1_distance_to_ball}, forward_2_distance: {forward_2_distance_to_ball}")
        return out_of_control

    def close_to_ball(self):
        players_distance_to_ball = self.agent.get_players_distance_to_ball_without_goalkeeper()
        forward_1_distance_to_ball = players_distance_to_ball[self.agent.roles_to_id["forward_1"]]
        forward_2_distance_to_ball = players_distance_to_ball[self.agent.roles_to_id["forward_2"]]

        close = forward_1_distance_to_ball < 0.5 or forward_2_distance_to_ball < 0.5
        self.logger.info(f"{self.get_current_state_info()}, close_to_ball result: {close}, forward_1_distance: {forward_1_distance_to_ball}, forward_2_distance: {forward_2_distance_to_ball}")
        return close

    def dribble_forward(self):
        """
        判断非守门员机器人中谁拿到球，然后进行带球
        其它两个机器人距离球门最近的进行防守补位，另一个跟随进攻
        """
        self.logger.info(f"{self.get_current_state_info()}, Starting dribble_forward method")
        players_status = self.agent.get_players_status()
        players_distance = self.agent.get_players_distance_to_ball_without_goalkeeper()
        for role, id in self.agent.roles_to_id.items():
            if players_status[id] == "controlling_ball":
                if role == "defender_1":
                    # 将players中的defender_1的角色与前锋中距离球门最近的角色交换
                    # 比较两名前锋与球门的距离
                    forward_1_distance = players_distance[self.agent.roles_to_id["forward_1"]]
                    forward_2_distance = players_distance[self.agent.roles_to_id["forward_2"]]
                    # 将前锋中距离球门最近的角色与defender_1的角色交换
                    if forward_1_distance < forward_2_distance:
                        self.agent.switch_players_role("defender_1", "forward_1")
                        self.agent.publish_command(self.agent.roles_to_id["forward_1"], "dribble")
                        self.agent.publish_command(self.agent.roles_to_id["forward_2"], "forward")
                        self.logger.info(f"{self.get_current_state_info()}, Switched defender_1 with forward_1, commanded forward_1 to dribble and forward_2 to forward")
                    else:
                        self.agent.switch_players_role("defender_1", "forward_2")
                        self.agent.publish_command(self.agent.roles_to_id["forward_2"], "dribble")
                        self.agent.publish_command(self.agent.roles_to_id["forward_1"], "forward")
                        self.logger.info(f"{self.get_current_state_info()}, Switched defender_1 with forward_2, commanded forward_2 to dribble and forward_1 to forward")
                else:
                    if role == "forward_1":
                        self.agent.publish_command(self.agent.roles_to_id["forward_1"], "dribble")
                        self.agent.publish_command(self.agent.roles_to_id["forward_2"], "forward")
                        self.logger.info(f"{self.get_current_state_info()}, Commanded forward_1 to dribble and forward_2 to forward")
                    else:
                        self.agent.publish_command(self.agent.roles_to_id["forward_2"], "dribble")
                        self.agent.publish_command(self.agent.roles_to_id["forward_1"], "forward")
                        self.logger.info(f"{self.get_current_state_info()}, Commanded forward_2 to dribble and forward_1 to forward")
        # 防守补位
        self.agent.publish_command(self.agent.roles_to_id["defender_1"], "go_to_defend_position")
        self.logger.info(f"{self.get_current_state_info()}, Commanded defender_1 to go_to_defend_position")

    def go_for_possession(self):
        """
        争夺控球
        """
        self.logger.info(f"{self.get_current_state_info()}, Starting go_for_possession method")
        # self.agent.t_no_ball = 0
        self.agent.publish_command(self.agent.roles_to_id["forward_1"], "chase_ball")
        self.agent.publish_command(self.agent.roles_to_id["forward_2"], "chase_ball")
        self.agent.publish_command(self.agent.roles_to_id["defender_1"], "chase_ball")
        self.logger.info(f"{self.get_current_state_info()}, Commanded forward_1, forward_2 and defender_1 to chase_ball")

    def go_for_possession_avoid_collsion(self):
        """
        防止距离球太近时机器人发生碰撞
        """
        self.logger.info(f"{self.get_current_state_info()}, Starting go_for_possession_avoid_collsion method")
        # 获取各球员与球的距离
        players_distances = self.agent.get_players_distance_to_ball_without_goalkeeper()

        # 将球员与他们的距离组合成列表
        players_distances = [(id, distance) for id, distance in players_distances.items()]

        # 根据距离从大到小排序
        players_distances_sorted = sorted(players_distances, key=lambda x: x[1], reverse=True)

        # 向最远的两个球员发送停止指令
        for role, _ in players_distances_sorted[:2]:
            self.agent.publish_command(role, "stop")
            self.logger.info(f"{self.get_current_state_info()}, Commanded {role} to stop_moving")

        # 向最近的球员发送追球指令
        closest_player = players_distances_sorted[0][0]
        self.agent.publish_command(closest_player, "chase_ball")
