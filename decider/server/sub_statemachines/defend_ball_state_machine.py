from transitions import Machine
import logging


class DefendBallStateMachine:
    def __init__(self, agent):
        self.logger = logging.getLogger(__name__)  # 类级logger
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
                "after": "dribble_forward",
            },
            {
                "trigger": "run",
                "source": ["have_no_ball", "have_ball", "close_to_ball"],
                "dest": "close_to_ball",
                "conditions": "close_to_ball",
                "after": "go_for_possession_avoid_collsion",
            },
            {
                "trigger": "run",
                "source": ["close_to_ball", "have_ball", "have_no_ball"],
                "dest": "have_no_ball",
                "conditions": "ball_out_of_control",
                "after": "go_for_possession",
            },
        ]
        self.machine = Machine(
            model=self,
            states=self.states,
            initial="have_no_ball",
            transitions=self.transitions,
        )
        self.state_machine_name = "DefendBallStateMachine"

    def read_params(self):
        defend_config = self._config.get("defending", {})
        self.ball_in_control_threshold_m = defend_config.get("ball_in_control_threshold_m", 0.4)
        self.ball_out_of_control_threshold_m = defend_config.get("ball_out_of_control_threshold_m", 0.5)
        self.close_to_ball_threshold_m = defend_config.get("close_to_ball_threshold_m", 0.5)

    def get_current_state_info(self):
        return f"{self.state_machine_name} - Current state: {self.state}"

    def ball_in_control(self):
        players_distance_to_ball = self.agent.get_players_distance_to_ball_without_goalkeeper()
        forward_1_distance = players_distance_to_ball[self.agent.roles_to_id["forward_1"]]
        forward_2_distance = players_distance_to_ball[self.agent.roles_to_id["forward_2"]]
        defender_1_distance = players_distance_to_ball[self.agent.roles_to_id["defender_1"]]
        result = (
            forward_1_distance < self.ball_in_control_threshold_m
            or forward_2_distance < self.ball_in_control_threshold_m
            or defender_1_distance < self.ball_in_control_threshold_m
        )

        self.logger.debug(f"检查球权控制状态: {'已控球' if result else '未控球'}")
        return result

    def ball_out_of_control(self):
        players_distance_to_ball = self.agent.get_players_distance_to_ball_without_goalkeeper()
        forward_1_distance = players_distance_to_ball[self.agent.roles_to_id["forward_1"]]
        forward_2_distance = players_distance_to_ball[self.agent.roles_to_id["forward_2"]]
        defender_1_distance = players_distance_to_ball[self.agent.roles_to_id["defender_1"]]

        out_of_control = (
            forward_1_distance > self.ball_out_of_control_threshold_m
            and forward_2_distance > self.ball_out_of_control_threshold_m
            and defender_1_distance > self.ball_out_of_control_threshold_m
        )
        self.logger.info(
            f"{self.get_current_state_info()}, ball_out_of_control result: {out_of_control}, "
            f"forward_1_distance: {forward_1_distance}, forward_2_distance: {forward_2_distance}, "
            f"defender_1_distance: {defender_1_distance}"
        )
        return out_of_control

    def close_to_ball(self):
        players_distance_to_ball = self.agent.get_players_distance_to_ball_without_goalkeeper()
        forward_1_distance = players_distance_to_ball[self.agent.roles_to_id["forward_1"]]
        forward_2_distance = players_distance_to_ball[self.agent.roles_to_id["forward_2"]]
        defender_1_distance = players_distance_to_ball[self.agent.roles_to_id["defender_1"]]

        close = (
            forward_1_distance < self.close_to_ball_threshold_m
            or forward_2_distance < self.close_to_ball_threshold_m
            or defender_1_distance < self.close_to_ball_threshold_m
        )
        self.logger.info(
            f"{self.get_current_state_info()}, close_to_ball result: {close}, "
            f"forward_1_distance: {forward_1_distance}, forward_2_distance: {forward_2_distance}, "
            f"defender_1_distance: {defender_1_distance}"
        )
        return close

    def dribble_forward(self):
        """
        根据距离球的远近判断谁带球，其它两个机器人距离球门最近的进行防守补位，另一个跟随进攻
        """
        self.logger.info(f"{self.get_current_state_info()}, Starting dribble_forward method")
        players_distance_to_ball = self.agent.get_players_distance_to_ball_without_goalkeeper()
        # 找出距离球最近的机器人
        closest_to_ball_role = min(players_distance_to_ball, key=players_distance_to_ball.get)

        non_closest_roles = [role for role in self.agent.roles_to_id if role != closest_to_ball_role]
        # 获取这两个非最近球的机器人距离球门的距离（假设这里用距离球的距离近似表示）
        non_closest_distances = {
            role: players_distance_to_ball[self.agent.roles_to_id[role]] for role in non_closest_roles
        }
        sorted_non_closest = sorted(non_closest_distances.items(), key=lambda x: x[1])
        closest_to_goal = sorted_non_closest[0][0]
        farther_from_goal = sorted_non_closest[1][0]

        self.agent.publish_command(self.agent.roles_to_id[closest_to_ball_role], "dribble")
        self.agent.publish_command(self.agent.roles_to_id[closest_to_goal], "chase_ball", {"chase_distance": 1})
        self.agent.publish_command(self.agent.roles_to_id[farther_from_goal], "find_ball")
        self.logger.info(
            f"{self.get_current_state_info()}, Commanded {closest_to_ball_role} to dribble, "
            f"{closest_to_goal} to chase_ball, {farther_from_goal} to find_ball"
        )

    def go_for_possession(self):
        """
        争夺控球
        """
        self.logger.info(f"{self.get_current_state_info()}, Starting go_for_possession method")
        self.agent.publish_command(self.agent.roles_to_id["forward_1"], "chase_ball")
        self.agent.publish_command(self.agent.roles_to_id["forward_2"], "chase_ball")
        self.agent.publish_command(self.agent.roles_to_id["defender_1"], "chase_ball")
        self.logger.info(
            f"{self.get_current_state_info()}, Commanded forward_1, forward_2 and defender_1 to chase_ball"
        )

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
            self.agent.publish_command(role, "find_ball")
            self.logger.info(f"{self.get_current_state_info()}, Commanded {role} to stop_moving")

        # 向最近的球员发送追球指令
        closest_player = players_distances_sorted[0][0]
        self.agent.publish_command(closest_player, "chase_ball")
    