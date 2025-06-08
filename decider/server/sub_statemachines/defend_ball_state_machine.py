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
        self.no_control_chase_distance = defend_config.get("no_control_chase_distance", 1.2)
        # self.go_back_to_field_x = defend_config.get("self.go_back_to_field_x", 0)
        # self.go_back_to_field_x = defend_config.get("self.go_back_to_field_x", 0)

    def get_current_state_info(self):
        return f"{self.state_machine_name} - Current state: {self.state}"

    def ball_in_control(self):
        players_distance_to_ball = self.agent.get_players_distance_to_ball_without_goalkeeper()
        forward_1_distance = players_distance_to_ball[self.agent.roles_to_id["forward_1"]]
        forward_2_distance = players_distance_to_ball[self.agent.roles_to_id["forward_2"]]
        defender_1_distance = players_distance_to_ball[self.agent.roles_to_id["defender_1"]]
        defender_2_distance = players_distance_to_ball[self.agent.roles_to_id["defender_2"]]
        result = (
            forward_1_distance < self.ball_in_control_threshold_m
            or forward_2_distance < self.ball_in_control_threshold_m
            or defender_1_distance < self.ball_in_control_threshold_m
            or defender_2_distance < self.ball_in_control_threshold_m
        )

        self.logger.debug(f"检查球权控制状态: {'已控球' if result else '未控球'}")
        return result

    def ball_out_of_control(self):
        players_distance_to_ball = self.agent.get_players_distance_to_ball_without_goalkeeper()
        forward_1_distance = players_distance_to_ball[self.agent.roles_to_id["forward_1"]]
        forward_2_distance = players_distance_to_ball[self.agent.roles_to_id["forward_2"]]
        defender_1_distance = players_distance_to_ball[self.agent.roles_to_id["defender_1"]]
        defender_2_distance = players_distance_to_ball[self.agent.roles_to_id["defender_2"]]

        out_of_control = (
            forward_1_distance > self.ball_out_of_control_threshold_m
            and forward_2_distance > self.ball_out_of_control_threshold_m
            and defender_1_distance > self.ball_out_of_control_threshold_m
            and defender_2_distance > self.ball_out_of_control_threshold_m
        )
        self.logger.info(
            f"{self.get_current_state_info()}, ball_out_of_control result: {out_of_control}, "
            f"forward_1_distance: {forward_1_distance}, forward_2_distance: {forward_2_distance}, "
            f"defender_1_distance: {defender_1_distance}, defender_2_distance: {defender_2_distance}"
        )
        return out_of_control

    def close_to_ball(self):
        players_distance_to_ball = self.agent.get_players_distance_to_ball_without_goalkeeper()
        forward_1_distance = players_distance_to_ball[self.agent.roles_to_id["forward_1"]]
        forward_2_distance = players_distance_to_ball[self.agent.roles_to_id["forward_2"]]
        defender_1_distance = players_distance_to_ball[self.agent.roles_to_id["defender_1"]]
        defender_2_distance = players_distance_to_ball[self.agent.roles_to_id["defender_2"]]

        close = (
            forward_1_distance < self.close_to_ball_threshold_m
            or forward_2_distance < self.close_to_ball_threshold_m
            or defender_1_distance < self.close_to_ball_threshold_m
            or defender_2_distance < self.close_to_ball_threshold_m
        )
        self.logger.info(
            f"{self.get_current_state_info()}, close_to_ball result: {close}, "
            f"forward_1_distance: {forward_1_distance}, forward_2_distance: {forward_2_distance}, "
            f"defender_1_distance: {defender_1_distance}, defender_2_distance: {defender_2_distance}"
        )
        return close

    def dribble_forward(self):
        """
        根据距离球的远近判断谁带球，若为后卫带球，让距离己方最近且无控球的前锋回场并互换角色，无前锋则忽略
        """
        self.logger.info(f"{self.get_current_state_info()}, Starting dribble_forward method")
        players_distance_to_ball = self.agent.get_players_distance_to_ball_without_goalkeeper()
        # 找出距离球最近的机器人，即控球机器人
        closest_to_ball_role = min(players_distance_to_ball, key=players_distance_to_ball.get)

        # 获取各机器人的位置信息
        players_positions = self.agent.get_players_positions_without_goalkeeper()

        # 找出没有控球的机器人
        non_ball_roles = [role for role in self.agent.roles_to_id if role != closest_to_ball_role]

        # 若控球机器人是后卫
        if closest_to_ball_role in ["defender_1", "defender_2"]:
            # 筛选出没有控球的前锋机器人
            non_ball_forwards = [role for role in non_ball_roles if role in ["forward_1", "forward_2"]]

            if non_ball_forwards:
                # 找出距离己方球门最近（y最小）且没有控球的前锋机器人
                min_y = float('inf')
                closest_own_goal_forward = None
                for role in non_ball_forwards:
                    position = players_positions[role]
                    y = position[1]  # 假设位置信息中 y 坐标在索引 1 处
                    if y < min_y:
                        min_y = y
                        closest_own_goal_forward = role

                if closest_own_goal_forward:
                    # 让距离己方球门最近且没有控球的前锋机器人回场
                    self.agent.publish_command(self.agent.roles_to_id[closest_own_goal_forward], "go_back_to_field")

                    # 互换角色
                    temp = self.agent.roles_to_id[closest_to_ball_role]
                    self.agent.roles_to_id[closest_to_ball_role] = self.agent.roles_to_id[closest_own_goal_forward]
                    self.agent.roles_to_id[closest_own_goal_forward] = temp
                    closest_to_ball_role = closest_own_goal_forward

        # 让所有前锋前进
        for role in ["forward_1", "forward_2"]:
            if role != closest_to_ball_role:
                self.agent.publish_command(self.agent.roles_to_id[role], "chase_ball", {"chase_distance": self.no_control_chase_distance})
                self.logger.debug(f"{self.get_current_state_info()}, Commanded {role} to chase_ball with chase_distance 1.2")
            else:
                self.agent.publish_command(self.agent.roles_to_id[role], "dribble")
                self.logger.debug(f"{self.get_current_state_info()}, Commanded {role} to dribble")

        self.logger.debug(
            f"{self.get_current_state_info()}, Commanded {closest_to_ball_role} to dribble"
        )

    def go_for_possession(self):
        """
        争夺控球
        """
        self.logger.debug(f"{self.get_current_state_info()}, Starting go_for_possession method")
        self.agent.publish_command(self.agent.roles_to_id["forward_1"], "chase_ball")
        self.agent.publish_command(self.agent.roles_to_id["forward_2"], "chase_ball")
        self.agent.publish_command(self.agent.roles_to_id["defender_1"], "chase_ball")
        self.agent.publish_command(self.agent.roles_to_id["defender_2"], "chase_ball")
        self.logger.debug(
            f"{self.get_current_state_info()}, Commanded forward_1, forward_2, defender_1 and defender_2 to chase_ball"
        )

    def go_for_possession_avoid_collsion(self):
        """
        防止距离球太近时机器人发生碰撞
        """
        self.logger.debug(f"{self.get_current_state_info()}, Starting go_for_possession_avoid_collsion method")
        # 获取各球员与球的距离
        players_distances = self.agent.get_players_distance_to_ball_without_goalkeeper()

        # 将球员与他们的距离组合成列表
        players_distances = [(id, distance) for id, distance in players_distances.items()]

        # 根据距离从小到大排序
        players_distances_sorted = sorted(players_distances, key=lambda x: x[1])

        # 找出最近的球员
        closest_player = players_distances_sorted[0][0]

        # 向除了最近的球员之外的其他球员发送追球指令
        for id, _ in players_distances_sorted[1:]:
            self.agent.publish_command(id, "chase_ball", {"chase_distance": self.no_control_chase_distance})
            self.logger.debug(f"{self.get_current_state_info()}, Commanded {id} to chase_ball with chase_distance {self.no_control_chase_distance}")

        # 向最近的球员发送追球指令
        self.agent.publish_command(closest_player, "chase_ball")
        self.logger.debug(f"{self.get_current_state_info()}, Commanded {closest_player} to chase_ball")
