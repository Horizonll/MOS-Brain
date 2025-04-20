from transitions import Machine
import logging


class ShootBallStateMachine:
    def __init__(self, agent):
        self.logger = logging.getLogger(__name__)  # 类级logger
        self.agent = agent

        # 初始化状态机
        self.states = ["have_no_ball", "close_to_ball", "have_ball"]
        self.transitions = [
            {
                "trigger": "run",
                "source": "close_to_ball",
                "dest": "have_ball",
                "conditions": "ball_in_control",
                "after": ["shoot", "log_transition"],  # 添加日志方法
            },
            {
                "trigger": "run",
                "source": "have_ball",
                "dest": "close_to_ball",
                "conditions": "dont_have_ball",
                "after": ["go_for_possession_avoid_collsion", "log_transition"],  # 添加日志方法
            },
            {
                "trigger": "run",
                "source": ["have_no_ball"],
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
            after_state_change=self.on_state_change,  # 添加全局状态变更回调
        )
        self.logger.info("状态机初始化完成，初始状态: have_no_ball")

    def log_transition(self):
        """记录状态迁移"""
        self.logger.debug(f"执行迁移后的动作，当前状态: {self.state}")

    def on_state_change(self, event=None):
        """全局状态变更回调"""
        if event:
            self.logger.info(f"状态变更: {event.transition.source} -> {event.transition.dest}")

    def ball_in_control(self):
        players_distance_to_ball = self.agent.get_players_distance_to_ball_without_goalkeeper()
        forward_1_distance = players_distance_to_ball[self.agent.roles_to_id["forward_1"]]
        forward_2_distance = players_distance_to_ball[self.agent.roles_to_id["forward_2"]]
        result1 = (forward_1_distance < 0.4 or forward_2_distance < 0.4)

        self.logger.debug(f"检查球权控制状态: {'已控球' if result1 else '未控球'}")
        return result1

    def ball_out_of_control(self):
        players_distance_to_ball = self.agent.get_players_distance_to_ball_without_goalkeeper()
        forward_1_distance = players_distance_to_ball[self.agent.roles_to_id["forward_1"]]
        forward_2_distance = players_distance_to_ball[self.agent.roles_to_id["forward_2"]]
        result = (forward_1_distance > 0.8 and forward_2_distance > 0.8)
        self.logger.debug(
            f"检查球权丢失状态: 前锋1距离={forward_1_distance:.2f}, "
            f"前锋2距离={forward_2_distance:.2f}, {'已丢失' if result else '仍保持'}"
        )
        return result
    
    def dont_have_ball(self):
        self.logger.debug(f"检查球权状态: 丢球：{self.ball_out_of_control()} , 控球：{self.ball_in_control()}")
        return not self.ball_in_control() and not self.ball_out_of_control()

    def close_to_ball(self):
        players_distance_to_ball = self.agent.get_players_distance_to_ball_without_goalkeeper()
        forward_1_distance = players_distance_to_ball[self.agent.roles_to_id["forward_1"]]
        forward_2_distance = players_distance_to_ball[self.agent.roles_to_id["forward_2"]]
        result = (forward_1_distance < 0.8 or forward_2_distance < 0.8)
        self.logger.debug(
            f"检查近距离状态: 前锋1距离={forward_1_distance:.2f}, "
            f"前锋2距离={forward_2_distance:.2f}, {'在范围内' if result else '未接近'}"
        )
        return result

    def shoot(self):
        """射门动作（添加距离信息日志）"""
        players_distance = self.agent.get_players_distance_to_ball_without_goalkeeper()
        min_distance = float('inf')
        closest_player_id = None
        
        # 记录所有球员距离
        distance_log = ", ".join(
            [f"{role}: {players_distance[id]:.2f}" 
             for role, id in self.agent.roles_to_id.items()]
        )
        self.logger.debug(f"球员距离球门数据: {distance_log}")

        # 寻找最近球员
        for role, id in self.agent.roles_to_id.items():
            if players_distance[id] < min_distance:
                min_distance = players_distance[id]
                closest_player_id = id

        if closest_player_id is not None:
            self.logger.info(f"执行射门: 球员 {closest_player_id} (距离={min_distance:.2f})")
            self.agent.publish_command(closest_player_id, "dribble")
        else:
            self.logger.warning("射门失败: 未找到可用球员")

    def go_for_possession(self):
        """争夺控球（添加策略说明）"""
        self.logger.info("执行双前锋追球策略")
        self.agent.publish_command(self.agent.roles_to_id["forward_1"], "chase_ball")
        self.agent.publish_command(self.agent.roles_to_id["forward_2"], "chase_ball")

    def go_for_possession_avoid_collsion(self):
        """防碰撞策略（添加选择逻辑说明）"""
        players_distance_to_ball = self.agent.get_players_distance_to_ball_without_goalkeeper()
        forward_1_distance = players_distance_to_ball[self.agent.roles_to_id["forward_1"]]
        forward_2_distance = players_distance_to_ball[self.agent.roles_to_id["forward_2"]]
        
        # 记录比较结果
        self.logger.debug(
            f"防撞策略: 前锋1距离={forward_1_distance:.2f} vs 前锋2距离={forward_2_distance:.2f}"
        )

        if forward_1_distance < forward_2_distance:
            self.logger.info("执行策略: 前锋1追球，前锋2待命")
            self.agent.publish_command(self.agent.roles_to_id["forward_2"], "stop")
            self.agent.publish_command(self.agent.roles_to_id["forward_1"], "chase_ball")
        else:
            self.logger.info("执行策略: 前锋2追球，前锋1待命")
            self.agent.publish_command(self.agent.roles_to_id["forward_1"], "stop")
            self.agent.publish_command(self.agent.roles_to_id["forward_2"], "chase_ball")