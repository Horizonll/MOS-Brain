from transitions import Machine
import logging


class ShootBallStateMachine:
    def __init__(self, agent):
        self.logger = logging.getLogger(__name__)
        self.agent = agent
        self._config = self.agent.get_config()  # 假设代理对象有获取配置的方法
        self.read_params()  # 添加参数读取

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
        self.logger.info("状态机初始化完成，初始状态: have_no_ball")

    def read_params(self):
        """从配置文件读取参数"""
        shoot_config = self._config.get("shooting", {})
        self.ball_in_control_threshold_m = shoot_config.get("ball_in_control_threshold_m", 0.4)
        self.ball_out_of_control_threshold_m = shoot_config.get("ball_out_of_control_threshold_m", 0.8)
        self.close_to_ball_threshold_m = shoot_config.get("close_to_ball_threshold_m", 0.8)

    def log_transition(self):
        self.logger.debug(f"执行迁移后的动作，当前状态: {self.state}")

    def on_state_change(self, event=None):
        if event:
            self.logger.info(f"状态变更: {event.transition.source} -> {event.transition.dest}")

    def ball_in_control(self):
        """检查是否控球（参数化距离阈值）"""
        players_distance = self.agent.get_players_distance_to_ball_without_goalkeeper()
        f1_dist = players_distance[self.agent.roles_to_id["forward_1"]]
        f2_dist = players_distance[self.agent.roles_to_id["forward_2"]]
        result = (f1_dist < self.ball_in_control_threshold_m) or (f2_dist < self.ball_in_control_threshold_m)
        self.logger.debug(f"控球检查: 前锋1={f1_dist:.2f}m, 前锋2={f2_dist:.2f}m, 结果={result}")
        return result

    def ball_out_of_control(self):
        """检查是否丢球（参数化距离阈值）"""
        players_distance = self.agent.get_players_distance_to_ball_without_goalkeeper()
        f1_dist = players_distance[self.agent.roles_to_id["forward_1"]]
        f2_dist = players_distance[self.agent.roles_to_id["forward_2"]]
        result = (f1_dist > self.ball_out_of_control_threshold_m) and (f2_dist > self.ball_out_of_control_threshold_m)
        self.logger.debug(f"丢球检查: 前锋1={f1_dist:.2f}m, 前锋2={f2_dist:.2f}m, 结果={result}")
        return result
    
    def dont_have_ball(self):
        """检查是否无球（非控球且非丢球中间状态）"""
        control = self.ball_in_control()
        out_of_control = self.ball_out_of_control()
        result = not control and not out_of_control
        self.logger.debug(f"无球状态检查: 控球={control}, 丢球={out_of_control}, 结果={result}")
        return result

    def close_to_ball(self):
        """检查是否接近球（参数化距离阈值）"""
        players_distance = self.agent.get_players_distance_to_ball_without_goalkeeper()
        f1_dist = players_distance[self.agent.roles_to_id["forward_1"]]
        f2_dist = players_distance[self.agent.roles_to_id["forward_2"]]
        result = (f1_dist < self.close_to_ball_threshold_m) or (f2_dist < self.close_to_ball_threshold_m)
        self.logger.debug(f"接近球检查: 前锋1={f1_dist:.2f}m, 前锋2={f2_dist:.2f}m, 结果={result}")
        return result

    def shoot(self):
        """射门动作（带参数化距离日志）"""
        players_distance = self.agent.get_players_distance_to_ball_without_goalkeeper()
        distance_log = ", ".join([
            f"{role}: {players_distance[id]:.2f}m" 
            for role, id in self.agent.roles_to_id.items()
        ])
        self.logger.debug(f"球员距离球数据: {distance_log}")

        closest_id = min(self.agent.roles_to_id.values(), key=lambda x: players_distance[x])
        closest_dist = players_distance[closest_id]
        
        self.logger.info(f"执行射门: 最近球员{closest_id}（距离{closest_dist:.2f}m）")
        self.agent.publish_command(closest_id, "dribble")
        
        for role, id in self.agent.roles_to_id.items():
            if id != closest_id:
                self.agent.publish_command(id, "chase_ball", {"chase_distance": 1})
                self.logger.info(f"球员{id}执行支持任务")

    def go_for_possession(self):
        """双前锋追球策略（参数化策略说明）"""
        self.logger.info(f"执行追球策略（阈值:{self.ball_out_of_control_threshold_m}m）")
        self.agent.publish_command(self.agent.roles_to_id["forward_1"], "chase_ball")
        self.agent.publish_command(self.agent.roles_to_id["forward_2"], "chase_ball")

    def go_for_possession_avoid_collsion(self):
        """防碰撞策略（带参数化距离比较）"""
        players_distance = self.agent.get_players_distance_to_ball_without_goalkeeper()
        f1_dist = players_distance[self.agent.roles_to_id["forward_1"]]
        f2_dist = players_distance[self.agent.roles_to_id["forward_2"]]
        self.logger.debug(f"防撞距离比较: 前锋1={f1_dist:.2f}m vs 前锋2={f2_dist:.2f}m")

        if f1_dist < f2_dist:
            self.logger.info(f"启动单前锋追球（前锋1更近，阈值:{self.close_to_ball_threshold_m}m）")
            self.agent.publish_command(self.agent.roles_to_id["forward_1"], "chase_ball")
            self.agent.publish_command(self.agent.roles_to_id["forward_2"], "find_ball")
        else:
            self.logger.info(f"启动单前锋追球（前锋2更近，阈值:{self.close_to_ball_threshold_m}m）")
            self.agent.publish_command(self.agent.roles_to_id["forward_2"], "chase_ball")
            self.agent.publish_command(self.agent.roles_to_id["forward_1"], "find_ball")


# 配置文件示例（JSON格式）
"""
"shooting": {
    "ball_in_control_threshold_m": 0.4,        
    "ball_out_of_control_threshold_m": 0.8,   
    "close_to_ball_threshold_m": 0.8          
}
"""