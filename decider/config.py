class config:
    walk_x_vel = 0.1  # 行走前向速度
    walk_y_vel = 0.05  # 行走横向速度
    walk_theta_vel = 0.3  # 转向速度

    # 找球时观察6个点： 正前，左上，左下，正下，右下，右上
    find_head_pos = [0.2, 0.2, 0.85, 1, 0.85, 0.2]
    find_neck_pos = [0, 1, 1, 0, -1, -1]

    # 定位点,x,y
    field_params = {
        "LF": [-1000, -1200],
        "LB": [-1000, -2500],
        "RF": [0, 3000],
        "RB": [1000, -2500],
        "GK": [0, -4000],
    }
    id = 1
    IP = "localhost"  # 主机IP
