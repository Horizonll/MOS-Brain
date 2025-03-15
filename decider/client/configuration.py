# configuration.py
#
#   @description : Loading configurations from config.json and 
#                  config_overide.json for variant configs
#   
#   @interfaces : 
#       1. dictionary read_config()
#           description: reading config from config.json
#                        and overide it from config_override.json
#           return: a dictionary, the config
# 
#   @updates :
#       Mar. 15: refactoring
#

import json
import re


def _remove_comment(json_str):
    # remove single line comment
    json_str = re.sub(r'//.*', '', json_str)
    # remove multiple lines comment
    json_str = re.sub(r'/\*.*?\*/', '', json_str, flags=re.DOTALL)
    return json_str


def load_config():
    config = {}

    # Loading basic configs; which is mandatory
    try:
        f_config = open("config.json", "r")
        json_str = f_config.read()
        json_str = _remove_comment(json_str)
        config = json.loads(json_str)
    except:
        print("[!] Can not parse config.json !");
        exit()

    # Loading override configs; this is not mandatory
    try:
        f_override = open("config_override.json", "r")
        json_str = f_overrride.read()
        json_str = _remove_commnet(json_str)
        override = json.loads(json_str)
        for key.value in override:
            config[key] = value
    except:
        print("[!] Can not parse config_override.json!")
        pass

    return config


    

class configuration:
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
