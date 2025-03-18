# configuration.py
#
#   @description : Loading configurations from config.json and 
#                  config_overide.json for variant configs
#   
#   @interfaces : 
#       1. read_config(): dictionary
#           description: reading config from config.json
#                        and overide it from config_override.json
#           return: a dictionary, the config
# 
#   @note :
#       1. The .json parser allows annotation begins with '//' or
#          quoted with '/*' and '*/' (C++ flavor)
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
