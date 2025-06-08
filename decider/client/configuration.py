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
        f_config = open("/home/thmos/MOS-Brain/decider/client/config.json", "r")
        json_str = f_config.read()
        json_str = _remove_comment(json_str)
        config = json.loads(json_str)
    except:
        print("[!] Can not parse config.json !");
        print(json_str)
        exit()

    # Loading override configs; this is not mandatory
    try:
        f_override = open("/home/thmos/MOS-Brain/decider/client/config_override.json", "r")
        json_str = f_override.read()
        json_str = _remove_comment(json_str)
        override = json.loads(json_str)
        for key, value in override.items():
            config[key] = value
    except Exception as e:
        print("[!] Can not parse config_override.json! " + str(e))
        print(json_str)
        pass

    return config
