import json
import re
import yaml
from pathlib import Path
import os

def _remove_comment(json_str):
    # 移除单行注释
    json_str = re.sub(r'//.*', '', json_str)
    # 移除多行注释
    json_str = re.sub(r'/\*.*?\*/', '', json_str, flags=re.DOTALL)
    return json_str

def _deep_merge(dest, src):
    """递归合并字典，支持多级覆盖"""
    for key, value in src.items():
        if isinstance(value, dict) and key in dest and isinstance(dest[key], dict):
            dest[key] = _deep_merge(dest[key], value)
        else:
            dest[key] = value
    return dest

def _load_file(file_path):
    """根据文件扩展名自动选择解析器"""
    file_ext = Path(file_path).suffix.lower()
    content = Path(file_path).read_text()
    
    if file_ext in ['.json']:
        content = _remove_comment(content)
        return json.loads(content)
    elif file_ext in ['.yaml', '.yml']:
        return yaml.safe_load(content)
    else:
        raise ValueError(f"Unsupported file format: {file_ext}")

def load_config():
    config = {}
    
    # 获取当前Python文件所在目录
    current_dir = Path(os.path.dirname(os.path.abspath(__file__)))
    
    # 定义配置文件路径（相对于当前脚本目录）
    base_config_path = current_dir / "config.json"
    
    # 加载基础配置（必需）
    try:
        config = _load_file(base_config_path)
    except Exception as e:
        print(f"[!] 无法解析基础配置文件 {base_config_path} !")
        print(e)
        exit()

    # 加载覆盖配置（可选）
    override_paths = [
        current_dir / "config_override.json",
        current_dir / "config_override.yaml",
        current_dir / "config_override.yml"
    ]

    for path in override_paths:
        if path.exists():
            try:
                override = _load_file(path)
                config = _deep_merge(config, override)
            except Exception as e:
                print(f"[!] 无法解析覆盖配置文件 {path} !")
                print(e)

    return config