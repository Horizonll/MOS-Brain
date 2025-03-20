import os

def get_python_files(start_dir):
    """
    递归获取目录下所有 Python 文件
    :param start_dir: 起始目录路径
    :return: 包含所有 .py 文件路径的列表
    """
    py_files = []
    for root, dirs, files in os.walk(start_dir):
        for filename in files:
            if filename.endswith('.py'):
                full_path = os.path.join(root, filename)
                py_files.append(full_path[0:-3])
    return py_files

# 使用示例
files = get_python_files('sub_statemachines/')
print(f"找到 {len(files)} 个Python文件:")
for f in files[:5]:  # 打印前5个文件
    print(f"- {f}")

