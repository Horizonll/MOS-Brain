import asyncio


# 定义一个异步函数
async def fetch_data():
    print("开始获取数据...")
    await asyncio.sleep(2)  # 模拟一个耗时操作
    print("数据获取完成")
    return {"data": "示例数据"}


# 定义主函数
async def main():
    result = await fetch_data()
    print(f"获取到的数据: {result}")


# 运行主函数
asyncio.run(main())
