from langchain_openai import ChatOpenAI
from langchain.prompts import ChatPromptTemplate
from openai import OpenAI
from dotenv import load_dotenv

load_dotenv()

client = OpenAI(base_url="https://open.bigmodel.cn/api/paas/v4/")

robot_data = {
    "id": 1,
    "x": 0,
    "y": 0,
    "ballx": 0,
    "bally": 0,
    "orientation": 0,
    "info": "moving to target",
}
data = {
    "robots": [
        {
            "id": 1,
            "x": 0,
            "y": 0,
        },
        {
            "id": 2,
            "x": 100,
            "y": 0,
        },
        {
            "id": 3,
            "x": 0,
            "y": 100,
        },
        {
            "id": 4,
            "x": -50,
            "y": -50,
        },
        {
            "id": 5,
            "x": 50,
            "y": 50,
        },
        {
            "id": 6,
            "x": -100,
            "y": 100,
        },
    ],
    "ball": {
        "x": 0,
        "y": 300,
    },
}

prompt_template = ChatPromptTemplate.from_messages(
    [
        (
            "system",
            "你被用于足球机器人决策，原点在球场中心，面向对手球门为y正方向，范围(-400,400)，右手为x正方向，范围(-250,250)。给你机器人和球的位置信息，你输出每个机器人的指令，不要输出太多信息，只需要输出机器人的指令。指令包括：go_to_defence_position,chase_ball,chase_ball_help。",
        ),
        (
            "human",
            "{data}",
        ),
    ]
)

model = ChatOpenAI(
    model="glm-4-flash",
    openai_api_base="https://open.bigmodel.cn/api/paas/v4/",
)

chain = prompt_template | model
answer = chain.invoke(input={"data": data})
print(answer.content)
