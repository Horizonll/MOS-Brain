import math

data = {
    "robots": [
        {
            "id": 1,
            "x": 0,
            "y": 0,
            "ballx": 0,
            "bally": 0,
            "orientation": 0,
            "info": "moving to",
        },
        {
            "id": 2,
            "x": 0,
            "y": 0,
            "ballx": 0,
            "bally": 0,
            "orientation": 0,
            "info": "moving to",
        },
    ]
}


def distance_to_ball(robot):
    return math.sqrt(
        (robot["x"] - robot["ballx"]) ** 2 + (robot["y"] - robot["bally"]) ** 2
    )


closest_robot = min(data["robots"], key=distance_to_ball)
closest_robot["info"] = "chasing ball"
second_closest_robot = sorted(data["robots"], key=distance_to_ball)[1]
assist_x = -0.8 * second_closest_robot["ballx"]
assist_y = 400 - (400 - second_closest_robot["bally"]) * 0.8
second_closest_robot["info"] = f"moving to ({assist_x}, {assist_y})"
for robot in data["robots"]:
    if robot["info"] not in ["chasing ball", f"moving to ({assist_x}, {assist_y})"]:
        robot["info"] = "defending"
