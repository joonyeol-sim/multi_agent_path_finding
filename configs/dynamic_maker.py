import yaml
import random

max_x = random.randint(2, 100)
max_y = random.randint(2, 100)
start_point = [random.randint(0, max_x - 1), random.randint(0, max_y - 1)]
goal_point = [random.randint(0, max_x - 1), random.randint(0, max_y - 1)]
dynamic_obstacles = []
num_of_dynamic_obstacles = random.randint(1, max_x * max_y // 2)
for i in range(1, num_of_dynamic_obstacles):
    start_time = random.randint(
        0,
        abs(start_point[0] - goal_point[0]) + abs(start_point[1] - goal_point[1]),
    )
    end_time = random.randint(
        start_time,
        abs(start_point[0] - goal_point[0]) + abs(start_point[1] - goal_point[1]),
    )
    rand_x = random.randint(0, max_x - 1)
    rand_y = random.randint(0, max_y - 1)
    if [rand_x, rand_y] != start_point and [rand_x, rand_y] != goal_point:
        dynamic_obstacles.append(([rand_x, rand_y], [start_time, end_time]))

with open("space_time_astar/space_time_astar_input.yaml", "r") as stream:
    input_data = yaml.load(stream, Loader=yaml.FullLoader)

input_data["start_point"] = start_point
input_data["goal_point"] = goal_point
input_data["dynamic_obstacles"] = dynamic_obstacles
input_data["space_limits"] = [max_x, max_y]

with open("space_time_astar/space_time_astar_d_input.yaml", "w") as f:
    yaml.dump(input_data, f)
