import random

import yaml


def static_obstacle_generator(dimension, space_limit, num, start_points, goal_points):
    static_obstacles = []
    for i in range(num):
        rand_point = [random.randint(0, space_limit[i] - 1) for i in range(dimension)]
        generate_flag = True
        for start_point, goal_point in zip(start_points, goal_points):
            if rand_point == start_point or rand_point == goal_point:
                generate_flag = False
        if generate_flag:
            static_obstacles.append(rand_point)
    return static_obstacles


def dynamic_obstacle_generator(dimension, space_limit, num, start_points, goal_points):
    max_time_range = 0
    for start_point, goal_point in zip(start_points, goal_points):
        time_range = 0
        for i in range(dimension):
            time_range += abs(start_point[i] - goal_point[i])
        max_time_range = max(max_time_range, time_range)

    dynamic_obstacles = []
    for i in range(num):
        for j in range(dimension):
            start_time = random.randint(0, max_time_range)
            end_time = random.randint(start_time, max_time_range)
            rand_point = [
                random.randint(0, space_limit[i] - 1) for i in range(dimension)
            ]
            generate_flag = True
            for start_point, goal_point in zip(start_points, goal_points):
                if rand_point == start_point or rand_point == goal_point:
                    generate_flag = False
            if generate_flag:
                dynamic_obstacles.append((rand_point, [start_time, end_time]))
    return dynamic_obstacles


def config_generator(
    dimension, space_limits, agent_num, static_obstacles_num, dynamic_obstacles_num
):
    start_points = []
    goal_points = []

    while len(start_points) < agent_num:
        start_point = [
            random.randint(0, space_limits[0] - 1),
            random.randint(0, space_limits[1] - 1),
        ]
        goal_point = [
            random.randint(0, space_limits[0] - 1),
            random.randint(0, space_limits[1] - 1),
        ]

        if start_point not in start_points and goal_point not in goal_points:
            start_points.append(start_point)
            goal_points.append(goal_point)

    static_obstacles = static_obstacle_generator(
        dimension, space_limits, static_obstacles_num, start_points, goal_points
    )
    dynamic_obstacles = dynamic_obstacle_generator(
        dimension, space_limits, dynamic_obstacles_num, start_points, goal_points
    )
    config = {
        "dimension": dimension,
        "space_limits": space_limits,
        "static_obstacles": static_obstacles,
        "dynamic_obstacles": dynamic_obstacles,
        "start_points": start_points,
        "goal_points": goal_points,
    }
    return config


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--output", "-o", type=str, help="Output file path")
    args = parser.parse_args()

    config = config_generator(2, [100, 100], 75, 0, 0)
    with open(args.output, "w") as f:
        yaml.dump(config, f)
