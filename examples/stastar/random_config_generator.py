import random

import yaml


def static_obstacle_generator(dimension, space_limit, num, start_point, goal_point):
    static_obstacles = []
    for i in range(num):
        static_obstacle = []
        for j in range(dimension):
            static_obstacle.append(random.randint(0, space_limit[j]))
        if static_obstacle != start_point and static_obstacle != goal_point:
            static_obstacles.append(static_obstacle)
    return static_obstacles


def dynamic_obstacle_generator(dimension, space_limit, num, start_point, goal_point):
    time_range = 0
    for i in range(dimension):
        time_range += abs(start_point[i] - goal_point[i])
    dynamic_obstacles = []
    for i in range(num):
        for j in range(dimension):
            start_time = random.randint(0, time_range)
            end_time = random.randint(start_time, time_range)
            rand_point = [
                random.randint(0, space_limit[i] - 1) for i in range(dimension)
            ]
            if rand_point != start_point and rand_point != goal_point:
                dynamic_obstacles.append((rand_point, [start_time, end_time]))
    return dynamic_obstacles


def config_generator(
    dimension, space_limits, static_obstacles_num, dynamic_obstacles_num
):
    start_point = [
        random.randint(0, space_limits[0] - 1),
        random.randint(0, space_limits[1] - 1),
    ]
    goal_point = [
        random.randint(0, space_limits[0] - 1),
        random.randint(0, space_limits[1] - 1),
    ]
    static_obstacles = static_obstacle_generator(
        dimension, space_limits, static_obstacles_num, start_point, goal_point
    )
    dynamic_obstacles = dynamic_obstacle_generator(
        dimension, space_limits, dynamic_obstacles_num, start_point, goal_point
    )
    config = {
        "dimension": dimension,
        "space_limits": space_limits,
        "static_obstacles": static_obstacles,
        "dynamic_obstacles": dynamic_obstacles,
        "start_point": start_point,
        "goal_point": goal_point,
    }
    return config


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--output", "-o", type=str, help="Output file path")
    args = parser.parse_args()

    config = config_generator(2, [10, 10], 10, 0)
    with open(args.output, "w") as f:
        yaml.dump(config, f)
