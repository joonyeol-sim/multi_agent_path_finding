import yaml
import random


def static_obstacle_generator(dimension, space_limit, num):
    static_obstacles = []
    for i in range(num):
        static_obstacle = []
        for j in range(dimension):
            static_obstacle.append(random.randint(0, space_limit[j]))
        static_obstacles.append(static_obstacle)
    return static_obstacles


def dynamic_obstacle_generator(dimension, space_limit, num):
    dynamic_obstacles = []
    for i in range(num):
        dynamic_obstacle = []
        for j in range(dimension):
            dynamic_obstacle.append(random.randint(0, space_limit[j]))
        dynamic_obstacles.append(dynamic_obstacle)
    return dynamic_obstacles


def config_generator(
    dimension, space_limit, static_obstacles_num, dynamic_obstacles_num
):
    start_point = [
        random.randint(0, space_limit[0] - 1),
        random.randint(0, space_limit[1] - 1),
    ]
    goal_point = [
        random.randint(0, space_limit[0] - 1),
        random.randint(0, space_limit[1] - 1),
    ]
    static_obstacles = static_obstacle_generator(
        dimension, space_limit, static_obstacles_num
    )
    dynamic_obstacles = dynamic_obstacle_generator(
        dimension, space_limit, dynamic_obstacles_num
    )
    config = {
        "dimension": dimension,
        "space_limit": space_limit,
        "static_obstacles": static_obstacles,
        "dynamic_obstacles": dynamic_obstacles,
    }
    return config


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--input", "-i", type=str, help="Input file path")
    args = parser.parse_args()

    config = config_generator(2, [50, 50], 10, 10)
    with open(args.output, "w") as f:
        yaml.dump(config, f)
