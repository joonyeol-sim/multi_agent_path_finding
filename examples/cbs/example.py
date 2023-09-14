import yaml
import time
from common.environment import Environment
from common.point import Point2D, Point3D
from cbs.cbs import ConflictBasedSearch

if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--input", "-i", type=str, help="Input file path")
    parser.add_argument("--output", "-o", type=str, help="Output file path")
    args = parser.parse_args()

    with open(args.input, "r") as stream:
        input_data = yaml.load(stream, Loader=yaml.FullLoader)

    if input_data["dimension"] == 2:
        Point = Point2D
    elif input_data["dimension"] == 3:
        Point = Point3D
    else:
        raise ValueError(f"Dimension must be 2 or 3: {input_data['dimension']}")

    static_obstacles = [
        Point(*static_obstacle) for static_obstacle in input_data["static_obstacles"]
    ]

    dynamic_obstacles = [
        (Point(*dynamic_obstacle[0]), dynamic_obstacle[1])
        for dynamic_obstacle in input_data["dynamic_obstacles"]
    ]

    environment = Environment(
        input_data["dimension"],
        input_data["space_limits"],
        static_obstacles,
        dynamic_obstacles,
    )

    start_points = [Point(*start_point) for start_point in input_data["start_points"]]
    goal_points = [Point(*goal_point) for goal_point in input_data["goal_points"]]
    planner = ConflictBasedSearch(
        start_points,
        goal_points,
        environment,
    )

    start_time = time.time()
    result = planner.plan()
    print(f"Time elapsed: {time.time() - start_time}")
    if result is None:
        print("No path found")
    else:
        print(*result, sep="\n")
        # change Point to list for yaml dump
        for i in range(len(result)):
            result[i] = [
                ([*point.__dict__.values()], time) for point, time in result[i]
            ]
        with open(args.output, "w") as f:
            yaml.dump(result, f)
