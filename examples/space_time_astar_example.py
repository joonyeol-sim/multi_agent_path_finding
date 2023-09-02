import yaml
from space_time_astar.environment import Environment
from space_time_astar.space_time_astar import SpaceTimeAstar

if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--input", "-i", type=str, help="Input file path")
    parser.add_argument("--output", "-o", type=str, help="Output file path")
    args = parser.parse_args()

    with open(args.input, "r") as stream:
        input_data = yaml.load(stream, Loader=yaml.FullLoader)

    environment = Environment(
        input_data["dimension"],
        input_data["space_limits"],
        input_data["static_obstacles"],
        input_data["dynamic_obstacles"],
    )
    planner = SpaceTimeAstar(
        input_data["start_point"], input_data["goal_point"], environment
    )

    result = planner.plan()
    if result is None:
        print("No path found")
    else:
        print(*result, sep="\n")
        # save path to yaml file
        with open(args.output, "w") as f:
            yaml.dump(result, f)
