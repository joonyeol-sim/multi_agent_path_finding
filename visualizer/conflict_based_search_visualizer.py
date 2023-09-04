import matplotlib.animation as animation
import matplotlib.pyplot as plt
import yaml


def cbs_animate(input_data, path_data):
    x_coords_list = []
    y_coords_list = []
    times = []
    for i in range(len(path_data)):
        x_coords_list.append([point[0][0] for point in path_data[i]])
        y_coords_list.append([point[0][1] for point in path_data[i]])
        if len(path_data[i]) > len(times):
            times = [point[1] for point in path_data[i]]

    fig, ax = plt.subplots(figsize=(10, 6))
    ax.set_xlim(0, input_data["space_limits"][0])
    ax.set_ylim(0, input_data["space_limits"][1])
    ax.grid(True, which="both", linestyle="-", linewidth=2)
    ax.set_xticks(range(input_data["space_limits"][0] + 1))
    ax.set_yticks(range(input_data["space_limits"][1] + 1))
    ax.set_aspect("equal")
    ax.set_xticklabels([])
    ax.set_yticklabels([])
    ax.tick_params(size=0)

    # Static obstacles
    for obstacle in input_data["static_obstacles"]:
        ax.add_patch(plt.Rectangle((obstacle[0], obstacle[1]), 1, 1, facecolor="gray"))

    # Start and goal points
    for start_point, goal_point in zip(
        input_data["start_points"],
        input_data["goal_points"],
    ):
        ax.plot(
            start_point[0] + 0.5,
            start_point[1] + 0.5,
            "go",
            markersize=10,
        )
        ax.plot(
            goal_point[0] + 0.5,
            goal_point[1] + 0.5,
            "ro",
            markersize=10,
        )

    time_text = ax.text(
        0.05, 0.95, "", transform=ax.transAxes, color="red", fontsize=12
    )

    rects = []
    for (
        x_coords,
        y_coords,
    ) in zip(x_coords_list, y_coords_list):
        rects.append(plt.Rectangle((x_coords[0], y_coords[0]), 1, 1, facecolor="blue"))

    # Dynamic obstacles (initially set to out-of-bounds location)
    dynamic_obstacles_rects = [
        plt.Rectangle((-10, -10), 1, 1, facecolor="orange")
        for _ in input_data["dynamic_obstacles"]
    ]
    for obstacle in dynamic_obstacles_rects:
        ax.add_patch(obstacle)

    def init():
        for rect in rects:
            ax.add_patch(rect)
        time_text.set_text("")
        for obstacle in dynamic_obstacles_rects:
            obstacle.set_xy((-10, -10))
        return [*rects, time_text] + dynamic_obstacles_rects

    def animate(i):
        for (
            x_coords,
            y_coords,
            rect,
        ) in zip(x_coords_list, y_coords_list, rects):
            if len(x_coords) <= i:
                continue
            rect.set_xy((x_coords[i], y_coords[i]))
        time_text.set_text(f"Time: {times[i]}")

        # Update dynamic obstacles
        for idx, obstacle_data in enumerate(input_data["dynamic_obstacles"]):
            start_time, end_time = obstacle_data[1]
            if start_time <= times[i] and (end_time == -1 or end_time >= times[i]):
                dynamic_obstacles_rects[idx].set_xy(obstacle_data[0])
            else:
                dynamic_obstacles_rects[idx].set_xy(
                    (-10, -10)
                )  # Move it out of the plot

        return [*rects, time_text] + dynamic_obstacles_rects

    anim = animation.FuncAnimation(
        fig,
        animate,
        init_func=init,
        frames=len(times),
        interval=500,
        repeat=False,
        blit=True,
    )
    plt.show()


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--input", "-i", type=str, help="Input file path")
    parser.add_argument("--output", "-o", type=str, help="Output file path")
    args = parser.parse_args()
    with open(args.input, "r") as stream:
        input_data = yaml.load(stream, Loader=yaml.FullLoader)

    with open(args.output, "r") as stream:
        path_data = yaml.load(stream, Loader=yaml.FullLoader)

    cbs_animate(input_data, path_data)
