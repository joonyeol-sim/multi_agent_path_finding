import matplotlib.animation as animation
import matplotlib.pyplot as plt
import yaml
import numpy as np


def animate(input_data, path_data, interp_steps=10):
    # Calculate interpolated coordinates for smoother transitions
    x_coords = [point[0][0] for point in path_data]
    y_coords = [point[0][1] for point in path_data]
    times = [point[1] for point in path_data]

    x_coords_interp = []
    y_coords_interp = []
    times_interp = []

    for i in range(len(x_coords) - 1):
        for j in range(interp_steps):
            fraction = j / interp_steps
            x_coords_interp.append(
                x_coords[i] + fraction * (x_coords[i + 1] - x_coords[i])
            )
            y_coords_interp.append(
                y_coords[i] + fraction * (y_coords[i + 1] - y_coords[i])
            )
            times_interp.append(times[i] + fraction * (times[i + 1] - times[i]))

    x_coords_interp.append(x_coords[-1])
    y_coords_interp.append(y_coords[-1])
    times_interp.append(times[-1])

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
    ax.plot(
        input_data["start_point"][0] + 0.5,
        input_data["start_point"][1] + 0.5,
        "go",
        markersize=10,
    )
    ax.plot(
        input_data["goal_point"][0] + 0.5,
        input_data["goal_point"][1] + 0.5,
        "ro",
        markersize=10,
    )

    time_text = ax.text(
        0.05, 0.95, "", transform=ax.transAxes, color="red", fontsize=12
    )

    circle = plt.Circle(
        (x_coords_interp[0] + 0.5, y_coords_interp[0] + 0.5), 0.5, facecolor="blue"
    )

    # Dynamic obstacles (initially set to out-of-bounds location)
    dynamic_obstacles_rects = [
        plt.Rectangle((-10, -10), 1, 1, facecolor="orange")
        for _ in input_data["dynamic_obstacles"]
    ]
    for obstacle in dynamic_obstacles_rects:
        ax.add_patch(obstacle)

    def init():
        ax.add_patch(circle)
        time_text.set_text("")
        for obstacle in dynamic_obstacles_rects:
            obstacle.set_xy((-10, -10))
        return [circle, time_text] + dynamic_obstacles_rects

    def animate(i):
        circle.set_center((x_coords_interp[i] + 0.5, y_coords_interp[i] + 0.5))
        time_text.set_text(f"Time: {times_interp[i]}")

        # Update dynamic obstacles
        for idx, obstacle_data in enumerate(input_data["dynamic_obstacles"]):
            start_time, end_time = obstacle_data[1]
            if start_time <= times_interp[i] and (
                end_time == -1 or end_time >= times_interp[i]
            ):
                dynamic_obstacles_rects[idx].set_xy(obstacle_data[0])
            else:
                dynamic_obstacles_rects[idx].set_xy(
                    (-10, -10)
                )  # Move it out of the plot

        return [circle, time_text] + dynamic_obstacles_rects

    anim = animation.FuncAnimation(
        fig,
        animate,
        init_func=init,
        frames=len(x_coords_interp),
        interval=500 / interp_steps,  # Adjusted interval for smoother transitions
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

    animate(input_data, path_data)
