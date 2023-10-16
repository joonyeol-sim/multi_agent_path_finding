import math

import matplotlib.animation as animation
import matplotlib.pyplot as plt
import yaml
import numpy as np


def cbs_animate(input_data, path_data, interp_steps=100):
    # Calculate interpolated coordinates for smoother transitions
    x_coords_list_interp = []
    y_coords_list_interp = []
    times_interp = []

    for path in path_data:
        x_coords = [point[0][0] for point in path]
        y_coords = [point[0][1] for point in path]
        times = [point[1] for point in path]

        x_coords_new = []
        y_coords_new = []
        times_new = []
        for i in range(len(x_coords) - 1):
            for j in range(interp_steps):
                fraction = j / interp_steps
                x_coords_new.append(
                    x_coords[i] + fraction * (x_coords[i + 1] - x_coords[i])
                )
                y_coords_new.append(
                    y_coords[i] + fraction * (y_coords[i + 1] - y_coords[i])
                )
                times_new.append(times[i] + fraction * (times[i + 1] - times[i]))

        x_coords_list_interp.append(x_coords_new + [x_coords[-1]])
        y_coords_list_interp.append(y_coords_new + [y_coords[-1]])
        times_interp.extend(times_new + [times[-1]])

    fig, ax = plt.subplots(figsize=(10, 6))
    ax.set_xlim(0, input_data["space_limits"][0])
    ax.set_ylim(0, input_data["space_limits"][1])
    ax.grid(True, which="both", linestyle="-", linewidth=1)
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

    circles = []
    agent_texts = []
    for index, (
        x_coords,
        y_coords,
    ) in enumerate(zip(x_coords_list_interp, y_coords_list_interp)):
        circle = plt.Circle(
            (x_coords[0] + 0.5, y_coords[0] + 0.5), math.sqrt(2) / 4, facecolor="blue"
        )
        circles.append(circle)
        agent_text = ax.text(
            x_coords[0] + 0.5,
            y_coords[0] + 0.5,
            str(index),
            ha="center",
            va="center",
            color="white",
        )
        agent_texts.append(agent_text)

    # Dynamic obstacles (initially set to out-of-bounds location)
    dynamic_obstacles_rects = [
        plt.Rectangle((-10, -10), 1, 1, facecolor="orange")
        for _ in input_data["dynamic_obstacles"]
    ]
    for obstacle in dynamic_obstacles_rects:
        ax.add_patch(obstacle)

    def init():
        for circle in circles:
            ax.add_patch(circle)
        time_text.set_text("")
        for obstacle in dynamic_obstacles_rects:
            obstacle.set_xy((-10, -10))
        for agent_text in agent_texts:
            agent_text.set_visible(True)
        return [*circles, time_text, *agent_texts] + dynamic_obstacles_rects

    def animate(i):
        for index, (x_coords, y_coords, circle, agent_text) in enumerate(
            zip(x_coords_list_interp, y_coords_list_interp, circles, agent_texts)
        ):
            if i < len(x_coords):
                circle.set_center((x_coords[i] + 0.5, y_coords[i] + 0.5))
                agent_text.set_position((x_coords[i] + 0.5, y_coords[i] + 0.5))
        if i < len(times_interp):
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

        return [*circles, time_text, *agent_texts] + dynamic_obstacles_rects

    anim = animation.FuncAnimation(
        fig,
        animate,
        init_func=init,
        frames=len(times_interp),
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

    cbs_animate(input_data, path_data)
