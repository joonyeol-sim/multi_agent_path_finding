from typing import List, Set, Tuple

from multi_agent_path_finding.common.environment import Environment
from multi_agent_path_finding.common.point import Point
from multi_agent_path_finding.common.constraint import (
    Constraint,
    VertexConstraint,
    EdgeConstraint,
)
from multi_agent_path_finding.stastar.node import Node
import matplotlib.pyplot as plt


class SpaceTimeAstar:
    def __init__(self, start_point: Point, goal_point: Point, env: Environment):
        self.env = env
        self.start_point = start_point
        self.goal_point = goal_point
        if env.dimension != len(start_point.__dict__.keys()):
            raise ValueError(
                f"Dimension does not match the length of start: {start_point}"
            )
        if env.dimension != len(goal_point.__dict__.keys()):
            raise ValueError(
                f"Dimension does not match the length of goal: {goal_point}"
            )
        if not self.is_valid_point(start_point, 0):
            raise ValueError(f"Start point is not valid: {start_point}")
        if not self.is_valid_point(goal_point, 0):
            raise ValueError(f"Goal point is not valid: {goal_point}")

        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection="3d")

    def plan(
        self, constraints: List[Constraint] = None
    ) -> List[Tuple[Point, int]] | None:
        open_set: Set[Node] = set()
        closed_set: Set[Node] = set()
        open_set.add(Node(self.start_point, 0))
        while open_set:
            current = min(open_set)
            open_set.remove(current)
            closed_set.add(current)
            if current.point == self.goal_point:
                # self.visualize(current, open_set, closed_set)
                # plt.show()
                return self.reconstruct_path(current)
            neighbors = self.get_neighbors(current, constraints)
            for neighbor in neighbors:
                if neighbor in closed_set:
                    continue

                if neighbor not in open_set:
                    open_set.add(neighbor)
                    neighbor.parent = current
                    neighbor.g_score = current.g_score + 1
                    neighbor.h_score = self.heuristic(neighbor)
                    neighbor.f_score = neighbor.g_score + neighbor.h_score

                if current.g_score + 1 < neighbor.g_score:
                    neighbor.parent = current
                    neighbor.g_score = current.g_score + 1
                    neighbor.h_score = self.heuristic(neighbor)
                    neighbor.f_score = neighbor.g_score + neighbor.h_score

            # self.visualize(current, open_set, closed_set)

        return None

    def visualize(self, node, open_set: Set[Node], closed_set: Set[Node]):
        # Clear the plot
        self.ax.clear()
        time_limit = max(10, max([node.time for node in open_set | closed_set]))

        # Plot obstacles
        for time in range(time_limit):
            self.ax.scatter(
                [obstacle.point.x for obstacle in self.env.obstacles],
                [obstacle.point.y for obstacle in self.env.obstacles],
                [time for obstacle in self.env.obstacles],
                c="black",
                marker="x",
            )

        # Plot open set
        self.ax.scatter(
            [node.point.x for node in open_set],
            [node.point.y for node in open_set],
            [node.time for node in open_set],
            c="b",
            marker="x",
            label="Open Set",
        )

        # Plot closed set
        self.ax.scatter(
            [node.point.x for node in closed_set],
            [node.point.y for node in closed_set],
            [node.time for node in closed_set],
            c="r",
            marker="o",
            label="Closed Set",
        )

        # Plot current node
        self.ax.scatter(
            node.point.x,
            node.point.y,
            node.time,
            c="g",
            marker="o",
            label="Current Node",
        )

        # Plot start and goal points without (x, o) markers
        self.ax.scatter(
            self.start_point.x,
            self.start_point.y,
            0,
            c="g",
            marker="^",
            label="Start Point",
        )

        self.ax.scatter(
            self.goal_point.x,
            self.goal_point.y,
            0,
            c="r",
            marker="^",
            label="Goal Point",
        )

        # Plot tree edges
        for node in open_set | closed_set:
            if node.parent is not None:
                self.ax.plot(
                    [node.point.x, node.parent.point.x],
                    [node.point.y, node.parent.point.y],
                    [node.time, node.parent.time],
                    c="y",
                )

        # Setting labels and title
        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")
        self.ax.set_zlabel("T")
        self.ax.set_xlim([0, self.env.space_limit[0]])
        self.ax.set_ylim([0, self.env.space_limit[1]])
        # time_limit = 10
        self.ax.set_zlim([0, time_limit])
        self.ax.set_title("3D Grid Map Visualization")
        self.ax.legend()

        # Show the plot
        plt.pause(0.5)

    def heuristic(self, node) -> int:
        # return manhattan distance
        return node.point.manhattan_distance(self.goal_point)

    @staticmethod
    def reconstruct_path(node: Node) -> List[Tuple[Point, int]]:
        path: List[Tuple[Point, int]] = [(node.point, node.time)]
        while node.parent is not None:
            path.append((node.parent.point, node.parent.time))
            node = node.parent
        return path[::-1]

    def get_neighbors(
        self, node: Node, constraints: List[Constraint] = None
    ) -> List[Node]:
        neighbors: List[Node] = []
        # move action
        for neighbor_point in node.point.get_neighbor_points():
            if self.is_valid_point(
                neighbor_point, node.time + 1
            ) and self.is_valid_given_constraints(
                node.point, neighbor_point, node.time, node.time + 1, constraints
            ):
                neighbors.append(Node(neighbor_point, node.time + 1))

        return neighbors

    def is_valid_point(self, point: Point, time: int) -> bool:
        if not self.is_valid_space(point):
            return False
        for obstacle in self.env.obstacles:
            if obstacle.is_colliding(point=point, time=time):
                return False
        return True

    @staticmethod
    def is_valid_given_constraints(
        prev_point: Point,
        next_point: Point,
        prev_time: int,
        next_time: int,
        constraints: List[Constraint],
    ) -> bool:
        if constraints is not None:
            for constraint in constraints:
                if isinstance(constraint, VertexConstraint):
                    if constraint.point == next_point and constraint.time == next_time:
                        return False
                elif isinstance(constraint, EdgeConstraint):
                    if (
                        constraint.points[0] == prev_point
                        and constraint.points[1] == next_point
                        and constraint.times[0] == prev_time
                        and constraint.times[1] == next_time
                    ):
                        return False
        return True

    def is_valid_space(self, point: Point) -> bool:
        for i, coordinate in enumerate(point.__dict__.values()):
            if coordinate < 0 or coordinate >= self.env.space_limit[i]:
                return False
        return True
