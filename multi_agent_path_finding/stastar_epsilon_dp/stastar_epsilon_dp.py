from typing import List, Set, Tuple

from multi_agent_path_finding.common.constraint import (
    Constraint,
    VertexConstraint,
    EdgeConstraint,
)
from multi_agent_path_finding.common.environment import Environment
from multi_agent_path_finding.common.point import Point
from multi_agent_path_finding.stastar_epsilon_dp.node import Node
import matplotlib.pyplot as plt

fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")


class SpaceTimeAstarEpsilonDP:
    def __init__(
        self, start_point: Point, goal_point: Point, env: Environment, w: float
    ):
        self.env = env
        self.start_point = start_point
        self.goal_point = goal_point
        self.w = w

        self.open_set: Set[Node] = set()
        self.focal_set: Set[Node] = set()
        self.closed_set: Set[Node] = set()

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
        if not w:
            raise ValueError(f"w must be given")

    def plan(
        self, constraints: List[Constraint] = None
    ) -> Tuple[List[Tuple[Point, int]], int] | None:
        if constraints is None:
            start_node = Node(self.start_point, 0)
            start_node.parent = None
            start_node.g_score = 0
            start_node.h_score = self.heuristic(start_node)
            start_node.f_score = start_node.g_score + start_node.h_score
            start_node.d_score = 0

            self.open_set.add(start_node)
            self.focal_set.add(start_node)
            min_f_score = start_node.f_score
        else:
            min_f_score = min([node.f_score for node in self.open_set])

        while self.open_set:
            # update focal set if min_f_score has increased
            new_min_f_score = min([node.f_score for node in self.open_set])
            if min_f_score < new_min_f_score:
                for node in self.open_set:
                    if self.w * min_f_score <= node.f_score <= self.w * new_min_f_score:
                        self.focal_set.add(node)
                min_f_score = new_min_f_score

            # select node from focal set
            current = min(self.focal_set)
            self.open_set.remove(current)
            self.focal_set.remove(current)
            self.closed_set.add(current)

            # check if current node is at goal
            if current.point == self.goal_point:
                return self.reconstruct_path(current), min_f_score

            # get neighbors
            neighbors = self.get_neighbors(current, constraints)
            for neighbor in neighbors:
                if neighbor in self.closed_set:
                    continue

                if neighbor not in self.open_set:
                    self.open_set.add(neighbor)
                    # If neighbor is not in open set, we set its parent
                    neighbor.parent = current
                    current.children.append(neighbor)

                    neighbor.g_score = current.g_score + 1
                    neighbor.h_score = self.heuristic(neighbor)
                    neighbor.f_score = neighbor.g_score + neighbor.h_score
                    neighbor.d_score = (
                        current.d_score
                        + self.focal_vertex_heuristic(neighbor)
                        + self.focal_edge_heuristic(current, neighbor)
                    )
                    if neighbor.f_score <= self.w * min_f_score:
                        self.focal_set.add(neighbor)

                if current.g_score + 1 < neighbor.g_score:
                    # If neighbor is in open set, we remove it from its parent's children
                    neighbor.parent.children.remove(neighbor)
                    # update neighbor's parent
                    neighbor.parent = current
                    current.children.append(neighbor)

                    neighbor.g_score = current.g_score + 1
                    neighbor.h_score = self.heuristic(neighbor)
                    neighbor.f_score = neighbor.g_score + neighbor.h_score
                    neighbor.d_score = (
                        current.d_score
                        + self.focal_vertex_heuristic(neighbor)
                        + self.focal_edge_heuristic(current, neighbor)
                    )

            self.visualize(current, self.open_set, self.closed_set, constraints)

        return None

    def visualize(self, node, open_set: Set[Node], closed_set: Set[Node], constraints):
        # Clear the plot
        ax.clear()
        time_limit = max(10, max([node.time for node in open_set | closed_set]))

        # Plot obstacles
        for time in range(time_limit):
            ax.scatter(
                [obstacle.point.x for obstacle in self.env.obstacles],
                [obstacle.point.y for obstacle in self.env.obstacles],
                [time for obstacle in self.env.obstacles],
                c="black",
                marker="x",
            )

        # Plot constraints
        if constraints is not None:
            for constraint in constraints:
                if isinstance(constraint, VertexConstraint):
                    ax.scatter(
                        constraint.point.x,
                        constraint.point.y,
                        constraint.time,
                        c="r",
                        marker="x",
                    )
                elif isinstance(constraint, EdgeConstraint):
                    ax.plot(
                        [constraint.points[0].x, constraint.points[1].x],
                        [constraint.points[0].y, constraint.points[1].y],
                        [constraint.times[0], constraint.times[1]],
                        c="r",
                    )

        # Plot open set
        ax.scatter(
            [node.point.x for node in open_set],
            [node.point.y for node in open_set],
            [node.time for node in open_set],
            c="b",
            marker="x",
            label="Open Set",
        )

        # Plot closed set
        ax.scatter(
            [node.point.x for node in closed_set],
            [node.point.y for node in closed_set],
            [node.time for node in closed_set],
            c="r",
            marker="o",
            label="Closed Set",
        )

        # Plot current node
        ax.scatter(
            node.point.x,
            node.point.y,
            node.time,
            c="g",
            marker="o",
            label="Current Node",
        )

        # Plot start and goal points without (x, o) markers
        ax.scatter(
            self.start_point.x,
            self.start_point.y,
            0,
            c="g",
            marker="^",
            label="Start Point",
        )

        ax.scatter(
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
                ax.plot(
                    [node.point.x, node.parent.point.x],
                    [node.point.y, node.parent.point.y],
                    [node.time, node.parent.time],
                    c="y",
                )

        # Setting labels and title
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("T")
        ax.set_xlim([0, self.env.space_limit[0]])
        ax.set_ylim([0, self.env.space_limit[1]])
        # time_limit = 10
        ax.set_zlim([0, time_limit])
        ax.set_title("3D Grid Map Visualization")
        ax.legend()

        # Show the plot
        plt.pause(0.01)

    def heuristic(self, node) -> int:
        # return manhattan distance
        return node.point.manhattan_distance(self.goal_point)

    def focal_vertex_heuristic(self, node) -> int:
        num_of_conflicts = 0
        for path in self.env.reservation_table:
            if not path:
                continue
            if len(path) <= node.time:
                other_node = path[-1]
            else:
                other_node = path[node.time]
            other_point, other_time = other_node
            if node.point == other_point and node.time == other_time:
                num_of_conflicts += 1
        return num_of_conflicts

    def focal_edge_heuristic(self, prev_node, next_node) -> int:
        num_of_conflicts = 0
        for path in self.env.reservation_table:
            if len(path) <= next_node.time:
                continue

            other_prev_point, other_prev_time = path[next_node.time - 1]
            other_next_point, other_next_time = path[next_node.time]

            if (
                prev_node.point == other_next_point
                and next_node.point == other_prev_point
            ):
                num_of_conflicts += 1
        return num_of_conflicts

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
