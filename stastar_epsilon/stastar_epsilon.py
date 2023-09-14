from typing import List, Set, Tuple

from common.environment import Environment
from common.point import Point
from common.constraint import Constraint, VertexConstraint, EdgeConstraint
from stastar_epsilon.node import Node


class SpaceTimeAstarEpsilon:
    def __init__(
        self, start_point: Point, goal_point: Point, env: Environment, w: float
    ):
        self.env = env
        self.start_point = start_point
        self.goal_point = goal_point
        self.w = w
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

    def plan(
        self, constraints: List[Constraint] = None
    ) -> tuple[list[tuple[Point, int]], int] | None:
        open_set: Set[Node] = set()
        focal_set: Set[Node] = set()
        closed_set: Set[Node] = set()

        start_node = Node(self.start_point, 0)
        start_node.parent = None
        start_node.g_score = 0
        start_node.h_score = self.heuristic(start_node)
        start_node.f_score = start_node.g_score + start_node.h_score
        start_node.d_score = 0

        open_set.add(start_node)
        focal_set.add(start_node)
        min_f_score = start_node.f_score

        while open_set:
            # update focal set if min_f_score has increased
            new_min_f_score = min([node.f_score for node in open_set])
            if min_f_score < new_min_f_score:
                for node in open_set:
                    if self.w * min_f_score <= node.f_score <= self.w * new_min_f_score:
                        focal_set.add(node)
                min_f_score = new_min_f_score

            # select node from focal set
            current = min(focal_set)
            open_set.remove(current)
            focal_set.remove(current)
            closed_set.add(current)

            # check if current node is at goal
            if current.point == self.goal_point:
                return self.reconstruct_path(current), min(open_set).f_score

            # get neighbors
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
                    neighbor.d_score = self.focal_vertex_heuristic(
                        neighbor
                    ) + self.focal_edge_heuristic(current, neighbor)
                    if neighbor.f_score <= self.w * min_f_score:
                        focal_set.add(neighbor)

                if current.g_score + 1 < neighbor.g_score:
                    neighbor.parent = current
                    neighbor.g_score = current.g_score + 1
                    neighbor.h_score = self.heuristic(neighbor)
                    neighbor.f_score = neighbor.g_score + neighbor.h_score
                    neighbor.d_score = self.focal_vertex_heuristic(
                        neighbor
                    ) + self.focal_edge_heuristic(current, neighbor)

        return None

    def heuristic(self, node) -> int:
        # return manhattan distance
        return node.point.manhattan_distance(self.goal_point)

    def focal_vertex_heuristic(self, node) -> int:
        num_of_conflicts = 0
        for path in self.env.reservation_table:
            if len(path) <= node.time:
                other_node = path[-1]
            else:
                other_node = path[node.time]
            if node.point == other_node.point:
                num_of_conflicts += 1
        return num_of_conflicts

    def focal_edge_heuristic(self, prev_node, next_node) -> int:
        num_of_conflicts = 0
        for path in self.env.reservation_table:
            if len(path) <= next_node.time:
                continue

            other_prev_node = path[next_node.time - 1]
            other_next_node = path[next_node.time]

            if (
                prev_node.point == other_next_node.point
                and next_node.point == other_prev_node.point
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

        # wait action
        if self.is_valid_point(
            node.point, node.time + 1
        ) and self.is_valid_given_constraints(
            node.point, node.point, node.time, node.time + 1, constraints
        ):
            neighbors.append(Node(node.point, node.time + 1))
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
