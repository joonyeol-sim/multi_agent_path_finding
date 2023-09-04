from typing import List, Set, Tuple

from common.environment import Environment
from common.point import Point
from common.constraint import Constraint, VertexConstraint, EdgeConstraint
from space_time_astar_fast.node import Node


class SpaceTimeAstarFast:
    def __init__(self, start_point: Point, goal_point: Point, env: Environment):
        self.env = env
        self.start_point = start_point
        self.goal_point = goal_point
        self.node_list: List[Node] = []
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
        self, agent_id: int, constraints: List[Constraint] = None
    ) -> List[Tuple[Point, int]] | None:
        if constraints is not None:
            for constraint in constraints:
                if constraint.agent_id != agent_id:
                    continue
                if isinstance(constraint, VertexConstraint):
                    to_be_pruned_node: Node = self.find_node_given_vertex_constraint(
                        constraint
                    )
                elif isinstance(constraint, EdgeConstraint):
                    to_be_pruned_node: Node = self.find_node_given_edge_constraint(
                        constraint
                    )
                else:
                    raise ValueError(f"Constraint type not supported: {constraint}")
                self.prune_preorder(to_be_pruned_node)
        open_set: Set[Node] = set()
        start_node = Node(self.start_point, 0)
        start_node.children = []
        open_set.add(start_node)
        self.node_list.append(start_node)

        while open_set:
            current = min(open_set)
            open_set.remove(current)
            if current.point == self.goal_point:
                return self.reconstruct_path(current)
            neighbors = self.get_neighbors(current, agent_id, constraints)
            for neighbor in neighbors:
                if neighbor not in open_set:
                    neighbor.children = []
                    neighbor.parent = current
                    current.children.append(neighbor)
                    open_set.add(neighbor)
                    self.node_list.append(neighbor)
                    neighbor.g_score = current.g_score + 1
                    neighbor.h_score = self.heuristic(neighbor)
                    neighbor.f_score = neighbor.g_score + neighbor.h_score

                if current.g_score + 1 < neighbor.g_score:
                    neighbor.parent.children.remove(neighbor)
                    neighbor.parent = current
                    current.children.append(neighbor)
                    neighbor.g_score = current.g_score + 1
                    neighbor.h_score = self.heuristic(neighbor)
                    neighbor.f_score = neighbor.g_score + neighbor.h_score

        return None

    def find_node_given_vertex_constraint(self, constraint: VertexConstraint) -> Node:
        for node in self.node_list:
            if node.point == constraint.point and node.time == constraint.time:
                return node

    def find_node_given_edge_constraint(self, constraint: EdgeConstraint) -> Node:
        for node in self.node_list:
            if node.point == constraint.points[0] and node.time == constraint.times[0]:
                return node

    def prune_preorder(self, prune_node: Node) -> None:
        if prune_node is None:
            return
        for child in prune_node.children:
            self.prune_preorder(child)
        if prune_node.parent is not None:
            self.node_list.remove(prune_node)
            prune_node.parent.children.remove(prune_node)
            prune_node.parent = None

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
        self, node: Node, agent_id: int, constraints: List[Constraint] = None
    ) -> List[Node]:
        neighbors: List[Node] = []
        # move action
        for neighbor_point in node.point.get_neighbor_points():
            if self.is_valid_point(
                neighbor_point, node.time + 1
            ) and self.is_valid_given_constraints(
                agent_id,
                node.point,
                neighbor_point,
                node.time,
                node.time + 1,
                constraints,
            ):
                neighbors.append(Node(neighbor_point, node.time + 1))

        # wait action
        if self.is_valid_point(
            node.point, node.time + 1
        ) and self.is_valid_given_constraints(
            agent_id, node.point, node.point, node.time, node.time + 1, constraints
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
        agent_id: int,
        prev_point: Point,
        next_point: Point,
        prev_time: int,
        next_time: int,
        constraints: List[Constraint],
    ) -> bool:
        if constraints is not None:
            for constraint in constraints:
                if constraint.agent_id != agent_id:
                    continue
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
