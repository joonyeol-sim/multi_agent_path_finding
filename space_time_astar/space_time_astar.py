from typing import List, Set, Tuple
from space_time_astar.node import Node
from space_time_astar.environment import Environment


class SpaceTimeAstar:
    def __init__(self, start_point: List[int], goal_point: List[int], env: Environment):
        self.env = env
        self.start_point = start_point
        self.goal_point = goal_point
        if env.dimension != len(start_point):
            raise ValueError(
                f"Dimension does not match the length of start: {start_point}"
            )
        if env.dimension != len(goal_point):
            raise ValueError(
                f"Dimension does not match the length of goal: {goal_point}"
            )

    def plan(self):
        open_set: Set[Node] = set()
        open_set.add(Node(self.start_point, 0))
        while open_set:
            current = min(open_set)
            open_set.remove(current)
            if current.point == self.goal_point:
                return self.reconstruct_path(current)
            neighbors = self.get_neighbors(current)
            for neighbor in neighbors:
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

        return None

    def heuristic(self, node) -> int:
        # return manhattan distance
        return sum(
            [abs(node.point[i] - self.goal_point[i]) for i in range(self.env.dimension)]
        )

    @staticmethod
    def reconstruct_path(node: Node) -> List[tuple[List[int], int]]:
        path: List[Tuple[List[int], int]] = [(node.point, node.time)]
        while node.parent is not None:
            path.append((node.parent.point, node.parent.time))
            node = node.parent
        return path[::-1]

    def get_neighbors(self, node: Node) -> List[Node]:
        neighbors: List[Node] = []
        for i in range(self.env.dimension):
            # move action
            for j in [-1, 1]:
                neighbor_point = node.point.copy()
                neighbor_point[i] += j
                if self.is_valid_point(neighbor_point, node.time + 1):
                    neighbors.append(Node(neighbor_point, node.time + 1))
            # wait action
            if self.is_valid_point(node.point, node.time + 1):
                neighbors.append(Node(node.point, node.time + 1))
        return neighbors

    def is_valid_point(self, point: List[int], time: int) -> bool:
        if not self.is_valid_space(point):
            return False
        for obstacle in self.env.obstacles:
            if obstacle.is_colliding(point=point, time=time):
                return False
        return True

    def is_valid_space(self, point: List[int]) -> bool:
        for i in range(self.env.dimension):
            if point[i] < 0 or point[i] > self.env.space_limit[i]:
                return False
        return True
