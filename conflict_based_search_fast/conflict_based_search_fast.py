from queue import PriorityQueue
from typing import List, Tuple
from itertools import combinations


from common.environment import Environment
from common.point import Point
from common.conflict import Conflict, VertexConflict, EdgeConflict
from common.constraint import (
    Constraint,
    VertexConstraint,
    EdgeConstraint,
)
from conflict_based_search.constraint_tree_node import CTNode
from space_time_astar_fast.space_time_astar_fast import SpaceTimeAstarFast


class ConflictBasedSearchFast:
    def __init__(
        self, start_points: List[Point], goal_points: List[Point], env: Environment
    ):
        # check if the length of start_points and goal_points are the same
        if len(start_points) != len(goal_points):
            raise ValueError(
                f"Length of start_points and goal_points are not the same: {len(start_points)} != {len(goal_points)}"
            )

        self.start_points = start_points
        self.goal_points = goal_points
        self.robot_num = len(start_points)
        self.env = env

        self.constraints_tree = PriorityQueue()
        self.individual_planners = [
            SpaceTimeAstarFast(start_point, goal_point, env)
            for start_point, goal_point in zip(start_points, goal_points)
        ]

    def plan(self):
        constraints: List[Constraint] = []
        solution: List[List[Tuple[Point, int]]] = [
            self.individual_planners[agent_id].plan(agent_id)
            for agent_id in range(len(self.individual_planners))
        ]
        cost: int = self.calculate_cost(solution)
        root_node = CTNode(constraints, solution, cost)
        self.constraints_tree.put(root_node)
        while not self.constraints_tree.empty():
            cur_node = self.constraints_tree.get()
            conflict = self.find_first_conflict(cur_node.solution)
            if not conflict:
                return cur_node.solution
            for agent_id in conflict.agent_ids:
                constraint = self.generate_constraint_from_conflict(agent_id, conflict)

                new_constraints = cur_node.constraints.copy()
                new_constraints.append(constraint)
                new_solution = cur_node.solution.copy()
                new_solution[agent_id] = self.individual_planners[agent_id].plan(
                    agent_id=agent_id,
                    constraints=new_constraints,
                )
                if not new_solution[agent_id]:
                    continue
                new_cost = self.calculate_cost(new_solution)
                new_node = CTNode(new_constraints, new_solution, new_cost)
                self.constraints_tree.put(new_node)
        return None

    @staticmethod
    def generate_constraint_from_conflict(
        agent_id: int, conflict: Conflict
    ) -> Constraint:
        if isinstance(conflict, VertexConflict):
            return VertexConstraint(
                agent_id=agent_id,
                point=conflict.point,
                time=conflict.time,
            )
        elif isinstance(conflict, EdgeConflict):
            return EdgeConstraint(
                agent_id=agent_id,
                points=conflict.points[agent_id],
                times=conflict.times,
            )
        else:
            raise ValueError(f"Unknown conflict type: {type(conflict)}")

    @staticmethod
    def get_state(agent_id: int, time: int, solution: List[List[Tuple[Point, int]]]):
        if time >= len(solution[agent_id]):
            return solution[agent_id][-1]
        else:
            return solution[agent_id][time]

    def calculate_cost(self, solution: List[List[Tuple[Point, int]]]) -> int:
        cost = 0
        for i in range(self.robot_num):
            cost += len(solution[i])
        return cost

    def find_first_conflict(self, solution: List[List[Tuple[Point, int]]]) -> Conflict:
        # Vertex Conflict
        for agent1, agent2 in combinations(range(self.robot_num), 2):
            max_time = max(len(solution[agent1]), len(solution[agent2]))
            for time in range(max_time):
                point1, time1 = self.get_state(agent1, time, solution)
                point2, time2 = self.get_state(agent2, time, solution)
                if point1 == point2 and time1 == time2:
                    return VertexConflict(
                        agent_ids=[agent1, agent2],
                        point=point1,
                        time=time1,
                    )

        # Edge Conflict
        for agent1, agent2 in combinations(range(self.robot_num), 2):
            max_time = max(len(solution[agent1]), len(solution[agent2]))
            for time in range(max_time - 1):
                prev_point1, prev_time1 = self.get_state(agent1, time, solution)
                next_point1, next_time1 = self.get_state(agent1, time + 1, solution)
                prev_point2, prev_time2 = self.get_state(agent2, time, solution)
                next_point2, next_time2 = self.get_state(agent2, time + 1, solution)

                if (
                    prev_point1 == next_point2
                    and prev_point2 == next_point1
                    and prev_time1 == prev_time2
                    and next_time1 == next_time2
                ):
                    return EdgeConflict(
                        agent_ids=[agent1, agent2],
                        points={
                            agent1: (prev_point1, next_point1),
                            agent2: (prev_point2, next_point2),
                        },
                        times=(prev_time1, next_time1),
                    )
