from heapq import heappush, heappop
from typing import List, Tuple
from copy import deepcopy
from itertools import combinations


from common.environment import Environment
from common.point import Point
from common.conflict import Conflict, VertexConflict, EdgeConflict
from common.constraint import (
    Constraint,
    VertexConstraint,
    EdgeConstraint,
)
from cbs.ct_node import CTNode
from stastar.stastar import SpaceTimeAstar


class ConflictBasedSearch:
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

        self.open_set: List[CTNode] = []
        self.individual_planners = [
            SpaceTimeAstar(start_point, goal_point, env)
            for start_point, goal_point in zip(start_points, goal_points)
        ]

    def plan(self):
        root_node = CTNode(
            constraints=[],
            solution=[],
            cost=0,
        )
        for agent_id, individual_planner in enumerate(self.individual_planners):
            path = individual_planner.plan()
            if not path:
                print(f"Agent {agent_id} failed to find a path")
                return None
            root_node.solution.append(path)

        root_node.cost = self.calculate_cost(root_node.solution)
        heappush(self.open_set, root_node)
        while self.open_set:
            cur_node = heappop(self.open_set)
            conflict = self.find_first_conflict(cur_node.solution)
            if not conflict:
                return cur_node.solution
            for agent_id in conflict.agent_ids:
                new_node = deepcopy(cur_node)
                constraint = self.generate_constraint_from_conflict(agent_id, conflict)
                new_node.constraints.append(constraint)
                # TODO: change constraints to dict
                new_node.solution[agent_id] = self.individual_planners[agent_id].plan(
                    constraints=new_node.constraints
                )
                if not new_node.solution[agent_id]:
                    continue
                new_node.cost = self.calculate_cost(new_node.solution)
                heappush(self.open_set, new_node)
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
