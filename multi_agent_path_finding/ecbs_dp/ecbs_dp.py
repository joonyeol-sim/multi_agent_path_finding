import heapq
import time
from itertools import combinations
from typing import List, Tuple

from multi_agent_path_finding.common.conflict import (
    Conflict,
    VertexConflict,
    EdgeConflict,
)
from multi_agent_path_finding.common.constraint import (
    Constraint,
    VertexConstraint,
    EdgeConstraint,
)
from multi_agent_path_finding.common.environment import Environment
from multi_agent_path_finding.common.point import Point
from multi_agent_path_finding.ecbs_dp.ct_node import CTNode
from multi_agent_path_finding.stastar_epsilon_dp.stastar_epsilon_dp import SpaceTimeAstarEpsilonDP


class EnhancedConflictBasedSearchDP:
    def __init__(
        self,
        start_points: List[Point],
        goal_points: List[Point],
        env: Environment,
        w: float,
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
        self.w = w

        self.open_set: List[CTNode] = list()
        self.focal_set: List[CTNode] = list()

        for start_point, goal_point in zip(start_points, goal_points):
            if env.dimension != len(start_point.__dict__.keys()):
                raise ValueError(f"Dimension does not match the length of start: {start_point}")
            if env.dimension != len(goal_point.__dict__.keys()):
                raise ValueError(f"Dimension does not match the length of goal: {goal_point}")
            if not self.is_valid_point(start_point, 0):
                raise ValueError(f"Start point is not valid: {start_point}")
            if not self.is_valid_point(goal_point, 0):
                raise ValueError(f"Goal point is not valid: {goal_point}")
        if not w:
            raise ValueError(f"w must be given")

    def plan(self):
        # generate root node
        root_node = CTNode(
            constraints={},
            solution=[],
            cost=0,
            f_mins=[],
            lower_bound=0,
            focal_heuristic=0,
            individual_planners=[],
        )

        root_node.individual_planners = [
            SpaceTimeAstarEpsilonDP(start_point, goal_point, self.env, self.w)
            for start_point, goal_point in zip(self.start_points, self.goal_points)
        ]

        for agent_id, individual_planner in enumerate(root_node.individual_planners):
            path, f_min = individual_planner.plan()
            if not path:
                print(f"Agent {agent_id} failed to find a path")
                return None
            root_node.solution.append(path)
            root_node.f_mins.append(f_min)
            self.env.reservation_table.append(path)

        root_node.cost = self.calculate_cost(root_node.solution)
        root_node.lower_bound = sum(root_node.f_mins)
        root_node.focal_heuristic = self.focal_heuristic(root_node.solution)

        # put root node into the priority queue
        heapq.heappush(self.open_set, root_node)
        heapq.heappush(self.focal_set, root_node)

        # set min_lower_bound to the lower bound of root node
        min_lower_bound = root_node.lower_bound

        ct_size = 0
        planning_avg_time = 0
        pruning_avg_time = 0
        copy_avg_time = 0
        generate_avg_time = 0

        while self.open_set:
            # update focal set if min_lower_bound has increased
            new_min_lower_bound = min([node.lower_bound for node in self.open_set])
            if min_lower_bound < new_min_lower_bound:
                for node in self.open_set:
                    if self.w * min_lower_bound <= node.cost <= self.w * new_min_lower_bound:
                        self.focal_set.append(node)
                min_lower_bound = new_min_lower_bound

            # select node from focal set
            cur_node = heapq.heappop(self.focal_set)
            print(f"Current node: {cur_node.focal_heuristic}, {cur_node.cost}")
            print(f"CT size: {ct_size}")
            self.open_set.remove(cur_node)

            # find the first conflict
            conflict = self.find_first_conflict(cur_node.solution)

            # if there is no conflict, return the solution
            if not conflict:
                return cur_node.solution, min_lower_bound

            generate_start_time = time.time()
            # if there is a conflict, generate two child nodes
            for agent_id in conflict.agent_ids:
                # if the agent has already passed the conflict time, ignore it
                if (type(conflict) == VertexConflict and len(cur_node.solution[agent_id]) <= conflict.time) or (
                    type(conflict) == EdgeConflict and len(cur_node.solution[agent_id]) <= conflict.times[1]
                ):
                    continue
                # generate child node from the current node
                copy_start_time = time.time()
                new_node = cur_node.deepcopy(agent_id)
                copy_avg_time += time.time() - copy_start_time

                pruning_start_time = time.time()
                # generate constraint from the conflict
                new_constraint = self.generate_constraint_from_conflict(agent_id, conflict)

                # pruning node from the new constraint
                pruning_node = None
                if type(conflict) == VertexConflict:
                    pruning_point = new_node.solution[agent_id][conflict.time][0]
                    pruning_time = new_node.solution[agent_id][conflict.time][1]

                else:
                    pruning_point = new_node.solution[agent_id][conflict.times[1]][0]
                    pruning_time = new_node.solution[agent_id][conflict.times[1]][1]

                for closed_node in new_node.individual_planners[agent_id].closed_set:
                    if closed_node.point == pruning_point and closed_node.time == pruning_time:
                        pruning_node = closed_node
                        break

                # add constraint to the child node
                new_node.constraints.setdefault(agent_id, []).append(new_constraint)

                # pruning the node from the new constraint
                self.prune_successor(
                    pruning_node,
                    new_node.individual_planners[agent_id].open_set,
                    new_node.individual_planners[agent_id].focal_set,
                    new_node.individual_planners[agent_id].closed_set,
                )
                pruning_node.parent.children.remove(pruning_node)
                pruning_node.parent = None
                pruning_avg_time += time.time() - pruning_start_time

                plan_start_time = time.time()
                # update reservation table
                self.env.reservation_table = new_node.solution
                self.env.reservation_table[agent_id] = []
                # generate new path for the agent that has the conflict
                result = new_node.individual_planners[agent_id].plan(constraints=new_node.constraints[agent_id])
                if not result:
                    continue
                new_node.solution[agent_id], new_f_min = result

                # update cost, f_mins, lower_bound, and focal_heuristic
                new_node.cost = self.calculate_cost(new_node.solution)
                new_node.f_mins[agent_id] = new_f_min
                new_node.lower_bound = sum(new_node.f_mins)
                new_node.focal_heuristic = self.focal_heuristic(new_node.solution)

                # add the child node to the open set
                heapq.heappush(self.open_set, new_node)

                # add the child node to the focal set if its lower bound is lower than w * min_lower_bound
                if new_node.cost <= self.w * min_lower_bound:
                    heapq.heappush(self.focal_set, new_node)

                ct_size += 1
                planning_avg_time += time.time() - plan_start_time
            generate_avg_time += time.time() - generate_start_time
            print(f"Planning avg time: {planning_avg_time / ct_size}")
            print(f"Pruning avg time: {pruning_avg_time / ct_size}")
            print(f"Copy avg time: {copy_avg_time / ct_size}")
            print(f"Generate avg time: {generate_avg_time / ct_size}")
        return None

    def prune_successor(self, node, open_set, focal_set, closed_set):
        while node.children:
            child = node.children.pop(0)
            child.parent = None
            self.prune_successor(child, open_set, focal_set, closed_set)

        if node in open_set:
            open_set.remove(node)
            # TODO: this is different from cbs
            if node in focal_set:
                focal_set.remove(node)
        else:
            closed_set.remove(node)
            # if not node.children:
            #     open_set.add(node)
            #     node.parent.children.append(node)
            # else:
            #     node.parent = None

    @staticmethod
    def generate_constraint_from_conflict(agent_id: int, conflict: Conflict) -> Constraint:
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
            return solution[agent_id][-1][0]
        else:
            return solution[agent_id][time][0]

    def calculate_cost(self, solution: List[List[Tuple[Point, int]]]) -> int:
        cost = 0
        for i in range(self.robot_num):
            cost += len(solution[i]) - 1
        return cost

    def find_first_conflict(self, solution: List[List[Tuple[Point, int]]]) -> Conflict:
        # Vertex Conflict
        for agent1, agent2 in combinations(range(self.robot_num), 2):
            max_time = max(len(solution[agent1]), len(solution[agent2]))
            for time in range(1, max_time):
                point1 = self.get_state(agent1, time, solution)
                point2 = self.get_state(agent2, time, solution)
                if point1 == point2:
                    return VertexConflict(
                        agent_ids=[agent1, agent2],
                        point=point1,
                        time=time,
                    )

                prev_point1 = self.get_state(agent1, time - 1, solution)
                next_point1 = self.get_state(agent1, time, solution)
                prev_point2 = self.get_state(agent2, time - 1, solution)
                next_point2 = self.get_state(agent2, time, solution)

                if prev_point1 == next_point2 and prev_point2 == next_point1:
                    return EdgeConflict(
                        agent_ids=[agent1, agent2],
                        points={
                            agent1: (prev_point1, next_point1),
                            agent2: (prev_point2, next_point2),
                        },
                        times=(time - 1, time),
                    )

    def focal_heuristic(self, solution: List[List[Tuple[Point, int]]]) -> int:
        num_of_conflicts = 0
        for agent1, agent2 in combinations(range(self.robot_num), 2):
            max_time = max(len(solution[agent1]), len(solution[agent2]))
            for time in range(1, max_time):
                point1 = self.get_state(agent1, time, solution)
                point2 = self.get_state(agent2, time, solution)
                if point1 == point2:
                    num_of_conflicts += 1

                prev_point1 = self.get_state(agent1, time - 1, solution)
                next_point1 = self.get_state(agent1, time, solution)
                prev_point2 = self.get_state(agent2, time - 1, solution)
                next_point2 = self.get_state(agent2, time, solution)

                if prev_point1 == next_point2 and prev_point2 == next_point1:
                    num_of_conflicts += 1

        return num_of_conflicts

    def is_valid_point(self, point: Point, time: int) -> bool:
        if not self.is_valid_space(point):
            return False
        for obstacle in self.env.obstacles:
            if obstacle.is_colliding(point=point, time=time):
                return False
        return True

    def is_valid_space(self, point: Point) -> bool:
        for i, coordinate in enumerate(point.__dict__.values()):
            if coordinate < 0 or coordinate >= self.env.space_limit[i]:
                return False
        return True
