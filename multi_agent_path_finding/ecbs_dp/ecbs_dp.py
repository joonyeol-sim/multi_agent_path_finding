from copy import deepcopy, copy
from itertools import combinations
from typing import List, Tuple, Set
import heapq
import matplotlib.pyplot as plt

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
from multi_agent_path_finding.stastar_epsilon_dp.stastar_epsilon_dp import (
    SpaceTimeAstarEpsilonDP,
)


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

        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection="3d")

        for start_point, goal_point in zip(start_points, goal_points):
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

    def plan(self):
        # generate root node
        root_node = CTNode(
            constraints={},
            solution=[],
            cost=0,
            f_mins=[],
            lower_bound=0,
            focal_heuristic=0,
            individual_planners=[]
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
        while self.open_set:
            # update focal set if min_lower_bound has increased
            new_min_lower_bound = min([node.lower_bound for node in self.open_set])
            if min_lower_bound < new_min_lower_bound:
                for node in self.open_set:
                    if (
                        self.w * min_lower_bound
                        <= node.cost
                        <= self.w * new_min_lower_bound
                    ):
                        self.focal_set.add(node)
                min_lower_bound = new_min_lower_bound

            # select node from focal set
            cur_node = heapq.heappop(self.focal_set)
            print(cur_node)
            self.open_set.remove(cur_node)

            # find the first conflict
            conflict = self.find_first_conflict(cur_node.solution)

            # if there is no conflict, return the solution
            if not conflict:
                return cur_node.solution, min_lower_bound

            # if there is a conflict, generate two child nodes
            for agent_id in conflict.agent_ids:
                # if the agent has already passed the conflict time, ignore it
                if (
                    type(conflict) == VertexConflict
                    and len(cur_node.solution[agent_id]) <= conflict.time
                ) or (
                    type(conflict) == EdgeConflict
                    and len(cur_node.solution[agent_id]) <= conflict.times[1]
                ):
                    continue
                # generate child node from the current node
                new_node = CTNode(
                    constraints=deepcopy(cur_node.constraints),
                    solution=deepcopy(cur_node.solution),
                    cost=cur_node.cost,
                    f_mins=deepcopy(cur_node.f_mins),
                    lower_bound=cur_node.lower_bound,
                    focal_heuristic=cur_node.focal_heuristic,
                    individual_planners=deepcopy(cur_node.individual_planners)
                )
                # new_node.individual_planners = copy(cur_node.individual_planners)
                # new_node.individual_planners[agent_id] = deepcopy(
                #     cur_node.individual_planners[agent_id]
                # )

                # generate constraint from the conflict
                new_constraint = self.generate_constraint_from_conflict(
                    agent_id, conflict
                )

                # add constraint to the child node
                new_node.constraints.setdefault(agent_id, []).append(new_constraint)

                # pruning node from the new constraint
                pruning_node = None
                if type(conflict) == VertexConflict:
                    pruning_point = new_node.solution[agent_id][conflict.time][0]
                    pruning_time = new_node.solution[agent_id][conflict.time][1]

                else:
                    pruning_point = new_node.solution[agent_id][conflict.times[1]][0]
                    pruning_time = new_node.solution[agent_id][conflict.times[1]][1]

                for closed_node in new_node.individual_planners[agent_id].closed_set:
                    if (
                        closed_node.point == pruning_point
                        and closed_node.time == pruning_time
                    ):
                        pruning_node = closed_node
                        break

                self.prune_successor(
                    pruning_node,
                    pruning_node,
                    new_node.individual_planners[agent_id].open_set,
                    new_node.individual_planners[agent_id].focal_set,
                    new_node.individual_planners[agent_id].closed_set,
                    agent_id,
                    new_constraint
                )

                pruning_node.parent.children.remove(pruning_node)

                # update reservation table
                self.env.reservation_table = new_node.solution
                self.env.reservation_table[agent_id] = []
                # generate new path for the agent that has the conflict
                new_node.solution[agent_id], new_f_min = new_node.individual_planners[
                    agent_id
                ].plan(constraints=new_node.constraints[agent_id])
                if not new_node.solution[agent_id]:
                    continue

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
        return None

    def prune_successor(
        self, conflict_node, node, open_set, focal_set, closed_set, agent_id, constraint
    ):
        while node.children:
            child = node.children.pop(0)
            self.prune_successor(
                conflict_node, child, open_set, focal_set, closed_set, agent_id, constraint
            )

        # self.visualize(conflict_node, node, open_set, closed_set, agent_id)

        if node in open_set:
            open_set.remove(node)
            if node in focal_set:
                focal_set.remove(node)
        else:
            closed_set.remove(node)

    def visualize(self, conflict_node, node, open_set, closed_set, agent_id):
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

        # Plot conflict node highlight with size
        self.ax.scatter(
            conflict_node.point.x,
            conflict_node.point.y,
            conflict_node.time,
            c="cyan",
            marker="o",
            s=100,
        )

        # Plot start and goal points without (x, o) markers
        self.ax.scatter(
            self.start_points[agent_id].x,
            self.start_points[agent_id].y,
            0,
            c="g",
            marker="^",
            label="Start Point",
        )

        self.ax.scatter(
            self.goal_points[agent_id].x,
            self.goal_points[agent_id].y,
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
        plt.pause(0.01)

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
