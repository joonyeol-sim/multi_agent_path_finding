"""Tests for `space_time_astar` package."""

import random
from itertools import combinations

from cbs.cbs import ConflictBasedSearch
from common.environment import Environment
from common.point import Point2D, Point3D


def find_first_conflict(solution) -> bool:
    # Vertex Conflict
    for agent1, agent2 in combinations(range(len(solution)), 2):
        max_time = max(len(solution[agent1]), len(solution[agent2]))
        for time in range(max_time):
            point1, time1 = solution[agent1][time]
            point2, time2 = solution[agent2][time]
            if point1 == point2 and time1 == time2:
                return True

    # Edge Conflict
    for agent1, agent2 in combinations(range(len(solution)), 2):
        max_time = max(len(solution[agent1]), len(solution[agent2]))
        for time in range(max_time - 1):
            prev_point1, prev_time1 = solution[agent1][time]
            next_point1, next_time1 = solution[agent1][time + 1]
            prev_point2, prev_time2 = solution[agent2][time]
            next_point2, next_time2 = solution[agent2][time + 1]

            if (
                prev_point1 == next_point2
                and prev_point2 == next_point1
                and prev_time1 == prev_time2
                and next_time1 == next_time2
            ):
                return True
    return False


class TestConflictBasedSearch:
    def test_open_plan(self):
        for dimension in [2, 3]:
            space_limits = [random.randint(2, 30) for _ in range(dimension)]
            robot_num = 10

            if dimension == 2:
                Point = Point2D
            else:
                Point = Point3D

            start_points = []
            goal_points = []

            while len(start_points) < robot_num or len(goal_points) < robot_num:
                start_point = Point(
                    *[random.randint(0, space_limits[i] - 1) for i in range(dimension)]
                )
                goal_point = Point(
                    *[random.randint(0, space_limits[i] - 1) for i in range(dimension)]
                )
                if start_point in start_points:
                    continue
                if goal_point in goal_points:
                    continue
                start_points.append(start_point)
                goal_points.append(goal_point)

            env = Environment(dimension=dimension, space_limit=space_limits)
            astar = ConflictBasedSearch(
                start_points=start_points,
                goal_points=goal_points,
                env=env,
            )
            solution = astar.plan()

            # interpolate the solution
            interpolated_solution = []
            max_time = max([len(path) for path in solution])
            for agent_id, path in enumerate(solution):
                interpolated_path = []
                for time in range(max_time):
                    if time < len(path):
                        interpolated_path.append(path[time])
                    else:
                        interpolated_path.append((path[-1][0], time))
                interpolated_solution.append(interpolated_path)

            # check all robots have a path
            for agent_id, path in enumerate(solution):
                assert path[0] == (start_points[agent_id], 0)
                assert path[-1] == (goal_points[agent_id], len(path) - 1)
            # check if the solution is collision-free
            assert not find_first_conflict(interpolated_solution)
