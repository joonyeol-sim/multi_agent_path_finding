"""Tests for `space_time_astar` package."""

import random

import pytest

from common.environment import Environment
from common.point import Point
from space_time_astar.space_time_astar import SpaceTimeAstar

from itertools import product


class TestSpaceTimeAstar:
    def test_open_plan(self):
        for dimension in [2, 3]:
            space_limits = [random.randint(2, 10) for _ in range(dimension)]

            start_point = Point(
                *[random.randint(0, space_limits[i] - 1) for i in range(dimension)]
            )
            goal_point = Point(
                *[random.randint(0, space_limits[i] - 1) for i in range(dimension)]
            )

            env = Environment(dimension=dimension, space_limit=space_limits)
            astar = SpaceTimeAstar(
                start_point=start_point,
                goal_point=goal_point,
                env=env,
            )
            path = astar.plan()

            assert path[0] == (start_point, 0)
            assert path[-1] == (goal_point, len(path) - 1)
            # check if the path is optimal
            assert len(path) == start_point.manhattan_distance(goal_point) + 1

    def test_no_plan(self):
        for dimension in [2, 3]:
            space_limits = [random.randint(2, 10) for _ in range(dimension)]
            start_point = Point(
                *[random.randint(0, space_limits[i] - 1) for i in range(dimension)]
            )
            goal_point = Point(
                *[random.randint(0, space_limits[i] - 1) for i in range(dimension)]
            )

            static_obstacles = start_point.get_neighbor_points()
            if dimension == 2:
                dynamic_obstacles = [(Point(*[start_point.x, start_point.y]), [1, -1])]
            else:
                dynamic_obstacles = [
                    (Point(*[start_point.x, start_point.y, start_point.z]), [1, -1])
                ]

            env = Environment(
                dimension=dimension,
                space_limit=space_limits,
                static_obstacles=static_obstacles,
                dynamic_obstacles=dynamic_obstacles,
            )
            astar = SpaceTimeAstar(
                start_point=start_point, goal_point=goal_point, env=env
            )
            path = astar.plan()

            assert path is None

    def test_static_obstacle_plan(self):
        for dimension in [2, 3]:
            space_limits = [random.randint(2, 10) for _ in range(dimension)]
            start_point = Point(*[0 for _ in range(dimension)])
            goal_point = Point(*[space_limits[i] - 1 for i in range(dimension)])

            static_obstacles = []
            ranges = [range(1, space_limits[i] - 1) for i in range(dimension)]
            for coordinates in product(*ranges):
                static_obstacles.append(Point(*coordinates))

            env = Environment(
                dimension=dimension,
                space_limit=space_limits,
                static_obstacles=static_obstacles,
            )
            astar = SpaceTimeAstar(
                start_point=start_point, goal_point=goal_point, env=env
            )
            path = astar.plan()

            assert path[0] == (start_point, 0)
            assert path[-1] == (goal_point, len(path) - 1)
            # check if the path is optimal
            assert len(path) == start_point.manhattan_distance(goal_point) + 1
            for node in path:
                assert node[0] not in static_obstacles

    def test_dynamic_obstacle_plan(self):
        for dimension in [2, 3]:
            space_limits = [random.randint(2, 10) for _ in range(dimension)]
            start_point = Point(*[0 for _ in range(dimension)])
            goal_point = Point(*[space_limits[i] - 1 for i in range(dimension)])
            dynamic_obstacles = []
            num_of_cells = 1
            for i in range(dimension):
                num_of_cells *= space_limits[i]
            num_of_dynamic_obstacles = random.randint(1, num_of_cells // 2)
            time_range = start_point.manhattan_distance(goal_point)
            for i in range(1, num_of_dynamic_obstacles):
                start_time = random.randint(0, time_range)
                end_time = random.randint(start_time, time_range)
                rand_point = Point(
                    *[random.randint(0, space_limits[i] - 1) for i in range(dimension)]
                )
                if rand_point != start_point and rand_point != goal_point:
                    dynamic_obstacles.append((rand_point, [start_time, end_time]))

            env = Environment(
                dimension=dimension,
                space_limit=space_limits,
                dynamic_obstacles=dynamic_obstacles,
            )
            astar = SpaceTimeAstar(
                start_point=start_point, goal_point=goal_point, env=env
            )
            path = astar.plan()

            assert path[0] == (start_point, 0)
            assert path[-1] == (goal_point, len(path) - 1)
            for node in path:
                for obstacle in dynamic_obstacles:
                    if obstacle[1][0] <= node[1] <= obstacle[1][1]:
                        assert node[0] != obstacle[0]

    def test_point_is_in_invalid_area(self):
        for dimension in [2, 3]:
            space_limits = [random.randint(2, 10) for _ in range(dimension)]
            env = Environment(dimension=dimension, space_limit=space_limits)

            # start point is out of bounds
            goal_point = Point(
                *[random.randint(0, space_limits[i] - 1) for i in range(dimension)]
            )
            start_point = Point(*[-1 for _ in range(dimension)])
            with pytest.raises(ValueError):
                astar = SpaceTimeAstar(
                    start_point=start_point, goal_point=goal_point, env=env
                )

            start_point = Point(*[space_limits[i] for i in range(dimension)])
            with pytest.raises(ValueError):
                astar = SpaceTimeAstar(
                    start_point=start_point, goal_point=goal_point, env=env
                )

            # goal point is out of bounds
            start_point = Point(
                *[random.randint(0, space_limits[i] - 1) for i in range(dimension)]
            )
            goal_point = Point(*[-1 for _ in range(dimension)])
            with pytest.raises(ValueError):
                astar = SpaceTimeAstar(
                    start_point=start_point, goal_point=goal_point, env=env
                )

            goal_point = Point(*[space_limits[i] for i in range(dimension)])
            with pytest.raises(ValueError):
                astar = SpaceTimeAstar(
                    start_point=start_point, goal_point=goal_point, env=env
                )

    def test_dimension_does_not_match(self):
        space_limits = [random.randint(2, 10) for _ in range(3)]
        env = Environment(dimension=3, space_limit=space_limits)

        goal_point = Point(*[random.randint(0, space_limits[i] - 1) for i in range(2)])
        start_point = Point(*[random.randint(0, space_limits[i] - 1) for i in range(3)])
        with pytest.raises(ValueError):
            astar = SpaceTimeAstar(
                start_point=start_point, goal_point=goal_point, env=env
            )

        start_point = Point(*[random.randint(0, space_limits[i] - 1) for i in range(3)])
        goal_point = Point(*[random.randint(0, space_limits[i] - 1) for i in range(2)])
        with pytest.raises(ValueError):
            astar = SpaceTimeAstar(
                start_point=start_point, goal_point=goal_point, env=env
            )
