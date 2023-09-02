"""Tests for `space_time_astar` package."""

import pytest
import random

from space_time_astar.environment import Environment
from space_time_astar.space_time_astar import SpaceTimeAstar


class TestSpaceTimeAstar:
    def test_open_plan(self):
        max_x = random.randint(2, 100)
        max_y = random.randint(2, 100)
        start_point = [random.randint(0, max_x - 1), random.randint(0, max_y - 1)]
        goal_point = [random.randint(0, max_x - 1), random.randint(0, max_y - 1)]
        env = Environment(dimension=2, space_limit=[max_x, max_y])
        astar = SpaceTimeAstar(start_point=start_point, goal_point=goal_point, env=env)
        path = astar.plan()

        assert path[0] == (start_point, 0)
        assert path[-1] == (goal_point, len(path) - 1)
        assert (
            len(path)
            == abs(start_point[0] - goal_point[0])
            + abs(start_point[1] - goal_point[1])
            + 1
        )

    def test_no_plan(self):
        max_x = random.randint(2, 100)
        max_y = random.randint(2, 100)
        start_point = [random.randint(0, max_x - 1), random.randint(0, max_y - 1)]
        goal_point = [random.randint(0, max_x - 1), random.randint(0, max_y - 1)]
        static_obstacles = [
            [start_point[0], start_point[1]],
            [start_point[0] + 1, start_point[1]],
            [start_point[0] - 1, start_point[1]],
            [start_point[0], start_point[1] + 1],
            [start_point[0], start_point[1] - 1],
        ]
        env = Environment(
            dimension=2, space_limit=[max_x, max_y], static_obstacles=static_obstacles
        )
        astar = SpaceTimeAstar(start_point=start_point, goal_point=goal_point, env=env)
        path = astar.plan()

        assert path is None

    def test_static_obstacle_plan(self):
        max_x = random.randint(2, 100)
        max_y = random.randint(2, 100)
        start_point = [0, 0]
        goal_point = [max_x - 1, max_y - 1]
        static_obstacles = []
        for i in range(1, max_x - 1):
            for j in range(1, max_y - 1):
                static_obstacles.append([i, j])
        env = Environment(
            dimension=2, space_limit=[max_x, max_y], static_obstacles=static_obstacles
        )
        astar = SpaceTimeAstar(start_point=start_point, goal_point=goal_point, env=env)
        path = astar.plan()

        assert path[0] == ([0, 0], 0)
        assert path[-1] == ([max_x - 1, max_y - 1], len(path) - 1)
        assert (
            len(path)
            == abs(start_point[0] - goal_point[0])
            + abs(start_point[1] - goal_point[1])
            + 1
        )
        for node in path:
            assert node[0] not in static_obstacles

    def test_dynamic_obstacle_plan(self):
        max_x = random.randint(2, 100)
        max_y = random.randint(2, 100)
        start_point = [random.randint(0, max_x - 1), random.randint(0, max_y - 1)]
        goal_point = [random.randint(0, max_x - 1), random.randint(0, max_y - 1)]
        dynamic_obstacles = []
        num_of_dynamic_obstacles = random.randint(1, max_x * max_y // 2)
        for i in range(1, num_of_dynamic_obstacles):
            start_time = random.randint(
                0,
                abs(start_point[0] - goal_point[0])
                + abs(start_point[1] - goal_point[1]),
            )
            end_time = random.randint(
                start_time,
                abs(start_point[0] - goal_point[0])
                + abs(start_point[1] - goal_point[1]),
            )
            rand_x = random.randint(0, max_x - 1)
            rand_y = random.randint(0, max_y - 1)
            if [rand_x, rand_y] != start_point and [rand_x, rand_y] != goal_point:
                dynamic_obstacles.append(([rand_x, rand_y], [start_time, end_time]))

        env = Environment(
            dimension=2, space_limit=[max_x, max_y], dynamic_obstacles=dynamic_obstacles
        )
        astar = SpaceTimeAstar(start_point=start_point, goal_point=goal_point, env=env)
        path = astar.plan()

        assert path[0] == (start_point, 0)
        assert path[-1] == (goal_point, len(path) - 1)
        for node in path:
            for obstacle in dynamic_obstacles:
                if obstacle[1][0] <= node[1] <= obstacle[1][1]:
                    assert node[0] != obstacle[0]
