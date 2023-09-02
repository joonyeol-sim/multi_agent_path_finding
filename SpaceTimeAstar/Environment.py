from typing import List, Tuple
from Obstacle import StaticObstacle, DynamicObstacle, Obstacle


class Environment:
    def __init__(self, dimension: int, space_limit: List[int],
                 static_obstacles: List[List[int]] = None,
                 dynamic_obstacles: List[Tuple[List[int], int]] = None,
                 disappear_at_goal: bool = False):
        self.dimension = dimension
        self.space_limit = space_limit
        if self.dimension != len(self.space_limit):
            raise ValueError(f"Dimension does not match the length of space limit: {self.space_limit}")

        self.obstacles: List[Obstacle] = []
        for static_obstacle in static_obstacles:
            if self.dimension != len(static_obstacle):
                raise ValueError(f"Dimension does not match the length of static obstacle: {static_obstacle}")
            self.obstacles.append(StaticObstacle(static_obstacle))
        for dynamic_obstacle in dynamic_obstacles:
            if self.dimension != len(dynamic_obstacle[0]):
                raise ValueError(f"Dimension does not match the length of dynamic obstacle: {dynamic_obstacle}")
            self.obstacles.append(DynamicObstacle(dynamic_obstacle[0], dynamic_obstacle[1]))
        self.disappear_at_goal = disappear_at_goal
