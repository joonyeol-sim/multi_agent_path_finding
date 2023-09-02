from typing import List, Tuple
from space_time_astar.obstacle import StaticObstacle, DynamicObstacle, Obstacle


class Environment:
    def __init__(
        self,
        dimension: int,
        space_limit: List[int],
        static_obstacles: List[List[int]] = None,
        dynamic_obstacles: List[Tuple[List[int], List[int]]] = None,
    ):
        self.dimension = dimension
        self.space_limit = space_limit
        if self.dimension != len(self.space_limit):
            raise ValueError(
                f"Dimension does not match the length of space limit: {self.space_limit}"
            )

        self.obstacles: List[Obstacle] = []
        if static_obstacles is not None:
            for static_obstacle in static_obstacles:
                if self.dimension != len(static_obstacle):
                    raise ValueError(
                        f"Dimension does not match the length of static obstacle: {static_obstacle}"
                    )
                self.obstacles.append(StaticObstacle(static_obstacle))
        if dynamic_obstacles is not None:
            for dynamic_obstacle in dynamic_obstacles:
                if self.dimension != len(dynamic_obstacle[0]):
                    raise ValueError(
                        f"Dimension does not match the length of dynamic obstacle: {dynamic_obstacle}"
                    )
                self.obstacles.append(
                    DynamicObstacle(dynamic_obstacle[0], dynamic_obstacle[1])
                )
