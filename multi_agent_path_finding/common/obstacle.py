from dataclasses import dataclass
from abc import ABC, abstractmethod
from typing import Tuple
from multi_agent_path_finding.common.point import Point


@dataclass
class Obstacle(ABC):
    point: Point

    @abstractmethod
    def is_colliding(self, *args, **kwargs) -> bool:
        pass


@dataclass
class StaticObstacle(Obstacle):
    def is_colliding(self, point: Point, **kwargs) -> bool:
        return point == self.point


@dataclass
class DynamicObstacle(Obstacle):
    time: Tuple[int, int]

    def is_colliding(self, point: Point, time: int = None) -> bool:
        is_in_time_range = (
            self.time[0] <= time <= self.time[1]
            if self.time[1] != -1
            else self.time[0] <= time
        )
        return is_in_time_range and point == self.point
