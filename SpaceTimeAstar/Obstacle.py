from dataclasses import dataclass
from abc import ABC, abstractmethod
from typing import List, Tuple


@dataclass
class Obstacle(ABC):
    point: List[int]

    @abstractmethod
    def is_colliding(self, *args, **kwargs) -> bool:
        pass


@dataclass
class StaticObstacle(Obstacle):
    def is_colliding(self, point: List[int], **kwargs) -> bool:
        return point == self.point


@dataclass
class DynamicObstacle(Obstacle):
    time: Tuple[int, int]

    def __init__(self, point: List[int], time: Tuple[int, int]):
        super().__init__(point)
        self.time = time

    def is_colliding(self, point: List[int], time: int = None) -> bool:
        is_in_time_range = self.time[0] <= time <= self.time[1] if self.time[1] != -1 else self.time[0] <= time
        return is_in_time_range and point == self.point
