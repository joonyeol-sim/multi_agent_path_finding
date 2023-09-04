from abc import ABC
from dataclasses import dataclass
from typing import List, Tuple, Dict
from common.point import Point


@dataclass
class Conflict(ABC):
    time: int
    agent_ids: List[int]


@dataclass
class VertexConflict(Conflict):
    point: Point


@dataclass
class EdgeConflict(Conflict):
    # The first point is the previous point
    # The second point is the next point
    # points: {
    #     agent_id: (previous_point, next_point)
    # }
    points: Dict[int, Tuple[Point, Point]]
