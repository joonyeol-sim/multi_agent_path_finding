from abc import ABC
from dataclasses import dataclass
from typing import List, Tuple, Dict
from common.point import Point


@dataclass
class Conflict(ABC):
    agent_ids: List[int]


@dataclass
class VertexConflict(Conflict):
    time: int
    point: Point


@dataclass
class EdgeConflict(Conflict):
    times: Tuple[int, int]
    # The first point is the previous point
    # The second point is the next point
    # points: {
    #     agent_id: (previous_point, next_point)
    # }
    points: Dict[int, Tuple[Point, Point]]
