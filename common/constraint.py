from abc import ABC
from dataclasses import dataclass
from typing import List, Tuple
from common.point import Point


@dataclass
class Constraint(ABC):
    agent_id: int


@dataclass
class VertexConstraint(Constraint):
    time: int
    point: Point


@dataclass
class EdgeConstraint(Constraint):
    times: Tuple[int, int]
    # The first point is the previous point
    # The second point is the next point
    points: Tuple[Point, Point]
