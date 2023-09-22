from dataclasses import dataclass
from typing import List, Tuple

from multi_agent_path_finding.common.point import Point
from multi_agent_path_finding.common.constraint import Constraint


@dataclass
class CTNode:
    constraints: List[Constraint]
    solution: List[List[Tuple[Point, int]]]
    cost: int = 0

    def __lt__(self, other):
        return self.cost < other.cost
