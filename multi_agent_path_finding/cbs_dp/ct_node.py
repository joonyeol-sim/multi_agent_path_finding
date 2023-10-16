from dataclasses import dataclass
from typing import List, Tuple, Dict

from multi_agent_path_finding.common.point import Point
from multi_agent_path_finding.common.constraint import Constraint
from multi_agent_path_finding.stastar_dp.stastar_dp import SpaceTimeAstarDP


@dataclass
class CTNode:
    constraints: Dict[int, List[Constraint]]
    solution: List[List[Tuple[Point, int]]]
    cost: int = 0
    individual_planners: List[SpaceTimeAstarDP] = None

    def __lt__(self, other):
        return self.cost < other.cost

    def __hash__(self):
        return hash(str(self.solution))
