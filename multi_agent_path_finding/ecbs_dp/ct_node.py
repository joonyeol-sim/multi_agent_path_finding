from dataclasses import dataclass
from typing import List, Tuple, Dict

from multi_agent_path_finding.common.point import Point
from multi_agent_path_finding.common.constraint import Constraint
from multi_agent_path_finding.stastar_epsilon_dp.stastar_epsilon_dp import (
    SpaceTimeAstarEpsilonDP,
)


@dataclass
class CTNode:
    constraints: Dict[int, List[Constraint]]
    solution: List[List[Tuple[Point, int]]]
    cost: int
    f_mins: List[int]
    lower_bound: int
    focal_heuristic: int
    individual_planners: List[SpaceTimeAstarEpsilonDP]

    def __lt__(self, other):
        if self.focal_heuristic != other.focal_heuristic:
            return self.focal_heuristic < other.focal_heuristic
        return self.cost < other.cost

    def __hash__(self):
        return hash(str(self.solution))
