# Todo: implement the Point class
import math
from abc import ABC, abstractmethod
from typing import Union
from dataclasses import dataclass

# Redefining the classes using @dataclass and the __new__ method


@dataclass(init=False)
class Point(ABC):
    x: int
    y: int

    def __new__(cls, *args, **kwargs):
        if len(args) == 2:
            instance = super(Point, cls).__new__(Point2D)
        elif len(args) == 3:
            instance = super(Point, cls).__new__(Point3D)
        else:
            raise ValueError("Invalid number of arguments for Point")
        instance.__init__(*args, **kwargs)
        return instance

    @abstractmethod
    def __hash__(self):
        pass

    @abstractmethod
    def euclidean_distance(self, other: Union["Point2D", "Point3D"]):
        pass

    @abstractmethod
    def manhattan_distance(self, other: Union["Point2D", "Point3D"]):
        pass

    @abstractmethod
    def get_neighbor_points(self):
        pass


@dataclass(init=False)
class Point2D(Point):
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __hash__(self):
        return hash((self.x, self.y))

    def euclidean_distance(self, other: "Point2D"):
        return math.sqrt(math.pow(self.x - other.x, 2) + math.pow(self.y - other.y, 2))

    def manhattan_distance(self, other: "Point2D"):
        return abs(self.x - other.x) + abs(self.y - other.y)

    def get_neighbor_points(self):
        return [
            Point2D(self.x + 1, self.y),
            Point2D(self.x - 1, self.y),
            Point2D(self.x, self.y + 1),
            Point2D(self.x, self.y - 1),
        ]


@dataclass(init=False)
class Point3D(Point):
    z: int

    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

    def __hash__(self):
        return hash((self.x, self.y, self.z))

    def euclidean_distance(self, other: "Point3D"):
        return math.sqrt(
            math.pow(self.x - other.x, 2)
            + math.pow(self.y - other.y, 2)
            + math.pow(self.z - other.z, 2)
        )

    def manhattan_distance(self, other: "Point3D"):
        return abs(self.x - other.x) + abs(self.y - other.y) + abs(self.z - other.z)

    def get_neighbor_points(self):
        return [
            Point3D(self.x + 1, self.y, self.z),
            Point3D(self.x - 1, self.y, self.z),
            Point3D(self.x, self.y + 1, self.z),
            Point3D(self.x, self.y - 1, self.z),
            Point3D(self.x, self.y, self.z + 1),
            Point3D(self.x, self.y, self.z - 1),
        ]
