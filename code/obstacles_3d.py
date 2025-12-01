from abc import ABC, abstractmethod
import math


class Obstacle3D(ABC):
    @abstractmethod
    def contains(self, x: float, y: float, z: float) -> bool:
        
        pass


class SphericalObstacle(Obstacle3D):
    def __init__(self, center: tuple[float, float, float], radius: float):
        self.cx, self.cy, self.cz = center
        self.radius = radius

    def contains(self, x: float, y: float, z: float) -> bool:
        dx = x - self.cx
        dy = y - self.cy
        dz = z - self.cz
        return math.sqrt(dx * dx + dy * dy + dz * dz) < self.radius


class BoxObstacle(Obstacle3D):
    

    def __init__(self,
                 x: float, y: float, z: float,
                 width: float, depth: float, height: float):
        self.x = x
        self.y = y
        self.z = z
        self.width = width
        self.depth = depth
        self.height = height

    def contains(self, x: float, y: float, z: float) -> bool:
        return (
            self.x <= x <= self.x + self.width and
            self.y <= y <= self.y + self.depth and
            self.z <= z <= self.z + self.height
        )
