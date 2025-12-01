from abc import ABC, abstractmethod
import math
import random
from typing import List, Tuple


class Obstacle(ABC):
    

    @abstractmethod
    def contains(self, x: float, y: float) -> bool:
       
        pass


class CircularObstacle(Obstacle):
    

    def __init__(self, center: tuple[float, float], radius: float):
        self.cx, self.cy = center
        self.radius = radius

    def contains(self, x: float, y: float) -> bool:
        dx = x - self.cx
        dy = y - self.cy
        return math.sqrt(dx * dx + dy * dy) < self.radius


class RectangularObstacle(Obstacle):
   

    def __init__(self, x: float, y: float, width: float, height: float):
        self.x = x          # bottom-left corner
        self.y = y
        self.width = width
        self.height = height

    def contains(self, x: float, y: float) -> bool:
        return (
            self.x <= x <= self.x + self.width and
            self.y <= y <= self.y + self.height
        )


class PolygonObstacle(Obstacle):
   

    def __init__(self, vertices: List[Tuple[float, float]]):
        self.vertices = vertices

    def contains(self, x: float, y: float) -> bool:
        
        inside = False
        n = len(self.vertices)
        for i in range(n):
            x1, y1 = self.vertices[i]
            x2, y2 = self.vertices[(i + 1) % n]

            if ((y1 > y) != (y2 > y)):
                x_cross = x1 + (y - y1) * (x2 - x1) / (y2 - y1 + 1e-12)
                if x < x_cross:
                    inside = not inside
        return inside


def generate_random_obstacles(
    num_obstacles: int,
    bounds: tuple[tuple[float, float], tuple[float, float]],
) -> List[Obstacle]:
    
    (xmin, xmax), (ymin, ymax) = bounds
    obstacles: List[Obstacle] = []

    for _ in range(num_obstacles):
        shape = random.choice(["circle", "rect", "poly"])

        if shape == "circle":
            radius = random.uniform(0.5, 2.0)
            cx = random.uniform(xmin + radius, xmax - radius)
            cy = random.uniform(ymin + radius, ymax - radius)
            obstacles.append(CircularObstacle((cx, cy), radius))

        elif shape == "rect":
            width = random.uniform(1.0, 3.0)
            height = random.uniform(1.0, 3.0)
            x = random.uniform(xmin, xmax - width)
            y = random.uniform(ymin, ymax - height)
            obstacles.append(RectangularObstacle(x, y, width, height))

        else:  
            cx = random.uniform(xmin + 1.0, xmax - 1.0)
            cy = random.uniform(ymin + 1.0, ymax - 1.0)
            num_vertices = random.randint(3, 6)
            vertices: List[Tuple[float, float]] = []
            for _ in range(num_vertices):
                r = random.uniform(0.5, 2.0)
                angle = random.uniform(0, 2 * math.pi)
                vx = cx + r * math.cos(angle)
                vy = cy + r * math.sin(angle)
                vertices.append((vx, vy))
            obstacles.append(PolygonObstacle(vertices))

    return obstacles
