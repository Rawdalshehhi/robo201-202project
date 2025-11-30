from abc import ABC, abstractmethod


class Configuration:
   
    
    def __init__(self, x: float, y: float):
        
        self.x = x
        self.y = y
    
    def to_tuple(self) -> tuple[float, float]:
        
        return (self.x, self.y)
    
    def __str__(self) -> str:
        return f"Configuration({self.x}, {self.y})"
    
    def distance_to(self, other: 'Configuration') -> float:
        
        
        dx = self.x - other.x
        dy = self.y - other.y
        return (dx*dx + dy*dy) ** 0.5


class PathPlanner(ABC):
    
    def __init__(self, start: Configuration, 
                 goal: Configuration,
                 bounds: tuple[tuple[float, float], ...]):
        
        self.start = start
        self.goal = goal
        self.bounds = bounds
        self.obstacles = []
        self.path = None
    
    def set_obstacles(self, obstacles: list):
        self.obstacles = obstacles
    
    @abstractmethod
    def plan(self) -> bool:
       
        pass
    
    def get_path(self) -> list[Configuration]:
        
        return self.path if self.path else []
    
    def is_collision_free(self, config1: Configuration, config2: Configuration) -> bool:
    
        import numpy as np

        if not self.obstacles:
            return True

        xs = np.linspace(config1.x, config2.x, 20)
        ys = np.linspace(config1.y, config2.y, 20)

        for x, y in zip(xs, ys):
            for obs in self.obstacles:
                if obs.contains(float(x), float(y)):
                    return False  

        return True  
    
    @abstractmethod
    def get_planning_time(self) -> float:
       
        pass
    
    @abstractmethod
    def get_num_nodes(self) -> int:
        
        pass
