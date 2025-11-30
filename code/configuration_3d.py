"""
3D configuration class (BONUS).
Extends the 2D Configuration as required in the assignment.
"""

from .path_planner_interface import Configuration
import math


class Configuration3D(Configuration):
    """3D configuration with coordinates (x, y, z)."""

    def __init__(self, x: float, y: float, z: float):
        super().__init__(x, y)
        self.z = z

    def distance_to(self, other: "Configuration3D") -> float:
        """Override to compute 3D Euclidean distance."""
        dx = self.x - other.x
        dy = self.y - other.y
        dz = self.z - other.z
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def to_tuple(self) -> tuple[float, float, float]:
        """Return (x, y, z)."""
        return (self.x, self.y, self.z)

    def __str__(self) -> str:
        return f"Configuration3D({self.x}, {self.y}, {self.z})"
