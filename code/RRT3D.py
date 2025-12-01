import time
import random
import math

from .path_planner_interface import PathPlanner
from .configuration_3d import Configuration3D


class Node3D(Configuration3D):
    

    def __init__(self, x: float, y: float, z: float):
        super().__init__(x, y, z)
        self.parent: "Node3D | None" = None


class RRT3DPlanner(PathPlanner):
   

    def __init__(
        self,
        start: Configuration3D,
        goal: Configuration3D,
        bounds: tuple[tuple[float, float], ...],
        step_size: float = 1.0,
        max_iterations: int = 5000,
        goal_threshold: float = 1.0,
    ):
        super().__init__(start, goal, bounds)
        self.step_size = step_size
        self.max_iterations = max_iterations
        self.goal_threshold = goal_threshold

        self.start_node = Node3D(start.x, start.y, start.z)
        self.goal_node = Node3D(goal.x, goal.y, goal.z)
        self.nodes: list[Node3D] = [self.start_node]

        self._planning_time: float = 0.0

    def _sample_random_point(self) -> Node3D:
       
        if random.random() < 0.1:
            return Node3D(self.goal.x, self.goal.y, self.goal.z)

        (xmin, xmax), (ymin, ymax), (zmin, zmax) = self.bounds
        return Node3D(
            random.uniform(xmin, xmax),
            random.uniform(ymin, ymax),
            random.uniform(zmin, zmax),
        )

    def _distance(self, a: Node3D, b: Node3D) -> float:
        return a.distance_to(b)

    def _nearest_node(self, target: Node3D) -> Node3D:
        best = self.nodes[0]
        best_d = float("inf")
        for n in self.nodes:
            d = self._distance(n, target)
            if d < best_d:
                best = n
                best_d = d
        return best

    def _steer(self, from_node: Node3D, to_node: Node3D) -> Node3D:
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        dz = to_node.z - from_node.z
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)

        if dist == 0.0:
            new = Node3D(from_node.x, from_node.y, from_node.z)
        elif dist <= self.step_size:
            new = Node3D(to_node.x, to_node.y, to_node.z)
        else:
            new = Node3D(
                from_node.x + self.step_size * dx / dist,
                from_node.y + self.step_size * dy / dist,
                from_node.z + self.step_size * dz / dist,
            )
        new.parent = from_node
        return new

    def _edge_collision_free(self, a: Node3D, b: Node3D) -> bool:
        
        import numpy as np

        if not self.obstacles:
            return True

        xs = np.linspace(a.x, b.x, 20)
        ys = np.linspace(a.y, b.y, 20)
        zs = np.linspace(a.z, b.z, 20)

        for x, y, z in zip(xs, ys, zs):
            for obs in self.obstacles:
                if obs.contains(float(x), float(y), float(z)):
                    return False
        return True

    def _extract_path(self, node: Node3D) -> list[Configuration3D]:
        path_nodes: list[Node3D] = []
        cur: Node3D | None = node
        while cur is not None:
            path_nodes.append(cur)
            cur = cur.parent
        path_nodes.reverse()
        return [Configuration3D(n.x, n.y, n.z) for n in path_nodes]

    def plan(self) -> bool:
        
        start_time = time.perf_counter()

        self.nodes = [self.start_node]
        found_path: list[Configuration3D] | None = None

        for _ in range(self.max_iterations):
            x_rand = self._sample_random_point()
            x_near = self._nearest_node(x_rand)
            x_new = self._steer(x_near, x_rand)

            if not self._edge_collision_free(x_near, x_new):
                continue

            self.nodes.append(x_new)

            if self._distance(x_new, self.goal_node) < self.goal_threshold:
                found_path = self._extract_path(x_new)
                break

        self._planning_time = time.perf_counter() - start_time

        if found_path is None:
            self.path = []
            return False

        self.path = found_path
        return True

    def get_planning_time(self) -> float:
        return self._planning_time

    def get_num_nodes(self) -> int:
        return len(self.nodes)
