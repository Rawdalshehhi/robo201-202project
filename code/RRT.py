import math
import random
import time
from typing import List
from .path_planner_interface import PathPlanner, Configuration


class Node:
    def __init__(self, x: float, y: float):
        self.x = x
        self.y = y
        self.parent = None
        self.cost = 0.0


class RRTPlanner(PathPlanner):
    def __init__(
        self,
        start: Configuration,
        goal: Configuration,
        bounds: tuple[tuple[float, float], ...],
        step_size: float = 0.5,
        max_iterations: int = 5000,
        goal_threshold: float = 0.5,
    ):
        super().__init__(start, goal, bounds)
        self.start_node = Node(start.x, start.y)
        self.goal_node = Node(goal.x, goal.y)
        self.nodes: List[Node] = [self.start_node]
        self.step_size = step_size
        self.max_iterations = max_iterations
        self.goal_threshold = goal_threshold
        self._planning_time = 0.0

    def sample_random_point(self) -> Node:
        if random.random() < 0.1:
            return Node(self.goal.x, self.goal.y)
        (xmin, xmax), (ymin, ymax) = self.bounds[:2]
        return Node(random.uniform(xmin, xmax), random.uniform(ymin, ymax))

    def distance(self, a: Node, b: Node) -> float:
        return math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2)

    def find_nearest_node(self, target: Node) -> Node:
        best = self.nodes[0]
        best_dist = float("inf")
        for n in self.nodes:
            d = self.distance(n, target)
            if d < best_dist:
                best = n
                best_dist = d
        return best

    def steer(self, from_node: Node, to_node: Node) -> Node:
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        dist = math.sqrt(dx * dx + dy * dy)
        if dist == 0.0:
            new = Node(from_node.x, from_node.y)
        elif dist <= self.step_size:
            new = Node(to_node.x, to_node.y)
        else:
            new = Node(
                from_node.x + self.step_size * dx / dist,
                from_node.y + self.step_size * dy / dist,
            )
        new.parent = from_node
        return new

    def extend(self, x_target: Node) -> str:
        x_near = self.find_nearest_node(x_target)
        x_new = self.steer(x_near, x_target)
        if self.is_collision_free(Configuration(x_near.x, x_near.y),
                                   Configuration(x_new.x, x_new.y)):
            self.nodes.append(x_new)
            if self.distance(x_new, self.goal_node) < self.goal_threshold:
                return "Reached"
            return "Advanced"
        return "Trapped"

    def extract_path(self, node: Node) -> list[Configuration]:
        path_nodes = []
        cur = node
        while cur is not None:
            path_nodes.append(cur)
            cur = cur.parent
        path_nodes.reverse()
        return [Configuration(n.x, n.y) for n in path_nodes]

    def build_rrt(self) -> list[Configuration] | None:
        for _ in range(self.max_iterations):
            x_rand = self.sample_random_point()
            status = self.extend(x_rand)
            if status == "Reached":
                return self.extract_path(self.nodes[-1])
        return None

    def plan(self) -> bool:
        start_time = time.perf_counter()
        self.nodes = [self.start_node]
        path = self.build_rrt()
        self._planning_time = time.perf_counter() - start_time
        if path is None:
            self.path = []
            return False
        self.path = path
        return True

    def get_planning_time(self) -> float:
        return self._planning_time

    def get_num_nodes(self) -> int:
        return len(self.nodes)
