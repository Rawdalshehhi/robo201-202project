import random
import math
import heapq
from .path_planner_interface import PathPlanner, Configuration


class PRMPlanner(PathPlanner):
    def __init__(
        self,
        start: Configuration,
        goal: Configuration,
        bounds: tuple[tuple[float, float], ...],
        num_samples: int = 1000,
        k_neighbors: int = 10,
    ):
        super().__init__(start, goal, bounds)
        self.num_samples = num_samples
        self.k_neighbors = k_neighbors
        self.samples: list[Configuration] = []
        self.edges: list[tuple[int, int]] = []
        self._planning_time: float = 0.0

    def dist(self, a: Configuration, b: Configuration) -> float:
        return a.distance_to(b)

    def random_free_points(self) -> list[Configuration]:
        (xmin, xmax), (ymin, ymax) = self.bounds[:2]
        points: list[Configuration] = []
        while len(points) < self.num_samples:
            x = random.uniform(xmin, xmax)
            y = random.uniform(ymin, ymax)
            c = Configuration(x, y)
            if self.is_collision_free(c, c):
                points.append(c)
        return points

    def k_nearest(self, idx: int, points: list[Configuration]) -> list[int]:
        p = points[idx]
        arr = []
        for j, q in enumerate(points):
            if j == idx:
                continue
            d = self.dist(p, q)
            arr.append((d, j))
        arr.sort(key=lambda t: t[0])
        return [j for (_, j) in arr[: self.k_neighbors]]

    def intersects_obstacle(self, p1: Configuration, p2: Configuration) -> bool:
        return not self.is_collision_free(p1, p2)

    def build_roadmap(self) -> None:
        self.samples = self.random_free_points()
        self.samples.extend([self.start, self.goal])
        n = len(self.samples)
        self.edges = []
        for i in range(n):
            neighbors = self.k_nearest(i, self.samples)
            for j in neighbors:
                v = self.samples[i]
                u = self.samples[j]
                if not self.intersects_obstacle(v, u):
                    self.edges.append((i, j))
                    self.edges.append((j, i))

    def shortest_path(self, start_idx: int, goal_idx: int) -> list[int] | None:
        adj: dict[int, list[int]] = {}
        for u, v in self.edges:
            adj.setdefault(u, []).append(v)

        n = len(self.samples)
        dist = [math.inf] * n
        prev: list[int | None] = [None] * n
        dist[start_idx] = 0.0

        pq: list[tuple[float, int]] = []
        heapq.heappush(pq, (0.0, start_idx))

        while pq:
            d, u = heapq.heappop(pq)
            if d > dist[u]:
                continue
            if u == goal_idx:
                break
            for v in adj.get(u, []):
                w = self.dist(self.samples[u], self.samples[v])
                nd = d + w
                if nd < dist[v]:
                    dist[v] = nd
                    prev[v] = u
                    heapq.heappush(pq, (nd, v))

        if dist[goal_idx] == math.inf:
            return None

        path_idx: list[int] = []
        cur: int | None = goal_idx
        while cur is not None:
            path_idx.append(cur)
            cur = prev[cur]
        path_idx.reverse()
        return path_idx

    def plan(self) -> bool:
        import time

        t0 = time.perf_counter()
        self.build_roadmap()
        start_idx = len(self.samples) - 2
        goal_idx = len(self.samples) - 1
        idx_path = self.shortest_path(start_idx, goal_idx)
        self._planning_time = time.perf_counter() - t0

        if idx_path is None:
            self.path = []
            return False

        self.path = [self.samples[i] for i in idx_path]
        return True

    def get_planning_time(self) -> float:
        return self._planning_time

    def get_num_nodes(self) -> int:
        return len(self.samples)
