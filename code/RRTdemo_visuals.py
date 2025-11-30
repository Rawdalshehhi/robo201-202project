from .path_planner_interface import Configuration
from .RRT import RRTPlanner
from .PRM import PRMPlanner
from .visualize import plot_planner
from .obstacles import CircularObstacle, RectangularObstacle, PolygonObstacle


def make_clean_obstacles():
    # A few small obstacles (much cleaner)
    obs = []

    # Circles
    obs.append(CircularObstacle((3, 3), 0.8))
    obs.append(CircularObstacle((7, 7), 1.0))

    # Rectangles
    obs.append(RectangularObstacle(5, 2, 1.5, 1))
    obs.append(RectangularObstacle(2, 7, 1, 1.5))

    # Polygons
    obs.append(PolygonObstacle([(6,3),(7,4),(6.5,5)]))

    return obs


def main():
    bounds = ((0, 10), (0, 10))
    start = Configuration(1, 1)
    goal = Configuration(9, 9)

    obstacles = make_clean_obstacles()

    # RRT visualization
    rrt = RRTPlanner(start, goal, bounds,
                     step_size=1.0,
                     max_iterations=5000)
    rrt.set_obstacles(obstacles)
    rrt.plan()
    plot_planner(rrt, "RRT (Clean Visualization)")


if __name__ == "__main__":
    main()
