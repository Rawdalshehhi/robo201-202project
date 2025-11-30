"""
Quick test for 3D RRT bonus.
"""

from .configuration_3d import Configuration3D
from .obstacles_3d import SphericalObstacle, BoxObstacle
from .RRT3D import RRT3DPlanner
from .visualize import plot_planner_3d


def main():
    bounds = ((0.0, 10.0), (0.0, 10.0), (0.0, 10.0))
    start = Configuration3D(1.0, 1.0, 1.0)
    goal = Configuration3D(9.0, 9.0, 9.0)

    obstacles = [
        SphericalObstacle((5.0, 5.0, 5.0), 2.0),
        BoxObstacle(3.0, 3.0, 0.0, 2.0, 2.0, 6.0),
    ]

    planner = RRT3DPlanner(start, goal, bounds,
                           step_size=1.0,
                           max_iterations=7000)
    planner.set_obstacles(obstacles)

    success = planner.plan()
    print("3D RRT success:", success)
    print("3D planning time:", planner.get_planning_time())
    print("3D num nodes:", planner.get_num_nodes())

    plot_planner_3d(planner, title="3D RRT BONUS")


if __name__ == "__main__":
    main()
