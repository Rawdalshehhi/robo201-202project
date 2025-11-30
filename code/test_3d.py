from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
import matplotlib.pyplot as plt

from .configuration_3d import Configuration3D
from .obstacles_3d import SphericalObstacle, BoxObstacle
from .RRT3D import RRT3DPlanner


def plot_planner_3d(planner: RRT3DPlanner, title: str = "3D RRT"):
    fig = plt.figure(figsize=(8, 8))
    ax = fig.add_subplot(111, projection="3d")

    for node in planner.nodes:
        parent = node.parent
        if parent is not None:
            ax.plot(
                [parent.x, node.x],
                [parent.y, node.y],
                [parent.z, node.z],
                color="gray",
                alpha=0.4,
            )

    path = planner.get_path()
    if path:
        xs = [p.x for p in path]
        ys = [p.y for p in path]
        zs = [p.z for p in path]
        ax.plot(xs, ys, zs, color="green", linewidth=2, label="Path")

    ax.scatter(
        [planner.start.x], [planner.start.y], [planner.start.z],
        color="blue", s=50, label="Start",
    )
    ax.scatter(
        [planner.goal.x], [planner.goal.y], [planner.goal.z],
        color="red", s=50, label="Goal",
    )

    ax.set_xlim(planner.bounds[0])
    ax.set_ylim(planner.bounds[1])
    ax.set_zlim(planner.bounds[2])

    ax.set_title(title)
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.legend()
    plt.tight_layout()
    plt.show()


def main():
    bounds = ((0, 10), (0, 10), (0, 10))
    start = Configuration3D(1, 1, 1)
    goal = Configuration3D(9, 9, 9)

    obstacles = [
        SphericalObstacle((5, 5, 5), 2.0),
        BoxObstacle(3, 3, 0, 2, 2, 6),
    ]

    planner = RRT3DPlanner(
        start,
        goal,
        bounds,
        step_size=1.0,
        max_iterations=3000,
        goal_threshold=1.0,
    )

    planner.set_obstacles(obstacles)
    success = planner.plan()
    print(
        "3D RRT success:",
        success,
        "time:",
        planner.get_planning_time(),
        "nodes:",
        planner.get_num_nodes(),
    )
    plot_planner_3d(planner)


if __name__ == "__main__":
    main()
