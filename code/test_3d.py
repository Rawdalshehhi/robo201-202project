import matplotlib.pyplot as plt
from .configuration_3d import Configuration3D
from .obstacles_3d import SphericalObstacle, BoxObstacle
from .RRT3D import RRT3DPlanner


def project_3d_to_2d(x, y, z):
    """
    Simple manual projection to 2D (portable 3D visualization)
    This avoids Axes3D and works on ANY system.
    """
    scale = 0.7
    px = x + scale * z
    py = y - scale * z
    return px, py


def plot_planner_3d_portable(planner, title="3D RRT (Portable Visualization)"):
    fig, ax = plt.subplots(figsize=(8, 8))

    # Draw 3D-like tree edges
    for node in planner.nodes:
        parent = node.parent
        if parent is not None:
            x1, y1 = project_3d_to_2d(parent.x, parent.y, parent.z)
            x2, y2 = project_3d_to_2d(node.x, node.y, node.z)
            ax.plot([x1, x2], [y1, y2], color="gray", alpha=0.4)

    # Draw path
    path = planner.get_path()
    if path:
        xs = []
        ys = []
        for p in path:
            px, py = project_3d_to_2d(p.x, p.y, p.z)
            xs.append(px)
            ys.append(py)
        ax.plot(xs, ys, color="green", linewidth=3, label="Path")

    # Draw start and goal
    sx, sy = project_3d_to_2d(planner.start.x, planner.start.y, planner.start.z)
    gx, gy = project_3d_to_2d(planner.goal.x, planner.goal.y, planner.goal.z)

    ax.scatter([sx], [sy], color="blue", s=80, label="Start")
    ax.scatter([gx], [gy], color="red", s=80, label="Goal")

    ax.set_title(title)
    ax.set_xlabel("Projected X")
    ax.set_ylabel("Projected Y")
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.set_aspect("equal", adjustable="box")

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

    planner = RRT3DPlanner(start, goal, bounds, step_size=1.0, max_iterations=3000)
    planner.set_obstacles(obstacles)

    success = planner.plan()
    print(
        "3D RRT success:", success,
        "time:", planner.get_planning_time(),
        "nodes:", planner.get_num_nodes()
    )

    plot_planner_3d_portable(planner)


if __name__ == "__main__":
    main()
