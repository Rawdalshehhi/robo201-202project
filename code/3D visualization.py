from mpl_toolkits.mplot3d import Axes3D  


def plot_planner_3d(planner, title: str = "3D Path Planning"):
    """Simple 3D visualization for RRT3DPlanner."""
    import matplotlib.pyplot as plt

    fig = plt.figure(figsize=(8, 8))
    ax = fig.add_subplot(111, projection='3d')

    
    if hasattr(planner, "nodes"):
        for node in planner.nodes:
            parent = getattr(node, "parent", None)
            if parent is not None:
                ax.plot(
                    [parent.x, node.x],
                    [parent.y, node.y],
                    [parent.z, node.z],
                    color="gray",
                    alpha=0.3,
                )

   
    path = planner.get_path()
    if path:
        xs = [p.x for p in path]
        ys = [p.y for p in path]
        zs = [p.z for p in path]
        ax.plot(xs, ys, zs, color="green", linewidth=3, label="Path")

   
    if hasattr(planner.start, "z"):
        ax.scatter([planner.start.x], [planner.start.y], [planner.start.z],
                   color="g", s=40, label="Start")
    if hasattr(planner.goal, "z"):
        ax.scatter([planner.goal.x], [planner.goal.y], [planner.goal.z],
                   color="r", s=40, label="Goal")

    ax.set_title(title)
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.legend()
    plt.show()
