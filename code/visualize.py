import matplotlib.pyplot as plt
from .path_planner_interface import PathPlanner, Configuration
from .obstacles import PolygonObstacle


def plot_planner(planner: PathPlanner, 
                 title: str = "Path Planning Result",
                 figsize: tuple = (10, 10)):
    """
    Visualize the planning result including obstacles, tree/graph, and path.
    """
    fig, ax = plt.subplots(figsize=figsize)
    
    
    (xmin, xmax), (ymin, ymax) = planner.bounds[:2]
    ax.set_xlim(xmin, xmax)
    ax.set_ylim(ymin, ymax)
    
    
    for obs in getattr(planner, "obstacles", []):
        if hasattr(obs, "radius"):  # circle
            circle = plt.Circle((obs.cx, obs.cy), obs.radius,
                                color='red', alpha=0.3)
            ax.add_patch(circle)
        elif hasattr(obs, "width") and hasattr(obs, "height"):  # rectangle
            rect = plt.Rectangle((obs.x, obs.y), obs.width, obs.height,
                                 color='orange', alpha=0.3)
            ax.add_patch(rect)
        elif isinstance(obs, PolygonObstacle):  # polygon
            poly = plt.Polygon(obs.vertices, color='purple', alpha=0.3)
            ax.add_patch(poly)
    
   
    if hasattr(planner, "nodes"):
        for node in planner.nodes:
            parent = getattr(node, "parent", None)
            if parent is not None:
                ax.plot([parent.x, node.x], [parent.y, node.y],
                        'gray', alpha=0.3)
    
    
    if hasattr(planner, "samples") and hasattr(planner, "edges"):
        samples = planner.samples
        for u, v in planner.edges:
            cu = samples[u]
            cv = samples[v]
            ax.plot([cu.x, cv.x], [cu.y, cv.y],
                    'lightgray', alpha=0.3)
    
    
    start: Configuration = planner.start
    goal: Configuration = planner.goal
    ax.plot(start.x, start.y, 'go', markersize=15, label='Start')
    ax.plot(goal.x, goal.y, 'r*', markersize=20, label='Goal')
    
    
    path = planner.get_path()
    if path:
        xs = [c.x for c in path]
        ys = [c.y for c in path]
        ax.plot(xs, ys, 'b-', linewidth=3, label='Path')
    
  
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_title(title)
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    ax.legend()
    
    plt.tight_layout()
    plt.show()
