from .path_planner_interface import PathPlanner, Configuration


def _compute_path_length(path: list[Configuration]) -> float:
    """Sum distances between consecutive configurations."""
    if len(path) < 2:
        return 0.0
    length = 0.0
    for i in range(len(path) - 1):
        length += path[i].distance_to(path[i + 1])
    return length


def test_planner(planner: PathPlanner, 
                 scenario_name: str,
                 visualize: bool = True) -> dict:
    """
    Polymorphic test function for path planning algorithms.
    Works with ANY PathPlanner subclass (RRT, PRM, etc.).
    """
    # 1. Run planner
    success = planner.plan()

    # 2. Get path and length
    path = planner.get_path()
    path_length = _compute_path_length(path)

    # 3. Metrics from planner
    planning_time = planner.get_planning_time()
    num_nodes = planner.get_num_nodes()

    # 4. Optional visualization
    if visualize:
        from .visualize import plot_planner
        plot_planner(planner, title=scenario_name)

    # 5. Return dictionary
    return {
        'success': success,
        'path_length': path_length,
        'planning_time': planning_time,
        'num_nodes': num_nodes,
        'scenario': scenario_name,
        'planner': planner.__class__.__name__,
    }


def main():
    """
    Main function to run path planning experiments.
    You can modify this to sweep hyperparameters for the report.
    """
    from .RRT import RRTPlanner
    from .PRM import PRMPlanner
    from .obstacles import generate_random_obstacles

    print("=" * 70)
    print("Path Planning Assignment - ROBO 201/202")
    print("=" * 70)
    
    # Workspace
    start = Configuration(1.0, 1.0)
    goal = Configuration(9.0, 9.0)
    bounds = ((0.0, 10.0), (0.0, 10.0))

    # Environment densities (0, 10, 100 obstacles)
    env_obstacles_list = [0, 10, 100]

    results: list[dict] = []

    # ---- Example systematic testing (you can expand to 9+ combos) ----

    # RRT hyperparameters
    rrt_step_sizes = [0.5, 1.0, 2.0]
    rrt_iterations = [1000, 5000, 10000]

    for num_obs in env_obstacles_list:
        obstacles = generate_random_obstacles(num_obs, bounds)

        for step in rrt_step_sizes:
            for iters in rrt_iterations:
                rrt = RRTPlanner(start, goal, bounds,
                                 step_size=step,
                                 max_iterations=iters)
                rrt.set_obstacles(obstacles)
                res = test_planner(
                    rrt,
                    scenario_name=f"RRT_step{step}_iter{iters}_obs{num_obs}",
                    visualize=False,
                )
                results.append(res)
                print(res)

    # PRM hyperparameters
    prm_samples = [1000, 5000, 10000]
    prm_k = [5, 10, 15]

    for num_obs in env_obstacles_list:
        obstacles = generate_random_obstacles(num_obs, bounds)

        for ns in prm_samples:
            for k in prm_k:
                prm = PRMPlanner(start, goal, bounds,
                                 num_samples=ns,
                                 k_neighbors=k)
                prm.set_obstacles(obstacles)
                res = test_planner(
                    prm,
                    scenario_name=f"PRM_samples{ns}_k{k}_obs{num_obs}",
                    visualize=False,
                )
                results.append(res)
                print(res)

    print("Run 'python -m code' to execute experiments.")
    print("=" * 70)


if __name__ == "__main__":
    main()
