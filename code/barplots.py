import matplotlib.pyplot as plt
def plot_rrt_time():
    max_iters = [1000, 5000, 10000]
    avg_times = [
        0.0002942880,   
        0.0003789790,   
        0.0002330760,  
    ]

    plt.figure()
    plt.bar([str(m) for m in max_iters], avg_times)
    plt.xlabel("RRT max_iterations (step_size = 1.0, obs = 0)")
    plt.ylabel("Average planning time (s)")
    plt.title("RRT: planning time vs max_iterations")
    plt.tight_layout()
    plt.show()


def plot_rrt_path():
    max_iters = [1000, 5000, 10000]
    avg_lengths = [
        13.322390310891004,  
        13.0,                
        12.986094919690853,  
    ]

    plt.figure()
    plt.bar([str(m) for m in max_iters], avg_lengths)
    plt.xlabel("RRT max_iterations (step_size = 1.0, obs = 0)")
    plt.ylabel("Average path length")
    plt.title("RRT: path length vs max_iterations")
    plt.tight_layout()
    plt.show()




def plot_prm_time():
    num_samples = [1000, 5000]
    avg_times = [
        0.47906445100124984,   
        13.38125031799973,    
    ]

    plt.figure()
    plt.bar([str(n) for n in num_samples], avg_times)
    plt.xlabel("PRM num_samples (k_neighbors = 10, obs = 0)")
    plt.ylabel("Average planning time (s)")
    plt.title("PRM: planning time vs num_samples")
    plt.tight_layout()
    plt.show()


def plot_prm_path():
    num_samples = [1000, 5000]
    avg_lengths = [
        11.788253659077021,   
        11.806034188663462,   
    ]

    plt.figure()
    plt.bar([str(n) for n in num_samples], avg_lengths)
    plt.xlabel("PRM num_samples (k_neighbors = 10, obs = 0)")
    plt.ylabel("Average path length")
    plt.title("PRM: path length vs num_samples")
    plt.tight_layout()
    plt.show()


def main():
    
    plot_rrt_time()
    plot_prm_time()
    plot_rrt_path()
    plot_prm_path()


if __name__ == "__main__":
    main()
