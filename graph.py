import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import constants as c

def graph(filepath):

    num_runs = 5
    # num_generations = c.numberOfGenerations
    # population_size = c.populationSize

    # Load the CSV (already loaded into c, or read from file if needed)
    df = pd.read_csv(filepath, header=None)
    data = df[0].values

    # Total values expected: 5 runs × (population * generations + 40 extras per run)
    expected_size = num_runs * c.populationSize * c.numberOfGenerations
    if len(data) != expected_size:
        raise ValueError(f"Expected {expected_size} values, found {len(data)}")

    # Reshape: (runs, full_run_length)
    per_run_length = c.populationSize * c.numberOfGenerations
    valid_data = data.reshape((num_runs, per_run_length))

    # Slice off extra values per run
    # valid_data = data_per_run[:, :c.populationSize * c.numberOfGenerations]

    # Reshape into (runs, population, generations)
    reshaped = valid_data.reshape((num_runs, c.numberOfGenerations, c.populationSize)).transpose(0, 2, 1)

    #np.savetxt('output.csv', valid_data, delimiter=',', fmt='%d')

    # Compute best (max) fitness per generation for each run
    per_run_gen_best = reshaped.max(axis=1)  # Shape: (runs, generations)
    
    #np.savetxt('best.csv', per_run_gen_best, delimiter=',', fmt='%d')

    # Plot
    plt.figure(figsize=(10, 5))
    for i, run_best in enumerate(per_run_gen_best):
        plt.plot(run_best, label=f'Run {i+1}', alpha=0.8)

    data_type = 'Oscillations' if 'oscillation' in filepath else 'Fitness'

    plt.title(f'Best {data_type} per Generation with B ({num_runs} Runs)')
    plt.xlabel('Generation')
    plt.ylabel(f'Best {data_type}')
    plt.xticks(ticks=np.arange(0, c.numberOfGenerations, step=5)) 
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()

def graph_dual(filepath_a, filepath_b):
    num_runs = 5

    def process_file(filepath):
        df = pd.read_csv(filepath, header=None)
        data = df[0].values
        expected_size = num_runs * c.populationSize * c.numberOfGenerations
        if len(data) != expected_size:
            raise ValueError(f"Expected {expected_size} values, found {len(data)}")
        valid_data = data.reshape((num_runs, c.numberOfGenerations, c.populationSize)).transpose(0, 2, 1)
        return valid_data.max(axis=1)  # Shape: (runs, generations)

    best_a = process_file(filepath_a)
    best_b = process_file(filepath_b)

    plt.figure(figsize=(10, 5))
    colors_a = plt.cm.Blues(np.linspace(0.4, 1, num_runs))
    colors_b = plt.cm.Reds(np.linspace(0.4, 1, num_runs))

    for i in range(num_runs):
        plt.plot(best_a[i], label=f'Robot A - Run {i+1}', color=colors_a[i])
        plt.plot(best_b[i], label=f'Robot B - Run {i+1}', color=colors_b[i], linestyle='--')

    plt.title('Most Jumps per Generation (Robots A & B)')
    plt.xlabel('Generation')
    plt.ylabel('Most Jumps')
    plt.xticks(ticks=np.arange(0, c.numberOfGenerations, step=5))
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()



#graph_dual("robotA_oscillations.csv", "robotB_oscillations.csv")



#graph(f"robotA_oscillations.csv")
#graph(f"robotB_oscillations.csv")
# graph(f"robotA_fitness.csv")
# graph(f"robotB_fitness.csv")