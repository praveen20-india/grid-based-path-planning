"""Main pipeline script for grid-based path planning.

Runs potential field and wavefront planning algorithms on sample maps,
evaluates path generation, and saves result visualizations.
"""

import os
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image

from attractive_field import compute_attractive_field
from brushfire import compute_obstacle_distance
from repulsive_field import compute_repulsive_field
from gradient_descent import compute_total_potential, gradient_descent_path
from wavefront import compute_wavefront
from greedy_descent import greedy_descent_path


def load_map(file_path):
    """Load and binarize an image map.
    Returns: 2D numpy array where 0 is free space and 1 is occupied.
    """
    image = Image.open(file_path).convert('L')
    grid_map = np.array(image.getdata()).reshape((image.size[1], image.size[0])) / 255.0
    grid_map[grid_map > 0.5] = 1
    grid_map[grid_map <= 0.5] = 0
    # Invert so 0 = free, 1 = occupied
    grid_map = (grid_map * -1) + 1
    return grid_map


def plot_path_on_map(grid_map, xs, ys, start, goal, title, save_path):
    """Plot the map, start, goal, and the extracted path."""
    plt.figure(figsize=(8, 8))
    plt.matshow(grid_map, fignum=0, cmap='gray_r')
    plt.plot(start[1], start[0], 'go', markersize=10, label='Start')
    plt.plot(goal[1], goal[0], 'r*', markersize=10, label='Goal')
    if xs and ys:
        plt.plot(ys, xs, 'b-', linewidth=2, label='Path')
    plt.title(title, pad=20)
    plt.legend()
    plt.savefig(save_path, bbox_inches='tight')
    plt.close()


def plot_potential(potential_map, goal, title, save_path):
    """Plot a generic potential or distance map."""
    plt.figure(figsize=(8, 8))
    plt.matshow(potential_map, fignum=0)
    plt.plot(goal[1], goal[0], 'r*', markersize=10, label='Goal')
    plt.colorbar()
    plt.title(title, pad=20)
    plt.legend()
    plt.savefig(save_path, bbox_inches='tight')
    plt.close()


def run_scenario(map_name, map_file, start, goal, save_dir, results_file):
    """Run all algorithms for a specific map and start/goal configuration."""
    print(f"Running scenario: {map_name}...")
    
    # Create directories if they don't exist
    os.makedirs(save_dir, exist_ok=True)
    os.makedirs(os.path.dirname(results_file), exist_ok=True)

    grid_map = load_map(map_file)

    # 1. Attractive Potential Field
    attr_map = compute_attractive_field(grid_map, goal, scaling_factor=0.01, mode='euclidean')

    # 2. Brushfire Distance to Obstacle (8-connectivity)
    dist_map = compute_obstacle_distance(grid_map, connectivity=8)

    # 3. Repulsive Potential Field
    rep_map = compute_repulsive_field(dist_map + 1, influence_distance=10, scaling_factor=100)

    # 4. Total Potential Field
    total_map = compute_total_potential(attr_map, rep_map)
    # Capping total potential for visualization purposes avoiding infinity/huge spikes
    total_vis = np.clip(total_map, a_min=None, a_max=np.percentile(total_map, 95))
    plot_potential(total_vis, goal, f"Total Potential ({map_name})", os.path.join(save_dir, f"{map_name}_total_potential.png"))

    # 5. Gradient Descent Path
    path_xs, path_ys = gradient_descent_path(total_map, start, goal, connectivity=8, max_iters=2000)
    
    reached_goal_gd = (path_xs[-1] == goal[0] and path_ys[-1] == goal[1])
    plot_path_on_map(grid_map, path_xs, path_ys, start, goal, 
                     f"Gradient Descent Path ({map_name})", 
                     os.path.join(save_dir, f"{map_name}_gd_path.png"))

    # 6. Wavefront Planner
    wf_map, wf_vis = compute_wavefront(grid_map, goal, connectivity=8)
    plot_potential(wf_vis, goal, f"Wavefront Map ({map_name})", os.path.join(save_dir, f"{map_name}_wavefront_map.png"))

    # 7. Greedy Descent on Wavefront
    wf_xs, wf_ys = greedy_descent_path(wf_map, start, goal, connectivity=8)
    
    reached_goal_wf = (wf_xs[-1] == goal[0] and wf_ys[-1] == goal[1])
    plot_path_on_map(grid_map, wf_xs, wf_ys, start, goal, 
                     f"Wavefront Path ({map_name})", 
                     os.path.join(save_dir, f"{map_name}_wavefront_path.png"))

    # Write metrics
    with open(results_file, 'a') as f:
        f.write(f"--- Scenario: {map_name} ---\n")
        f.write(f"Gradient Descent (PF) - Length: {len(path_xs)}, Reached Goal: {reached_goal_gd}\n")
        f.write(f"Wavefront Planner     - Length: {len(wf_xs)}, Reached Goal: {reached_goal_wf}\n\n")

    return reached_goal_gd, reached_goal_wf


def main():
    base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    data_dir = os.path.join(base_dir, 'data')
    save_dir = os.path.join(base_dir, 'report', 'result_images')
    results_file = os.path.join(base_dir, 'results', 'performance_summary.txt')

    # Clear previous results
    if os.path.exists(results_file):
        os.remove(results_file)

    scenarios = [
        {'name': 'map0_goal1', 'file': 'map0.png', 'start': [10, 10], 'goal': [40, 110]},
        {'name': 'map0_goal2', 'file': 'map0.png', 'start': [10, 10], 'goal': [70, 90]},
        {'name': 'map1', 'file': 'map1.png', 'start': [60, 60], 'goal': [60, 90]},
        {'name': 'map2', 'file': 'map2.png', 'start': [8, 31], 'goal': [38, 139]},
        {'name': 'map3', 'file': 'map3.png', 'start': [50, 90], 'goal': [375, 375]}
    ]

    for sc in scenarios:
        # Note on goal coordinates: in notebook, plotting was plt.plot(goal[1], goal[0]) 
        # meaning goal[0] is y (row) and goal[1] is x (col). 
        # The dictionaries in notebook: map_0_first_position: [110, 40] where 110 is likely X and 40 is Y.
        # But we define them as [row, col] here for consistency, so reversed.
        
        run_scenario(
            map_name=sc['name'],
            map_file=os.path.join(data_dir, sc['file']),
            start=sc['start'],
            goal=sc['goal'],
            save_dir=save_dir,
            results_file=results_file
        )
        
    print("Pipeline execution completed. Check 'results/' and 'report/result_images/' for outputs.")

if __name__ == "__main__":
    main()
