"""Gradient descent path extraction on potential fields.

Implements discrete gradient descent for navigating a robot from start to goal
on a total potential field (attractive + repulsive). Supports both 4-connectivity
and 8-connectivity grid topologies.
"""

import numpy as np


def compute_total_potential(attractive_map, repulsive_map):
    """Compute the total potential field as the sum of attractive and repulsive.

    U_total(q) = U_att(q) + U_rep(q)

    Args:
        attractive_map: 2D numpy array of attractive potential.
        repulsive_map: 2D numpy array of repulsive potential.

    Returns:
        2D numpy array of total potential.
    """
    return attractive_map + repulsive_map


def gradient_descent_step(total_potential, current, connectivity=4):
    """Perform a single gradient descent step on the total potential field.

    Selects the neighboring cell (including the current cell) with the
    steepest decrease in potential.

    Args:
        total_potential: 2D numpy array of total potential values.
        current: Current position as [row, col].
        connectivity: Grid connectivity — 4 or 8.

    Returns:
        Next position as numpy array [row, col].
    """
    if connectivity == 4:
        move_type = [[0, 0], [0, 1], [-1, 0], [1, 0], [0, -1]]
    elif connectivity == 8:
        move_type = [
            [0, 0], [-1, -1], [-1, 0], [0, -1],
            [1, 0], [0, 1], [1, -1], [1, 1], [-1, 1],
        ]
    else:
        raise ValueError(f"Unsupported connectivity: {connectivity}. Use 4 or 8.")

    potential_diffs = []
    for move in move_type:
        diff = (
            total_potential[current[0] + move[0], current[1] + move[1]]
            - total_potential[current[0], current[1]]
        )
        potential_diffs.append(diff)

    # Select the direction with the minimum potential difference
    best_direction = min(zip(potential_diffs, move_type))[1]
    return np.array([
        current[0] + best_direction[0],
        current[1] + best_direction[1],
    ])


def gradient_descent_path(total_potential, start, goal, connectivity=4, max_iters=1000):
    """Extract a path from start to goal using iterative gradient descent.

    At each step, the algorithm moves to the neighboring cell with the
    steepest decrease in potential. The path terminates when the goal is
    reached or the maximum number of iterations is exceeded.

    Args:
        total_potential: 2D numpy array of total potential values.
        start: Start position as [row, col].
        goal: Goal position as [row, col].
        connectivity: Grid connectivity — 4 or 8.
        max_iters: Maximum number of iterations.

    Returns:
        Tuple (trajectory_rows, trajectory_cols) as lists of coordinates.
    """
    current = np.array(start)
    trajectory_rows = [int(current[0])]
    trajectory_cols = [int(current[1])]

    for _ in range(max_iters):
        if current[0] == goal[0] and current[1] == goal[1]:
            break
        current = gradient_descent_step(total_potential, current, connectivity)
        trajectory_rows.append(int(current[0]))
        trajectory_cols.append(int(current[1]))

    return trajectory_rows, trajectory_cols
