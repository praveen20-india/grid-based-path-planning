"""Greedy descent path extraction on wavefront maps.

Implements greedy gradient descent on a wavefront (distance-to-goal) map.
Unlike potential-field gradient descent, the wavefront approach guarantees
convergence to the goal since the wavefront map has a single global minimum.
"""

import numpy as np


def greedy_descent_path(wavefront_map, start, goal, connectivity=4):
    """Extract a path from start to goal using greedy descent on a wavefront map.

    At each step, the algorithm moves to the neighboring cell with the
    smallest distance-to-goal value.

    Args:
        wavefront_map: 2D numpy array of distance-to-goal values.
        start: Start position as [row, col].
        goal: Goal position as [row, col].
        connectivity: Grid connectivity — 4 or 8.

    Returns:
        Tuple (trajectory_rows, trajectory_cols) as lists of coordinates.
    """
    current = list(start)
    trajectory_rows = [current[0]]
    trajectory_cols = [current[1]]

    if connectivity == 4:
        move_type = [[1, 0], [0, 1], [-1, 0], [0, -1]]
    elif connectivity == 8:
        move_type = [
            [1, 0], [1, 1], [-1, 1], [-1, -1],
            [0, 1], [0, -1], [-1, 0], [1, -1],
        ]
    else:
        raise ValueError(f"Unsupported connectivity: {connectivity}. Use 4 or 8.")

    while current[0] != goal[0] or current[1] != goal[1]:
        best_value = wavefront_map[current[0]][current[1]]
        next_cell = current

        for move in move_type:
            nr, nc = current[0] + move[0], current[1] + move[1]
            if 0 <= nr < wavefront_map.shape[0] and 0 <= nc < wavefront_map.shape[1]:
                if wavefront_map[nr][nc] < best_value:
                    best_value = wavefront_map[nr][nc]
                    next_cell = [nr, nc]

        if next_cell == current:
            # Reached a dead end or local minimum (shouldn't happen on a perfect wavefront but good safety)
            break

        current = next_cell
        trajectory_rows.append(current[0])
        trajectory_cols.append(current[1])

    return trajectory_rows, trajectory_cols
