"""Brushfire (distance transform) algorithm for obstacle proximity computation.

Implements a BFS-based distance transform that computes the distance from each
free cell to the nearest obstacle. Supports both 4-connectivity and 8-connectivity
grid topologies.
"""

import numpy as np
from collections import deque


def compute_obstacle_distance(grid_map, connectivity=4):
    """Compute the distance to the nearest obstacle for every cell.

    Uses a BFS-based brushfire algorithm starting from all obstacle cells
    simultaneously, propagating outward through free space.

    Args:
        grid_map: 2D numpy array (0 = free, 1 = occupied).
        connectivity: Grid connectivity — 4 or 8.

    Returns:
        2D numpy array where each cell contains the distance to the
        nearest obstacle.
    """
    result = grid_map.copy().astype(float)
    queue = deque([])

    # Initialize: collect all obstacle cells
    for i, j in np.ndindex(result.shape):
        if result[i][j] == 1:
            queue.append([i, j])

    # Define movement patterns
    if connectivity == 4:
        move_type = [[-1, 0], [0, -1], [1, 0], [0, 1]]
    elif connectivity == 8:
        move_type = [
            [-1, 0], [-1, -1], [0, -1], [1, -1],
            [1, 0], [1, 1], [0, 1], [-1, 1],
        ]
    else:
        raise ValueError(f"Unsupported connectivity: {connectivity}. Use 4 or 8.")

    # BFS propagation
    while len(queue) != 0:
        t = queue.popleft()
        for move in move_type:
            nr, nc = t[0] + move[0], t[1] + move[1]
            if nr >= result.shape[0] or nc >= result.shape[1] or nr < 0 or nc < 0:
                continue
            if result[nr][nc] == 0:
                result[nr][nc] = result[t[0], t[1]] + 1
                queue.append([nr, nc])

    # Adjust so obstacle cells have distance 0
    result[:] = result[:] - 1

    return result
