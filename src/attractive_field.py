"""Attractive potential field computation for grid-based path planning.

Implements quadratic attractive potential functions using various distance
metrics: Euclidean, Manhattan, and grid-based (4- or 8-connectivity) distances.
The attractive field draws the robot toward the goal position.
"""

import numpy as np
from collections import deque


def euclidean_distance(p1, p2):
    """Compute the Euclidean distance between two grid points.

    Args:
        p1: First point as [row, col].
        p2: Second point as [row, col].

    Returns:
        Euclidean distance as a float.
    """
    return np.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)


def manhattan_distance(p1, p2):
    """Compute the Manhattan distance between two grid points.

    Args:
        p1: First point as [row, col].
        p2: Second point as [row, col].

    Returns:
        Manhattan distance as a float.
    """
    return np.abs(p1[0] - p2[0]) + np.abs(p1[1] - p2[1])


def compute_attractive_field(grid_map, goal, scaling_factor, mode="euclidean"):
    """Compute the attractive potential field over a grid map.

    The quadratic attractive potential is defined as:
        U_att(q) = 0.5 * xi * d(q, q_goal)^2

    where xi is the scaling factor and d() is the chosen distance metric.

    Args:
        grid_map: 2D numpy array representing the grid map.
        goal: Goal position as [row, col].
        scaling_factor: Scaling factor xi for the potential.
        mode: Distance metric — 'euclidean', 'manhattan', '4_point', or '8_point'.

    Returns:
        2D numpy array representing the attractive potential map.
    """
    result = grid_map.copy().astype(float)
    goal = np.array([goal[0], goal[1]])
    result[:] = 0

    if mode in ["euclidean", "manhattan"]:
        dist_fn = euclidean_distance if mode == "euclidean" else manhattan_distance
        for i, j in np.ndindex(result.shape):
            result[i][j] = 0.5 * scaling_factor * (dist_fn([i, j], goal) ** 2)
        return result

    # BFS-based distance for grid connectivity modes
    move_type = []
    result[goal[0]][goal[1]] = 1
    queue = deque([[goal[0], goal[1]]])

    if mode == "4_point":
        move_type = [[-1, 0], [0, -1], [1, 0], [0, 1]]
    elif mode == "8_point":
        move_type = [
            [-1, 0], [-1, -1], [0, -1], [1, -1],
            [1, 0], [1, 1], [0, 1], [-1, 1],
        ]

    while len(queue) != 0:
        t = queue.popleft()
        for move in move_type:
            nr, nc = t[0] + move[0], t[1] + move[1]
            if nr >= result.shape[0] or nc >= result.shape[1] or nr < 0 or nc < 0:
                continue
            if result[nr][nc] == 0:
                result[nr][nc] = result[t[0], t[1]] + 1
                queue.append([nr, nc])

    result[:] = result[:] - 1

    for i, j in np.ndindex(result.shape):
        result[i][j] = 0.5 * scaling_factor * (result[i][j] ** 2)

    return result
