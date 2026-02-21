"""Wavefront planner for grid-based path planning.

Implements a BFS-based wavefront expansion from the goal position, assigning
monotonically increasing distance values to all reachable free cells. This
creates a navigation function with a guaranteed global minimum at the goal,
eliminating the local-minima problem inherent in potential field methods.
"""

import numpy as np


from collections import deque

def is_valid_cell(grid_map, point):
    """Check if a cell is within bounds and not an obstacle.

    Args:
        grid_map: 2D numpy array representing the grid.
        point: Cell coordinates as (row, col).

    Returns:
        True if the cell is valid for movement.
    """
    if point[0] < 0 or point[0] >= grid_map.shape[0]:
        return False
    if point[1] < 0 or point[1] >= grid_map.shape[1]:
        return False
    if grid_map[point[0]][point[1]] == np.inf:
        return False
    return True


def compute_wavefront(grid_map, goal, connectivity=4):
    """Compute the wavefront distance-to-goal map via BFS.

    Starting from the goal cell (distance 0), the algorithm propagates
    outward through free space, assigning increasing distance values.
    Obstacle cells are marked as infinity.

    Args:
        grid_map: 2D numpy array (0 = free, 1 = occupied).
        goal: Goal position as [row, col].
        connectivity: Grid connectivity — 4 or 8.

    Returns:
        Tuple (distance_map, visualization_map):
            - distance_map: obstacles are np.inf, free cells have distance values.
            - visualization_map: same but obstacles are set to 0 for display.
    """
    result = grid_map.copy().astype(float)

    # Mark obstacles as infinity, free cells with a large sentinel value
    for i, j in np.ndindex(result.shape):
        if result[i][j] == 1:
            result[i][j] = np.inf
        else:
            result[i][j] = result[i][j] + 1e9

    # Set goal distance to 0
    result[goal[0]][goal[1]] = 0
    queue = deque([[goal[0], goal[1]]])

    # Define movement patterns
    if connectivity == 4:
        move_type = [[1, 0], [0, 1], [-1, 0], [0, -1]]
    elif connectivity == 8:
        move_type = [
            [-1, 0], [-1, -1], [0, -1], [1, -1],
            [1, 0], [1, 1], [0, 1], [-1, 1],
        ]
    else:
        raise ValueError(f"Unsupported connectivity: {connectivity}. Use 4 or 8.")

    # BFS expansion
    while len(queue) != 0:
        t = queue.popleft()
        for move in move_type:
            nr, nc = t[0] + move[0], t[1] + move[1]
            if not is_valid_cell(result, (nr, nc)):
                continue
            if result[nr][nc] != np.inf and result[nr][nc] > result[t[0], t[1]] + 1:
                result[nr][nc] = result[t[0], t[1]] + 1
                queue.append([nr, nc])

    # Create visualization map (replace inf with 0 for display)
    vis_map = result.copy()
    for i, j in np.ndindex(vis_map.shape):
        if vis_map[i][j] == np.inf:
            vis_map[i][j] = 0

    return result, vis_map
