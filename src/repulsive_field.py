"""Repulsive potential field computation for grid-based path planning.

Implements the standard repulsive potential function that generates a repulsive
force from obstacles. Cells within the influence distance Q receive a non-zero
potential, pushing the robot away from obstacles.
"""

import numpy as np


def compute_repulsive_field(distance_map, influence_distance, scaling_factor):
    """Compute the repulsive potential field from an obstacle distance map.

    The repulsive potential is defined as:
        U_rep(q) = 0.5 * eta * (1/d(q) - 1/Q)^2   if d(q) <= Q
        U_rep(q) = 0                                 if d(q) > Q

    where eta is the scaling factor, d(q) is the distance to the nearest
    obstacle, and Q is the influence distance.

    Args:
        distance_map: 2D numpy array of distances to nearest obstacle.
                      Obstacle cells should have been offset by +1 before
                      passing to avoid division by zero.
        influence_distance: Influence distance Q (cells within this range
                            are affected).
        scaling_factor: Scaling factor eta for the potential.

    Returns:
        2D numpy array representing the repulsive potential map.
    """
    result = distance_map.copy().astype(float)

    for i, j in np.ndindex(result.shape):
        d = result[i][j]
        if d <= influence_distance:
            result[i][j] = 0.5 * scaling_factor * ((1.0 / d - 1.0 / influence_distance) ** 2)
        else:
            result[i][j] = 0.0

    return result
