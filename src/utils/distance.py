import math
import numpy as np

def compute_2d_distance(pts: np.ndarray) -> np.ndarray:
    """Compute the 2D Euclidean distance for each point."""
    return np.sqrt(np.square(pts[:, 0]) + np.square(pts[:, 1]))


def euclid_dist(loc1, loc2):
    """Compute the Euclidean distance between two locations."""
    dx = loc1.x - loc2.x
    dy = loc1.y - loc2.y
    dz = loc1.z - loc2.z
    return math.sqrt(dx * dx + dy * dy + dz * dz)