"""Planar geometry for true footprint contact and conservative static visibility."""
import math
import numpy as np


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def rectangle(x, y, heading, length, width):
    local = np.array([[length, width], [-length, width], [-length, -width], [length, -width]]) / 2
    c, s = math.cos(heading), math.sin(heading)
    return local @ np.array([[c, s], [-s, c]]) + [x, y]


def overlap(a, b):
    """Separating-axis test: rotated polygons, not circle/AABB overlaps."""
    for p in (a, b):
        edges = np.roll(p, -1, axis=0) - p
        for edge in edges:
            axis = np.array([-edge[1], edge[0]])
            if np.dot(axis, axis) == 0:
                continue
            u, v = a @ axis, b @ axis
            if u.max() < v.min() or v.max() < u.min():
                return False
    return True


def separation(a, b):
    if overlap(a, b):
        return 0.0
    distances = []
    for points, edges in [(a, b), (b, a)]:
        for start, end in zip(edges, np.roll(edges, -1, axis=0)):
            delta = end-start
            t = np.clip((points-start) @ delta / (delta @ delta), 0, 1)
            distances.extend(np.linalg.norm(points-(start+t[:, None]*delta), axis=1))
    return float(min(distances))


def polygon_contains(poly, points):
    """Convex polygon containment, vectorised for occupancy rasterisation."""
    edges = np.roll(poly, -1, axis=0) - poly
    rel = points[:, None, :] - poly[None, :, :]
    cross = edges[None, :, 0]*rel[:, :, 1] - edges[None, :, 1]*rel[:, :, 0]
    return np.all(cross >= 0, axis=1) | np.all(cross <= 0, axis=1)
