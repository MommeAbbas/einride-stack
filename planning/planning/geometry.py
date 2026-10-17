"""Planar geometry for footprint contact and conservative static visibility."""
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


def rotation(heading):
    return np.array([[np.cos(heading),-np.sin(heading)],[np.sin(heading),np.cos(heading)]])


class FrameGrid:
    """Grid queried in odom, stored in map; rigid corrections never move particles."""
    def __init__(self,grid,map_from_odom):
        self.grid=grid; self.occupied=grid.occupied; self.resolution=grid.resolution
        self.transform=np.asarray(map_from_odom); self.rot=rotation(self.transform[2])
    def to_map(self,points): return np.asarray(points)@self.rot.T+self.transform[:2]
    def to_odom(self,points): return (np.asarray(points)-self.transform[:2])@self.rot
    def cells(self,p): return self.grid.cells(self.to_map(p))
    def points(self,c): return self.to_odom(self.grid.points(c))
    def cell(self,p): return self.grid.cell(self.to_map(p))
    def point(self,c): return self.to_odom(self.grid.point(c))
    def inside(self,c): return self.grid.inside(c)
    def clear_line(self,a,b,blocked=None): return self.grid.clear_line(self.to_map(a),self.to_map(b),blocked)
