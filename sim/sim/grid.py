"""Simulator-only static oracle occupancy and clearance-aware A*; no ROS imports."""
import heapq
import math
import numpy as np
from scipy.ndimage import distance_transform_edt
from .geometry import polygon_contains


class Grid:
    def __init__(self, occupied, resolution, origin):
        self.occupied = np.asarray(occupied, dtype=bool)
        self.resolution, self.origin = resolution, np.asarray(origin)

    @classmethod
    def from_polygons(cls, polygons, resolution, extent):
        count = math.ceil(2*extent/resolution)
        yy, xx = np.mgrid[:count, :count]
        points = np.column_stack([xx.ravel(), yy.ravel()])*resolution-extent+resolution/2
        occupied = np.zeros(len(points), dtype=bool)
        # Expand each convex footprint by half cell diagonal so thin dock panels cannot disappear.
        from scipy.spatial import ConvexHull
        for polygon in polygons:
            p = np.asarray(polygon)
            offsets = np.array([[1,1], [1,-1], [-1,1], [-1,-1]])*resolution/2
            expanded = (p[:, None, :] + offsets).reshape(-1, 2)
            expanded = expanded[ConvexHull(expanded).vertices]
            occupied |= polygon_contains(expanded, points)
        return cls(occupied.reshape(count, count), resolution, [-extent, -extent])

    def cell(self, point):
        x, y = np.floor((np.asarray(point)-self.origin)/self.resolution).astype(int)
        return int(y), int(x)

    def point(self, cell):
        return self.origin + (np.array([cell[1], cell[0]])+.5)*self.resolution

    def inside(self, cell):
        return 0 <= cell[0] < self.occupied.shape[0] and 0 <= cell[1] < self.occupied.shape[1]

    def clear_line(self, start, end, blocked=None):
        blocked = self.occupied if blocked is None else blocked
        steps = max(1, math.ceil(np.linalg.norm(np.asarray(end)-start)/(self.resolution/2)))
        for point in np.linspace(start, end, steps+1):
            cell = self.cell(point)
            if not self.inside(cell) or blocked[cell]:
                return False
        return True

    def plan(self, start, goal, radius):
        blocked = distance_transform_edt(~self.occupied)*self.resolution <= radius
        a, b = self.cell(start), self.cell(goal)
        if any(not self.inside(p) or blocked[p] for p in (a, b)):
            return []
        frontier, costs, previous = [(0., a)], {a: 0.}, {}
        while frontier:
            _, here = heapq.heappop(frontier)
            if here == b:
                cells = [b]
                while cells[-1] != a:
                    cells.append(previous[cells[-1]])
                points = [np.asarray(start)] + [self.point(c) for c in reversed(cells)][1:] + [np.asarray(goal)]
                # Visibility simplification checked on the inflated grid, no corner cutting.
                result, i = [points[0]], 0
                while i < len(points)-1:
                    j = i+1
                    while j+1 < len(points) and self.clear_line(points[i], points[j+1], blocked):
                        j += 1
                    result.append(points[j]); i = j
                return [p.tolist() for p in result]
            for dy, dx in [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]:
                nxt = here[0]+dy, here[1]+dx
                if not self.inside(nxt) or blocked[nxt]:
                    continue
                if dx and dy and (blocked[here[0]+dy, here[1]] or blocked[here[0], here[1]+dx]):
                    continue
                cost = costs[here]+math.hypot(dx,dy)
                if cost < costs.get(nxt, math.inf):
                    costs[nxt], previous[nxt] = cost, here
                    heapq.heappush(frontier, (cost+math.hypot(nxt[0]-b[0],nxt[1]-b[1]), nxt))
        return []
