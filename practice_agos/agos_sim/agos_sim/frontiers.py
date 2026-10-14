"""
Frontier detection and TAD frontier scoring on an occupancy grid.

Plain numpy, no ROS, so it can be tested on its own (test/test_frontiers.py).
agos_sim/explorer.py runs it on RTAB-Map's /map.

TAD (Trapezoid, Adjacent, Distance) is the frontier selection the AGOS
proposal names for SO3: Buriboev, Choi and Jeon, "Optimized frontier-based
path planning using the TAD algorithm for efficient autonomous exploration",
Electronics 14(1):74, 2025. Frontiers are clusters of free cells next to
unknown space. Each one gets a rectangle (its bounding box) and the score

    F_n = d_n + a_n + t_n                     (equal weights, as in the paper)

    d_n  Distance: robot to frontier centre in metres, scaled up by how much
         of the area around the frontier is still unexplored:
         d_n * (1 + A_unexplored / A_total), counted in a disc of
         AREA_RADIUS around the centre.
    a_n  Adjacent: 1 + the number of other frontier rectangles that overlap
         or touch this one (so 1..n).
    t_n  Trapezoid: +1 if the frontier opens out (the gap into unknown space
         is wider than the free passage leading up to it), 0 if they are the
         same width, -1 if it narrows.

The frontier with the highest score is the next goal, and its centre (moved
to the nearest cell the robot can reach) is where the robot is sent.

The grid is first pooled to COARSE metres per cell (0.05 m RTAB-Map cells ->
0.1 m), which keeps everything here fast enough for a Raspberry Pi.

In a pipe the LiDAR meets the walls at a shallow angle a few metres ahead,
so RTAB-Map's map has gaps in the walls there, and the free cells next to
those gaps look like frontiers. Gaps of one or two cells (up to 0.2 m) are
closed before frontiers are looked for (closing wider gaps would also fill
in narrow passages: this keeps any passage 0.5 m or wider open). The wider
gaps further ahead fill in as the robot drives up to them; any that never
do are skipped once the robot has been there (explorer.py).
"""
import heapq
import math

import numpy as np

COARSE = 0.1            # m per cell for all of the processing here
AREA_RADIUS = 2.0       # m, disc around a frontier centre for A_unexplored / A_total
ADJACENT_GAP = 0.5      # m, frontier rectangles closer than this count as adjacent
INNER_OFFSET = 0.6      # m, how far behind a frontier its passage width is measured
TRAPEZOID_TOL = 0.25    # outer/inner within +-25 % counts as the same width (t = 0)
WALL_GAP = 0.2          # m, closing radius: closes gaps up to 0.2 m, keeps passages >= 0.5 m

OCCUPIED = 50           # occupancy value at or above which a cell is an obstacle

# 8-connected neighbours: (row step, column step, length in cells)
NEIGHBOURS = [(-1, -1, math.sqrt(2)), (-1, 0, 1.0), (-1, 1, math.sqrt(2)), (0, -1, 1.0),
              (0, 1, 1.0), (1, -1, math.sqrt(2)), (1, 0, 1.0), (1, 1, math.sqrt(2))]
# path steps for distances: the 8 neighbours plus the 8 knight's moves, which
# keep grid path lengths within 3 % of the straight-line length at any angle
# (8 neighbours alone over-count by up to 8 %). A knight's move needs both
# cells it passes over to be open too: [(row, col), (row, col)] relative.
KNIGHT = [(dr, dc) for dr in (-2, -1, 1, 2) for dc in (-2, -1, 1, 2) if abs(dr) != abs(dc)]


def _over(dr, dc):
    if abs(dr) == 1:
        return [(0, dc // 2), (dr, dc // 2)]
    return [(dr // 2, 0), (dr // 2, dc)]


def shift(mask, dr, dc, fill=False):
    """Move a mask by (dr, dc) cells; cells moved in from outside get `fill`."""
    out = np.full_like(mask, fill)
    h, w = mask.shape
    out[max(dr, 0):h + min(dr, 0), max(dc, 0):w + min(dc, 0)] = \
        mask[max(-dr, 0):h + min(-dr, 0), max(-dc, 0):w + min(-dc, 0)]
    return out


def dilate(mask, radius):
    """
    Grow a boolean mask by a disc of `radius` cells.

    The disc is r^2 + r rather than r^2, so its edge is round on a grid (a
    plain r^2 disc ends in a single cell, and closing gaps with it fails).
    """
    out = mask.copy()
    for dr in range(-radius, radius + 1):
        for dc in range(-radius, radius + 1):
            if (dr or dc) and dr * dr + dc * dc <= radius * (radius + 1):
                out |= shift(mask, dr, dc)
    return out


def erode(mask, radius):
    """Shrink a boolean mask by a disc of `radius` cells."""
    return ~dilate(~mask, radius)


def close_gaps(mask, radius):
    """Fill gaps up to 2 * radius cells wide between True cells."""
    return erode(dilate(mask, radius), radius) | mask


def box_sum(mask, radius):
    """Count the True cells in the (2 radius + 1)^2 square around each cell."""
    c = np.pad(mask.astype(np.int32), radius + 1).cumsum(0).cumsum(1)
    k = 2 * radius + 1
    return c[k:, k:] - c[:-k, k:] - c[k:, :-k] + c[:-k, :-k]


def label(mask):
    """8-connected components of a boolean mask: (labels, count), 0 = background."""
    labels = np.zeros(mask.shape, np.int32)
    h, w = mask.shape
    count = 0
    for r0, c0 in zip(*np.nonzero(mask)):
        if labels[r0, c0]:
            continue
        count += 1
        labels[r0, c0] = count
        stack = [(r0, c0)]
        while stack:
            r, c = stack.pop()
            for dr, dc, _ in NEIGHBOURS:
                rr, cc = r + dr, c + dc
                if 0 <= rr < h and 0 <= cc < w and mask[rr, cc] and not labels[rr, cc]:
                    labels[rr, cc] = count
                    stack.append((rr, cc))
    return labels, count


def angle_diff(a, b):
    """Smallest difference between two angles in radians (0..pi)."""
    return abs((a - b + math.pi) % (2 * math.pi) - math.pi)


class Grid:
    """
    An occupancy grid pooled to COARSE m cells: free, occupied and unknown masks.

    The data is nav_msgs/OccupancyGrid data: row-major, row 0 at origin_y.
    """

    def __init__(self, data, width, height, resolution, origin_x, origin_y):
        a = np.asarray(data, np.int16).reshape(height, width)
        k = max(1, int(round(COARSE / resolution)))
        h, w = -(-height // k), -(-width // k)
        pad = np.full((h * k, w * k), -1, np.int16)
        pad[:height, :width] = a
        blocks = pad.reshape(h, k, w, k)
        self.occ = (blocks >= OCCUPIED).any(axis=(1, 3))
        self.free = ((blocks >= 0) & (blocks < OCCUPIED)).any(axis=(1, 3)) & ~self.occ
        self.unknown = ~self.free & ~self.occ
        self.res = resolution * k
        self.ox, self.oy = origin_x, origin_y
        self.shape = self.free.shape

    def cell(self, x, y):
        return int(math.floor((y - self.oy) / self.res)), int(math.floor((x - self.ox) / self.res))

    def xy(self, r, c):
        return self.ox + (c + 0.5) * self.res, self.oy + (r + 0.5) * self.res

    def inside(self, r, c):
        return 0 <= r < self.shape[0] and 0 <= c < self.shape[1]


def geodesic(mask, start, exit_cells=None):
    """
    Dijkstra over the True cells of `mask` (8-connected) from cell `start`.

    Returns (dist, branch): dist in cells (inf where unreachable), and, if
    exit_cells is given, branch[r, c] = the bearing (rad) from `start` of the
    point where the shortest path to (r, c) first gets exit_cells away from
    the start (nan for cells closer than that). The branch tells which
    pipe out of the start manhole a cell is reached through.
    """
    h, w = mask.shape
    n = h * w
    dist = [math.inf] * n
    branch = [math.nan] * n if exit_cells is not None else None
    flat = mask.ravel().tolist()
    r0, c0 = start
    if not (0 <= r0 < h and 0 <= c0 < w) or not flat[r0 * w + c0]:
        out = np.full((h, w), np.inf)
        return out, (np.full((h, w), np.nan) if exit_cells is not None else None)
    s = r0 * w + c0
    dist[s] = 0.0
    heap = [(0.0, s)]
    steps = [(dr * w + dc, dr, dc, length, ()) for dr, dc, length in NEIGHBOURS]
    steps += [(dr * w + dc, dr, dc, math.sqrt(5.0), tuple(a * w + b for a, b in _over(dr, dc)))
              for dr, dc in KNIGHT]
    while heap:
        d, i = heapq.heappop(heap)
        if d > dist[i]:
            continue
        r, c = divmod(i, w)
        for off, dr, dc, length, over in steps:
            rr, cc = r + dr, c + dc
            if 0 <= rr < h and 0 <= cc < w:
                j = i + off
                nd = d + length
                if flat[j] and nd < dist[j] and all(flat[i + o] for o in over):
                    dist[j] = nd
                    if branch is not None:
                        if nd >= exit_cells and d < exit_cells:
                            branch[j] = math.atan2(rr - r0, cc - c0)
                        else:
                            branch[j] = branch[i]
                    heapq.heappush(heap, (nd, j))
    dist = np.array(dist).reshape(h, w)
    if branch is not None:
        branch = np.array(branch).reshape(h, w)
    return dist, branch


class Frontier:
    """One frontier cluster and its TAD score."""

    def __init__(self, cells, centroid, goal_cell, goal, size, rect):
        self.cells = cells          # (k, 2) array of (row, col)
        self.centroid = centroid    # (x, y) m
        self.goal_cell = goal_cell  # (row, col) reachable cell nearest the centre
        self.goal = goal            # (x, y) m
        self.size = size            # m along the frontier
        self.rect = rect            # (xmin, ymin, xmax, ymax) m
        self.d = self.d_adj = self.unknown_ratio = 0.0
        self.a = 1
        self.t = 0
        self.outer = self.inner = 0.0
        self.path = math.inf        # path length from the robot, m
        self.from_start = math.inf  # path length from the mission start, m
        self.branch = math.nan      # bearing of the pipe it is reached through
        self.score = 0.0

    def __repr__(self):
        return ('Frontier(goal=(%.2f, %.2f) size=%.2f F=%.2f = d %.2f + a %d + t %d)'
                % (self.goal[0], self.goal[1], self.size, self.score, self.d_adj, self.a, self.t))


class Snapshot:
    """
    Everything the explorer needs from one map.

    Where the robot can go, how far everything is from it (and from the
    mission start), the frontiers and their TAD scores.

    clearance: m an obstacle must be from a cell for the robot's centre to go
    there (the robot is 0.38 m wide). exit_radius: m from the start beyond
    which a path counts as having left the start manhole through a pipe.
    """

    def __init__(self, grid, robot_xy, start_xy=None, clearance=0.2, exit_radius=2.5):
        self.g = grid
        self.robot_xy = robot_xy
        # walls with their small gaps closed (see WALL_GAP)
        self.walls = close_gaps(grid.occ, max(1, int(round(WALL_GAP / grid.res))))
        radius = max(1, int(math.ceil(clearance / grid.res - 1e-9)))
        self.traversable = grid.free & ~dilate(self.walls, radius)
        self.robot_cell = self.snap(robot_xy)
        self.dist, _ = geodesic(self.traversable, self.robot_cell) if self.robot_cell else \
            (np.full(grid.shape, np.inf), None)
        self.start_xy = start_xy
        self.dist_start = self.branch = None
        if start_xy is not None:
            sc = self.snap(start_xy)
            self.dist_start, self.branch = geodesic(
                self.traversable, sc if sc else (-1, -1), exit_cells=exit_radius / grid.res)

    def snap(self, xy, max_m=0.6):
        """Nearest traversable cell to a point (within max_m), or None."""
        r0, c0 = self.g.cell(*xy)
        reach = int(math.ceil(max_m / self.g.res))
        best, best_d = None, math.inf
        for r in range(r0 - reach, r0 + reach + 1):
            for c in range(c0 - reach, c0 + reach + 1):
                if self.g.inside(r, c) and self.traversable[r, c]:
                    d = (r - r0) ** 2 + (c - c0) ** 2
                    if d < best_d:
                        best, best_d = (r, c), d
        return best

    def path_length(self, cell):
        return self.dist[cell] * self.g.res

    # ------------------------------------------------------------ frontiers

    def frontiers(self, min_size=0.3):
        """Reachable frontier clusters with their TAD scores, best first."""
        g = self.g
        unknown = g.unknown & ~self.walls
        unknown_next = (shift(unknown, 1, 0, True) | shift(unknown, -1, 0, True)
                        | shift(unknown, 0, 1, True) | shift(unknown, 0, -1, True))
        edge = g.free & ~self.walls & unknown_next
        # frontier cells one cell apart are the same frontier
        labels, count = label(dilate(edge, 1))
        reachable = np.isfinite(self.dist)
        # how much room there is around each cell (traversable cells within 0.2 m)
        self._room = box_sum(self.traversable, 2)
        out = []
        for k in range(1, count + 1):
            cells = np.argwhere((labels == k) & edge)
            if len(cells) * g.res < min_size:
                continue
            f = self._make(cells, reachable)
            if f is not None:
                out.append(f)
        self._score(out)
        out.sort(key=lambda f: -f.score)
        return out

    def _make(self, cells, reachable):
        g = self.g
        xs = g.ox + (cells[:, 1] + 0.5) * g.res
        ys = g.oy + (cells[:, 0] + 0.5) * g.res
        centroid = (float(xs.mean()), float(ys.mean()))
        # goal: a reachable cell close to the frontier, as far from the walls
        # as possible (the middle of the pipe), nearest the centre after that
        near = np.zeros(g.shape, bool)
        near[cells[:, 0], cells[:, 1]] = True
        near = dilate(near, 3) & reachable
        cand = np.argwhere(near)
        if len(cand) == 0:
            return None
        room = self._room[cand[:, 0], cand[:, 1]]
        cand = cand[room >= room.max()]
        cx = g.ox + (cand[:, 1] + 0.5) * g.res
        cy = g.oy + (cand[:, 0] + 0.5) * g.res
        i = int(np.argmin((cx - centroid[0]) ** 2 + (cy - centroid[1]) ** 2))
        goal_cell = (int(cand[i, 0]), int(cand[i, 1]))
        rect = (float(xs.min() - g.res / 2), float(ys.min() - g.res / 2),
                float(xs.max() + g.res / 2), float(ys.max() + g.res / 2))
        f = Frontier(cells, centroid, goal_cell, g.xy(*goal_cell), len(cells) * g.res, rect)
        f.path = self.path_length(goal_cell)
        if self.dist_start is not None:
            f.from_start = self.dist_start[goal_cell] * g.res
            f.branch = float(self.branch[goal_cell])
        return f

    def _score(self, frontiers):
        g = self.g
        rx, ry = self.robot_xy
        rad = int(round(AREA_RADIUS / g.res))
        yy, xx = np.mgrid[-rad:rad + 1, -rad:rad + 1]
        disc = xx ** 2 + yy ** 2 <= rad ** 2
        for f in frontiers:
            # Distance, scaled by the unexplored share of the area around it
            f.d = math.hypot(f.centroid[0] - rx, f.centroid[1] - ry)
            r0, c0 = g.cell(*f.centroid)
            rr, cc = r0 + yy[disc], c0 + xx[disc]
            inside = (rr >= 0) & (rr < g.shape[0]) & (cc >= 0) & (cc < g.shape[1])
            unknown = (~inside).sum() + g.unknown[rr[inside], cc[inside]].sum()
            f.unknown_ratio = float(unknown) / float(disc.sum())
            f.d_adj = f.d * (1.0 + f.unknown_ratio)
            # Adjacent: rectangles that overlap or touch (within ADJACENT_GAP)
            f.a = 1 + sum(1 for o in frontiers
                          if o is not f and _touch(f.rect, o.rect, ADJACENT_GAP))
            # Trapezoid: does the space open out beyond the frontier?
            f.outer, f.inner, f.t = self._trapezoid(f)
            f.score = f.d_adj + f.a + f.t

    def _trapezoid(self, f):
        g = self.g
        # direction into the unknown: from the frontier cells to their unknown neighbours
        v = np.zeros(2)
        for r, c in f.cells:
            for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                rr, cc = r + dr, c + dc
                if not g.inside(rr, cc) or g.unknown[rr, cc]:
                    v += (dc, dr)
        if not v.any():
            return 0.0, 0.0, 0
        n = v / np.linalg.norm(v)
        p = np.array([-n[1], n[0]])
        xy = np.c_[f.cells[:, 1] + 0.5, f.cells[:, 0] + 0.5] * g.res
        across = xy @ p
        outer = float(across.max() - across.min() + g.res)
        inner = 0.0
        for back in (INNER_OFFSET, INNER_OFFSET * 0.5, INNER_OFFSET * 1.5):
            q = np.array(f.centroid) - n * back
            inner = self._width(q, p)
            if inner > 0:
                break
        if inner <= 0:
            return outer, inner, 0
        ratio = outer / inner
        t = 1 if ratio > 1 + TRAPEZOID_TOL else (-1 if ratio < 1 - TRAPEZOID_TOL else 0)
        return outer, inner, t

    def _width(self, q, p):
        """Width of free space through point q along direction p (m)."""
        g = self.g
        r, c = g.cell(*q)
        if not g.inside(r, c) or not g.free[r, c]:
            return 0.0
        width = 0.0
        for sign in (1.0, -1.0):
            s = g.res / 2
            while s < 5.0:
                x, y = q + sign * s * p
                r, c = g.cell(x, y)
                if not g.inside(r, c) or not g.free[r, c]:
                    break
                s += g.res / 2
            width += s
        return width

    # ------------------------------------------------------------ mission legs

    def in_leg(self, f, bearing, half_angle, max_distance):
        """
        Check whether frontier f is in the leg that leaves the start along `bearing`.

        That is: within half_angle of it, and no further than max_distance
        along the pipe (0 = no limit).
        """
        if self.dist_start is None or not math.isfinite(f.from_start):
            return False
        if max_distance > 0 and f.from_start > max_distance:
            return False
        if math.isnan(f.branch):
            # Close to the start (in the manhole or just inside a pipe): no
            # pipe to go by yet, so go by the direction from the start. (A
            # frontier just inside the pipe behind would otherwise count in
            # both legs, and pull the robot back from far along the leg.)
            if self.start_xy is None:
                return True
            towards = math.atan2(f.goal[1] - self.start_xy[1], f.goal[0] - self.start_xy[0])
            return angle_diff(towards, bearing) <= half_angle
        return angle_diff(f.branch, bearing) <= half_angle

    def leg_end(self, bearing, half_angle, max_distance):
        """
        Find the reachable cell furthest along the leg: ((x, y), distance) or None.

        At most max_distance from the start along the pipe (0 = no limit).
        """
        if self.dist_start is None:
            return None
        d = self.dist_start * self.g.res
        ok = np.isfinite(d) & np.isfinite(self.dist) & ~np.isnan(self.branch)
        if max_distance > 0:
            ok &= d <= max_distance
        b = np.where(ok, self.branch, 0.0)
        diff = np.abs((b - bearing + np.pi) % (2 * np.pi) - np.pi)
        ok &= diff <= half_angle
        if not ok.any():
            return None
        d = np.where(ok, d, -1.0)
        r, c = np.unravel_index(int(np.argmax(d)), d.shape)
        return self.g.xy(r, c), float(d[r, c])

    def explored_along(self, bearing, half_angle):
        """Measure how far (m along the pipes) the known free space in this leg reaches."""
        end = self.leg_end(bearing, half_angle, 0.0)
        return end[1] if end else 0.0


def _touch(a, b, gap):
    return not (a[2] + gap < b[0] or b[2] + gap < a[0]
                or a[3] + gap < b[1] or b[3] + gap < a[1])
