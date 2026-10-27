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

Exploring the whole map (explorer.py, mode full) uses the same scores, but
only among the frontiers nearest the robot (nearest_first): TAD favours
distant frontiers, and with the whole network open it would send the robot
back and forth between the two ends of what it has mapped. The robot also
keeps to the pipe it is in, and looks into the other pipes of a manhole
before it leaves them far behind (WholeMap). Routes longer than one Nav2
goal are driven in steps along the mapped pipes (route, waypoint). Where the
robot has already driven counts as passable whatever the map shows there
now: after a long drive RTAB-Map can draw a wall twice, a little apart, and
the pipe then looks too narrow on the map where it is not (Snapshot, driven).

The grid is first pooled to COARSE metres per cell (0.05 m RTAB-Map cells ->
0.1 m), which keeps everything here fast enough for a Raspberry Pi.

Driving forwards into new pipe (NAV_ALGORITHMS.md): the robot can only
turn round where it has room (manholes: Snapshot.manholes), and the camera
looks ahead, so new pipe is entered forwards and the robot reverses only
over pipe already mapped. Snapshot.approach plans that for a goal: drive on
if the route leaves ahead of the robot, else turn where it is, back up to
the first manhole on the route and turn there, or go on to a manhole ahead,
turn and come back. Each turn costs TURN_COST m (the yaw term in FUEL's
motion cost: Zhou et al., "FUEL: Fast UAV Exploration using Incremental
Frontier Structure and Hierarchical Planning", RA-L 6(2), 2021). A frontier
that could only be reached in reverse is not chosen. In full mode the next
frontier is the first of a tour through all of them and back home
(plan_tour: the global tour of TARE, Cao et al., RSS 2021, and FUEL), and
PipeGraph turns the map into manholes and the pipes between them.

In a pipe the LiDAR meets the walls at a shallow angle a few metres ahead,
so RTAB-Map's map has gaps in the walls there, and the free cells next to
those gaps look like frontiers. Gaps of one or two cells (up to 0.2 m) are
closed before frontiers are looked for (closing wider gaps would also fill
in narrow passages: this keeps any passage 0.5 m or wider open). The wider
gaps further ahead fill in as the robot drives up to them; any that never
do are skipped once the robot has been there (explorer.py). One or two
wall cells on their own in the middle of free space count as free (Grid).
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
TRAIL_RADIUS = 0.1      # m: cells this close to where the robot has driven are passable
TRAIL_STEP = 1.0        # m: points of that path further apart than this are not joined
# Turning on the spot: the robot's corners are 0.40 m from its centre, and a
# 900 mm pipe leaves 0.45 m, too little for its tracks on the curved wall. A
# cell is a place to turn if no wall or unmapped cell is within TURN_ROOM
# cells (Chebyshev distance, so at least that far in metres too): 0.5 m.
TURN_ROOM = 5           # cells (of COARSE m)
TURN_COST = 2.0         # m: a half turn on the spot, as distance (about the time it takes)
TURN_FREE = math.radians(45)    # a route this far off the heading needs no turn first
LOOK_AHEAD = 1.0        # m along a route: the direction it leaves the robot in
CLEARANCE_CAP = 8       # cells: clearance is worked out up to this
TOUR_LIMIT = 9          # frontiers in the tour (the nearest); exact up to this

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
        # One or two wall cells with no unknown space round them are not a
        # wall: most likely the LiDAR's scan, tilted with the robot, met the
        # bottom or the top of the pipe a few metres ahead. Kept, one such
        # cell in the middle of a pipe left no room for the robot (see
        # clearance) and the pipe beyond it looked closed: in a test run the
        # robot turned back 3 m short of a pipe that went on for 20 m. (Nav2,
        # on the finer 0.05 m map, has room to go past one.)
        lone = self.occ & (box_sum(self.unknown, 1)[:h, :w] == 0) & \
            (box_sum(self.occ, 1)[:h, :w] <= 2)
        self.occ &= ~lone
        self.free |= lone
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
        self.travel = math.inf      # path length plus turns, driving forwards into it, m
        self.how = 'ahead'          # how to get there forwards (Snapshot.approach)
        self.spot = None            # (x, y): where to turn round on the way, if anywhere
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
    driven: (x, y) points along the path the robot has driven, oldest first
    (full mode). The cells along it are passable whatever the map shows
    there now (see trail). robot_yaw: the robot's heading (rad), for
    approach; without it every route counts as leaving ahead.
    """

    def __init__(self, grid, robot_xy, start_xy=None, clearance=0.2, exit_radius=2.5,
                 driven=None, robot_yaw=None):
        self.g = grid
        self.robot_xy = robot_xy
        self.robot_yaw = robot_yaw
        self._clear = self._turn = self._manholes = None
        # walls with their small gaps closed (see WALL_GAP)
        self.walls = close_gaps(grid.occ, max(1, int(round(WALL_GAP / grid.res))))
        radius = max(1, int(math.ceil(clearance / grid.res - 1e-9)))
        self.traversable = grid.free & ~dilate(self.walls, radius)
        if driven is not None and len(driven) > 1:
            self.traversable |= self.trail(driven)
        self.robot_cell = self.snap(robot_xy)
        self.dist, _ = geodesic(self.traversable, self.robot_cell) if self.robot_cell else \
            (np.full(grid.shape, np.inf), None)
        self._room = None
        self._routes = {}
        self.start_xy = start_xy
        self.dist_start = self.branch = None
        if start_xy is not None:
            sc = self.snap(start_xy)
            self.dist_start, self.branch = geodesic(
                self.traversable, sc if sc else (-1, -1), exit_cells=exit_radius / grid.res)

    def trail(self, points):
        """
        Mark the cells along a path the robot has driven.

        After a long drive RTAB-Map's map can show a wall twice, a little
        apart (the same wall seen before and after the map drifted), and a
        pipe then looks too narrow for the robot, or closed, where the robot
        has in fact just driven. Returns the cells within TRAIL_RADIUS of the
        lines joining the points (points more than TRAIL_STEP apart are not
        joined).
        """
        g = self.g
        p = np.asarray(points, float).reshape(-1, 2)
        a, b = p[:-1], p[1:]
        length = np.hypot(b[:, 0] - a[:, 0], b[:, 1] - a[:, 1])
        keep = length <= TRAIL_STEP
        a, b, length = a[keep], b[keep], length[keep]
        mask = np.zeros(g.shape, bool)
        if len(a) == 0:
            return mask
        # points half a cell apart along each piece
        n = np.maximum(1, np.ceil(length / (g.res * 0.5)).astype(int))
        piece = np.repeat(np.arange(len(a)), n + 1)
        t = np.concatenate([np.linspace(0.0, 1.0, k + 1) for k in n])
        xy = a[piece] + (b[piece] - a[piece]) * t[:, None]
        cols = np.floor((xy[:, 0] - g.ox) / g.res).astype(int)
        rows = np.floor((xy[:, 1] - g.oy) / g.res).astype(int)
        ok = (rows >= 0) & (rows < g.shape[0]) & (cols >= 0) & (cols < g.shape[1])
        mask[rows[ok], cols[ok]] = True
        return dilate(mask, max(1, int(round(TRAIL_RADIUS / g.res))))

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

    def room(self):
        """Traversable cells in the 5 x 5 cells round each cell (most in the middle of a pipe)."""
        if self._room is None:
            self._room = box_sum(self.traversable, 2)
        return self._room

    # ------------------------------------------------------------ routes

    def route(self, cell):
        """
        Find the shortest path from the robot to `cell` through the mapped space.

        Returns its cells, robot first, or None if the cell cannot be reached.
        """
        if self.robot_cell is None or not math.isfinite(self.dist[cell]):
            return None
        key = (int(cell[0]), int(cell[1]))
        if key in self._routes:
            return self._routes[key]
        h, w = self.g.shape
        dist = self.dist
        r, c = int(cell[0]), int(cell[1])
        path = [(r, c)]
        # downhill on the distance from the robot (it always has a lower 8-neighbour)
        while dist[r, c] > 0:
            best, best_d = None, dist[r, c]
            for dr, dc, _ in NEIGHBOURS:
                rr, cc = r + dr, c + dc
                if 0 <= rr < h and 0 <= cc < w and dist[rr, cc] < best_d:
                    best, best_d = (rr, cc), dist[rr, cc]
            if best is None:
                break
            r, c = best
            path.append(best)
        path.reverse()
        self._routes[key] = path
        return path

    def route_passes(self, cell, xy, radius):
        """Check whether the route to `cell` passes within `radius` m of the point xy."""
        path = self.route(cell)
        if not path:
            return False
        p = np.array(path, float)
        x = self.g.ox + (p[:, 1] + 0.5) * self.g.res
        y = self.g.oy + (p[:, 0] + 0.5) * self.g.res
        return bool((np.hypot(x - xy[0], y - xy[1]) < radius).any())

    def waypoint(self, cell, along):
        """
        Find the point `along` m along the route to `cell`, in the middle of the pipe.

        Returns (x, y), or None if the route is no longer than that (then the
        cell itself is the next goal).
        """
        path = self.route(cell)
        if path is None or self.dist[cell] * self.g.res <= along:
            return None
        for p in path:
            if self.dist[p] * self.g.res >= along:
                return self.g.xy(*self.centre(p))
        return None

    def centre(self, cell, radius=0.3):
        """
        Move a reachable cell to the middle of the passage it is in.

        Returns the reachable cell within `radius` m with the most room round
        it, the nearest to `cell` if there are several.
        """
        k = int(math.ceil(radius / self.g.res))
        r0, c0 = max(cell[0] - k, 0), max(cell[1] - k, 0)
        r1, c1 = min(cell[0] + k + 1, self.g.shape[0]), min(cell[1] + k + 1, self.g.shape[1])
        ok = np.isfinite(self.dist[r0:r1, c0:c1])
        if not ok.any():
            return int(cell[0]), int(cell[1])
        room = np.where(ok, self.room()[r0:r1, c0:c1], -1)
        best = np.argwhere(room >= room.max())
        d = (best[:, 0] + r0 - cell[0]) ** 2 + (best[:, 1] + c0 - cell[1]) ** 2
        i = int(np.argmin(d))
        return int(best[i, 0] + r0), int(best[i, 1] + c0)

    # ------------------------------------------------------------ turning round

    def clearance(self):
        """
        Cells to the nearest wall or unmapped cell, for each cell (up to CLEARANCE_CAP).

        Chebyshev distance (squares round the cell), so the true distance
        is at least this.
        """
        if self._clear is None:
            ok = ~(self.walls | self.g.unknown)
            level = np.zeros(self.g.shape, np.int16)
            for _ in range(CLEARANCE_CAP):
                level += ok
                ok = erode(ok, 1)
                if not ok.any():
                    break
            self._clear = level
        return self._clear

    def turn_spots(self):
        """Reachable cells where the robot has room to turn on the spot (TURN_ROOM)."""
        if self._turn is None:
            self._turn = (self.clearance() > TURN_ROOM) & np.isfinite(self.dist)
        return self._turn

    def manholes(self):
        """
        Find the places to turn round (in a sewer, the manholes).

        Returns [(centre cell, entry cell, cells)]: the cell with the most
        room (nearest the middle if several), the one the robot reaches
        first, and all of them (the places to turn), nearest first.
        """
        if self._manholes is None:
            labels, count = label(self.turn_spots())
            clear = self.clearance()
            out = []
            for k in range(1, count + 1):
                cells = np.argwhere(labels == k)
                d = self.dist[cells[:, 0], cells[:, 1]]
                entry = tuple(int(v) for v in cells[int(np.argmin(d))])
                room = clear[cells[:, 0], cells[:, 1]]
                best = cells[room >= room.max()]
                mid = cells.mean(axis=0)
                i = int(np.argmin(((best - mid) ** 2).sum(axis=1)))
                out.append((tuple(int(v) for v in best[i]), entry, cells))
            out.sort(key=lambda m: self.dist[m[1]])
            self._manholes = out
            # each manhole's floor: its places to turn and the passable
            # cells round them, out to its walls and pipe openings
            area = np.zeros(self.g.shape, np.int32)
            reach = TURN_ROOM + 2
            for i, (_, _, cells) in enumerate(out):
                m = np.zeros(self.g.shape, bool)
                m[cells[:, 0], cells[:, 1]] = True
                area[dilate(m, reach) & self.traversable & (area == 0)] = i + 1
            self._manhole_area = area
        return self._manholes

    def manhole_at(self, cell):
        """Return the manhole (as in manholes) whose floor a cell is on, or None."""
        mh = self.manholes()
        k = int(self._manhole_area[cell])
        return mh[k - 1] if k else None

    def leaving(self, path, start=0):
        """
        Find the bearing (rad) in which a route leaves its cell path[start].

        That is the direction to the point LOOK_AHEAD m further on; None if
        less than 0.3 m of the route is left. From the robot (start 0) it is
        measured from where the robot really is.
        """
        g = self.g
        d0 = self.dist[path[start]]
        end = None
        for c in path[start + 1:]:
            end = c
            if (self.dist[c] - d0) * g.res >= LOOK_AHEAD:
                break
        if end is None or (self.dist[end] - d0) * g.res < 0.3:
            return None
        x0, y0 = self.robot_xy if start == 0 else g.xy(*path[start])
        x1, y1 = g.xy(*end)
        return math.atan2(y1 - y0, x1 - x0)

    def approach(self, cell):
        """
        Plan how to drive to `cell` forwards: a heading-aware travel cost.

        Returns (cost in m, how, spot). how is
          'ahead'    the route leaves ahead of the robot: drive there
          'turn'     the robot has room to turn where it is: turn to face the
                     route first (spot is None)
          'back'     reverse along the route to `spot`, the middle of the first
                     manhole on it, turn there, and drive on forwards
          'loop'     drive forwards to `spot`, the middle of a manhole ahead,
                     turn there and come back
          'reverse'  only reachable in reverse (no room to turn on the way):
                     the cost is inf
        The cost is the path length plus TURN_COST for a half turn (in
        proportion for less), plus the drive there and back for 'loop'.
        Reversing ('back') only ever goes over the route, which is mapped
        pipe. Without the robot's heading every route counts as 'ahead'.
        """
        path = self.route(cell)
        if path is None:
            return math.inf, 'reverse', None
        length = self.dist[cell] * self.g.res
        bearing = self.leaving(path) if self.robot_yaw is not None else None
        if bearing is None:
            return length, 'ahead', None
        off = angle_diff(bearing, self.robot_yaw)
        turn = self.turn_spots()
        if turn[self.robot_cell] and off > TURN_FREE:
            return length + TURN_COST * off / math.pi, 'turn', None
        if off <= math.pi / 2:
            return length, 'ahead', None
        best = (math.inf, 'reverse', None)
        on = next((m for m in (self.manhole_at(c) for c in path) if m is not None), None)
        if on is not None:
            best = (length + TURN_COST, 'back', self.g.xy(*on[0]))
        for centre, entry, _ in self.manholes():
            d = self.dist[entry] * self.g.res
            if length + 2.0 * d + TURN_COST >= best[0]:
                break                   # (nearest first: the others are further still)
            b = self.leaving(self.route(entry))
            if b is None or angle_diff(b, self.robot_yaw) <= math.pi / 2:
                best = (length + 2.0 * self.dist[centre] * self.g.res + TURN_COST, 'loop',
                        self.g.xy(*centre))
                break
        return best

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
        # (worked out in a window round the frontier: the whole map is slow)
        r0, c0 = np.maximum(cells.min(axis=0) - 3, 0)
        r1, c1 = np.minimum(cells.max(axis=0) + 4, g.shape)
        near = np.zeros((r1 - r0, c1 - c0), bool)
        near[cells[:, 0] - r0, cells[:, 1] - c0] = True
        near = dilate(near, 3) & reachable[r0:r1, c0:c1]
        cand = np.argwhere(near) + (r0, c0)
        if len(cand) == 0:
            return None
        room = self.room()[cand[:, 0], cand[:, 1]]
        cand = cand[room >= room.max()]
        cx = g.ox + (cand[:, 1] + 0.5) * g.res
        cy = g.oy + (cand[:, 0] + 0.5) * g.res
        i = int(np.argmin((cx - centroid[0]) ** 2 + (cy - centroid[1]) ** 2))
        goal_cell = (int(cand[i, 0]), int(cand[i, 1]))
        rect = (float(xs.min() - g.res / 2), float(ys.min() - g.res / 2),
                float(xs.max() + g.res / 2), float(ys.max() + g.res / 2))
        f = Frontier(cells, centroid, goal_cell, g.xy(*goal_cell), len(cells) * g.res, rect)
        f.path = self.path_length(goal_cell)
        f.travel, f.how, f.spot = self.approach(goal_cell)
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


def nearest_first(frontiers, reach):
    """
    Choose a frontier when exploring the whole map: TAD among the nearest.

    Returns the highest TAD score among the frontiers whose travel cost
    from the robot (path length plus turns, Snapshot.approach) is at most
    `reach` m more than the nearest one's. So the robot maps the pipes round
    it before it drives on, rather than coming back for them across the
    network.
    """
    nearest = min(f.travel for f in frontiers)
    return max((f for f in frontiers if f.travel <= nearest + reach), key=lambda f: f.score)


def plan_tour(snap, frontiers, home=None, limit=TOUR_LIMIT):
    """
    Plan one tour from the robot through every frontier and back home.

    The global tour of TARE (Cao, Zhu, Choset and Zhang, RSS 2021) and FUEL
    (Zhou et al., RA-L 2021) on the pipes: the cost of a tour is its length
    through the mapped pipes plus its turns. The first leg is each
    frontier's travel cost (Snapshot.approach); every leg after a frontier
    adds TURN_COST, because the robot faces into the pipe it has just
    explored and has to turn back. home: (x, y) to end at (None: the tour
    ends at its last frontier).

    The `limit` frontiers with the lowest travel cost are put in order
    exactly (Held-Karp dynamic programming, 2^n n^2 steps); the others are
    left out: they are further away and change little about which comes
    first. Returns ({frontier: cost of the best tour that starts with it},
    best tour as a list).
    """
    cand = sorted((f for f in frontiers if math.isfinite(f.travel)),
                  key=lambda f: f.travel)[:limit]
    n = len(cand)
    if n == 0:
        return {}, []
    res = snap.g.res
    pair = [[0.0] * n for _ in range(n)]
    for i, f in enumerate(cand):
        dist, _ = geodesic(snap.traversable, f.goal_cell)
        for j, g in enumerate(cand):
            if i != j:
                pair[i][j] = dist[g.goal_cell] * res + TURN_COST
    end = [0.0] * n
    if home is not None:
        cell = snap.snap(home)
        dist = geodesic(snap.traversable, cell)[0] if cell else None
        for i, f in enumerate(cand):
            d = dist[f.goal_cell] * res if dist is not None else math.inf
            end[i] = d + TURN_COST if math.isfinite(d) else 0.0
    # rest[mask][j]: cheapest way from frontier j through the frontiers in
    # mask (j among them) and on home
    full = (1 << n) - 1
    rest = [[math.inf] * n for _ in range(full + 1)]
    after = [[-1] * n for _ in range(full + 1)]
    for j in range(n):
        rest[1 << j][j] = end[j]
    for mask in range(1, full + 1):
        for j in range(n):
            if not mask & (1 << j) or mask == 1 << j:
                continue
            others = mask & ~(1 << j)
            for k in range(n):
                if others & (1 << k):
                    c = pair[j][k] + rest[others][k]
                    if c < rest[mask][j]:
                        rest[mask][j], after[mask][j] = c, k
    costs = {f: f.travel + rest[full][i] for i, f in enumerate(cand)}
    i = min(range(n), key=lambda i: cand[i].travel + rest[full][i])
    order, mask = [], full
    while i >= 0:
        order.append(cand[i])
        nxt = after[mask][i]
        mask &= ~(1 << i)
        i = nxt
    return costs, order


def _near(xy, point, radius):
    return math.hypot(xy[0] - point[0], xy[1] - point[1]) < radius


class WholeMap:
    """
    Where to go next when exploring the whole map (explorer.py, mode full).

    The robot keeps to the pipe it is in: when the frontier it is driving to
    moves on (more of the pipe has been mapped), it goes on to the frontier
    beyond it. When the frontier is gone (the pipe is mapped: the end of a
    dead end has been seen, or it has met a part mapped before), it chooses
    again, nearest_first.

    A frontier in another pipe that the robot comes within `peek` m of (a
    side pipe in a manhole it drives through) is remembered with where the
    robot was. If the robot gets `peek` m past that point and its own pipe
    still goes on, it goes back for the other pipe, once, and its own pipe
    waits. A short dead end off a manhole is then mapped while the robot is
    there, not on a trip back across the network at the end. On such a trip
    back it does not go back again (for the pipe it left, or any other) until
    it has mapped the pipe it went back for. The pipes behind the robot at
    the start wait too (the pipe ahead comes first, as in the proposal's
    mission).

    A frontier counts as the same one as before if it is within hold_radius
    of it, or beyond it on the route from the robot and at most `track` m
    further (the pipe was mapped further on in the meantime). The robot
    leaves a frontier it is driving to for later and goes to one 2 x peek m
    nearer that has come up since (the map round the robot changed after it
    chose), if that one was there at the last choice too (not a gap that
    opens in a wall for a moment).

    When the frontier it is driving to goes from the map, it drives on
    towards where it was for up to `wait` s before it chooses another. The
    end of a pipe at a bend can go from the map until the robot sees round
    the bend, and turning back for another frontier then took the robot
    away from the bend. (Only with `now`, the time, given to choose.)

    With tour=True a new choice is the first frontier of the best tour
    through all of them and back to `home` (plan_tour), TAD deciding between
    first frontiers whose tours are within `reach` m of the best, instead of
    the nearest (nearest_first). Either way the costs include the turns the
    robot needs to drive into each frontier forwards.
    """

    def __init__(self, reach=2.0, hold_radius=1.0, peek=12.0, track=6.0, wait=8.0,
                 tour=False, home=None):
        self.reach = reach
        self.hold_radius = hold_radius
        self.peek = peek
        self.track = track
        self.wait = wait
        self.tour = tour
        self.home = home         # (x, y): where the tour ends
        self.order = []          # the frontiers in the order of the last tour
        self.target = None       # (x, y): the frontier the robot is driving to
        self.side = []           # [(frontier (x, y), where the robot was (x, y))]: to go back for
        self.waiting = []        # (x, y): frontiers left for later, never gone back for
        self.detour = False      # the target is a pipe it went back for
        self.seen = []           # (x, y): the frontiers at the last choice
        self.lost = None         # when the target's frontier went from the map (s)
        self.note = ''           # why the last choice was made, for the log

    def drop(self):
        """Forget the target (reached, unreachable, or the robot is stuck)."""
        self.target = None
        self.detour = False
        self.lost = None

    def expects(self, snap, f):
        """Check whether f is the frontier it is going to, or one it left or passed for later."""
        points = (([self.target] if self.target is not None else []) + self.waiting
                  + [p for p, _ in self.side])
        return any(self._find(snap, [f], p) is f for p in points)

    def choose(self, snap, frontiers, heading=None, now=None):
        """
        Decide where to go next.

        snap: the Snapshot; frontiers: the ones that may be chosen (not
        skipped); heading: the robot's heading for the first choice of a
        mission, else None; now: the time in s. Returns (frontier, hold):

          (None, True)   keep going: the target's frontier is still there
          (f, False)     go to frontier f
          (None, False)  no frontier is left
        """
        self.note = ''
        seen, self.seen = self.seen, [f.goal for f in frontiers]
        side, self.side = self.side, []
        for p, at in side:
            f = self._find(snap, frontiers, p)
            if f is not None and all(q != f.goal for q, _ in self.side):
                self.side.append((f.goal, at))
        waiting, self.waiting = self.waiting, []
        for p in waiting:
            f = self._find(snap, frontiers, p)
            if f is not None and f.goal not in self.waiting:
                self.waiting.append(f.goal)
        current, held = None, False
        if self.target is not None:
            current = self._find(snap, frontiers, self.target)
            held = current is not None and _near(current.goal, self.target, self.hold_radius)
        if current is not None:
            self.lost = None
            if not self.detour:
                self._remember_side_pipes(snap, frontiers, current)
                back = [(p, at) for p, at in self.side if self._depth(snap, at) >= self.peek]
                if back:
                    # the pipe it is in goes on: map the one it passed first
                    p, _ = min(back, key=lambda s: self._depth(snap, s[0]))
                    f = next(f for f in frontiers if f.goal == p)
                    self.waiting.append(current.goal)
                    self.note = 'the pipe it is in goes on: back to a pipe it passed'
                    f = self._go(snap, f)
                    self.detour = True
                    return f, False
            nearest = min(frontiers, key=lambda f: f.path)
            if current.path > nearest.path + 2 * self.peek and \
                    any(_near(nearest.goal, p, self.hold_radius) for p in seen):
                self.note = 'a frontier far nearer than the one it was going to has come up'
                self.waiting.append(current.goal)
                self.detour = False
                return self._go(snap, self._pick(snap, frontiers)), False
            if held:
                return None, True
            return self._go(snap, current), False
        if self.target is not None and frontiers and now is not None:
            # its frontier has gone from the map: drive on for a while first
            if self.lost is None:
                self.lost = now
                self.note = 'the frontier it was going to has gone from the map: going on'
            if now - self.lost < self.wait:
                return None, True
            self.note = 'the frontier it was going to has not come back'
        self.lost = None
        self.detour = False
        if not frontiers:
            return None, False
        pool = frontiers
        if heading is not None:
            rx, ry = snap.robot_xy
            ahead = [f for f in frontiers if angle_diff(
                math.atan2(f.goal[1] - ry, f.goal[0] - rx), heading) < math.pi / 2]
            pool = ahead or frontiers
        f = self._pick(snap, pool)
        if heading is not None:
            self.waiting += [g.goal for g in frontiers
                             if g is not f and not self._same_pipe(snap, g, f.goal)]
        return self._go(snap, f), False

    def _pick(self, snap, frontiers):
        """Choose among frontiers: the tour's first, or the nearest (with TAD among the close)."""
        if not self.tour or len(frontiers) < 2:
            self.order = []
            return nearest_first(frontiers, self.reach)
        costs, self.order = plan_tour(snap, frontiers, self.home)
        if not costs:
            return nearest_first(frontiers, self.reach)
        best = min(costs.values())
        return max((f for f, c in costs.items() if c <= best + self.reach),
                   key=lambda f: f.score)

    def _go(self, snap, f):
        self.target = f.goal
        self.lost = None
        self.side = [(p, at) for p, at in self.side if not self._same_pipe(snap, f, p)]
        self.waiting = [p for p in self.waiting if not self._same_pipe(snap, f, p)]
        return f

    def _remember_side_pipes(self, snap, frontiers, current):
        """Remember the frontiers in other pipes within peek m, with where the robot is."""
        for f in frontiers:
            if f is current or f.path > self.peek or \
                    self._same_pipe(snap, f, current.goal) or \
                    any(self._same_pipe(snap, f, p) for p, _ in self.side) or \
                    any(self._same_pipe(snap, f, p) for p in self.waiting):
                continue
            self.side.append((f.goal, snap.robot_xy))

    def _find(self, snap, frontiers, point):
        """Find the frontier that `point` (a frontier earlier) is now; None if it is gone."""
        near = [f for f in frontiers if _near(f.goal, point, self.hold_radius)]
        if near:
            return min(near, key=lambda f: math.hypot(f.goal[0] - point[0],
                                                      f.goal[1] - point[1]))
        d = self._depth(snap, point)
        moved = [f for f in frontiers if d <= f.path <= d + self.track
                 and snap.route_passes(f.goal_cell, point, self.hold_radius)]
        return min(moved, key=lambda f: f.path) if moved else None

    def _same_pipe(self, snap, f, point):
        """Check whether frontier f and a frontier at `point` are in the same pipe, close."""
        if _near(f.goal, point, self.hold_radius):
            return True
        d = self._depth(snap, point)
        if not math.isfinite(d) or abs(f.path - d) > self.track:
            return False
        if f.path >= d:
            return snap.route_passes(f.goal_cell, point, self.hold_radius)
        cell = snap.snap(point)
        return cell is not None and snap.route_passes(cell, f.goal, self.hold_radius)

    def _depth(self, snap, point):
        """Path length from the robot to a point (inf if not reachable)."""
        cell = snap.snap(point)
        return snap.path_length(cell) if cell else math.inf


def _touch(a, b, gap):
    return not (a[2] + gap < b[0] or b[2] + gap < a[0]
                or a[3] + gap < b[1] or b[3] + gap < a[1])


class PipeGraph:
    """
    The mapped network as a graph: manholes and the pipes between them.

    The topological view of a sewer (Choset and Burdick, "Sensor-Based
    Exploration: The Hierarchical Generalized Voronoi Graph", IJRR 19(2),
    2000; for pipe networks, Nguyen et al., Frontiers in Robotics and AI 9,
    2022): manholes, where pipes meet and the robot can turn round, are the
    nodes, and each pipe is an edge between the two manholes it joins, or a
    branch off one manhole that ends in a dead end or, with a frontier at its
    end, in pipe not explored yet.

    Built from a Snapshot: a manhole is a place to turn round
    (Snapshot.manholes) with the passable cells round it up to its walls; a
    pipe is a connected stretch of passable cells outside the manholes.

      nodes  [(x, y)]: the middle of each manhole, nearest the robot first
      edges  [(i, j, length m)]: pipes from manhole i to manhole j
      ends   [(i, length m, open, (x, y))]: pipes off manhole i that end;
             open = a frontier is at the end; (x, y) = the end
      loose  [(x, y)]: pipes that reach no manhole (both ends unexplored)
    """

    def __init__(self, snap, frontiers=()):
        self.nodes, self.edges, self.ends, self.loose = [], [], [], []
        mh = snap.manholes()
        if not mh:
            return
        g = snap.g
        trav = snap.traversable & np.isfinite(snap.dist)
        grow = TURN_ROOM + 2
        areas = []
        for centre, _, cells in mh:
            m = np.zeros(g.shape, bool)
            m[cells[:, 0], cells[:, 1]] = True
            areas.append(dilate(m, grow) & trav)
            self.nodes.append(g.xy(*centre))
        inside = np.zeros(g.shape, bool)
        for a in areas:
            inside |= a
        labels, count = label(trav & ~inside)
        open_cells = np.zeros(g.shape, bool)
        for f in frontiers:
            open_cells[f.goal_cell] = True
        open_cells = dilate(open_cells, 3)
        for k in range(1, count + 1):
            seg = labels == k
            if seg.sum() * g.res * g.res < 0.3:        # a speck, not a pipe
                continue
            rim = dilate(seg, 1)
            touch = [i for i, a in enumerate(areas) if (rim & a).any()]
            cells = np.argwhere(seg)
            if not touch:
                mid = cells[len(cells) // 2]
                self.loose.append(g.xy(*mid))
                continue
            a = touch[0]
            region = seg | areas[a]
            for b in touch[1:]:
                region = region | areas[b]
            dist, _ = geodesic(region, mh[a][0])
            dist = dist * g.res
            if len(touch) >= 2:
                for b in touch[1:]:
                    d = float(dist[mh[b][0]])
                    if math.isfinite(d):
                        self.edges.append((a, b, d))
                continue
            d = np.where(seg, dist, -1.0)
            r, c = np.unravel_index(int(np.argmax(d)), d.shape)
            is_open = bool((seg & open_cells).any())
            self.ends.append((a, float(d[r, c]), is_open, g.xy(r, c)))

    def length(self):
        """Pipe mapped in all, m (manhole to manhole, and out to the ends)."""
        return sum(e[2] for e in self.edges) + sum(e[1] for e in self.ends)
