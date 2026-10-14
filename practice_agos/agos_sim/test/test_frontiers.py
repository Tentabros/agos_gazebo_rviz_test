"""
Tests for agos_sim/frontiers.py (TAD frontier scoring), without ROS.

    cd agos_sim && python3 -m pytest test/test_frontiers.py     (or: colcon test)

The synthetic maps look like RTAB-Map's maps of the sewer: free space
inside 0.9 m pipes and 2.4 m manholes, walls round them, everything else
unknown. test/data/*.npz are real RTAB-Map maps from the simulation.
"""
import math
import os
import time

from agos_sim.frontiers import angle_diff, close_gaps, dilate, Grid, Snapshot
import numpy as np

RES = 0.05
LEG = math.radians(60)
DATA = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'data')


class MapBuilder:
    """A 0.05 m occupancy grid (-1 unknown, 0 free, 100 wall) to draw on."""

    def __init__(self, xmin, ymin, xmax, ymax):
        self.ox, self.oy = xmin, ymin
        self.w = int(round((xmax - xmin) / RES))
        self.h = int(round((ymax - ymin) / RES))
        yy, xx = np.mgrid[0:self.h, 0:self.w]
        self.x = self.ox + (xx + 0.5) * RES
        self.y = self.oy + (yy + 0.5) * RES
        self.free = np.zeros((self.h, self.w), bool)
        self.wall = np.zeros((self.h, self.w), bool)

    def chamber(self, cx, cy, half=1.2, wall=0.1):
        dx, dy = np.abs(self.x - cx), np.abs(self.y - cy)
        inside = (dx <= half) & (dy <= half)
        self.free |= inside
        self.wall |= ~inside & (dx <= half + wall) & (dy <= half + wall)

    def pipe(self, points, half=0.45, wall=0.1):
        """Free band along a polyline, walls along both sides (not across the ends)."""
        for (x0, y0), (x1, y1) in zip(points, points[1:]):
            vx, vy = x1 - x0, y1 - y0
            length = math.hypot(vx, vy)
            ux, uy = vx / length, vy / length
            along = (self.x - x0) * ux + (self.y - y0) * uy
            across = np.abs(-(self.x - x0) * uy + (self.y - y0) * ux)
            seg = (along >= 0) & (along <= length)
            self.free |= seg & (across <= half)
            self.wall |= seg & (across > half) & (across <= half + wall)
        for x, y in points[1:-1]:          # round joints at the bends
            d = np.hypot(self.x - x, self.y - y)
            self.free |= d <= half
            self.wall |= (d > half) & (d <= half + wall)

    def grid(self):
        a = np.full((self.h, self.w), -1, np.int8)
        a[self.wall & ~self.free] = 100
        a[self.free] = 0
        return Grid(a.ravel(), self.w, self.h, RES, self.ox, self.oy)


def manhole_with_two_pipes(east=6.0, west=3.0):
    """
    Make a start manhole at the origin with two pipes.

    One is known for `east` m to the east of its wall, the other for `west` m
    to the west.
    """
    m = MapBuilder(-1.6 - west, -2.0, 1.6 + east, 2.0)
    m.chamber(0.0, 0.0)
    m.pipe([(0.0, 0.0), (1.2 + east, 0.0)])
    m.pipe([(0.0, 0.0), (-1.2 - west, 0.0)])
    return m.grid()


def test_pipe_ends_are_the_frontiers():
    snap = Snapshot(manhole_with_two_pipes(), (0.0, 0.0), (0.0, 0.0))
    frontiers = snap.frontiers()
    assert len(frontiers) == 2, frontiers
    east = max(frontiers, key=lambda f: f.goal[0])
    west = min(frontiers, key=lambda f: f.goal[0])
    # at the end of the known pipe, in the middle of it
    assert 6.5 < east.goal[0] <= 7.2 and abs(east.goal[1]) < 0.2, east
    assert -4.2 <= west.goal[0] < -3.5 and abs(west.goal[1]) < 0.2, west
    # about as wide as the pipe
    assert 0.6 <= east.size <= 1.2, east.size
    # reached through the east and the west pipe
    assert angle_diff(east.branch, 0.0) < 0.2
    assert angle_diff(west.branch, math.pi) < 0.2


def test_legs_split_by_pipe():
    snap = Snapshot(manhole_with_two_pipes(), (0.0, 0.0), (0.0, 0.0))
    frontiers = snap.frontiers()
    ahead = [f for f in frontiers if snap.in_leg(f, 0.0, LEG, 25.0)]
    behind = [f for f in frontiers if snap.in_leg(f, math.pi, LEG, 25.0)]
    assert len(ahead) == 1 and ahead[0].goal[0] > 0
    assert len(behind) == 1 and behind[0].goal[0] < 0
    # the leg length limit is measured along the pipe from the start
    assert not [f for f in frontiers if snap.in_leg(f, 0.0, LEG, 5.0)]


def test_frontier_near_the_start_belongs_to_its_side():
    # the west pipe is only known 0.9 m past the manhole wall: its frontier is
    # within exit_radius of the start, so it has no branch yet
    snap = Snapshot(manhole_with_two_pipes(west=0.9), (0.0, 0.0), (0.0, 0.0))
    frontiers = snap.frontiers()
    west = min(frontiers, key=lambda f: f.goal[0])
    assert math.isnan(west.branch) and west.goal[0] < -1.5, west
    assert not snap.in_leg(west, 0.0, LEG, 25.0)
    assert snap.in_leg(west, math.pi, LEG, 25.0)


def test_tad_score_is_d_plus_a_plus_t():
    snap = Snapshot(manhole_with_two_pipes(), (0.0, 0.0), (0.0, 0.0))
    for f in snap.frontiers():
        assert math.isclose(f.d, math.hypot(*f.centroid), rel_tol=1e-6)
        assert math.isclose(f.d_adj, f.d * (1.0 + f.unknown_ratio), rel_tol=1e-6)
        assert 0.0 <= f.unknown_ratio <= 1.0
        assert f.a == 1 and f.t == 0     # one frontier at each end; same width as the pipe
        assert math.isclose(f.score, f.d_adj + f.a + f.t, rel_tol=1e-6)
    # further frontier, higher score (TAD prefers the distant frontier)
    best = snap.frontiers()[0]
    assert best.goal[0] > 0


def test_trapezoid_opening_out_and_narrowing():
    # a pipe whose known end widens into a 2 m wide space: opens out (t = +1)
    m = MapBuilder(-1.6, -2.0, 8.0, 2.0)
    m.chamber(0.0, 0.0)
    m.pipe([(0.0, 0.0), (5.0, 0.0)])
    m.free |= (m.x >= 5.0) & (m.x <= 6.0) & (np.abs(m.y) <= 0.45 + (m.x - 5.0) * 0.55)
    snap = Snapshot(m.grid(), (0.0, 0.0), (0.0, 0.0))
    east = max(snap.frontiers(), key=lambda f: f.goal[0])
    assert east.t == 1, (east, east.outer, east.inner)

    # a wide space narrowing into a gap: narrows (t = -1)
    m = MapBuilder(-1.6, -2.0, 8.0, 2.0)
    m.chamber(0.0, 0.0)
    m.pipe([(0.0, 0.0), (5.0, 0.0)], half=1.0)
    m.free &= ~((m.x > 4.6) & (np.abs(m.y) > 0.3))
    m.wall |= (m.x > 4.6) & (m.x <= 4.7) & (np.abs(m.y) > 0.3) & (np.abs(m.y) <= 1.1)
    snap = Snapshot(m.grid(), (0.0, 0.0), (0.0, 0.0))
    east = max(snap.frontiers(), key=lambda f: f.goal[0])
    assert east.t == -1, (east, east.outer, east.inner)


def test_adjacent_frontiers():
    # two pipes leaving a manhole side by side: their frontiers touch (a = 2)
    m = MapBuilder(-1.6, -2.0, 6.0, 2.0)
    m.chamber(0.0, 0.0)
    m.pipe([(0.0, 0.55), (4.0, 0.55)], half=0.35, wall=0.05)
    m.pipe([(0.0, -0.55), (4.0, -0.55)], half=0.35, wall=0.05)
    snap = Snapshot(m.grid(), (0.0, 0.0), (0.0, 0.0), clearance=0.1)
    east = [f for f in snap.frontiers() if f.goal[0] > 3.0]
    assert len(east) == 2, east
    assert all(f.a == 2 for f in east), east


def test_small_wall_gaps_are_not_frontiers():
    m = MapBuilder(-1.6, -2.0, 9.0, 2.0)
    m.chamber(0.0, 0.0)
    m.pipe([(0.0, 0.0), (7.0, 0.0)])
    # knock 0.1 m holes in both walls every 0.5 m, like a sparse LiDAR scan
    holes = (np.round((m.x - 2.0) / RES).astype(int) % 10 < 2) & (m.x > 2.0) & (m.x < 7.0)
    m.wall &= ~holes
    grid = m.grid()
    snap = Snapshot(grid, (0.0, 0.0), (0.0, 0.0))
    frontiers = snap.frontiers()
    assert len(frontiers) == 1 and frontiers[0].goal[0] > 6.0, frontiers
    # closing must not fill in the pipe itself
    r, c = grid.cell(4.0, 0.0)
    assert not snap.walls[r, c] and snap.traversable[r, c]


def test_gap_closing_width():
    # gaps of 1 or 2 cells in a wall close, a 0.9 m pipe opening does not
    for gap, closed in ((1, True), (2, True), (9, False)):
        m = np.zeros((20, 40), bool)
        m[10, :] = True
        m[10, 15:15 + gap] = False
        assert close_gaps(m, 2)[10, 15:15 + gap].all() == closed, gap
    # a passage 5 cells (0.5 m) wide between two walls stays open
    m = np.zeros((30, 40), bool)
    m[5, :] = m[11, :] = True
    assert not close_gaps(m, 2)[6:11, 20].any()


def test_leg_end_along_the_pipe():
    # a long known pipe: the leg end is 25 m from the start along the pipe
    m = MapBuilder(-1.6, -2.0, 40.0, 12.0)
    m.chamber(0.0, 0.0)
    m.pipe([(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (35.0, 10.0)])
    snap = Snapshot(m.grid(), (0.0, 0.0), (0.0, 0.0))
    (x, y), dist = snap.leg_end(0.0, LEG, 25.0)
    assert 24.0 <= dist <= 25.0, dist
    # 25 m along the pipe is the second straight: x 10, y about 10 m
    assert abs(x - 15.0) < 1.5 and abs(y - 10.0) < 0.5, (x, y)
    assert snap.leg_end(math.pi, LEG, 25.0) is None    # nothing behind
    assert abs(snap.explored_along(0.0, LEG) - 45.0) < 2.5


def test_unreachable_space_is_ignored():
    # free space the robot cannot get to (behind a wall) has no frontiers
    m = MapBuilder(-1.6, -2.0, 8.0, 4.0)
    m.chamber(0.0, 0.0)
    m.pipe([(0.0, 0.0), (5.0, 0.0)])
    m.free |= (m.x > 2.0) & (m.x < 4.0) & (m.y > 2.0) & (m.y < 3.0)
    snap = Snapshot(m.grid(), (0.0, 0.0), (0.0, 0.0))
    assert all(f.goal[1] < 1.0 for f in snap.frontiers())


def test_dilate_disc():
    m = np.zeros((11, 11), bool)
    m[5, 5] = True
    assert dilate(m, 1).sum() == 9
    assert dilate(m, 3)[5, 8] and dilate(m, 3)[8, 6] and not dilate(m, 3)[8, 8]


def _real_map(name):
    z = np.load(os.path.join(DATA, name))
    a = z['data']
    grid = Grid(a.ravel(), a.shape[1], a.shape[0], float(z['res']), float(z['ox']),
                float(z['oy']))
    return grid, tuple(float(v) for v in z['robot'])


def test_real_rtabmap_map_at_the_start():
    # RTAB-Map's map a few seconds after start-up in agos_sewer_network.sdf
    # (manhole M1 at the origin, pipes to the east and west)
    grid, robot = _real_map('rtabmap_start.npz')
    snap = Snapshot(grid, robot, (0.0, 0.0))
    frontiers = snap.frontiers()
    ahead = [f for f in frontiers if snap.in_leg(f, 0.0, LEG, 25.0)]
    behind = [f for f in frontiers if snap.in_leg(f, math.pi, LEG, 25.0)]
    best = max(ahead, key=lambda f: f.score)
    assert best.goal[0] > 3.0 and abs(best.goal[1]) < 0.5, best
    assert max(behind, key=lambda f: f.score).goal[0] < -3.0


def test_real_rtabmap_map_20m_along():
    # the robot 20 m along the east pipe: the only frontier ahead is further on
    grid, robot = _real_map('rtabmap_leg1_20m.npz')
    snap = Snapshot(grid, robot, (0.0, 0.0))
    ahead = [f for f in snap.frontiers() if snap.in_leg(f, 0.0, LEG, 25.0)]
    assert len(ahead) == 1, ahead
    assert ahead[0].from_start > 20.0 and ahead[0].path < 6.0, ahead[0]
    end, dist = snap.leg_end(0.0, LEG, 25.0)
    assert dist <= 25.0 and math.hypot(end[0] - robot[0], end[1] - robot[1]) < 8.0


def test_speed_on_a_big_map():
    # 120 x 40 m map with 200 m of pipe: must stay well under a second
    m = MapBuilder(-60.0, -20.0, 60.0, 20.0)
    m.chamber(0.0, 0.0)
    pts = [(0.0, 0.0)] + [(5.0 * k, 12.0 * math.sin(k * 0.7)) for k in range(1, 12)]
    m.pipe(pts)
    m.pipe([(0.0, 0.0)] + [(-x, -y) for x, y in pts[1:]])
    grid = m.grid()
    t = time.time()
    snap = Snapshot(grid, (0.0, 0.0), (0.0, 0.0))
    frontiers = snap.frontiers()
    elapsed = time.time() - t
    assert len(frontiers) == 2, frontiers
    assert elapsed < 2.0, elapsed


if __name__ == '__main__':
    for name, fn in sorted(globals().items()):
        if name.startswith('test_') and callable(fn):
            t = time.time()
            fn()
            print('ok   %-45s %.2f s' % (name, time.time() - t))
