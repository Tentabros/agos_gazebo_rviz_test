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

from agos_sim.frontiers import (angle_diff, close_gaps, dilate, Grid, nearest_first, PipeGraph,
                                plan_tour, Snapshot, TURN_COST, WholeMap)
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

    def cap(self, x, y, heading, half=0.45, wall=0.1):
        """Close the end of a pipe at (x, y), heading out of it, with a wall."""
        ux, uy = math.cos(heading), math.sin(heading)
        along = (self.x - x) * ux + (self.y - y) * uy
        across = np.abs(-(self.x - x) * uy + (self.y - y) * ux)
        end = (along > 0.0) & (along <= wall) & (across <= half + wall)
        self.free &= ~end
        self.wall |= end

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


def test_specks_do_not_close_the_pipe():
    # wall cells on their own in the middle of the pipe, as when the LiDAR's
    # tilted scan meets the bottom of the pipe: one 0.05 m cell 3 m on and
    # two 0.1 m apart 1.3 m on. The end of the pipe beyond them is still a
    # frontier the robot can get to.
    m = MapBuilder(-1.6, -2.0, 9.0, 2.0)
    m.chamber(0.0, 0.0)
    m.pipe([(0.0, 0.0), (7.0, 0.0)])
    specks = [(4.225, 0.025), (2.525, 0.025), (2.625, 0.025)]
    for x, y in specks:
        speck = (np.abs(m.x - x) < 0.01) & (np.abs(m.y - y) < 0.01)
        assert speck.sum() == 1
        m.free &= ~speck
        m.wall |= speck
    grid = m.grid()
    assert all(grid.free[grid.cell(x, y)] for x, y in specks)
    snap = Snapshot(grid, (0.0, 0.0), (0.0, 0.0))
    east = [f for f in snap.frontiers() if f.goal[0] > 6.0]
    assert len(east) == 1 and 6.5 < east[0].path < 9.0, east
    # the walls are still walls
    assert grid.occ[grid.cell(4.225, 0.5)] and grid.occ[grid.cell(4.225, -0.5)]


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


def test_route_follows_the_pipe():
    # an L-shaped pipe: the route goes round the corner, inside the pipe
    m = MapBuilder(-1.6, -2.0, 14.0, 12.0)
    m.chamber(0.0, 0.0)
    m.pipe([(0.0, 0.0), (10.0, 0.0), (10.0, 10.0)])
    snap = Snapshot(m.grid(), (0.0, 0.0))
    end = snap.snap((10.0, 9.0))
    route = snap.route(end)
    assert route[0] == snap.robot_cell and route[-1] == end
    assert all(snap.traversable[c] for c in route)
    steps = sum(math.hypot(a[0] - b[0], a[1] - b[1]) for a, b in zip(route, route[1:]))
    assert abs(steps * snap.g.res - snap.path_length(end)) < 1.0
    assert snap.route_passes(end, (10.0, 0.0), 0.5)          # round the corner
    assert not snap.route_passes(end, (5.0, 5.0), 1.0)       # not across it


def test_waypoint_steps_along_the_route():
    m = MapBuilder(-1.6, -2.0, 14.0, 12.0)
    m.chamber(0.0, 0.0)
    m.pipe([(0.0, 0.0), (10.0, 0.0), (10.0, 10.0)])
    snap = Snapshot(m.grid(), (0.0, 0.0))
    end = snap.snap((10.0, 9.0))
    x, y = snap.waypoint(end, 8.0)
    # 8 m along the pipe, in the middle of it
    assert 7.0 < x < 8.8 and abs(y) < 0.1, (x, y)
    x, y = snap.waypoint(end, 13.0)
    assert abs(x - 10.0) < 0.1 and 1.5 < y < 4.0, (x, y)
    assert snap.waypoint(end, 25.0) is None      # shorter than that: go straight there


def test_driven_path_opens_a_wall_mapped_twice():
    # after drift, the pipe's top wall is in the map a second time, 0.3 m
    # further in, for 3 m: there is no room for the robot there on the map,
    # but the robot has just driven through, so the way back is open along
    # its path
    m = MapBuilder(-1.6, -2.0, 14.0, 2.0)
    m.chamber(0.0, 0.0)
    m.pipe([(0.0, 0.0), (12.0, 0.0)])
    twice = (m.x > 4.0) & (m.x < 7.0) & (m.y >= 0.05) & (m.y <= 0.15)
    m.free &= ~twice
    m.wall |= twice
    grid = m.grid()
    snap = Snapshot(grid, (10.0, 0.0))
    assert not math.isfinite(snap.path_length(snap.snap((0.0, 0.0))))
    driven = [(x, -0.2) for x in np.arange(0.0, 10.01, 0.3)]
    snap = Snapshot(grid, (10.0, 0.0), driven=driven)
    assert 9.5 < snap.path_length(snap.snap((0.0, 0.0))) < 10.5
    # only along the path: the wall itself is still a wall, and two points
    # far apart (a jump in the path) are not joined
    assert not snap.traversable[grid.cell(5.5, 0.5)]
    snap = Snapshot(grid, (10.0, 0.0), driven=[(0.0, -0.2), (10.0, -0.2)])
    assert not math.isfinite(snap.path_length(snap.snap((0.0, 0.0))))


def test_nearest_first():
    # the east pipe is known 6 m out, the west one 1.5 m: TAD alone takes
    # the far one, nearest_first the near one, and TAD again within reach
    snap = Snapshot(manhole_with_two_pipes(east=6.0, west=1.5), (0.0, 0.0))
    frontiers = snap.frontiers()
    assert max(frontiers, key=lambda f: f.score).goal[0] > 0
    assert nearest_first(frontiers, 2.0).goal[0] < 0
    assert nearest_first(frontiers, 10.0).goal[0] > 0


def test_whole_map_gives_up_a_far_frontier_for_a_near_one():
    # going to the end of a pipe 35 m east when the map round the robot shows
    # a pipe going on 1.5 m west: it goes west, once the west frontier has
    # been there for two choices (not a gap that opens for a moment), and
    # leaves the east one for later; 10 m east, it keeps going
    for east, keeps in ((35.0, False), (10.0, True)):
        snap = Snapshot(manhole_with_two_pipes(east=east, west=1.5), (0.0, 0.0))
        frontiers = snap.frontiers()
        far = max(frontiers, key=lambda f: f.path)
        chooser = WholeMap()
        chooser.target = far.goal
        f, hold = chooser.choose(snap, frontiers)
        assert hold and f is None
        f, hold = chooser.choose(snap, frontiers)
        if keeps:
            assert hold and f is None
        else:
            assert f is not None and f.goal[0] < 0, f and f.goal
            assert chooser.expects(snap, far)


def test_whole_map_does_not_go_back_from_a_pipe_it_went_back_for():
    # the robot is 3 m into a west pipe; it passed the east one 17 m ago. On
    # its own pipe it would go back for the east one now; on a trip back for
    # the west pipe it maps that one first.
    snap = Snapshot(manhole_with_two_pipes(east=20.0, west=8.0), (-3.0, 0.0))
    frontiers = snap.frontiers()
    east = max(frontiers, key=lambda f: f.goal[0])
    west = min(frontiers, key=lambda f: f.goal[0])
    for detour, goes_back in ((False, True), (True, False)):
        chooser = WholeMap()
        chooser.target = west.goal
        chooser.side = [(east.goal, (14.0, 0.0))]
        chooser.detour = detour
        f, hold = chooser.choose(snap, frontiers)
        if goes_back:
            assert f is not None and f.goal == east.goal and chooser.detour
        else:
            assert hold and f is None


def test_whole_map_expects_its_goal_and_the_pipes_it_left():
    # the explorer goes to a frontier beside the path driven only if it is
    # the one the robot is going to or one it left for later, also after the
    # map has grown past where that frontier was
    snap = Snapshot(manhole_with_two_pipes(east=6.0, west=3.0), (0.0, 0.0))
    frontiers = snap.frontiers()
    east = max(frontiers, key=lambda f: f.goal[0])
    west = min(frontiers, key=lambda f: f.goal[0])
    chooser = WholeMap()
    assert not chooser.expects(snap, east) and not chooser.expects(snap, west)
    chooser.target = (east.goal[0] - 0.5, east.goal[1])
    assert chooser.expects(snap, east) and not chooser.expects(snap, west)
    chooser.target = None
    chooser.waiting = [(west.goal[0] + 2.0, west.goal[1])]
    assert chooser.expects(snap, west) and not chooser.expects(snap, east)


def test_whole_map_drives_on_when_its_frontier_goes():
    # the frontier it was going to has gone from the map (the end of a pipe
    # at a bend can, until the robot sees round it): it keeps going for up to
    # `wait` s, then chooses another; given no time, it chooses at once
    snap = Snapshot(manhole_with_two_pipes(east=6.0, west=3.0), (0.0, 0.0))
    frontiers = snap.frontiers()
    gone = (30.0, 0.0)
    chooser = WholeMap(wait=8.0)
    chooser.target = gone
    assert chooser.choose(snap, frontiers, now=100.0) == (None, True)
    assert chooser.choose(snap, frontiers, now=107.0) == (None, True)
    f, hold = chooser.choose(snap, frontiers, now=108.5)
    assert f is not None and not hold and chooser.target == f.goal
    chooser = WholeMap(wait=8.0)
    chooser.target = gone
    f, hold = chooser.choose(snap, frontiers)
    assert f is not None and not hold


def test_turn_spots_are_the_manholes():
    # 2.4 m manhole: room to turn; 900 mm pipe: none
    snap = Snapshot(manhole_with_two_pipes(), (0.0, 0.0))
    turn = snap.turn_spots()
    assert turn[snap.g.cell(0.0, 0.0)]
    for x in (2.0, 4.0, -2.0):
        assert not turn[snap.g.cell(x, 0.0)], x
    (centre, entry, _), = snap.manholes()
    assert math.hypot(*snap.g.xy(*centre)) < 0.15


def test_approach_ahead_turn_and_back():
    grid = manhole_with_two_pipes(east=8.0, west=6.0)
    east, west = (8.5, 0.0), (-6.5, 0.0)
    # in the east pipe facing east: the east end is ahead; the west one is
    # behind, so back up to the manhole, turn there, and go on forwards
    snap = Snapshot(grid, (4.0, 0.0), robot_yaw=0.0)
    cost, how, spot = snap.approach(snap.snap(east))
    assert how == 'ahead' and abs(cost - 4.5) < 0.3, (cost, how)
    cost, how, spot = snap.approach(snap.snap(west))
    assert how == 'back' and math.hypot(*spot) < 0.3, (how, spot)
    assert abs(cost - (10.5 + TURN_COST)) < 0.4, cost
    # in the manhole facing east: turn there for the west pipe
    snap = Snapshot(grid, (0.0, 0.0), robot_yaw=0.0)
    assert snap.approach(snap.snap(west))[1] == 'turn'
    assert snap.approach(snap.snap(east))[1] == 'ahead'
    # without a heading every route is 'ahead'
    snap = Snapshot(grid, (4.0, 0.0))
    assert snap.approach(snap.snap(west))[1] == 'ahead'


def test_approach_loop_and_reverse_only():
    # a pipe with a manhole only at its east end; the robot faces it 2 m
    # off. The west end is behind with nowhere to turn on the way: go on to
    # the manhole, turn, come back. With no manhole at all: reverse only.
    m = MapBuilder(-12.0, -2.0, 8.0, 2.0)
    m.chamber(5.0, 0.0)
    m.pipe([(-11.0, 0.0), (5.0, 0.0)])
    snap = Snapshot(m.grid(), (2.0, 0.0), robot_yaw=0.0)
    west = snap.snap((-10.8, 0.0))
    cost, how, spot = snap.approach(west)
    assert how == 'loop' and abs(spot[0] - 5.0) < 0.3, (how, spot)
    assert abs(cost - (12.8 + 2 * 3.0 + TURN_COST)) < 0.6, cost
    m = MapBuilder(-12.0, -2.0, 8.0, 2.0)
    m.pipe([(-11.0, 0.0), (7.0, 0.0)])
    snap = Snapshot(m.grid(), (2.0, 0.0), robot_yaw=0.0)
    cost, how, spot = snap.approach(snap.snap((-10.8, 0.0)))
    assert how == 'reverse' and cost == math.inf


def test_frontiers_carry_their_travel_cost():
    snap = Snapshot(manhole_with_two_pipes(east=8.0, west=6.0), (4.0, 0.0), robot_yaw=0.0)
    by_x = sorted(snap.frontiers(), key=lambda f: f.goal[0])
    assert by_x[0].how == 'back' and by_x[-1].how == 'ahead'
    assert by_x[0].travel > by_x[0].path + 0.5 * TURN_COST
    assert by_x[-1].travel == by_x[-1].path


def test_tour_through_the_frontiers_and_home():
    # manhole at the origin, pipes known 2 m north, 6 m east and 9 m west
    # (each with a frontier at its end); the robot in the manhole facing
    # east. Out and back along each pipe is the same distance in any order,
    # so the turns decide: the pipe ahead first saves the turn at the start
    # (a quarter turn for the north pipe, a half turn for the west one).
    m = MapBuilder(-11.0, -2.0, 9.0, 5.0)
    m.chamber(0.0, 0.0)
    m.pipe([(0.0, 0.0), (7.2, 0.0)])
    m.pipe([(0.0, 0.0), (-10.2, 0.0)])
    m.pipe([(0.0, 0.0), (0.0, 3.2)])
    snap = Snapshot(m.grid(), (0.0, 0.0), robot_yaw=0.0)
    frontiers = snap.frontiers()
    assert len(frontiers) == 3, frontiers
    east, north, west = (max(frontiers, key=lambda f: f.goal[0]),
                         max(frontiers, key=lambda f: f.goal[1]),
                         min(frontiers, key=lambda f: f.goal[0]))
    costs, order = plan_tour(snap, frontiers, home=(0.0, 0.0))
    assert order[0] is east and set(order) == set(frontiers), order
    assert costs[east] == min(costs.values())
    assert 0.3 * TURN_COST < costs[north] - costs[east] < TURN_COST, costs
    assert abs(costs[west] - costs[east] - TURN_COST) < 0.5, costs
    # out and back along each pipe (less the corners it cuts across the
    # manhole between pipes), plus a turn after each frontier
    lengths = 2 * (east.path + north.path + west.path)
    assert lengths - 2.5 < costs[east] - 3 * TURN_COST < lengths, (costs[east], lengths)
    chooser = WholeMap(reach=0.5, tour=True, home=(0.0, 0.0))
    f, hold = chooser.choose(snap, frontiers)
    assert f is east and chooser.order == order


def test_tour_ends_near_home():
    # the robot in a manhole facing east, pipes known 3 m east and 4 m west
    # with frontiers at their ends, and home 20 m further west. Nearest
    # first goes east; so does the tour, but because it then ends in the
    # west, on the way home. With home in the east it goes west first.
    m = MapBuilder(-26.0, -2.0, 26.0, 2.0)
    m.chamber(0.0, 0.0)
    m.pipe([(0.0, 0.0), (4.2, 0.0)])
    m.pipe([(0.0, 0.0), (-5.2, 0.0)])
    snap = Snapshot(m.grid(), (0.0, 0.0), robot_yaw=0.0)
    frontiers = snap.frontiers()
    east = max(frontiers, key=lambda f: f.goal[0])
    _, order = plan_tour(snap, frontiers, home=(-4.5, 0.0))
    assert order[0] is east, order
    _, order = plan_tour(snap, frontiers, home=(3.5, 0.0))
    assert order[0] is not east, order


def test_pipe_graph_of_a_small_network():
    # four manholes in a 16 x 12 m loop and a dead end off B, all mapped
    m = MapBuilder(-2.0, -14.0, 25.0, 2.0)
    a, b, c, d = (0.0, 0.0), (16.0, 0.0), (16.0, -12.0), (0.0, -12.0)
    for ch in (a, b, c, d):
        m.chamber(*ch)
    for p, q in ((a, b), (b, c), (c, d), (d, a)):
        m.pipe([p, q])
    m.pipe([b, (23.0, 0.0)])
    m.cap(23.0, 0.0, 0.0)
    snap = Snapshot(m.grid(), (0.0, 0.0), robot_yaw=0.0)
    graph = PipeGraph(snap, snap.frontiers())
    assert len(graph.nodes) == 4, graph.nodes
    assert math.hypot(*graph.nodes[0]) < 0.3     # nearest the robot first
    lengths = sorted(round(e[2]) for e in graph.edges)
    assert lengths == [12, 12, 16, 16], graph.edges
    (node, length, is_open, end), = graph.ends
    assert math.hypot(graph.nodes[node][0] - 16.0, graph.nodes[node][1]) < 0.3
    assert 6.0 < length < 7.5 and not is_open, graph.ends
    assert 60.0 < graph.length() < 64.0


def explore_whole_map(m, start, lidar=8.0, step=0.6):
    """
    Explore a MapBuilder world the way explorer.py does in full mode.

    A point robot with a 360 deg LiDAR moves `step` m at a time along the
    route to the frontier WholeMap chooses, and when none is left, drives
    back to the start. Returns (known map, path).
    """
    known = np.full((m.h, m.w), -1, np.int8)
    walls = m.wall & ~m.free             # (pipes open their way through the manhole walls)
    blocked = walls | ~m.free
    ang = np.linspace(-math.pi, math.pi, 720, endpoint=False)
    dist = np.arange(0.05, lidar, RES * 0.7)
    chooser = WholeMap()
    pose = list(start)
    path, reached = [tuple(start[:2])], []
    for tick in range(2000):
        cc = ((pose[0] + np.cos(ang)[:, None] * dist - m.ox) / RES).astype(int)
        rr = ((pose[1] + np.sin(ang)[:, None] * dist - m.oy) / RES).astype(int)
        inside = (cc >= 0) & (cc < m.w) & (rr >= 0) & (rr < m.h)
        cc, rr = np.clip(cc, 0, m.w - 1), np.clip(rr, 0, m.h - 1)
        stop = blocked[rr, cc] | ~inside
        first = np.where(stop.any(axis=1), stop.argmax(axis=1), len(dist))
        seen = np.arange(len(dist))[None, :] < first[:, None]
        known[rr[seen], cc[seen]] = 0
        hit = np.nonzero(first < len(dist))[0]
        wall = walls[rr[hit, first[hit]], cc[hit, first[hit]]]
        known[rr[hit[wall], first[hit[wall]]], cc[hit[wall], first[hit[wall]]]] = 100
        snap = Snapshot(Grid(known.ravel(), m.w, m.h, RES, m.ox, m.oy), tuple(pose[:2]))
        passed = [p for p in path if math.hypot(p[0] - pose[0], p[1] - pose[1]) > 3.0]
        frontiers = [f for f in snap.frontiers()
                     if not any(math.hypot(f.goal[0] - x, f.goal[1] - y) < 0.25
                                for x, y in reached)
                     and not any(math.hypot(f.goal[0] - x, f.goal[1] - y) < 0.75
                                 for x, y in passed)]
        f, hold = chooser.choose(snap, frontiers, heading=pose[2] if tick == 0 else None)
        if f is None and not hold:
            route = snap.route(snap.snap(start[:2]))
            return known, path + [snap.g.xy(*c) for c in route[1:]]
        route = snap.route(snap.snap(chooser.target))
        k = next((i for i, c in enumerate(route) if snap.path_length(c) >= step), None)
        if k is None:                 # got there
            x, y = chooser.target
            reached.append(chooser.target)
            chooser.drop()
        else:
            x, y = snap.g.xy(*route[k])
        pose = [x, y, math.atan2(y - pose[1], x - pose[0])]
        path.append((x, y))
    raise AssertionError('exploration did not end')


def test_whole_map_on_a_small_network():
    # four manholes in a 16 x 12 m loop, a dead end off B that can be seen to
    # its end from B, and one round a bend off D that cannot
    m = MapBuilder(-2.0, -18.0, 25.0, 2.0)
    a, b, c, d = (0.0, 0.0), (16.0, 0.0), (16.0, -12.0), (0.0, -12.0)
    for ch in (a, b, c, d):
        m.chamber(*ch)
    for p, q in ((a, b), (b, c), (c, d), (d, a)):
        m.pipe([p, q])
    m.pipe([b, (23.0, 0.0)])
    m.cap(23.0, 0.0, 0.0)
    m.pipe([d, (0.0, -15.0), (-1.5, -15.0)])
    m.cap(-1.5, -15.0, math.pi)
    known, path = explore_whole_map(m, (0.0, 0.0, 0.0))
    # all of it mapped: every free cell 0.2 m or more from a wall is known
    inner = m.free & ~dilate(m.wall & ~m.free, 4)
    assert (known[inner] == 0).mean() > 0.99
    # once round, into both dead ends and back to the start: about 71 m for a
    # robot that knew the map beforehand, over 112 m for one that went round
    # twice
    driven = sum(math.hypot(p[0] - q[0], p[1] - q[1]) for p, q in zip(path, path[1:]))
    assert driven < 100.0, driven
    assert math.hypot(path[-1][0], path[-1][1]) < 0.3, path[-1]


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
