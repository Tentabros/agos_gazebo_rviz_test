"""
Tests for the explorer: the stuck check, the distance driven and the report's map.

    cd agos_sim && python3 -m pytest test/test_explorer.py     (or: colcon test)

Needs ROS 2 sourced (the module imports rclpy and Nav2's messages); skipped
without it. The methods run on stand-ins for the node, without ROS running.
"""
import ast
import types

import numpy as np
import pytest

explorer = pytest.importorskip('agos_sim.explorer')


def stand_in(goal):
    """Return an object with what check_stuck uses, a goal just sent."""
    s = types.SimpleNamespace(stuck_timeout=40.0, progress_pose=None, progress_time=0.0,
                              goal=goal, goal_info='frontier', goal_kind='frontier',
                              target=goal, hop=False, full=True, blacklist=[],
                              leg_stats=[{'failed': 0}], events=[])
    s.log = lambda text, warn=False: s.events.append(text)
    s.cancel_goal = lambda: None
    s.recover = lambda then: s.events.append('recover')
    return s


def first_stuck(s, poses, new_goal=None):
    """Run check_stuck once a second along poses; return when it found the robot stuck."""
    for t, (x, y) in enumerate(poses):
        if new_goal:
            new_goal(s, (x, y))
        if explorer.Explorer.check_stuck(s, (x, y, 0.0), float(t)):
            return t
    return None


def test_rocking_back_and_forth_is_stuck():
    # 0.3 m a second back and forth over 3 m, never further: it moves all
    # the time, but makes no headway towards a goal 10 m away
    poses, x, v = [], 0.0, 0.3
    for _ in range(300):
        poses.append((x, 0.0))
        x += v
        if x >= 3.0 or x <= 0.0:
            v = -v
    s = stand_in((10.0, 0.0))
    assert 40 <= first_stuck(s, poses) <= 60
    assert s.blacklist == [(10.0, 0.0)] and 'recover' in s.events


def test_goals_that_move_on_are_not_stuck():
    # each goal 1.5 m ahead, the next one sent as the robot gets there (as a
    # frontier moves on along a pipe): 300 m of headway, never stuck
    def new_goal(s, p):
        if abs(s.goal[0] - p[0]) < 0.2:
            s.goal = (p[0] + 1.5, 0.1)
    s = stand_in((1.5, 0.0))
    assert first_stuck(s, [(0.1 * i, 0.0) for i in range(3000)], new_goal) is None


def test_standing_still_is_stuck():
    assert first_stuck(stand_in((5.0, 0.0)), [(0.0, 0.0)] * 60) == 40


def test_distance_driven_leaves_out_jitter():
    s = types.SimpleNamespace(odom_frame='odom', robot_frame='base_footprint',
                              distance=0.0, last_odom=None)

    def at(x):
        tf = types.SimpleNamespace(transform=types.SimpleNamespace(
            translation=types.SimpleNamespace(x=x, y=0.0)))
        s.tf_buffer = types.SimpleNamespace(lookup_transform=lambda *args: tf)
        explorer.Explorer.add_driven(s)
    for i in range(1000):           # standing still, the odometry 1 cm back and forth
        at(0.01 * (i % 2))
    assert s.distance == 0.0
    for i in range(501):            # then 10 m in 2 cm steps
        at(0.02 * i)
    assert 9.9 <= s.distance <= 10.0


def report_map(**node):
    """Draw the report's map, 3 x 2 m of free space, on a stand-in; return where it is orange."""
    s = {'full': False, 'map_path': [], 'visited': [], 'leg_stats': [], 'start': None,
         'leftover': lambda: []}
    s.update(node)
    img = explorer.Explorer.map_image(types.SimpleNamespace(**s),
                                      np.zeros((40, 60), np.int16), 0.05, 0.0, 0.0)
    # 4 pixels a cell: x m is column 80 x, y m is row 159 - 80 y (the top row is y = 2 m)
    return (img == (255, 120, 0)).all(axis=2)


def test_report_map_draws_on_the_map_only():
    # a path that leaves the map (the trajectory can, after a loop closure
    # moves the map) is drawn up to the map's edge: a point past the bottom
    # edge once painted an orange line across the whole map
    orange = report_map(trajectory=[(0.0, 1.0, 1.0, 0.0, 'exploring', 1),
                                    (1.0, 1.0, -0.5, 0.0, 'exploring', 1)])
    assert orange[78:160, 80].all()             # from y = 1 m down to the edge
    assert not orange[:78].any()                # and nothing above


def test_report_map_shows_rtabmaps_path_in_full_mode():
    # in full mode the map shows RTAB-Map's path, which fits the map after
    # loop closures, and not the trajectory recorded on the way
    orange = report_map(full=True, map_path=[(0.5, 0.5), (2.5, 0.5)],
                        trajectory=[(0.0, 0.5, 1.5, 0.0, 'exploring', 1),
                                    (1.0, 2.5, 1.5, 0.0, 'exploring', 1)])
    assert orange[159 - 40, 40:200].all()       # y = 0.5 m, x = 0.5 to 2.5 m
    assert not orange[159 - 120].any()          # y = 1.5 m


def test_no_attribute_hides_a_method():
    # self.<name> = ... with the name of one of the node's methods hides the
    # method: a full-map mission once stopped at its first frontier with
    # "'float' object is not callable"
    tree = ast.parse(open(explorer.__file__).read())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'Explorer')
    methods = {n.name for n in cls.body if isinstance(n, ast.FunctionDef)}
    assigned = {t.attr for n in ast.walk(cls) if isinstance(n, (ast.Assign, ast.AugAssign))
                for t in (n.targets if isinstance(n, ast.Assign) else [n.target])
                for t in ast.walk(t)
                if isinstance(t, ast.Attribute) and isinstance(t.value, ast.Name)
                and t.value.id == 'self'}
    assert not methods & assigned, methods & assigned
