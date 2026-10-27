"""
Autonomous exploration for AGOS: the proposal's mission, with TAD frontiers.

The same explorer as practice_agos's (agos_sim/explorer.py), copied here so
the test robot's laptop folder stands on its own. On the test robot it runs
in full mode with room-sized settings (launch/robot.launch.py); the pipe
and manhole wording below is the sewer's, and in a room a "manhole" is any
open area wide enough to turn in.

The mission (the proposal's "AGOS Flow of Operation" figure), from the manhole the
robot starts in:

  1. explore up to leg_length (25 m) along the pipe the robot is facing,
  2. come back to the start (reversing: the robot cannot turn in a pipe),
  3. turn round in the manhole,
  4. explore up to leg_length along the pipe on the other side,
  5. come back to the start,
  6. write a report to <report_dir>/<date_time>/ (map, path, summary):
     ~/agos_missions, or the missions/ folder when started with ./agos_laptop.

Inside a leg the robot goes from frontier to frontier (a frontier is the
edge between mapped free space and unmapped space in RTAB-Map's /map).
Which frontier comes next is decided by TAD (frontiers.py): every frontier
in the leg's direction and within leg_length of the start along the pipe
gets the score F = d + a + t, and the robot drives to the highest with Nav2
(NavigateToPose). It keeps that goal until it gets there or until the
frontier has been mapped (then the next one is further on). A frontier
that has not moved although the robot got to it, or that Nav2 could not
reach, is not tried again, and neither is one on the stretch of pipe the
robot has already driven through (passed_margin or more behind it): that
is a gap in a wall it has seen up close, not unexplored pipe. When no
frontier is left in the leg, the robot drives to the far end of what it
has mapped in the leg (at most leg_length out), so the camera has seen all
of it, and comes back.

With mode:=full the robot maps the whole network instead: no legs and no
length limit. It goes on from frontier to frontier, along every pipe and
into every dead end it finds, until no frontier is left anywhere, then it
drives back to the start and writes the report (frontiers.py, WholeMap):

  - It takes the frontiers nearest it first (TAD decides between those
    within `reach` m of the nearest), and keeps to the pipe it is in.
  - It remembers the other pipes of a manhole it drives through. If its own
    pipe still goes on `peek` m past the manhole, it goes back and maps
    those first, so a short dead end is not left for a long trip back.
  - Long drives (back to a pipe left for later, and home at the end) go in
    steps of `hop_length` m along the mapped pipes: Nav2's global costmap
    only covers 50 x 50 m round the robot.
  - Gaps in the walls of pipes it has driven through are recognised by
    RTAB-Map's own record of where the robot has been (/mapPath), which
    stays right when a loop closure moves the map.
  - Frontiers it could not reach are tried once more before it goes home.

    ./agos_laptop explore                                   # the test robot, full mode
    ros2 launch agos_robot_nav robot.launch.py explore:=true
    ros2 run agos_robot_nav explorer                        # on its own
    ros2 service call /explore/start std_srvs/srv/Trigger  # when autostart:=false
    ros2 service call /explore/stop std_srvs/srv/Trigger   # stop and write the report

Forwards into new pipe (NAV_ALGORITHMS.md): the camera looks ahead and the
robot can only turn round in a manhole, so it drives to a frontier forwards
and reverses only over pipe it has already mapped. For each frontier,
Snapshot.approach (frontiers.py) says how: drive on if the route leaves
ahead of the robot; else turn on the spot if it is in a manhole; else back
up along the route to the first manhole on it and turn there; else drive on
to a manhole ahead, turn and come back. The turns count in the cost of
each frontier, and a frontier it could only reach in reverse is not chosen.
Every goal goes to Nav2 with one of two planner and controller pairs
(config/nav2_base.yaml), published on /planner_selector and
/controller_selector:

    forwards  Hybrid-A* (GridBased) + MPPI that never reverses (FollowPath)
    reverse   Smac 2D through mapped space only (Mapped) + MPPI that may
              reverse (Reverse): back to a manhole to turn, and goals in
              mapped pipe behind the robot (the leg end, the start)

In full mode the next frontier is the first of the shortest tour through
all of them and back to the start (frontiers.py, plan_tour), TAD deciding
between first stops whose tours are within `reach` m of the best. The
report has the network as a graph (manholes and the pipes between them,
frontiers.py PipeGraph) and how far the robot reversed. forward_only:=false
and tour:=false give the earlier behaviour, for comparison.

RViz: the Explorer display shows the frontiers (green: in this leg, or that
may be chosen; grey: not), the TAD score of each, the current goal (yellow;
orange: the next step on the way to it), the path driven, the manholes
(blue), the pipes between them (purple) and, in full mode, the tour
(magenta). /explore/status says what it is doing.
"""
import csv
import datetime
import math
import os
import struct
import time
import zlib

from action_msgs.msg import GoalStatus
from agos_robot_nav.frontiers import Grid, PipeGraph, Snapshot, WholeMap
from geometry_msgs.msg import Point
from nav2_msgs.action import BackUp, NavigateToPose, Spin
from nav2_msgs.srv import ClearEntireCostmap
from nav_msgs.msg import OccupancyGrid, Path
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from std_msgs.msg import ColorRGBA, String
from std_srvs.srv import Trigger
import tf2_ros
from visualization_msgs.msg import Marker, MarkerArray

WAITING, EXPLORING, TO_LEG_END, RETURNING, TURNING, RECOVERING, REPOSITIONING, DONE = (
    'waiting', 'exploring', 'to leg end', 'returning', 'turning round', 'recovering',
    'to a manhole to turn', 'done')

# How the robot drives to a goal: (controller, planner) in config/nav2_base.yaml
DRIVE = {
    'forward': ('FollowPath', 'GridBased'),     # Hybrid-A* + MPPI that never reverses
    'reverse': ('Reverse', 'Mapped'),           # mapped space only + MPPI that may reverse
    'either': ('Reverse', 'GridBased2D'),       # forward_only:=false: as before
}
SELECT_WAIT = 0.5   # s between choosing the planner and controller and sending a goal
GRAPH_EVERY = 10.0  # s between updates of the pipe network markers

HOP_LEAD = 3.0      # m (full mode): send the next step of a long drive this far before the last
# Nav2 errors that say nothing about the goal: the map -> odom transform came
# late (FollowPath / ComputePathToPose TF_ERROR). On a busy computer RTAB-Map's
# transform can be a second or two late now and then, and Nav2 then fails
# every goal at once: the goal is sent again a moment later, for a while.
TRANSIENT_ERRORS = (102, 202)
TRANSIENT_WAIT = 1.0     # s before sending the goal again
TRANSIENT_TIME = 15.0    # s of such errors in a row before the goal counts as failed
# Nav2 cannot plan because its map shows the robot inside a wall (a wall
# RTAB-Map drew twice after the map drifted): back up and send the goal again.
START_OCCUPIED = 205
# Full mode: frontiers this close beside the path driven are mostly gaps in a
# wall the map has drawn twice (after it drifted, or where a loop through the
# network has not been closed). The robot goes there only for the frontier it
# is driving to and for pipes it left for later; the report lists the others.
NEAR_TRAIL = 2.5    # m
NEAR_TRAIL_WHY = 'next to the path driven (most likely a wall the map drew twice)'
REVERSE_ONLY_WHY = 'only reachable in reverse (nowhere to turn round on the way)'
# After the robot gets to a frontier, RTAB-Map adds what the LiDAR sees from
# there at its next map update (once a second). Choosing before that, a
# frontier round a bend still looks like a dead end where the robot stands.
SETTLE = 2.0        # s to wait after reaching a frontier before choosing again
ARRIVED_RADIUS = 0.6   # m: a "reached" goal further than this from the robot was not reached
PROGRESS = 0.3      # m towards the goal that counts as progress (see check_stuck)


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def heading(a):
    """Return an angle in radians as whole degrees from 0 to 359."""
    return int(round(math.degrees(a))) % 360


def fmt_xy(p):
    """Return a point as '(x, y)' in metres, without '-0.00'."""
    return '(%.2f, %.2f)' % (round(p[0], 2) + 0.0, round(p[1], 2) + 0.0)


class Explorer(Node):

    def __init__(self):
        super().__init__('explorer')
        p = self.declare_parameter
        p('mode', 'legs')                # 'legs': the proposal's mission; 'full': the whole map
        p('legs', [0.0, 180.0])          # deg, relative to the heading at the start
        p('leg_length', 25.0)            # m along the pipe from the start (0 = no limit)
        p('leg_half_angle', 60.0)        # deg: pipes leaving the start manhole within this
        p('exit_radius', 2.5)            # m: further than this from the start = in a pipe
        p('min_frontier_size', 0.3)      # m: smaller frontiers are ignored
        p('clearance', 0.2)              # m: the robot's centre stays this far from walls
        p('hold_radius', 1.0)            # m: the goal frontier is still there if within this
        p('skip_radius', 1.0)            # m: frontiers this close to a failed goal are skipped
        p('reached_radius', 0.25)        # m: ...and this close to one the robot reached
        p('passed_radius', 0.75)         # m: ...and this close to the path driven in this leg
        p('passed_margin', 3.0)          # m: (only the part of it this far behind the robot)
        p('stuck_timeout', 40.0)         # s without 0.3 m of headway towards the goal = stuck
        p('turn_around', True)           # turn round in the start manhole between legs
        # full mode (see frontiers.py, WholeMap):
        p('reach', 2.0)                  # m: TAD picks among frontiers this far past the nearest
        p('peek', 12.0)                  # m past a manhole before going back for its other pipes
        p('hop_length', 8.0)             # m: longer drives go in steps of this
        p('look_round', True)            # turn once round in the start manhole first
        p('map_path_topic', '/mapPath')  # RTAB-Map's map nodes: where the robot has been
        p('tour', True)                  # full mode: next = first of the best tour (plan_tour)
        p('forward_only', True)          # into new pipe forwards only; reverse over mapped pipe
        p('autostart', True)             # start as soon as Nav2 and the map are up
        p('report_dir', os.environ.get('AGOS_MISSION_DIR', '~/agos_missions'))
        p('global_frame', 'map')
        p('robot_frame', 'base_footprint')
        p('odom_frame', 'odom')          # the distance driven is added up here (add_driven)
        prm = self.get_parameter
        self.mode = str(prm('mode').value).strip().lower()
        if self.mode not in ('legs', 'full'):
            self.get_logger().warning('Unknown mode "%s": using legs (or full)' % self.mode)
            self.mode = 'legs'
        self.full = self.mode == 'full'
        self.legs = [math.radians(a) for a in prm('legs').value]
        self.leg_length = float(prm('leg_length').value)
        self.half_angle = math.radians(float(prm('leg_half_angle').value))
        self.exit_radius = float(prm('exit_radius').value)
        self.min_size = float(prm('min_frontier_size').value)
        self.clearance = float(prm('clearance').value)
        self.hold_radius = float(prm('hold_radius').value)
        self.skip_radius = float(prm('skip_radius').value)
        self.reached_radius = float(prm('reached_radius').value)
        self.passed_radius = float(prm('passed_radius').value)
        self.passed_margin = float(prm('passed_margin').value)
        self.stuck_timeout = float(prm('stuck_timeout').value)
        self.turn_around = bool(prm('turn_around').value)
        self.reach = float(prm('reach').value)
        self.peek = float(prm('peek').value)
        self.hop_length = float(prm('hop_length').value)
        self.look_round = bool(prm('look_round').value)
        self.tour = bool(prm('tour').value)
        self.forward_only = bool(prm('forward_only').value)
        self.autostart = bool(prm('autostart').value)
        self.report_dir = os.path.expanduser(prm('report_dir').value)
        self.global_frame = prm('global_frame').value
        self.robot_frame = prm('robot_frame').value
        self.odom_frame = prm('odom_frame').value

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, latched)
        self.map_path = []             # (x, y) of RTAB-Map's map nodes (full mode)
        if self.full:
            self.create_subscription(Path, prm('map_path_topic').value, self.on_map_path, 1)
        self.markers_pub = self.create_publisher(MarkerArray, '/explore/markers', latched)
        self.status_pub = self.create_publisher(String, '/explore/status', latched)
        # (the behaviour tree's selector nodes take the latest, latched)
        self.controller_pub = self.create_publisher(String, '/controller_selector', latched)
        self.planner_pub = self.create_publisher(String, '/planner_selector', latched)
        self.drive_mode = None
        self.create_service(Trigger, '/explore/start', self.srv_start)
        self.create_service(Trigger, '/explore/stop', self.srv_stop)

        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.spin_ac = ActionClient(self, Spin, 'spin')
        self.backup_ac = ActionClient(self, BackUp, 'backup')
        self.nav2_active = self.create_client(Trigger, '/lifecycle_manager_navigation/is_active')
        self.clear_clients = [
            self.create_client(ClearEntireCostmap, name) for name in (
                '/global_costmap/clear_entirely_global_costmap',
                '/local_costmap/clear_entirely_local_costmap')]
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.map_msg = None
        self.pose_time = None          # s, time of the last robot pose (see now_s)
        self.state = WAITING
        self.start_requested = self.autostart
        self.nav2_up = False
        self.active_future = None
        self.last_active_check = None
        self.status_text = ''
        self.reset_mission()
        self.timer = self.create_timer(1.0, self.step)
        self.log('Waiting for Nav2 and the map' if self.autostart
                 else 'Ready: call /explore/start to begin')

    # ------------------------------------------------------------ bookkeeping

    def reset_mission(self):
        self.start = None              # (x, y, yaw)
        self.leg = 0
        self.goal = None               # (x, y): where Nav2 is sending the robot
        self.target = None             # (x, y): where it is going (full mode: goal may be a step)
        self.target_facing = 0.0
        self.hop = False               # goal is a step on the way to target (full mode)
        self.goal_kind = None          # 'frontier', 'leg end', 'home'
        self.goal_seq = 0
        self.goal_active = False
        self.goal_handle = None
        self.goal_info = ''
        self.goal_facing = 0.0
        self.goal_mode = 'forward'     # how Nav2 drives there (DRIVE)
        self.pending = None            # (kind, xy, info, facing, state): goal after a turn
        self.turn_for = None           # why the robot is turning: 'start', 'leg' or 'approach'
        self.turns = 0                 # turns on the spot to face a pipe before driving into it
        self.reverse_total = 0.0       # m driven backwards (add_driven)
        self.reverse_spot = 0.0        # ... of that, back to a manhole to turn
        self.reverse_frontier = 0.0    # ... and on the way to a frontier
        self.reverse_only = []         # frontiers it could have reached only in reverse
        self.graph = None              # PipeGraph of the map (markers and report)
        self.graph_time = None
        self.transient_since = None    # first of the TRANSIENT_ERRORS in a row
        self.transient_last = None     # and the last
        self.resend_at = None          # when to send the goal again after one of them
        self.resend_after = False      # send the goal again after the recovery (START_OCCUPIED)
        self.occupied = 0              # START_OCCUPIED errors in a row
        self.arrived = None            # when the robot last got to a frontier (see SETTLE)
        self.short = 0                 # "reached" results short of the goal in a row
        self.blacklist = []
        self.reached = []
        self.visited = []
        self.failures = 0
        self.home_failures = 0
        self.trajectory = []           # (t, x, y, yaw, state, leg)
        self.distance = 0.0            # m driven, on the odometry (add_driven)
        self.last_odom = None
        self.events = []
        self.leg_stats = []
        self.progress_pose = None      # where the robot last made headway (check_stuck)
        self.progress_time = None
        self.after_recovery = None
        self.mission_start_time = None
        self.result = ''
        self.last_frontiers = []
        self.near_trail = []           # full mode: frontiers left beside the path (NEAR_TRAIL)
        self.last_snapshot = None
        self.chooser = WholeMap(self.reach, self.hold_radius, self.peek,   # full mode
                                tour=self.tour)
        self.retried = False           # full mode: unreachable frontiers tried again

    def now_s(self):
        # Mission time is the time of the last robot pose on /tf: simulation
        # time with Gazebo, wall time on the real robot. The node itself runs
        # on the wall clock (use_sim_time false in agos.launch.py), because
        # following /clock (500 messages a second) would cost this Python node
        # a sixth of a CPU core.
        if self.pose_time is not None:
            return self.pose_time
        return self.get_clock().now().nanoseconds * 1e-9

    def log(self, text, warn=False):
        if not rclpy.ok():
            # shut down by Ctrl+C: the ROS logger would add a line saying it
            # cannot publish to /rosout any more
            print('[explorer] %s' % text, flush=True)
        # (two separate calls: an rclpy logger call site may not change severity)
        elif warn:
            self.get_logger().warning(text)
        else:
            self.get_logger().info(text)
        if self.mission_start_time is not None:
            self.events.append('t = %5.0f s  %s' % (self.now_s() - self.mission_start_time, text))

    def set_status(self, text):
        if text != self.status_text:
            self.status_text = text
            self.status_pub.publish(String(data=text))

    def on_map(self, msg):
        self.map_msg = msg

    def on_map_path(self, msg):
        self.map_path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]

    def robot_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(self.global_frame, self.robot_frame, Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return None
        self.pose_time = Time.from_msg(t.header.stamp).nanoseconds * 1e-9
        tr = t.transform
        return tr.translation.x, tr.translation.y, yaw_of(tr.rotation)

    def add_driven(self):
        """
        Add up the distance driven, on the odometry.

        On the map the robot jumps when a loop closure moves the map (4 m at
        the end of a whole-network run), and the report counted those jumps
        as driven. Moves under 5 cm wait until they add up, so the odometry's
        jitter while the robot stands still does not count either.
        """
        try:
            t = self.tf_buffer.lookup_transform(self.odom_frame, self.robot_frame, Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return
        x, y = t.transform.translation.x, t.transform.translation.y
        if self.last_odom is None:
            self.last_odom = (x, y)
            return
        dx, dy = x - self.last_odom[0], y - self.last_odom[1]
        step = math.hypot(dx, dy)
        if step > 0.05:
            self.distance += step
            self.last_odom = (x, y)
            yaw = yaw_of(t.transform.rotation)
            if dx * math.cos(yaw) + dy * math.sin(yaw) < 0.0:      # backwards
                self.reverse_total += step
                if self.state == REPOSITIONING:
                    self.reverse_spot += step
                elif self.state == EXPLORING and self.goal_kind == 'frontier':
                    self.reverse_frontier += step

    def leg_bearing(self):
        return wrap(self.start[2] + self.legs[self.leg])

    def leg_name(self):
        if self.full:
            return 'whole map'
        deg = math.degrees(self.legs[self.leg]) % 360.0
        side = 'ahead' if deg < 1.0 or deg > 359.0 else ('behind' if abs(deg - 180.0) < 1.0
                                                         else '%.0f deg' % deg)
        return 'leg %d of %d (%s)' % (self.leg + 1, len(self.legs), side)

    # ------------------------------------------------------------ services

    def srv_start(self, request, response):
        if self.state not in (WAITING, DONE):
            response.success = False
            response.message = 'Already running (%s)' % self.state
            return response
        if self.state == DONE:
            self.reset_mission()
            self.state = WAITING
        self.start_requested = True
        response.success = True
        response.message = 'Starting as soon as Nav2 and the map are ready'
        return response

    def srv_stop(self, request, response):
        if self.state in (WAITING, DONE):
            self.start_requested = False
            response.success = True
            response.message = 'Not running'
            return response
        self.cancel_goal()
        self.finish('stopped (/explore/stop)')
        response.success = True
        response.message = 'Stopped; report written'
        return response

    # ------------------------------------------------------------ main loop

    def step(self):
        pose = self.robot_pose()
        if self.state == WAITING:
            self.wait_for_start(pose)
            return
        if self.state == DONE or pose is None:
            return
        t = self.now_s()
        if not self.trajectory or math.hypot(pose[0] - self.trajectory[-1][1],
                                             pose[1] - self.trajectory[-1][2]) > 0.05:
            self.trajectory.append((t, pose[0], pose[1], pose[2], self.state, self.leg + 1))
        self.add_driven()
        if self.goal_active and self.check_stuck(pose, t):
            return
        if self.resend_at is not None and t >= self.resend_at and \
                self.state not in (RECOVERING, TURNING):
            # after one of the TRANSIENT_ERRORS or START_OCCUPIED (a new
            # choice below replaces it)
            self.send_goal(self.goal, self.goal_kind, self.goal_info, facing=self.goal_facing,
                           target=self.target if self.hop else None, fresh=False,
                           mode=self.goal_mode)
        if self.arrived is not None and t - self.arrived < SETTLE and self.state == EXPLORING:
            self.publish_markers()
            return
        self.arrived = None

        if self.state == EXPLORING:
            if self.full:
                self.explore_full(pose)
            else:
                self.explore(pose)
        elif self.state == RETURNING and self.full:
            self.return_full(pose)
        elif self.state == REPOSITIONING and self.full and self.hop:
            self.keep_driving(self.snapshot(pose), pose)
        # TO_LEG_END, RETURNING, TURNING, REPOSITIONING and RECOVERING advance in the
        # action callbacks (in full mode RETURNING and REPOSITIONING also go on here,
        # one step of the route at a time)
        if self.last_snapshot is not None and (self.graph_time is None
                                               or t - self.graph_time >= GRAPH_EVERY):
            self.graph_time = t
            self.graph = PipeGraph(self.last_snapshot)
        self.publish_markers()
        self.set_status('%s, %s%s' % (self.state, self.leg_name(),
                                      (', goal ' + self.goal_info) if self.goal_active else ''))

    def wait_for_start(self, pose):
        if not self.start_requested:
            self.set_status('ready: call /explore/start')
            return
        if not self.nav2_up:
            self.set_status('waiting for Nav2')
            self.poll_nav2()
            return
        if self.map_msg is None or pose is None:
            self.set_status('waiting for the map and the robot position')
            return
        self.start = pose
        self.chooser.home = (pose[0], pose[1])
        self.mission_start_time = self.now_s()
        self.leg = 0
        self.state = EXPLORING
        if self.full:
            self.log('Mission started at %s, facing %d deg: the whole map, then back here'
                     % (fmt_xy(pose), heading(pose[2])))
        else:
            self.log('Mission started at %s, facing %d deg: %d legs of %s'
                     % (fmt_xy(pose), heading(pose[2]), len(self.legs),
                        ('%.0f m' % self.leg_length) if self.leg_length > 0 else 'any length'))
        if self.full and self.look_round:
            # The camera only looks ahead, and RTAB-Map recognises a place by
            # what the camera sees. Back at the start after the whole network
            # the robot may come through facing the other way (in a test run
            # it reversed through), and the start then looks new: the loop is
            # not closed and the map stays off by the odometry's drift.
            self.state = TURNING
            self.turn_for = 'start'
            self.log('Turning once round in the manhole first, so that RTAB-Map knows it '
                     'from every side when the robot comes back')
            self.send_spin(2.0 * math.pi)
            return
        self.begin_leg()

    def poll_nav2(self):
        # wall-clock seconds: a reply that never comes must not stall the start
        now = time.monotonic()
        if self.active_future is not None:
            if not self.active_future.done():
                if now - self.last_active_check < 10.0:
                    return
                self.active_future.cancel()      # no reply: ask again
            elif self.active_future.result() is not None and \
                    self.active_future.result().success:
                self.active_future = None
                self.nav2_up = True
                self.get_logger().info('Nav2 is active')
                return
            self.active_future = None
        if self.last_active_check is None or now - self.last_active_check > 2.0:
            self.last_active_check = now
            if self.nav2_active.service_is_ready():
                self.active_future = self.nav2_active.call_async(Trigger.Request())

    # ------------------------------------------------------------ legs

    def begin_leg(self):
        self.blacklist = []
        self.reached = []
        self.failures = 0
        self.leg_stats.append({'leg': self.leg + 1, 'bearing': self.leg_bearing(),
                               'start': self.now_s(), 'end': None, 'sent': 0, 'goals': 0,
                               'failed': 0, 'explored': 0.0, 'end_point': None, 'note': ''})
        if self.full:
            self.log('Exploring the whole map: the pipe ahead first, then the nearest frontiers')
        else:
            self.log('Starting %s: heading %d deg' % (self.leg_name(),
                                                      heading(self.leg_bearing())))

    def snapshot(self, pose):
        m = self.map_msg
        grid = Grid(m.data, m.info.width, m.info.height, m.info.resolution,
                    m.info.origin.position.x, m.info.origin.position.y)
        # (full mode: no legs, so no paths from the start are needed)
        snap = Snapshot(grid, (pose[0], pose[1]),
                        None if self.full else (self.start[0], self.start[1]),
                        clearance=self.clearance, exit_radius=self.exit_radius,
                        driven=self.driven(pose) if self.full else None,
                        robot_yaw=pose[2] if self.forward_only else None)
        self.last_snapshot = snap
        return snap

    def driven(self, pose):
        """
        Where the robot has driven, oldest first, up to where it is (full mode).

        RTAB-Map's map nodes (/mapPath) when they have arrived (they move with
        the map when a loop closure corrects it), else the path recorded here.
        """
        if self.map_path:
            points = list(self.map_path)
        else:
            points = [(x, y) for _, x, y, _, _, _ in self.trajectory]
        return points + [(pose[0], pose[1])]

    def explore(self, pose):
        snap = self.snapshot(pose)
        bearing = self.leg_bearing()
        frontiers = snap.frontiers(self.min_size)
        passed = self.passed(pose)
        in_leg = [f for f in frontiers
                  if snap.in_leg(f, bearing, self.half_angle, self.leg_length)
                  and not self.near(f.goal, self.blacklist, self.skip_radius)
                  and not self.near(f.goal, self.reached, self.reached_radius)
                  and not self.near(f.goal, passed, self.passed_radius)]
        self.reverse_only = [f for f in in_leg if not math.isfinite(f.travel)]
        in_leg = [f for f in in_leg if math.isfinite(f.travel)]
        self.last_frontiers = [(f, f in in_leg) for f in frontiers]
        stats = self.leg_stats[-1]
        stats['explored'] = max(stats['explored'], min(
            snap.explored_along(bearing, self.half_angle),
            self.leg_length if self.leg_length > 0 else math.inf))

        if self.goal_active and self.goal_kind == 'frontier':
            # TAD holds the goal while its frontier is still there
            if any(math.hypot(f.goal[0] - self.goal[0], f.goal[1] - self.goal[1])
                   < self.hold_radius for f in in_leg):
                return
            if not in_leg:
                return       # mapped; let the robot finish this goal, then go to the leg end

        if in_leg:
            self.send_frontier(max(in_leg, key=lambda f: f.score), pose, snap)
            return
        if self.goal_active:
            return
        # nothing left to explore in this leg: drive to its far end, then go back
        end = snap.leg_end(bearing, self.half_angle, self.leg_length)
        stats['end_point'] = end[0] if end else None
        if end and math.hypot(end[0][0] - pose[0], end[0][1] - pose[1]) > 1.0:
            self.log('%s explored (%.1f m along the pipe); driving to its far end (%.2f, %.2f)'
                     % (self.leg_name().capitalize(), stats['explored'], end[0][0], end[0][1]))
            self.state = TO_LEG_END
            # (mapped pipe: forwards if it is ahead, else it may reverse there)
            cell = snap.snap(end[0])
            ahead = cell is None or snap.approach(cell)[1] == 'ahead'
            self.send_goal(end[0], 'leg end', 'leg end (%.2f, %.2f)' % end[0],
                           mode='forward' if ahead else 'reverse')
        else:
            if end is None:
                stats['note'] = 'no pipe in this direction'
                self.log('No pipe found in the direction of %s' % self.leg_name(), warn=True)
            self.go_home()

    @staticmethod
    def near(xy, points, radius):
        if len(points) == 0:
            return False
        p = np.asarray(points, float).reshape(-1, 2)
        return bool((np.hypot(p[:, 0] - xy[0], p[:, 1] - xy[1]) < radius).any())

    def skipped(self, f, passed):
        """Check whether a frontier is one not to go to (failed, reached, or passed)."""
        return (self.near(f.goal, self.blacklist, self.skip_radius)
                or self.near(f.goal, self.reached, self.reached_radius)
                or self.near(f.goal, passed, self.passed_radius))

    def passed(self, pose):
        """
        Where the robot explored in this leg, leaving out the last passed_margin.

        Seen from far off, RTAB-Map's map of a pipe the robot has driven
        through gets small gaps in its walls again (LiDAR rays that graze a
        wall clear some wall cells), and the free cells next to those gaps
        look like frontiers. They are behind the robot and nothing new is
        there, so they are skipped rather than driven back to.

        In full mode it is everywhere the robot has been: RTAB-Map's map
        nodes (/mapPath), which move with the map when a loop closure
        corrects it (the path recorded here does not), or until they arrive
        the path driven while exploring.
        """
        if self.full and self.map_path:
            source = self.map_path
        else:
            leg = self.leg + 1
            source = [(x, y) for _, x, y, _, state, n in self.trajectory
                      if state == EXPLORING and (self.full or n == leg)]
        points = []
        for x, y in source:
            if math.hypot(x - pose[0], y - pose[1]) > self.passed_margin and \
                    (not points or math.hypot(x - points[-1][0], y - points[-1][1]) > 0.25):
                points.append((x, y))
        return np.array(points, float).reshape(-1, 2)

    def send_frontier(self, f, pose, snap=None):
        info = ('frontier (%.2f, %.2f), F = %.1f = d %.1f + a %d + t %+d'
                % (f.goal[0], f.goal[1], f.score, f.d_adj, f.a, f.t))
        if self.goal_active and self.target is not None and \
                math.hypot(f.goal[0] - self.target[0], f.goal[1] - self.target[1]) < 0.3:
            return
        self.log('Goal: ' + info)
        self.leg_stats[-1]['sent'] += 1
        facing = math.atan2(f.goal[1] - pose[1], f.goal[0] - pose[0])
        if f.how != 'ahead' and snap is not None:
            # it faces the wrong way: turn round first (Snapshot.approach)
            self.reposition(f.how, f.spot, ('frontier', f.goal, info, facing, EXPLORING),
                            snap, pose)
        elif self.full:
            # (a new choice restarts the stuck timer; following the same pipe does not)
            self.drive_to(snap, f.goal, 'frontier', info, facing, fresh=self.target is None)
        else:
            self.send_goal(f.goal, 'frontier', info, facing=facing)

    def reposition(self, how, spot, pending, snap, pose):
        """
        Turn round before driving to a goal forwards (Snapshot.approach).

        how 'turn': turn on the spot now; 'back': reverse to the manhole at
        `spot` (over mapped pipe) and turn there; 'loop': drive on to the
        manhole at `spot` and turn there. Then the robot drives to the goal
        in `pending` (kind, xy, info, facing, state to go on in).
        """
        self.cancel_goal()
        self.pending = pending
        if how == 'turn':
            self.turn_to_route(snap, pose)
            return
        self.log('%s to the manhole at %s to turn round there first'
                 % ('Backing up' if how == 'back' else 'Driving on', fmt_xy(spot)))
        self.state = REPOSITIONING
        self.drive_to(snap, spot, 'turn spot', 'manhole ' + fmt_xy(spot),
                      mode='reverse' if how == 'back' else 'forward')

    def turn_to_route(self, snap, pose):
        """Turn on the spot to face the route to the pending goal, then go on (resume)."""
        cell = snap.snap(self.pending[1]) if snap is not None and pose is not None else None
        path = snap.route(cell) if cell else None
        bearing = snap.leaving(path) if path else None
        angle = wrap(bearing - pose[2]) if bearing is not None else 0.0
        if abs(angle) < math.radians(20):
            self.resume()
            return
        self.turns += 1
        self.state = TURNING
        self.turn_for = 'approach'
        self.log('Turning %.0f deg to face the pipe to %s'
                 % (math.degrees(angle), self.pending[2]))
        self.send_spin(angle)

    def resume(self):
        """Drive to the goal the robot turned round for, forwards."""
        if self.pending is None:
            self.state = EXPLORING
            return
        kind, xy, info, facing, state = self.pending
        self.pending = None
        self.state = state
        pose = self.robot_pose()
        snap = self.snapshot(pose) if pose is not None and self.map_msg is not None else None
        self.drive_to(snap, xy, kind, info, facing)

    def pending_failed(self, pending):
        """Give up on a goal when the robot could not get to the manhole to turn round in."""
        if pending is None:
            self.state = EXPLORING
            return
        kind, xy, info = pending[0], pending[1], pending[2]
        if kind == 'home':
            # back over the mapped pipes as it is, then
            self.log('Could not turn round on the way: going back to the start as it is',
                     warn=True)
            self.state = RETURNING
            self.drive_to(self.last_snapshot, xy, 'home', info, pending[3], mode='reverse')
            return
        self.blacklist.append(xy)
        self.leg_stats[-1]['failed'] += 1
        self.log('Could not get to a manhole to turn round for ' + info, warn=True)
        self.state = EXPLORING
        if self.full:
            self.target, self.hop = None, False      # choose again

    def go_home(self, snap=None):
        stats = self.leg_stats[-1]
        if stats['end'] is None:
            stats['end'] = self.now_s()
        self.state = RETURNING
        self.home_failures = 0
        self.log('Returning to the start ' + fmt_xy(self.start))
        if self.full:
            self.head_home(snap, self.robot_pose())
        else:
            # the proposal's mission: back down the leg (mapped), then turn in the manhole
            self.send_goal(self.start[:2], 'home', 'start ' + fmt_xy(self.start),
                           facing=self.start[2], mode='reverse')

    def head_home(self, snap, pose):
        """Drive back to the start, forwards where it can turn round first (full mode)."""
        info = 'start ' + fmt_xy(self.start)
        cell = snap.snap(self.start[:2]) if snap is not None else None
        how, spot = (snap.approach(cell)[1:] if cell is not None and pose is not None
                     else ('ahead', None))
        if how in ('turn', 'back', 'loop'):
            self.reposition(how, spot, ('home', self.start[:2], info, self.start[2], RETURNING),
                            snap, pose)
        else:
            # ahead, or (nowhere to turn) back over the mapped pipes
            self.drive_to(snap, self.start[:2], 'home', info, self.start[2],
                          mode='forward' if how == 'ahead' else 'reverse')

    # ------------------------------------------------------------ full mode

    def explore_full(self, pose):
        """Go from frontier to frontier until none is left anywhere, then home (full mode)."""
        snap = self.snapshot(pose)
        frontiers = snap.frontiers(self.min_size)
        passed = self.passed(pose)
        choosable = [f for f in frontiers if not self.skipped(f, passed)]
        # (only reachable in reverse: nowhere to turn round on the way)
        self.reverse_only = [f for f in choosable if not math.isfinite(f.travel)]
        choosable = [f for f in choosable if math.isfinite(f.travel)]
        chooser = self.chooser
        chooser.target = self.target if self.goal_kind == 'frontier' else None
        self.near_trail = [f for f in choosable if self.near(f.goal, passed, NEAR_TRAIL)
                           and not chooser.expects(snap, f)]
        away = [f for f in choosable if f not in self.near_trail]
        self.last_frontiers = [(f, f in away) for f in frontiers]
        f, hold = chooser.choose(snap, away,
                                 heading=pose[2] if self.leg_stats[-1]['sent'] == 0 else None,
                                 now=self.now_s())
        if chooser.note:
            self.log(chooser.note[0].upper() + chooser.note[1:])
        if hold:
            self.keep_driving(snap, pose)
            return
        if f is not None:
            self.send_frontier(f, pose, snap)
            return
        if self.goal_active:
            return          # its frontier has been mapped: finish this goal, then look again
        again = [g for g in frontiers if self.near(g.goal, self.blacklist, self.skip_radius)]
        if again and not self.retried:
            self.retried = True
            self.blacklist = []
            self.log('No frontier left but %d it could not reach: trying those once more'
                     % len(again))
            return
        self.log('No frontier left: the whole map is explored')
        self.go_home(snap)

    def drive_to(self, snap, xy, kind, info, facing=0.0, fresh=True, mode='forward'):
        """
        Send the robot to xy, in steps of hop_length m along the route (full mode).

        Nav2's global costmap is a 50 x 50 m window round the robot, so it
        cannot plan to a goal further away; and its planner may take a short
        cut through unmapped space, where this route keeps to mapped pipe.
        (Legs mode: in one go; no leg is longer than the window.)
        """
        goal, target = xy, None
        cell = snap.snap(xy) if snap is not None and self.full else None
        if cell is not None:
            step = snap.waypoint(cell, self.hop_length)
            if step is not None:
                goal, target = step, xy
        self.target_facing = facing
        self.send_goal(goal, kind, info, facing=facing, target=target, fresh=fresh, mode=mode)

    def keep_driving(self, snap, pose):
        """Send the next step of a long drive when the robot is nearly at the last one."""
        if not self.hop or self.target is None:
            return
        if self.goal_active and \
                math.hypot(self.goal[0] - pose[0], self.goal[1] - pose[1]) > HOP_LEAD:
            return
        self.drive_to(snap, self.target, self.goal_kind, self.goal_info, self.target_facing,
                      fresh=False, mode=self.goal_mode)

    def return_full(self, pose):
        """Drive back to the start along the mapped pipes (full mode)."""
        snap = self.snapshot(pose)
        if self.goal_active:
            self.keep_driving(snap, pose)
        else:
            self.head_home(snap, pose)

    def leftover(self):
        """Frontiers still in the map when the mission ended (full mode): [((x, y), why)]."""
        out = []
        for f, choosable in self.last_frontiers:
            if choosable:
                why = 'not explored'
            elif f in self.near_trail:
                why = NEAR_TRAIL_WHY
            elif f in self.reverse_only:
                why = REVERSE_ONLY_WHY
            elif self.near(f.goal, self.blacklist, self.skip_radius):
                why = 'could not be reached'
            elif self.near(f.goal, self.reached, self.reached_radius):
                why = 'reached, but nothing more could be mapped from there'
            else:
                why = 'gap in the wall of a pipe already driven through'
            out.append((f.goal, why))
        return out

    def next_leg(self):
        if self.leg + 1 >= len(self.legs):
            self.finish('completed')
            return
        pose = self.robot_pose()
        self.leg += 1
        turn = wrap(self.leg_bearing() - (pose[2] if pose else self.start[2]))
        if self.turn_around and abs(turn) > math.radians(20):
            self.state = TURNING
            self.turn_for = 'leg'
            self.log('Turning %.0f deg in the manhole' % math.degrees(turn))
            self.send_spin(turn)
        else:
            self.state = EXPLORING
            self.begin_leg()

    # ------------------------------------------------------------ Nav2 actions

    def select(self, mode):
        """Choose Nav2's planner and controller (DRIVE[mode]); True if that changed them."""
        if mode == self.drive_mode:
            return False
        controller, planner = DRIVE[mode]
        self.controller_pub.publish(String(data=controller))
        self.planner_pub.publish(String(data=planner))
        self.drive_mode = mode
        return True

    def send_goal(self, xy, kind, info, facing=0.0, target=None, fresh=True, mode='forward'):
        """
        Send a NavigateToPose goal.

        target: where the robot is really going, when xy is only a step on
        the way (full mode). fresh: restart the stuck timer. mode: how Nav2
        drives there (DRIVE): 'forward' into new pipe, 'reverse' where it
        may back up over mapped pipe.
        """
        self.resend_at = None
        if not self.forward_only:
            mode = 'either'
        if not self.nav.server_is_ready():
            self.log('Nav2 navigate_to_pose is not available', warn=True)
            return
        changed = self.select(mode)
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self.global_frame
        goal.pose.header.stamp = Time().to_msg()     # 0 = the latest (see now_s)
        goal.pose.pose.position.x = float(xy[0])
        goal.pose.pose.position.y = float(xy[1])
        goal.pose.pose.orientation.z = math.sin(facing / 2.0)
        goal.pose.pose.orientation.w = math.cos(facing / 2.0)
        self.goal_seq += 1
        seq = self.goal_seq
        self.goal = (float(xy[0]), float(xy[1]))
        self.target = self.goal if target is None else (float(target[0]), float(target[1]))
        self.hop = target is not None
        self.goal_kind = kind
        self.goal_info = info
        self.goal_facing = facing
        self.goal_mode = mode
        self.goal_active = True
        self.goal_handle = None
        if fresh or self.progress_time is None:
            self.progress_time = self.now_s()
            self.progress_pose = None        # (set at the next check_stuck)
        if changed:
            # let the behaviour tree take the new planner and controller
            # first: step() sends the goal a moment later
            self.resend_at = self.now_s() + SELECT_WAIT
            return
        future = self.nav.send_goal_async(goal)
        future.add_done_callback(lambda fut: self.on_goal_response(fut, seq))

    def on_goal_response(self, future, seq):
        handle = future.result()
        if seq != self.goal_seq:
            return
        if handle is None or not handle.accepted:
            self.goal_active = False
            self.on_goal_done(seq, GoalStatus.STATUS_ABORTED)
            return
        self.goal_handle = handle
        handle.get_result_async().add_done_callback(lambda fut: self.on_result(seq, fut.result()))

    def on_result(self, seq, res):
        if res is None:
            self.on_goal_done(seq, GoalStatus.STATUS_ABORTED)
        else:
            # (NavigateToPose results have had an error code since Nav2 Iron)
            self.on_goal_done(seq, res.status, getattr(res.result, 'error_code', 0))

    def on_goal_done(self, seq, status, error=0):
        if seq != self.goal_seq or self.state == DONE:
            return              # an older goal, replaced by a newer one
        self.goal_active = False
        ok = status == GoalStatus.STATUS_SUCCEEDED
        kind = self.goal_kind
        if not ok and error in TRANSIENT_ERRORS:
            t = self.now_s()
            if self.transient_since is None or t - self.transient_last > 5 * TRANSIENT_WAIT:
                # (a goal sent again that then ran for a while starts a new row)
                self.transient_since = t
                self.log('Nav2 lost the map -> odom transform for a moment (error %d); '
                         'sending the goal again' % error, warn=True)
            self.transient_last = t
            if t - self.transient_since < TRANSIENT_TIME:
                # still "active": step() sends it again after TRANSIENT_WAIT
                self.goal_active = True
                self.resend_at = t + TRANSIENT_WAIT
                return
            self.log('Nav2 has had no map -> odom transform for %.0f s' % TRANSIENT_TIME,
                     warn=True)
        self.transient_since = None
        if not ok and error == START_OCCUPIED and self.occupied < 2:
            self.occupied += 1
            self.log('Nav2 finds the robot inside a wall on its map (error %d): backing up, '
                     'then sending the goal again' % error, warn=True)
            self.goal_active = True
            self.resend_after = True
            self.recover(self.state)
            return
        self.occupied = 0
        if self.hop and ok:
            # a step on the way (full mode): the next one is sent from step()
            self.home_failures = 0
            return
        if kind == 'frontier':
            stats = self.leg_stats[-1]
            pose = self.robot_pose()
            short = math.hypot(pose[0] - self.goal[0], pose[1] - self.goal[1]) if pose else 0.0
            if ok and short > ARRIVED_RADIUS and self.short < 2:
                # Nav2 can report a goal reached from further away (once in
                # testing, 1.2 m short, as RTAB-Map corrected the map): choose
                # again, and the frontier is still there to choose
                self.short += 1
                self.log('Nav2 reported %s reached, but the robot is %.1f m short of it'
                         % (self.goal_info, short), warn=True)
            elif ok:
                # A frontier that has not moved although the robot got there
                # cannot be mapped from where the robot can go: skip it. (One
                # that has moved on, e.g. round a bend, is followed.) Wait for
                # the map from here first (SETTLE).
                self.reached.append(self.target)
                stats['goals'] += 1
                self.visited.append(self.target)
                self.failures = 0
                self.short = 0
                self.arrived = self.now_s()
            else:
                self.blacklist.append(self.target)
                stats['failed'] += 1
                self.failures += 1
                self.log('Could not reach ' + self.goal_info, warn=True)
                if self.failures % 3 == 0:
                    self.recover(EXPLORING)
            if self.full:
                self.target, self.hop = None, False      # choose again
        elif kind == 'turn spot':
            if ok:
                pose = self.robot_pose()
                snap = self.snapshot(pose) if pose is not None else None
                self.turn_to_route(snap, pose)
            else:
                pending, self.pending = self.pending, None
                self.pending_failed(pending)
        elif kind == 'leg end':
            if not ok:
                self.log('Could not reach the leg end; returning from here', warn=True)
            self.go_home()
        elif kind == 'home':
            if ok:
                self.log('Back at the start')
                self.leg_stats[-1]['end'] = self.now_s()
                if self.full:
                    self.finish('completed')
                else:
                    self.next_leg()
            else:
                self.home_failures += 1
                if self.home_failures >= 3:
                    self.finish('stopped: could not get back to the start')
                else:
                    self.log('Could not get back to the start, trying again', warn=True)
                    self.recover(RETURNING)

    def cancel_goal(self):
        self.goal_seq += 1
        self.goal_active = False
        self.resend_at = None
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None

    def check_stuck(self, pose, t):
        """
        Check whether the robot has made no headway towards its goal for stuck_timeout s.

        Headway is PROGRESS m towards the goal from where the robot last made
        some, so driving back and forth does not count: in a test run the
        robot rocked 3 m back and forth for 10 minutes in a pipe its map
        showed closed. The goal may move on meanwhile (a frontier as more of
        the pipe is mapped, or the next step of a long drive): what counts is
        the direction to the goal now.
        """
        if self.progress_time is None:
            self.progress_time = t
        if self.progress_pose is None:
            self.progress_pose = pose
            return False
        x0, y0 = self.progress_pose[0], self.progress_pose[1]
        gx, gy = self.goal[0] - x0, self.goal[1] - y0
        norm = math.hypot(gx, gy)
        headway = ((pose[0] - x0) * gx + (pose[1] - y0) * gy) / norm if norm > 1e-6 else 0.0
        if headway >= PROGRESS:
            self.progress_pose, self.progress_time = pose, t
            return False
        if t - self.progress_time < self.stuck_timeout:
            return False
        self.log('Stuck for %.0f s on the way to %s' % (self.stuck_timeout, self.goal_info),
                 warn=True)
        kind, target = self.goal_kind, self.target
        self.cancel_goal()
        if kind == 'frontier':
            self.blacklist.append(target)
            self.leg_stats[-1]['failed'] += 1
            if self.full:
                self.target, self.hop = None, False      # choose again
            self.recover(EXPLORING)
        elif kind == 'leg end':
            self.go_home()
        elif kind == 'turn spot':
            pending, self.pending = self.pending, None
            self.pending_failed(pending)
            if self.state == EXPLORING:
                self.recover(EXPLORING)
        else:
            self.home_failures += 1
            if self.home_failures >= 3:
                self.finish('stopped: could not get back to the start')
            else:
                self.recover(RETURNING)
        return True

    def recover(self, then):
        """Clear the costmaps and back up a little, then carry on with `then`."""
        self.log('Recovering: clearing the costmaps and backing up 0.25 m')
        for client in self.clear_clients:
            if client.service_is_ready():
                client.call_async(ClearEntireCostmap.Request())
        self.state = RECOVERING
        self.after_recovery = then
        if not self.backup_ac.server_is_ready():
            self.recovered()
            return
        goal = BackUp.Goal()
        goal.target = Point(x=0.25)
        goal.speed = 0.1
        goal.time_allowance = Duration(seconds=10.0).to_msg()
        future = self.backup_ac.send_goal_async(goal)

        def accepted(fut):
            handle = fut.result()
            if handle is None or not handle.accepted:
                self.recovered()
            else:
                handle.get_result_async().add_done_callback(lambda _: self.recovered())
        future.add_done_callback(accepted)

    def recovered(self):
        if self.state != RECOVERING:
            return
        self.state = self.after_recovery
        # a new try: stuck_timeout again from here
        self.progress_time, self.progress_pose = self.now_s(), None
        if self.resend_after:
            self.resend_after = False
            self.resend_at = self.now_s()      # step() sends the goal again
        elif self.state == RETURNING and not self.full:   # (full mode: return_full goes on)
            self.send_goal(self.start[:2], 'home', 'start ' + fmt_xy(self.start),
                           facing=self.start[2], mode='reverse')

    def send_spin(self, angle):
        if not self.spin_ac.server_is_ready():
            self.spun(GoalStatus.STATUS_ABORTED)
            return
        goal = Spin.Goal()
        goal.target_yaw = float(angle)
        goal.time_allowance = Duration(seconds=60.0).to_msg()
        future = self.spin_ac.send_goal_async(goal)

        def accepted(fut):
            handle = fut.result()
            if handle is None or not handle.accepted:
                self.spun(GoalStatus.STATUS_ABORTED)
            else:
                handle.get_result_async().add_done_callback(
                    lambda f: self.spun(f.result().status if f.result()
                                        else GoalStatus.STATUS_ABORTED))
        future.add_done_callback(accepted)

    def spun(self, status):
        if self.state != TURNING:
            return
        why, self.turn_for = self.turn_for, None
        if why == 'approach':
            if status != GoalStatus.STATUS_SUCCEEDED:
                self.log('Could not turn fully; going on anyway', warn=True)
            self.resume()
            return
        if status != GoalStatus.STATUS_SUCCEEDED:
            self.log('Could not turn round fully; exploring %s anyway'
                     % ('the whole map' if self.full else 'the next leg'), warn=True)
        self.state = EXPLORING
        self.begin_leg()

    # ------------------------------------------------------------ RViz markers

    def publish_markers(self):
        stamp = Time().to_msg()      # 0 = draw with the latest transforms (see now_s)
        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        def marker(ns, mid, mtype, color, scale):
            m = Marker()
            m.header.frame_id = self.global_frame
            m.header.stamp = stamp
            m.ns, m.id, m.type, m.action = ns, mid, mtype, Marker.ADD
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = scale
            m.color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=color[3])
            return m

        snap = self.last_snapshot
        if snap is not None and self.state == EXPLORING:
            for k, (in_leg, color) in enumerate(((True, (0.1, 0.9, 0.2, 0.9)),
                                                 (False, (0.6, 0.6, 0.6, 0.6)))):
                m = marker('frontiers', k, Marker.CUBE_LIST, color, snap.g.res)
                for f, inside in self.last_frontiers:
                    if inside == in_leg:
                        for r, c in f.cells:
                            x, y = snap.g.xy(r, c)
                            m.points.append(Point(x=x, y=y, z=0.05))
                if m.points:
                    markers.markers.append(m)
            for i, (f, inside) in enumerate(self.last_frontiers):
                if inside:
                    m = marker('scores', i, Marker.TEXT_VIEW_FACING, (1.0, 1.0, 1.0, 0.9), 0.25)
                    m.pose.position.x, m.pose.position.y = f.goal
                    m.pose.position.z = 0.6
                    m.text = 'F %.1f' % f.score
                    markers.markers.append(m)
        if self.goal_active and self.goal is not None:
            m = marker('goal', 0, Marker.SPHERE, (1.0, 0.85, 0.0, 0.9), 0.3)
            m.pose.position.x, m.pose.position.y = self.target if self.hop else self.goal
            m.pose.position.z = 0.15
            markers.markers.append(m)
            if self.hop:        # the next step on the way to it (full mode)
                m = marker('goal', 1, Marker.SPHERE, (1.0, 0.5, 0.0, 0.9), 0.2)
                m.pose.position.x, m.pose.position.y = self.goal
                m.pose.position.z = 0.15
                markers.markers.append(m)
        if self.start is not None and not self.full:
            for i, a in enumerate(self.legs):
                m = marker('legs', i, Marker.ARROW, (0.2, 0.5, 1.0, 0.8), 0.1)
                b = wrap(self.start[2] + a)
                m.points = [Point(x=self.start[0], y=self.start[1], z=0.1),
                            Point(x=self.start[0] + 1.5 * math.cos(b),
                                  y=self.start[1] + 1.5 * math.sin(b), z=0.1)]
                m.scale.y, m.scale.z = 0.2, 0.2
                markers.markers.append(m)
        graph = self.graph
        if graph is not None and graph.nodes:
            m = marker('manholes', 0, Marker.CUBE_LIST, (0.2, 0.4, 1.0, 0.6), 0.5)
            m.scale.z = 0.05
            m.points = [Point(x=x, y=y, z=0.03) for x, y in graph.nodes]
            markers.markers.append(m)
            m = marker('pipes', 0, Marker.LINE_LIST, (0.6, 0.3, 0.9, 0.8), 0.06)
            for i, j, _ in graph.edges:
                m.points += [Point(x=graph.nodes[i][0], y=graph.nodes[i][1], z=0.3),
                             Point(x=graph.nodes[j][0], y=graph.nodes[j][1], z=0.3)]
            for i, _, _, end in graph.ends:
                m.points += [Point(x=graph.nodes[i][0], y=graph.nodes[i][1], z=0.3),
                             Point(x=end[0], y=end[1], z=0.3)]
            if m.points:
                markers.markers.append(m)
        if self.full and self.state == EXPLORING and self.chooser.order and snap is not None:
            m = marker('tour', 0, Marker.LINE_STRIP, (1.0, 0.2, 0.8, 0.8), 0.04)
            m.points = [Point(x=snap.robot_xy[0], y=snap.robot_xy[1], z=0.25)] + [
                Point(x=f.goal[0], y=f.goal[1], z=0.25) for f in self.chooser.order]
            markers.markers.append(m)
        if len(self.trajectory) > 1:
            m = marker('path', 0, Marker.LINE_STRIP, (1.0, 0.45, 0.0, 0.9), 0.05)
            m.points = [Point(x=p[1], y=p[2], z=0.05) for p in self.trajectory]
            markers.markers.append(m)
        self.markers_pub.publish(markers)

    # ------------------------------------------------------------ report

    def finish(self, result):
        if self.state == DONE:
            return
        self.result = result
        if self.leg_stats and self.leg_stats[-1]['end'] is None:
            self.leg_stats[-1]['end'] = self.now_s()
        self.log('Mission %s' % result)
        self.state = DONE
        self.goal_active = False
        try:
            folder = self.write_report()
            self.log('Report written to %s' % folder)
            status = 'done (%s): report in %s' % (result, folder)
        except OSError as e:
            self.log('Could not write the report: %s' % e, warn=True)
            status = 'done (%s)' % result
        if rclpy.ok():      # (shut down by Ctrl+C: nothing can be published any more)
            self.set_status(status)
            self.publish_markers()

    def write_report(self):
        folder = os.path.join(self.report_dir,
                              datetime.datetime.now().strftime('%Y%m%d_%H%M%S'))
        os.makedirs(folder, exist_ok=True)
        m = self.map_msg
        grid = None
        self.graph = None
        if self.last_snapshot is not None:
            # the network as a graph, from the last map (frontiers from the same map)
            snap = self.last_snapshot
            self.graph = PipeGraph(snap, snap.frontiers(self.min_size))
        if m is not None:
            grid = np.asarray(m.data, np.int16).reshape(m.info.height, m.info.width)
            res, ox, oy = m.info.resolution, m.info.origin.position.x, m.info.origin.position.y
            write_pgm(os.path.join(folder, 'map.pgm'), grid)
            with open(os.path.join(folder, 'map.yaml'), 'w') as f:
                f.write('image: map.pgm\nmode: trinary\nresolution: %.4f\n'
                        'origin: [%.4f, %.4f, 0.0]\nnegate: 0\noccupied_thresh: 0.65\n'
                        'free_thresh: 0.25\n' % (res, ox, oy))
            write_png(os.path.join(folder, 'map.png'),
                      self.map_image(grid, res, ox, oy))
        with open(os.path.join(folder, 'trajectory.csv'), 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['time_s', 'x_m', 'y_m', 'yaw_deg', 'state', 'leg'])
            for t, x, y, yaw, state, leg in self.trajectory:
                w.writerow(['%.2f' % (t - self.mission_start_time), '%.3f' % x, '%.3f' % y,
                            '%.1f' % math.degrees(yaw), state, leg])
        with open(os.path.join(folder, 'report.md'), 'w') as f:
            f.write(self.report_text(grid))
        return folder

    def report_text(self, grid):
        t0 = self.mission_start_time or self.now_s()
        if self.last_odom is not None:
            driven = self.distance
        else:       # (no odometry: on the map, jumps and all)
            driven = sum(math.hypot(b[1] - a[1], b[2] - a[2])
                         for a, b in zip(self.trajectory, self.trajectory[1:]))
        lines = ['# AGOS mission report', '',
                 '- Date: %s' % datetime.datetime.now().strftime('%Y-%m-%d %H:%M'),
                 '- Result: %s' % self.result,
                 '- Mission time: %.0f s (simulation time)' % (self.now_s() - t0),
                 '- Start: %s m in the map, facing %d deg'
                 % (fmt_xy(self.start), heading(self.start[2]))
                 if self.start else '- Start: not set',
                 '- Distance driven: %.1f m' % driven,
                 '- Driving: ' + ('forwards into new pipe (Hybrid-A* and an MPPI that never '
                                  'reverses); reversing only over mapped pipe'
                                  if self.forward_only else
                                  'either way (forward_only:=false, as before)'),
                 '- Reversed: %.1f m in all; of that %.1f m back to a manhole to turn round '
                 'and %.1f m on the way to a frontier'
                 % (self.reverse_total, self.reverse_spot, self.reverse_frontier),
                 '- Turned on the spot to face a pipe before driving into it: %d times'
                 % self.turns]
        if grid is not None:
            res = self.map_msg.info.resolution
            lines.append('- Mapped: %.1f m^2 of free space, %.1f m^2 of walls'
                         % (np.count_nonzero((grid >= 0) & (grid < 50)) * res * res,
                            np.count_nonzero(grid >= 50) * res * res))
        if self.full:
            return '\n'.join(lines + self.full_report_lines() + self.network_lines())
        lines += ['- Leg length limit: %s along the pipe'
                  % (('%.0f m' % self.leg_length) if self.leg_length > 0 else 'none'), '',
                  '| Leg | Heading | Explored along the pipe | Frontier goals sent '
                  '| Reached | Failed | Time | Note |',
                  '|---|---|---|---|---|---|---|---|']
        for s in self.leg_stats:
            lines.append('| %d | %d deg | %.1f m | %d | %d | %d | %.0f s | %s |' % (
                s['leg'], heading(s['bearing']), s['explored'], s['sent'], s['goals'],
                s['failed'], (s['end'] or self.now_s()) - s['start'], s['note']))
        lines += ['', 'Frontier selection: TAD (Buriboev, Choi and Jeon, 2025), '
                  'F = d + a + t; see agos_robot_nav/frontiers.py. A goal that was sent but '
                  'neither reached nor failed was replaced by a better frontier as the '
                  'map grew.', '',
                  '![Map with the path driven](map.png)', '',
                  'Map: white = free, black = wall, grey = not mapped. Orange line = path '
                  'driven, green dot = start, yellow dots = frontier goals reached, '
                  'blue dots = leg ends, purple dots = manholes.', '', '## Events', '', '```']
        lines += self.events
        lines += ['```', '']
        return '\n'.join(lines + self.network_lines())

    def network_lines(self):
        """Describe the network as a graph: manholes and the pipes between them (PipeGraph)."""
        g = self.graph
        if g is None or not g.nodes:
            return []
        names = ['M%d' % (i + 1) for i in range(len(g.nodes))]
        dead = [e for e in g.ends if not e[2]]
        lines = ['## Pipe network', '',
                 'The map as a graph (PipeGraph in agos_robot_nav/frontiers.py): %d manholes '
                 '(places with room to turn round, purple on the map, M1 nearest the end of '
                 'the mission), %d pipes between them, %d dead ends and %d pipes not explored '
                 'to the end; %.1f m of pipe in all, measured from the middle of the manholes.'
                 % (len(g.nodes), len(g.edges), len(dead), len(g.ends) - len(dead), g.length()),
                 '', '| Manhole | Position (m) |', '|---|---|']
        lines += ['| %s | %s |' % (n, fmt_xy(p)) for n, p in zip(names, g.nodes)]
        lines += ['', '| Pipe | Length | Ends at |', '|---|---|---|']
        lines += ['| %s to %s | %.1f m | %s |' % (names[i], names[j], d, names[j])
                  for i, j, d in g.edges]
        lines += ['| %s, %s | %.1f m | %s |'
                  % (names[i], 'not explored to the end' if is_open else 'dead end', d,
                     fmt_xy(end)) for i, d, is_open, end in g.ends]
        if g.loose:
            lines.append('| (reaches no manhole) | | %s |'
                         % ', '.join(fmt_xy(p) for p in g.loose))
        return lines + ['']

    def full_report_lines(self):
        """Write the rest of the report in full mode, after the lines both modes share."""
        s = self.leg_stats[-1] if self.leg_stats else {'sent': 0, 'goals': 0, 'failed': 0}
        left = self.leftover()
        lines = ['- Mode: the whole map, then back to the start',
                 '- Frontier goals: %d sent, %d reached, %d could not be reached'
                 % (s['sent'], s['goals'], s['failed'])]
        if not left:
            lines.append('- Frontiers left at the end: none')
        else:
            kinds = {}
            for xy, why in left:
                kinds.setdefault(why, []).append(xy)
            lines.append('- Frontiers left at the end: %d' % len(left))
            for why, points in kinds.items():
                lines.append('    - %d %s: %s' % (len(points), why,
                                                  ', '.join(fmt_xy(p) for p in points[:8])
                                                  + (' ...' if len(points) > 8 else '')))
        lines += ['', 'Frontier selection: TAD (Buriboev, Choi and Jeon, 2025), F = d + a + t, '
                  'among the frontiers %s within %.0f m of the best one, counting the turns '
                  'needed to drive into each forwards; the robot keeps to the pipe it is in, '
                  'and goes back for the other pipes of a manhole once it is %.0f m past it '
                  '(WholeMap and plan_tour in agos_robot_nav/frontiers.py). A goal that was '
                  'sent but neither reached nor failed was replaced as the map grew.'
                  % ('that start a tour through all of them and back to the start'
                     if self.tour else 'nearest the robot', self.reach, self.peek), '',
                  '![Map with the path driven](map.png)', '',
                  'Map: white = free, black = wall, grey = not mapped. Orange line = path '
                  "driven (RTAB-Map's, which loop closures move with the map; "
                  'trajectory.csv has where the robot was on the map at the time), '
                  'green dot = start, yellow dots = frontier goals reached, '
                  'red dots = frontiers left (other than gaps in walls and the ones next '
                  'to the path driven), purple dots = manholes.', '',
                  '## Events', '', '```']
        lines += self.events
        lines += ['```', '']
        return lines

    def map_image(self, grid, res, ox, oy):
        h, w = grid.shape
        img = np.full((h, w, 3), 205, np.uint8)
        img[(grid >= 0) & (grid < 50)] = 255
        img[grid >= 50] = 0
        scale = max(1, min(4, int(800 // max(w, h, 1))))
        img = np.repeat(np.repeat(img, scale, axis=0), scale, axis=1)

        def px(x, y):
            return ((y - oy) / res * scale, (x - ox) / res * scale)   # (row, col), row from y=oy

        def dot(x, y, color, radius):
            r0, c0 = px(x, y)
            k = int(radius * scale) + 1
            a, b = max(0, int(r0) - k), max(0, int(c0) - k)
            win = img[a:int(r0) + k + 1, b:int(c0) + k + 1]
            rr, cc = np.ogrid[a:a + win.shape[0], b:b + win.shape[1]]
            win[(rr - r0) ** 2 + (cc - c0) ** 2 <= (radius * scale) ** 2] = color

        if self.full and len(self.map_path) > 1:
            # RTAB-Map's path fits this map: loop closures move it with the
            # map. The trajectory has where the robot was on the map at the
            # time, up to 4 m off this map after a whole-network run.
            pts = list(self.map_path)
        else:
            pts = [(p[1], p[2]) for p in self.trajectory]
        for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
            n = int(max(abs(x1 - x0), abs(y1 - y0)) / res * scale * 2) + 1
            for k in range(n + 1):
                r, c = px(x0 + (x1 - x0) * k / n, y0 + (y1 - y0) * k / n)
                r, c = int(r), int(c)
                # (off the map: a negative index would paint from the far side)
                if 0 <= r < img.shape[0] and 0 <= c < img.shape[1]:
                    img[max(0, r - 1):r + 2, max(0, c - 1):c + 2] = (255, 120, 0)
        for x, y in self.visited:
            dot(x, y, (230, 190, 0), 2)
        for s in self.leg_stats:
            if s['end_point']:
                dot(s['end_point'][0], s['end_point'][1], (40, 90, 255), 3)
        if self.full:
            for (x, y), why in self.leftover():
                if not why.startswith('gap') and why != NEAR_TRAIL_WHY:
                    dot(x, y, (220, 30, 30), 3)
        if self.graph is not None:
            for x, y in self.graph.nodes:
                dot(x, y, (140, 60, 210), 5)
        if self.start:
            dot(self.start[0], self.start[1], (0, 170, 0), 4)
        return img[::-1]    # image rows go down, map rows go up


def write_pgm(path, grid):
    img = np.full(grid.shape, 205, np.uint8)
    img[(grid >= 0) & (grid < 50)] = 254
    img[grid >= 50] = 0
    img = img[::-1]
    with open(path, 'wb') as f:
        f.write(b'P5\n%d %d\n255\n' % (img.shape[1], img.shape[0]))
        f.write(img.tobytes())


def write_png(path, rgb):
    """Minimal PNG writer (8-bit RGB), so no imaging library is needed."""
    h, w, _ = rgb.shape
    raw = b''.join(b'\x00' + np.ascontiguousarray(rgb[r]).tobytes() for r in range(h))

    def chunk(tag, data):
        return (struct.pack('>I', len(data)) + tag + data
                + struct.pack('>I', zlib.crc32(tag + data) & 0xffffffff))
    with open(path, 'wb') as f:
        f.write(b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', struct.pack('>IIBBBBB', w, h, 8, 2, 0, 0, 0))
                + chunk(b'IDAT', zlib.compress(raw, 6)) + chunk(b'IEND', b''))


def main(args=None):
    rclpy.init(args=args)
    node = Explorer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node.state not in (WAITING, DONE):
            if rclpy.ok():
                node.cancel_goal()
            node.finish('stopped (shut down)')
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
