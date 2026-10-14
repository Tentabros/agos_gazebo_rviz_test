"""
Autonomous exploration for AGOS: the proposal's mission, with TAD frontiers.

The mission (the proposal's "AGOS Flow of Operation" figure), from the manhole the
robot starts in:

  1. explore up to leg_length (25 m) along the pipe the robot is facing,
  2. come back to the start (reversing: the robot cannot turn in a pipe),
  3. turn round in the manhole,
  4. explore up to leg_length along the pipe on the other side,
  5. come back to the start,
  6. write a report to <report_dir>/<date_time>/ (map, path, summary):
     ~/agos_missions, or the missions/ folder when started with ./agos.

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

    ros2 launch agos_sim agos.launch.py explore:=true      # everything
    ros2 run agos_sim explorer                             # on its own
    ros2 service call /explore/start std_srvs/srv/Trigger  # when autostart:=false
    ros2 service call /explore/stop std_srvs/srv/Trigger   # stop and write the report

RViz: the Explorer display shows the frontiers (green: in this leg, grey:
not), the TAD score of each, the current goal (yellow) and the path driven.
/explore/status says what it is doing.
"""
import csv
import datetime
import math
import os
import struct
import time
import zlib

from action_msgs.msg import GoalStatus
from agos_sim.frontiers import Grid, Snapshot
from geometry_msgs.msg import Point
from nav2_msgs.action import BackUp, NavigateToPose, Spin
from nav2_msgs.srv import ClearEntireCostmap
from nav_msgs.msg import OccupancyGrid
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

WAITING, EXPLORING, TO_LEG_END, RETURNING, TURNING, RECOVERING, DONE = (
    'waiting', 'exploring', 'to leg end', 'returning', 'turning round', 'recovering', 'done')


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
        p('stuck_timeout', 40.0)         # s without moving 0.3 m = stuck
        p('turn_around', True)           # turn round in the start manhole between legs
        p('autostart', True)             # start as soon as Nav2 and the map are up
        p('report_dir', os.environ.get('AGOS_MISSION_DIR', '~/agos_missions'))
        p('global_frame', 'map')
        p('robot_frame', 'base_footprint')
        prm = self.get_parameter
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
        self.autostart = bool(prm('autostart').value)
        self.report_dir = os.path.expanduser(prm('report_dir').value)
        self.global_frame = prm('global_frame').value
        self.robot_frame = prm('robot_frame').value

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, latched)
        self.markers_pub = self.create_publisher(MarkerArray, '/explore/markers', latched)
        self.status_pub = self.create_publisher(String, '/explore/status', latched)
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
        self.goal = None               # (x, y)
        self.goal_kind = None          # 'frontier', 'leg end', 'home'
        self.goal_seq = 0
        self.goal_active = False
        self.goal_handle = None
        self.goal_info = ''
        self.blacklist = []
        self.reached = []
        self.visited = []
        self.failures = 0
        self.home_failures = 0
        self.trajectory = []           # (t, x, y, yaw, state, leg)
        self.events = []
        self.leg_stats = []
        self.progress_pose = None
        self.progress_time = None
        self.after_recovery = None
        self.mission_start_time = None
        self.result = ''
        self.last_frontiers = []
        self.last_snapshot = None

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
        # (two separate calls: an rclpy logger call site may not change severity)
        if warn:
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

    def robot_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(self.global_frame, self.robot_frame, Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return None
        self.pose_time = Time.from_msg(t.header.stamp).nanoseconds * 1e-9
        tr = t.transform
        return tr.translation.x, tr.translation.y, yaw_of(tr.rotation)

    def leg_bearing(self):
        return wrap(self.start[2] + self.legs[self.leg])

    def leg_name(self):
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
        if self.goal_active and self.check_stuck(pose, t):
            return

        if self.state == EXPLORING:
            self.explore(pose)
        # TO_LEG_END, RETURNING, TURNING and RECOVERING advance in the action callbacks
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
        self.mission_start_time = self.now_s()
        self.leg = 0
        self.state = EXPLORING
        self.log('Mission started at %s, facing %d deg: %d legs of %s'
                 % (fmt_xy(pose), heading(pose[2]), len(self.legs),
                    ('%.0f m' % self.leg_length) if self.leg_length > 0 else 'any length'))
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
        self.log('Starting %s: heading %d deg' % (self.leg_name(), heading(self.leg_bearing())))

    def snapshot(self, pose):
        m = self.map_msg
        grid = Grid(m.data, m.info.width, m.info.height, m.info.resolution,
                    m.info.origin.position.x, m.info.origin.position.y)
        snap = Snapshot(grid, (pose[0], pose[1]), (self.start[0], self.start[1]),
                        clearance=self.clearance, exit_radius=self.exit_radius)
        self.last_snapshot = snap
        return snap

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
            self.send_frontier(max(in_leg, key=lambda f: f.score), pose)
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
            self.send_goal(end[0], 'leg end', 'leg end (%.2f, %.2f)' % end[0])
        else:
            if end is None:
                stats['note'] = 'no pipe in this direction'
                self.log('No pipe found in the direction of %s' % self.leg_name(), warn=True)
            self.go_home()

    @staticmethod
    def near(xy, points, radius):
        return any(math.hypot(xy[0] - x, xy[1] - y) < radius for x, y in points)

    def passed(self, pose):
        """
        Where the robot explored in this leg, leaving out the last passed_margin.

        Seen from far off, RTAB-Map's map of a pipe the robot has driven
        through gets small gaps in its walls again (LiDAR rays that graze a
        wall clear some wall cells), and the free cells next to those gaps
        look like frontiers. They are behind the robot and nothing new is
        there, so they are skipped rather than driven back to.
        """
        leg = self.leg + 1
        points = []
        for _, x, y, _, state, n in self.trajectory:
            if n == leg and state == EXPLORING and \
                    math.hypot(x - pose[0], y - pose[1]) > self.passed_margin and \
                    (not points or math.hypot(x - points[-1][0], y - points[-1][1]) > 0.25):
                points.append((x, y))
        return points

    def send_frontier(self, f, pose):
        info = ('frontier (%.2f, %.2f), F = %.1f = d %.1f + a %d + t %+d'
                % (f.goal[0], f.goal[1], f.score, f.d_adj, f.a, f.t))
        if self.goal_active and self.goal is not None and \
                math.hypot(f.goal[0] - self.goal[0], f.goal[1] - self.goal[1]) < 0.3:
            return
        self.log('Goal: ' + info)
        self.leg_stats[-1]['sent'] += 1
        facing = math.atan2(f.goal[1] - pose[1], f.goal[0] - pose[0])
        self.send_goal(f.goal, 'frontier', info, facing=facing)

    def go_home(self):
        stats = self.leg_stats[-1]
        if stats['end'] is None:
            stats['end'] = self.now_s()
        self.state = RETURNING
        self.home_failures = 0
        self.log('Returning to the start ' + fmt_xy(self.start))
        self.send_goal(self.start[:2], 'home', 'start ' + fmt_xy(self.start), facing=self.start[2])

    def next_leg(self):
        if self.leg + 1 >= len(self.legs):
            self.finish('completed')
            return
        pose = self.robot_pose()
        self.leg += 1
        turn = wrap(self.leg_bearing() - (pose[2] if pose else self.start[2]))
        if self.turn_around and abs(turn) > math.radians(20):
            self.state = TURNING
            self.log('Turning %.0f deg in the manhole' % math.degrees(turn))
            self.send_spin(turn)
        else:
            self.state = EXPLORING
            self.begin_leg()

    # ------------------------------------------------------------ Nav2 actions

    def send_goal(self, xy, kind, info, facing=0.0):
        if not self.nav.server_is_ready():
            self.log('Nav2 navigate_to_pose is not available', warn=True)
            return
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
        self.goal_kind = kind
        self.goal_info = info
        self.goal_active = True
        self.goal_handle = None
        pose = self.robot_pose()
        self.progress_pose = pose
        self.progress_time = self.now_s()
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
        handle.get_result_async().add_done_callback(
            lambda fut: self.on_goal_done(seq, fut.result().status if fut.result() else
                                          GoalStatus.STATUS_ABORTED))

    def on_goal_done(self, seq, status):
        if seq != self.goal_seq or self.state == DONE:
            return              # an older goal, replaced by a newer one
        self.goal_active = False
        ok = status == GoalStatus.STATUS_SUCCEEDED
        kind = self.goal_kind
        if kind == 'frontier':
            stats = self.leg_stats[-1]
            if ok:
                # A frontier that has not moved although the robot got there
                # cannot be mapped from where the robot can go: skip it. (One
                # that has moved on, e.g. round a bend, is followed.)
                self.reached.append(self.goal)
                stats['goals'] += 1
                self.visited.append(self.goal)
                self.failures = 0
            else:
                self.blacklist.append(self.goal)
                stats['failed'] += 1
                self.failures += 1
                self.log('Could not reach ' + self.goal_info, warn=True)
                if self.failures % 3 == 0:
                    self.recover(EXPLORING)
        elif kind == 'leg end':
            if not ok:
                self.log('Could not reach the leg end; returning from here', warn=True)
            self.go_home()
        elif kind == 'home':
            if ok:
                self.log('Back at the start')
                self.leg_stats[-1]['end'] = self.now_s()
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
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None

    def check_stuck(self, pose, t):
        if self.progress_pose is None:
            self.progress_pose, self.progress_time = pose, t
            return False
        if math.hypot(pose[0] - self.progress_pose[0], pose[1] - self.progress_pose[1]) > 0.3:
            self.progress_pose, self.progress_time = pose, t
            return False
        if t - self.progress_time < self.stuck_timeout:
            return False
        self.log('Stuck for %.0f s on the way to %s' % (self.stuck_timeout, self.goal_info),
                 warn=True)
        kind, goal = self.goal_kind, self.goal
        self.cancel_goal()
        if kind == 'frontier':
            self.blacklist.append(goal)
            self.leg_stats[-1]['failed'] += 1
            self.recover(EXPLORING)
        elif kind == 'leg end':
            self.go_home()
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
        if self.state == RETURNING:
            self.send_goal(self.start[:2], 'home', 'start ' + fmt_xy(self.start),
                           facing=self.start[2])

    def send_spin(self, angle):
        if not self.spin_ac.server_is_ready():
            self.state = EXPLORING
            self.begin_leg()
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
        if status != GoalStatus.STATUS_SUCCEEDED:
            self.log('Could not turn round fully; exploring the next leg anyway', warn=True)
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
            m.pose.position.x, m.pose.position.y = self.goal
            m.pose.position.z = 0.15
            markers.markers.append(m)
        if self.start is not None:
            for i, a in enumerate(self.legs):
                m = marker('legs', i, Marker.ARROW, (0.2, 0.5, 1.0, 0.8), 0.1)
                b = wrap(self.start[2] + a)
                m.points = [Point(x=self.start[0], y=self.start[1], z=0.1),
                            Point(x=self.start[0] + 1.5 * math.cos(b),
                                  y=self.start[1] + 1.5 * math.sin(b), z=0.1)]
                m.scale.y, m.scale.z = 0.2, 0.2
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
            self.set_status('done (%s): report in %s' % (result, folder))
        except OSError as e:
            self.log('Could not write the report: %s' % e, warn=True)
            self.set_status('done (%s)' % result)
        self.publish_markers()

    def write_report(self):
        folder = os.path.join(self.report_dir,
                              datetime.datetime.now().strftime('%Y%m%d_%H%M%S'))
        os.makedirs(folder, exist_ok=True)
        m = self.map_msg
        grid = None
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
        driven = sum(math.hypot(b[1] - a[1], b[2] - a[2])
                     for a, b in zip(self.trajectory, self.trajectory[1:]))
        lines = ['# AGOS mission report', '',
                 '- Date: %s' % datetime.datetime.now().strftime('%Y-%m-%d %H:%M'),
                 '- Result: %s' % self.result,
                 '- Mission time: %.0f s (simulation time)' % (self.now_s() - t0),
                 '- Start: %s m in the map, facing %d deg'
                 % (fmt_xy(self.start), heading(self.start[2]))
                 if self.start else '- Start: not set',
                 '- Distance driven: %.1f m' % driven]
        if grid is not None:
            res = self.map_msg.info.resolution
            lines.append('- Mapped: %.1f m^2 of free space, %.1f m^2 of walls'
                         % (np.count_nonzero((grid >= 0) & (grid < 50)) * res * res,
                            np.count_nonzero(grid >= 50) * res * res))
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
                  'F = d + a + t; see agos_sim/frontiers.py. A goal that was sent but '
                  'neither reached nor failed was replaced by a better frontier as the '
                  'map grew.', '',
                  '![Map with the path driven](map.png)', '',
                  'Map: white = free, black = wall, grey = not mapped. Orange line = path '
                  'driven, green dot = start, yellow dots = frontier goals reached, '
                  'blue dots = leg ends.', '', '## Events', '', '```']
        lines += self.events
        lines += ['```', '']
        return '\n'.join(lines)

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
            rr, cc = np.ogrid[:img.shape[0], :img.shape[1]]
            img[(rr - r0) ** 2 + (cc - c0) ** 2 <= (radius * scale) ** 2] = color

        pts = [(p[1], p[2]) for p in self.trajectory]
        for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
            n = int(max(abs(x1 - x0), abs(y1 - y0)) / res * scale * 2) + 1
            for k in range(n + 1):
                r, c = px(x0 + (x1 - x0) * k / n, y0 + (y1 - y0) * k / n)
                r, c = int(r), int(c)
                img[max(0, r - 1):r + 2, max(0, c - 1):c + 2] = (255, 120, 0)
        for x, y in self.visited:
            dot(x, y, (230, 190, 0), 2)
        for s in self.leg_stats:
            if s['end_point']:
                dot(s['end_point'][0], s['end_point'][1], (40, 90, 255), 3)
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
            node.cancel_goal()
            node.finish('stopped (shut down)')
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
