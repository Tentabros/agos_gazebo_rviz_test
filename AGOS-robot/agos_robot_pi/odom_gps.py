#!/usr/bin/env python3
"""
Give RTAB-Map the robot's odometry as a GPS fix, so that it only looks for loop closures nearby.

RTAB-Map finds a loop closure by how alike two camera images are. In the
simulated network every pipe has the same concrete texture and every
manhole looks the same, and in a run round the whole network RTAB-Map
matched a pipe with another one 33 m away: the map turned 100 deg and the
robot could not find its way back. With a GPS fix in each map node,
RTAB-Map only compares a new node with the nodes whose fix is within
RGBD/LocalRadius of its own (Rtabmap/LoopGPS, on by default).

A buried pipe has no GPS, so this node publishes the odometry (/odom, the
EKF) as one: odometry x east and y north of a point on the equator. It
drifts slowly (about 2 % of the distance driven in testing), so a closure
where the robot comes back round a loop is still looked for, and matches
between places further apart are not. RTAB-Map uses these fixes for
nothing else: it ignores GPS in its graph optimization
(Optimizer/PriorsIgnored, true by default).

A copy of practice_agos/agos_sim/agos_sim/odom_gps.py. On the robot,
robot.launch.py starts it when it makes a new map (not with
localization:=true: the fixes in a saved map come from that run's odometry),
and /odom is icp_odometry's.
"""
import math

from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus

EARTH_RADIUS = 6378137.0    # m, at the equator


def to_fix(x, y):
    """Return (latitude, longitude) in degrees for x m east and y m north of (0, 0)."""
    return math.degrees(y / EARTH_RADIUS), math.degrees(x / EARTH_RADIUS)


class OdomGps(Node):

    def __init__(self):
        super().__init__('odom_gps')
        self.declare_parameter('rate', 2.0)     # fixes a second (RTAB-Map adds a node a second)
        self.odom = None
        self.pub = self.create_publisher(NavSatFix, 'gps/fix', 1)
        self.create_subscription(Odometry, 'odom', self.on_odom, 1)
        self.create_timer(1.0 / max(float(self.get_parameter('rate').value), 0.1), self.publish)

    def on_odom(self, msg):
        self.odom = msg

    def publish(self):
        if self.odom is None:
            return
        p = self.odom.pose.pose.position
        fix = NavSatFix()
        fix.header.stamp = self.odom.header.stamp
        fix.header.frame_id = self.odom.child_frame_id
        fix.status.status = NavSatStatus.STATUS_FIX
        fix.status.service = NavSatStatus.SERVICE_GPS
        fix.latitude, fix.longitude = to_fix(p.x, p.y)
        fix.altitude = 0.0
        fix.position_covariance = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self.pub.publish(fix)


def main(args=None):
    rclpy.init(args=args)
    node = OdomGps()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
