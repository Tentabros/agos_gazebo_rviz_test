"""
Start Nav2 once RTAB-Map is ready, and start it again if it goes down.

Nav2's costmaps need the map -> base_footprint transform before they can
be activated, and RTAB-Map only publishes map -> odom once it has made its
first map. A lifecycle manager that starts Nav2 straight away (autostart)
waits for that transform, times out and aborts the whole bring-up, which
leaves every Nav2 node inactive: no lifecycle manager in the node list,
plans that are drawn but never driven, nothing on /cmd_vel (the problems in
the Fix NAV notes).

This node replaces the manual "ros2 service call
/lifecycle_manager_navigation/manage_nodes ... {command: 0}" step:

  1. wait until the map -> base_footprint transform exists,
  2. ask the lifecycle manager to start Nav2 (STARTUP), unless it is
     already running (e.g. Startup was pressed in RViz's Navigation 2 panel),
  3. every check_period seconds ask it whether Nav2 is active, and if a
     start failed or a node went down, reset Nav2 and start it again.

Started by slam_nav.launch.py when autostart:=true (the default).
"""
from nav2_msgs.srv import ManageLifecycleNodes
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from std_srvs.srv import Trigger
import tf2_ros

WAIT_TF, PRECHECK, STARTING, ACTIVE, CHECKING, RESETTING = range(6)


class NavSupervisor(Node):

    def __init__(self):
        super().__init__('nav_supervisor')
        self.declare_parameter('manager', 'lifecycle_manager_navigation')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('robot_frame', 'base_footprint')
        self.declare_parameter('check_period', 5.0)
        manager = self.get_parameter('manager').value
        self.global_frame = self.get_parameter('global_frame').value
        self.robot_frame = self.get_parameter('robot_frame').value
        self.check_period = float(self.get_parameter('check_period').value)

        self.tf_buffer = None
        self.tf_listener = None
        self.listen_tf(True)
        self.manage = self.create_client(ManageLifecycleNodes, manager + '/manage_nodes')
        self.is_active = self.create_client(Trigger, manager + '/is_active')

        self.state = WAIT_TF
        self.future = None
        self.attempts = 0
        self.last_check = self.get_clock().now()
        self.waiting_since = self.get_clock().now()
        self.retry_at = self.get_clock().now()
        self.timer = self.create_timer(1.0, self.step)
        self.get_logger().info('Waiting for %s -> %s (RTAB-Map) before starting Nav2'
                               % (self.global_frame, self.robot_frame))

    def listen_tf(self, on):
        # /tf is only needed until Nav2 is up. Decoding it in Python costs a few
        # percent of a CPU core, so stop then, and start afresh after a reset.
        if self.tf_listener is not None:
            self.tf_listener.unregister()
            self.tf_listener = None
        if on:
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def tf_ready(self):
        return self.tf_buffer.can_transform(self.global_frame, self.robot_frame, Time(),
                                            timeout=Duration(seconds=0.0))

    def call_manage(self, command):
        if not self.manage.service_is_ready():
            return False
        request = ManageLifecycleNodes.Request()
        request.command = command
        self.future = self.manage.call_async(request)
        return True

    def step(self):
        now = self.get_clock().now()
        if self.state == WAIT_TF:
            if not self.tf_ready():
                if (now - self.waiting_since).nanoseconds > 30e9:
                    self.get_logger().info(
                        'Still no %s -> %s transform. Is RTAB-Map running and getting '
                        'camera, LiDAR and odometry data?' % (self.global_frame, self.robot_frame))
                    self.waiting_since = now
                return
            if now < self.retry_at or not self.is_active.service_is_ready():
                return
            # already running? (then there is nothing to start)
            self.future = self.is_active.call_async(Trigger.Request())
            self.last_check = now
            self.state = PRECHECK

        elif self.state == PRECHECK:
            if not self.future.done():
                if (now - self.last_check).nanoseconds > 10e9:
                    self.state = WAIT_TF     # manager busy, ask again later
                return
            result = self.future.result()
            if result is not None and result.success:
                self.get_logger().info('Nav2 is already active')
                self.state = ACTIVE
                self.last_check = now
                self.listen_tf(False)
            elif self.call_manage(ManageLifecycleNodes.Request.STARTUP):
                self.attempts += 1
                self.get_logger().info('RTAB-Map is up: starting Nav2 (attempt %d)'
                                       % self.attempts)
                self.state = STARTING
            else:
                self.state = WAIT_TF

        elif self.state == STARTING:
            if not self.future.done():
                return
            ok = self.future.result() is not None and self.future.result().success
            if ok:
                self.get_logger().info('Nav2 is active')
                self.state = ACTIVE
                self.last_check = now
                self.listen_tf(False)
            else:
                self.get_logger().warning('Nav2 did not start, resetting it and trying again')
                self.reset()

        elif self.state == ACTIVE:
            if (now - self.last_check).nanoseconds < self.check_period * 1e9:
                return
            self.last_check = now
            if self.is_active.service_is_ready():
                self.future = self.is_active.call_async(Trigger.Request())
                self.state = CHECKING

        elif self.state == CHECKING:
            if not self.future.done():
                if (now - self.last_check).nanoseconds > 10e9:
                    self.state = ACTIVE      # manager busy, ask again later
                return
            result = self.future.result()
            if result is not None and result.success:
                self.state = ACTIVE
            else:
                self.get_logger().warning('Nav2 is no longer active, restarting it')
                self.reset()

        elif self.state == RESETTING:
            if self.future.done():
                self.state = WAIT_TF

    def reset(self):
        # wait a little longer after each failed start (5, 10 ... 30 s)
        now = self.get_clock().now()
        self.retry_at = now + Duration(seconds=min(30.0, 5.0 * max(1, self.attempts)))
        self.waiting_since = now
        self.listen_tf(True)
        if self.call_manage(ManageLifecycleNodes.Request.RESET):
            self.state = RESETTING
        else:
            self.state = WAIT_TF


def main(args=None):
    rclpy.init(args=args)
    node = NavSupervisor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
