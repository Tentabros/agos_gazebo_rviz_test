"""
Give the EKF the rate at which the robot's heading turns, not the gyro's z rate.

The EKF (config/ekf.yaml) works in 2D: it adds up the turn rate it is given
as the robot's heading. The gyro measures turn rates about the robot's own
axes, and in a pipe the robot is often not level: a track rides up the
curved wall in a bend, or when the robot turns round in a manhole. Tilted,
the gyro's z axis is not vertical and measures only part of a turn: through
a bend in the simulation, tilted up to 12 deg, it measured an 88 deg turn
0.6 deg short. Out and back the errors cancel; round a loop they add up, and
the odometry drifts sideways.

This node republishes /imu/data on /imu/level with the z rate replaced by the
heading rate, from the gyro's x, y and z rates (p, q, r) and the IMU's roll
and pitch:

    (q sin(roll) + r cos(roll)) / cos(pitch)

Roll and pitch come from the IMU's orientation: an IMU's own filter takes
them from gravity, to about a degree in a low-cost one, and in Gazebo they are
exact. The heading in that orientation would drift without a compass, and is
not used.
"""
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

MAX_TILT = math.radians(60.0)   # more than this, and the orientation is not believed


def roll_pitch(x, y, z, w):
    """Return (roll, pitch) in radians of a quaternion, in the ZYX (yaw, pitch, roll) order."""
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    return roll, pitch


def heading_rate(q, r, roll, pitch):
    """Return the heading's rate of turn for gyro rates q (y axis) and r (z axis), in rad/s."""
    return (q * math.sin(roll) + r * math.cos(roll)) / math.cos(pitch)


class ImuLevel(Node):

    def __init__(self):
        super().__init__('imu_level')
        self.pub = self.create_publisher(Imu, 'imu/level', 10)
        self.create_subscription(Imu, 'imu/data', self.on_imu, qos_profile_sensor_data)

    def on_imu(self, msg):
        o, w = msg.orientation, msg.angular_velocity
        roll, pitch = roll_pitch(o.x, o.y, o.z, o.w)
        # orientation_covariance[0] == -1: the IMU gives no orientation
        if msg.orientation_covariance[0] >= 0.0 and abs(roll) < MAX_TILT and \
                abs(pitch) < MAX_TILT:
            w.z = heading_rate(w.y, w.z, roll, pitch)
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImuLevel()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
