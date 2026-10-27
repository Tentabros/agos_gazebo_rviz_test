#!/usr/bin/env python3
"""
How the robot's Raspberry Pi keeps up, and whether every part is alive.

Started by robot.launch.py. Once a second it publishes a one-line summary on
/agos_robot/status (./agos_laptop ping on the laptop and ./agos_robot status
on the Pi read it), prints one every 10 s, and writes a row to a CSV file in
log_dir:

    time_s       seconds since this node started
    scan_hz      RPLiDAR C1 scans a second (/scan, about 10)
    imu_hz       IMU samples a second (/imu/data, about 200; 0 with camera:=false)
    odom_hz      icp_odometry updates a second (/odom)
    odom_lost    1 if icp_odometry has lost track
    icp_inliers  share of the scan that matched the map (icp_odometry), 0..1
    update_ms    RTAB-Map's last update: the SLAM, then making and sending the map
    nodes        map nodes in RTAB-Map's working memory
    loops        loop closures and proximity links so far
    esp32_age_s  seconds since the ESP32's last heartbeat (-1: never heard)
    esp32_cmds   /cmd_vel messages the ESP32 has received
    cpu_pct      the Pi's CPU use, all cores (100 = every core busy)
    ram_mb       the Pi's memory in use
    temp_c       the CPU temperature
    throttled    1 if the Pi is slowing itself down (too hot or low voltage)

./agos_robot stats prints the totals of the last run:

    python3 robot_monitor.py --summary logs/<file>.csv
"""
import csv
import datetime
import glob
import math
import os
import platform
import sys
import time

THROTTLED = '/sys/devices/platform/soc/soc:firmware/get_throttled'
THERMAL = '/sys/class/thermal/thermal_zone0/temp'
COLUMNS = ['time_s', 'scan_hz', 'imu_hz', 'odom_hz', 'odom_lost', 'icp_inliers', 'update_ms',
           'nodes', 'loops', 'esp32_age_s', 'esp32_cmds', 'cpu_pct', 'ram_mb', 'temp_c',
           'throttled']


def read(path):
    try:
        with open(path) as f:
            return f.read().strip()
    except OSError:
        return None


def pi_model():
    """Return the board's name, e.g. 'Raspberry Pi 4 Model B Rev 1.5'."""
    # Docker hides /proc/device-tree: ./agos_robot passes it in AGOS_PI_MODEL
    model = os.environ.get('AGOS_PI_MODEL') or read('/proc/device-tree/model')
    if model:
        return model.replace('\x00', '').strip()
    return '%s (%s)' % (platform.node(), platform.machine())


class Load:
    """CPU use (from /proc/stat), memory, temperature and throttling of this computer."""

    def __init__(self):
        self.last = self.cpu_times()

    @staticmethod
    def cpu_times():
        line = read('/proc/stat')
        if not line:
            return None
        values = [int(v) for v in line.splitlines()[0].split()[1:]]
        idle = values[3] + (values[4] if len(values) > 4 else 0)   # idle + iowait
        return sum(values), idle

    def cpu(self):
        """Return the CPU use in % since the last call."""
        now = self.cpu_times()
        if now is None or self.last is None:
            return float('nan')
        total, idle = now[0] - self.last[0], now[1] - self.last[1]
        self.last = now
        return 100.0 * (1.0 - idle / total) if total > 0 else 0.0

    @staticmethod
    def ram_mb():
        info = {}
        for line in (read('/proc/meminfo') or '').splitlines():
            key, _, value = line.partition(':')
            info[key] = float(value.split()[0]) / 1024.0 if value.split() else 0.0
        if 'MemTotal' not in info:
            return float('nan'), float('nan')
        return info['MemTotal'] - info.get('MemAvailable', 0.0), info['MemTotal']

    @staticmethod
    def temp_c():
        value = read(THERMAL)
        return float(value) / 1000.0 if value else float('nan')

    @staticmethod
    def throttled():
        """Return 1 if the Pi is throttling now (too hot, low voltage), 0 if not, -1 unknown."""
        value = read(THROTTLED)
        if value is None:
            return -1
        try:
            return 1 if int(value, 16) & 0xF else 0     # bits 0-3: happening now
        except ValueError:
            return -1


def mean(values):
    return sum(values) / len(values) if values else float('nan')


def percentile(values, q):
    if not values:
        return float('nan')
    values = sorted(values)
    return values[min(len(values) - 1, int(round(q / 100.0 * (len(values) - 1))))]


def summary_lines(rows, model=None):
    def col(name):
        out = []
        for r in rows:
            try:
                v = float(r[name])
            except (KeyError, TypeError, ValueError):
                continue
            if not math.isnan(v):
                out.append(v)
        return out

    lines = []
    if model:
        lines.append(model)
    if not rows:
        return lines + ['No data yet.']
    span = col('time_s')
    lines.append('Run of %.0f s' % (span[-1] if span else 0.0))
    for name, label, unit in (('scan_hz', 'LiDAR scans', 'a second'),
                              ('imu_hz', 'IMU samples', 'a second'),
                              ('odom_hz', 'Odometry updates', 'a second'),
                              ('update_ms', 'RTAB-Map update time', 'ms'),
                              ('cpu_pct', 'CPU use', '%'),
                              ('temp_c', 'Temperature', 'C')):
        v = col(name)
        if v:
            lines.append('%s: mean %.1f, 95%% %.1f, max %.1f %s'
                         % (label, mean(v), percentile(v, 95), max(v), unit))
    lost = col('odom_lost')
    if lost:
        lines.append('Odometry lost: %.0f %% of the time' % (100.0 * mean(lost)))
    loops = col('loops')
    if loops:
        lines.append('Loop closures and proximity links: %d' % max(loops))
    ram = col('ram_mb')
    if ram:
        lines.append('Memory in use: max %.0f MB' % max(ram))
    throttled = [v for v in col('throttled') if v >= 0]
    if throttled and max(throttled) > 0:
        lines.append('THROTTLED %.0f %% of the time: add a fan, or a better power supply'
                     % (100.0 * mean(throttled)))
    return lines


def summarize(path):
    if path == 'last':
        files = sorted(glob.glob(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                              'logs', 'robot_*.csv')))
        if not files:
            print('No runs in logs/ yet.')
            return
        path = files[-1]
    with open(path, newline='') as f:
        rows = list(csv.DictReader(f))
    print(path)
    print('\n'.join(summary_lines(rows, pi_model())))


def main():
    if len(sys.argv) > 2 and sys.argv[1] == '--summary':
        summarize(sys.argv[2])
        return

    import rclpy
    from rclpy.clock import Clock, ClockType
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from nav_msgs.msg import Odometry
    from rtabmap_msgs.msg import Info, OdomInfo
    from sensor_msgs.msg import Imu, LaserScan
    from std_msgs.msg import Int32, String

    class Monitor(Node):

        def __init__(self):
            super().__init__('agos_robot_monitor')
            self.log_dir = os.path.expanduser(
                self.declare_parameter('log_dir', '~/agos_robot_logs').value)
            self.rate = float(self.declare_parameter('detection_rate', 1.0).value)
            self.camera = bool(self.declare_parameter('camera', True).value)
            os.makedirs(self.log_dir, exist_ok=True)
            stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
            self.path = os.path.join(self.log_dir, 'robot_%s.csv' % stamp)
            self.file = open(self.path, 'w', newline='')
            self.csv = csv.DictWriter(self.file, fieldnames=COLUMNS)
            self.csv.writeheader()
            self.rows = []
            self.t0 = time.monotonic()
            self.model = pi_model()
            self.load = Load()
            self.counts = {'scan': 0, 'imu': 0, 'odom': 0}
            self.odom_lost = None
            self.icp_inliers = float('nan')
            self.update_ms = float('nan')
            self.nodes = 0
            self.loops = 0
            self.esp32_time = None
            self.esp32_cmds = 0
            self.last_print = 0.0

            def counter(name):
                def count(_msg):
                    self.counts[name] += 1
                return count

            self.create_subscription(LaserScan, '/scan', counter('scan'), qos_profile_sensor_data)
            self.create_subscription(Imu, '/imu/data', counter('imu'), qos_profile_sensor_data)
            self.create_subscription(Odometry, '/odom', counter('odom'), 10)
            self.create_subscription(OdomInfo, '/odom_info', self.on_odom_info, 10)
            self.create_subscription(Info, '/info', self.on_info, 10)
            self.create_subscription(Int32, '/esp32/heartbeat', self.on_heartbeat, 10)
            self.status = self.create_publisher(String, '/agos_robot/status', 1)
            self.create_timer(1.0, self.on_timer, clock=Clock(clock_type=ClockType.STEADY_TIME))
            self.last_tick = time.monotonic()
            self.get_logger().info('%s, ROS 2 %s: writing %s'
                                   % (self.model, os.environ.get('ROS_DISTRO', '?'), self.path))

        def on_odom_info(self, msg):
            self.odom_lost = bool(msg.lost)
            self.icp_inliers = float(msg.icp_inliers_ratio)

        def on_info(self, msg):
            stats = dict(zip(msg.stats_keys, msg.stats_values))
            # RTAB-Map's ROS node sends an update's total time with the next one
            self.update_ms = stats.get('RtabmapROS/TimeTotal/ms', self.update_ms)
            self.nodes = int(stats.get('Memory/Working_memory_size/', self.nodes))
            if msg.loop_closure_id > 0 or msg.proximity_detection_id > 0:
                self.loops += 1

        def on_heartbeat(self, msg):
            self.esp32_time = time.monotonic()
            self.esp32_cmds = msg.data

        def on_timer(self):
            now = time.monotonic()
            dt = max(now - self.last_tick, 1e-3)
            self.last_tick = now
            rates = {k: v / dt for k, v in self.counts.items()}
            for k in self.counts:
                self.counts[k] = 0
            ram, total = Load.ram_mb()
            esp_age = now - self.esp32_time if self.esp32_time is not None else -1.0
            row = {
                'time_s': '%.1f' % (now - self.t0),
                'scan_hz': '%.1f' % rates['scan'],
                'imu_hz': '%.0f' % rates['imu'],
                'odom_hz': '%.1f' % rates['odom'],
                'odom_lost': '' if self.odom_lost is None else '%d' % self.odom_lost,
                'icp_inliers': '%.2f' % self.icp_inliers,
                'update_ms': '%.0f' % self.update_ms,
                'nodes': '%d' % self.nodes,
                'loops': '%d' % self.loops,
                'esp32_age_s': '%.1f' % esp_age,
                'esp32_cmds': '%d' % self.esp32_cmds,
                'cpu_pct': '%.0f' % self.load.cpu(),
                'ram_mb': '%.0f' % ram,
                'temp_c': '%.1f' % Load.temp_c(),
                'throttled': '%d' % Load.throttled(),
            }
            self.csv.writerow(row)
            self.file.flush()
            self.rows.append(row)
            text = self.line(row, total)
            self.status.publish(String(data=text))
            if now - self.last_print >= 10.0:
                self.last_print = now
                self.get_logger().info(text)

        def line(self, row, total):
            parts = ['%s | CPU %s %% | RAM %s/%.0f MB'
                     % (self.model, row['cpu_pct'], row['ram_mb'], total)]
            if row['temp_c'] != 'nan':
                parts.append('%s C' % row['temp_c'])
            if row['throttled'] == '1':
                parts.append('THROTTLED')
            parts.append('LiDAR %s Hz' % row['scan_hz'])
            if self.camera:
                parts.append('IMU %s Hz' % row['imu_hz'])
            odom = 'odometry %s Hz' % row['odom_hz']
            if row['odom_lost'] == '1':
                odom += ' LOST'
            parts.append(odom)
            if row['update_ms'] != 'nan':
                parts.append('RTAB-Map %s ms, %s nodes, %s loops'
                             % (row['update_ms'], row['nodes'], row['loops']))
            else:
                parts.append('RTAB-Map: no update yet')
            age = float(row['esp32_age_s'])
            if age < 0:
                parts.append('ESP32: not heard yet')
            elif age > 2.0:
                parts.append('ESP32: SILENT for %.0f s' % age)
            else:
                parts.append('ESP32: ok, %s commands' % row['esp32_cmds'])
            return ' | '.join(parts)

        def close(self):
            self.file.close()
            lines = ['', 'Totals of this run (%s):' % self.path]
            print('\n'.join(lines + summary_lines(self.rows, self.model)), flush=True)

    rclpy.init()
    node = Monitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
