#!/usr/bin/env python3
"""
Carry /cmd_vel to the ESP32 and its heartbeat back, between Zenoh and DDS.

Everything in AGOS talks through Zenoh (rmw_zenoh_cpp), like the simulation
with SLAM on the Pi. The micro-ROS agent, through which the ESP32 joins ROS 2,
speaks only DDS (Fast DDS), and one ROS 2 process can use only one of the two.
So robot.launch.py starts this script twice:

    --side zenoh   RMW_IMPLEMENTATION=rmw_zenoh_cpp, the robot's ROS graph
    --side dds     RMW_IMPLEMENTATION=rmw_fastrtps_cpp, ROS_DOMAIN_ID=77: the agent
                   and the ESP32, in a domain of their own

and the two halves pass each message to each other as its serialized bytes
over a UDP socket on localhost (both middlewares serialize messages as CDR).
Nothing else crosses: the topics are listed in TOPICS.

    zenoh half :<port>   receives the "up" topics from the dds half
    dds half   :<port+1> receives the "down" topics from the zenoh half
"""
import argparse
import socket
import struct
import threading

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile
from rosidl_runtime_py.utilities import get_message

# (topic, type, direction): down = Zenoh -> DDS (to the ESP32), up = DDS -> Zenoh
TOPICS = [
    ('/cmd_vel', 'geometry_msgs/msg/Twist', 'down'),
    ('/esp32/heartbeat', 'std_msgs/msg/Int32', 'up'),
]
HEADER = struct.Struct('!B')        # which entry of TOPICS the packet carries
MAX_PACKET = 65000


class Relay(Node):

    def __init__(self, side, port):
        super().__init__('uros_relay_' + side)
        self.side = side
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        here, there = (port, port + 1) if side == 'zenoh' else (port + 1, port)
        self.sock.bind(('127.0.0.1', here))
        self.peer = ('127.0.0.1', there)
        self.publishers_by_index = {}
        self.seen = set()
        # The zenoh half subscribes to the "down" topics and publishes the
        # "up" ones; the dds half the other way round
        for index, (topic, type_name, direction) in enumerate(TOPICS):
            msg_type = get_message(type_name)
            outgoing = (direction == 'down') == (side == 'zenoh')
            # Towards the ESP32 everything is best effort (a late speed
            # command is useless); on the Zenoh side the ROS defaults.
            qos = qos_profile_sensor_data if side == 'dds' else QoSProfile(depth=10)
            if outgoing:
                self.create_subscription(msg_type, topic, self.sender(index), qos, raw=True)
            else:
                self.publishers_by_index[index] = self.create_publisher(msg_type, topic, qos)
        self.thread = threading.Thread(target=self.receive, daemon=True)
        self.thread.start()
        self.get_logger().info('%s half ready: %s' % (side, ', '.join(
            '%s (%s)' % (t, d) for t, _, d in TOPICS)))

    def sender(self, index):
        def send(data):
            if len(data) <= MAX_PACKET:
                self.sock.sendto(HEADER.pack(index) + bytes(data), self.peer)
        return send

    def receive(self):
        while rclpy.ok():
            try:
                packet = self.sock.recv(MAX_PACKET + HEADER.size)
            except OSError:
                return
            if len(packet) < HEADER.size:
                continue
            index = HEADER.unpack_from(packet)[0]
            pub = self.publishers_by_index.get(index)
            if pub is None:
                continue
            pub.publish(packet[HEADER.size:])       # already serialized
            if index not in self.seen:
                self.seen.add(index)
                self.get_logger().info('First message on %s passed on' % TOPICS[index][0])


def main():
    parser = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    parser.add_argument('--side', choices=('zenoh', 'dds'), required=True)
    parser.add_argument('--port', type=int, default=7550)
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = Relay(args.side, args.port)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.sock.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
