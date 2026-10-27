#!/usr/bin/env python3
"""
Drive the motors in short, slow steps to check the wiring and directions.

Run it while ./agos_robot start is running, with the robot's wheels off the
floor (on a box) the first time:

    ./agos_robot motor-test               the steps below at 0.15 m/s and 1.5 rad/s
    ./agos_robot motor-test 0.3 2.0       faster
    ./agos_robot motor-test duty [top]    full power for 2 s, to measure the top speed
                                          (top: MAX_WHEEL_SPEED in config.h, default 0.5)

Each step publishes /cmd_vel at 20 Hz for 2 s, then stops for 1 s. Watch
the wheels: the printed line says what they should do.
"""
import sys
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node


def steps(v, w):
    return [
        ('forwards: both wheels roll forwards', v, 0.0),
        ('backwards: both wheels roll backwards', -v, 0.0),
        ('turn left (anticlockwise seen from above): left wheel back, right forwards', 0.0, w),
        ('turn right (clockwise): left wheel forwards, right back', 0.0, -w),
    ]


def main():
    calibrate = len(sys.argv) > 1 and sys.argv[1] == 'duty'
    v = float(sys.argv[1]) if len(sys.argv) > 1 and not calibrate else 0.15
    w = float(sys.argv[2]) if len(sys.argv) > 2 and not calibrate else 1.5
    rclpy.init()
    node = Node('agos_motor_test')
    pub = node.create_publisher(Twist, '/cmd_vel', 10)
    # give Zenoh a moment to connect the publisher to the relay
    time.sleep(1.5)

    def drive(speed, turn, seconds):
        msg = Twist()
        msg.linear.x = float(speed)
        msg.angular.z = float(turn)
        end = time.monotonic() + seconds
        while time.monotonic() < end and rclpy.ok():
            pub.publish(msg)
            time.sleep(0.05)
        pub.publish(Twist())

    try:
        if calibrate:
            # Asking for MAX_WHEEL_SPEED (or more) gives both motors MAX_DUTY:
            # the distance driven in 2 s at full power is the real top speed.
            top = float(sys.argv[2]) if len(sys.argv) > 2 else 0.5   # config.h default
            print('Calibration: the robot drives straight ahead for 2 s at full power. Put it '
                  'on the floor with 2 m free ahead and mark where it starts.')
            input('Enter to drive... ')
            drive(top, 0.0, 2.0)
            print('Measure the distance in m and divide by 2: that is MAX_WHEEL_SPEED for '
                  'esp32_motor_node/include/config.h. If it curved, the weaker side is the one '
                  'it curved towards (see the guide).')
        else:
            for text, speed, turn in steps(v, w):
                print(text)
                drive(speed, turn, 2.0)
                time.sleep(1.0)
            print('Done. A wheel that turned the wrong way: INVERT_LEFT / INVERT_RIGHT in '
                  'esp32_motor_node/include/config.h. Left and right swapped: swap the two '
                  "motors' L298N outputs (OUT1/OUT2 with OUT3/OUT4).")
    except KeyboardInterrupt:
        pass
    finally:
        pub.publish(Twist())
        time.sleep(0.2)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
