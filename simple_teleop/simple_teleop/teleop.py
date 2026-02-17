#!/usr/bin/env python3
import sys
import select
import termios
import tty
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped


class TeleopNode(Node):
    def __init__(self):
        super().__init__('simple_teleop')
        self.pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self._lock = threading.Lock()
        self._twist = TwistStamped()
        self._twist.twist.linear.x = 0.0
        self._twist.twist.angular.z = 0.0
        self._running = True
        self.get_logger().info('simple_teleop started')
        thread = threading.Thread(target=self._publisher_loop)
        thread.daemon = True
        thread.start()

    def _publisher_loop(self):
        rate_hz = 10.0
        delay = 1.0 / rate_hz
        while rclpy.ok() and self._running:
            with self._lock:
                msg = TwistStamped()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'base_link'
                msg.twist = self._twist.twist
                self.pub.publish(msg)
            time.sleep(delay)

    def set_twist(self, linear=None, angular=None):
        with self._lock:
            if linear is not None:
                self._twist.twist.linear.x = float(linear)
            if angular is not None:
                self._twist.twist.angular.z = float(angular)

    def stop(self):
        self.set_twist(0.0, 0.0)


def _get_key(timeout=0.1):
    dr, _, _ = select.select([sys.stdin], [], [], timeout)
    if dr:
        c = sys.stdin.read(1)
        if c == '\x1b':
            # possible arrow key sequence
            seq = sys.stdin.read(2)
            return c + seq
        return c
    return None


def main(args=None):
    settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        rclpy.init(args=args)
        node = TeleopNode()

        # Speed control variables
        current_linear_speed = 0.0
        current_angular_speed = 0.0
        linear_increment = 0.1
        angular_increment = 0.2
        max_linear_speed = 1.0
        max_angular_speed = 2.0

        print('Use arrow keys to move (repeated presses increase speed)')
        print('Press "k" to stop, space to stop, q to quit')
        try:
            while rclpy.ok():
                key = _get_key(0.1)
                if not key:
                    continue
                if key == '\x1b[A':  # up - increase forward speed
                    current_linear_speed = min(current_linear_speed + linear_increment, max_linear_speed)
                    node.set_twist(current_linear_speed, None)
                elif key == '\x1b[B':  # down - increase backward speed
                    current_linear_speed = max(current_linear_speed - linear_increment, -max_linear_speed)
                    node.set_twist(current_linear_speed, None)
                elif key == '\x1b[C':  # right - increase right turn speed
                    current_angular_speed = max(current_angular_speed - angular_increment, -max_angular_speed)
                    node.set_twist(None, current_angular_speed)
                elif key == '\x1b[D':  # left - increase left turn speed
                    current_angular_speed = min(current_angular_speed + angular_increment, max_angular_speed)
                    node.set_twist(None, current_angular_speed)
                elif key == 'k':  # k = stop all motion
                    current_linear_speed = 0.0
                    current_angular_speed = 0.0
                    node.stop()
                elif key == ' ':  # space = stop
                    current_linear_speed = 0.0
                    current_angular_speed = 0.0
                    node.stop()
                elif key == 'q':
                    break
        finally:
            node._running = False
            rclpy.shutdown()
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


if __name__ == '__main__':
    main()
