#!/usr/bin/env python3
"""Keyboard teleop for Yahboom ROSMASTER X3 mecanum-wheel robot.

Latched velocity state: each key sets one axis (vx, vy or wz) and the others
are kept, so motions combine (e.g. W then E -> drive forward while rotating).
Space stops everything. The current command is published continuously to
mecanum_drive_controller so the latched motion persists.
"""
import curses

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped

HELP = """\
Keyboard Teleop - ROSMASTER X3   (latched: motions combine)
-----------------------------------------------------------
  Up / W    forward          Down / S   backward
  Left / A  strafe left      Right / D  strafe right
  Q         rotate CCW        E          rotate CW
  Space     STOP all          Ctrl+C     quit
-----------------------------------------------------------
  Each key sets one axis; others stay. e.g. W then E = forward + rotate."""


class KeyboardTeleop(Node):

    def __init__(self):
        super().__init__('keyboard_teleop')
        self.declare_parameter('max_linear_vel', 0.3)
        self.declare_parameter('max_angular_vel', 1.0)
        self._max_v = self.get_parameter('max_linear_vel').value
        self._max_w = self.get_parameter('max_angular_vel').value

        self._pub = self.create_publisher(
            TwistStamped, '/mecanum_drive_controller/cmd_vel', 10)

        # latched command state
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0

    def stop(self):
        self.vx = self.vy = self.wz = 0.0

    def publish(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_footprint'
        msg.twist.linear.x = self.vx
        msg.twist.linear.y = self.vy
        msg.twist.angular.z = self.wz
        self._pub.publish(msg)


def _run(stdscr, node: KeyboardTeleop):
    curses.cbreak()
    curses.noecho()
    stdscr.keypad(True)
    stdscr.nodelay(True)   # non-blocking getch

    v = node._max_v
    w = node._max_w

    # key -> (axis attribute, value). Each key sets only its own axis.
    KEY_SET = {
        curses.KEY_UP:    ('vx',  v),
        ord('w'):         ('vx',  v),
        ord('W'):         ('vx',  v),
        curses.KEY_DOWN:  ('vx', -v),
        ord('s'):         ('vx', -v),
        ord('S'):         ('vx', -v),
        curses.KEY_LEFT:  ('vy',  v),
        ord('a'):         ('vy',  v),
        ord('A'):         ('vy',  v),
        curses.KEY_RIGHT: ('vy', -v),
        ord('d'):         ('vy', -v),
        ord('D'):         ('vy', -v),
        ord('q'):         ('wz',  w),
        ord('Q'):         ('wz',  w),
        ord('e'):         ('wz', -w),
        ord('E'):         ('wz', -w),
    }

    help_lines = HELP.splitlines()
    for i, line in enumerate(help_lines):
        stdscr.addstr(i, 0, line)
    status_row = len(help_lines) + 1
    stdscr.addstr(status_row - 1, 0,
                  f'limits: linear {v:.2f} m/s   angular {w:.2f} rad/s')
    stdscr.refresh()

    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.05)
        key = stdscr.getch()
        if key in KEY_SET:
            axis, val = KEY_SET[key]
            # press the same direction again -> zero that axis (toggle)
            current = getattr(node, axis)
            setattr(node, axis, 0.0 if current == val else val)
        elif key == ord(' '):
            node.stop()
        # publish the latched command every cycle so motion persists
        node.publish()
        stdscr.addstr(status_row, 0,
                      f'cmd: vx={node.vx:+.2f}  vy={node.vy:+.2f}  wz={node.wz:+.2f}')
        stdscr.clrtoeol()
        stdscr.refresh()


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()
    try:
        curses.wrapper(_run, node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.publish()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
