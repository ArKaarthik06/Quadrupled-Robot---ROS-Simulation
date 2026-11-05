#!/usr/bin/env python3
"""Keyboard teleop for the Recon simulation. Publishes firmware-style letters on /recon/command."""
import select
import sys
import termios
import tty

import rclpy
from std_msgs.msg import String

HELP = """
Recon keyboard teleop
---------------------
  w : walk forward         x : walk backward (also 'b')
  a : turn left            d : turn right   (also 'l' / 'r')
  space / s : stand        k : sit          g : run
  z : hello   p : push-up  u : up-down      q : front-back
  t/e : body left/right    v : dance        m : moonwalk
  y : toggle head scan     c : crab stand   n : wheel mode
  CTRL-C : quit
"""

KEYMAP = {
    'w': 'f', 'x': 'b', 'a': 'l', 'd': 'r', ' ': 's', 's': 's',
    'b': 'b', 'l': 'l', 'r': 'r', 'f': 'f',
    'k': 'k', 'g': 'g', 'z': 'z', 'p': 'p', 'u': 'u', 'q': 'q',
    't': 't', 'e': 'w', 'v': 'd', 'm': 'm', 'y': 'y', 'c': 'c', 'n': 'n', 'i': 'i', 'o': 'x',
}


def main():
    rclpy.init()
    node = rclpy.create_node('recon_teleop')
    pub = node.create_publisher(String, '/recon/command', 10)
    print(HELP)
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        while rclpy.ok():
            if select.select([sys.stdin], [], [], 0.1)[0]:
                key = sys.stdin.read(1)
                if key == '\x03':
                    break
                cmd = KEYMAP.get(key)
                if cmd:
                    pub.publish(String(data=cmd))
                    print(f'  [{key!r}] -> command "{cmd}"')
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
