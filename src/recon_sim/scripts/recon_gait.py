#!/usr/bin/env python3
"""
Recon gait node - a ROS 2 port of Octosnake.cpp + the gaits of Recon.cpp.

Every servo is driven by a sine oscillator   angle = A*sin(2*pi*t/T + phase) + offset
(angles in servo degrees, 90 = centre), exactly like the ESP32 firmware.  The 8 servo
angles (+ the neck) are converted to joint radians (q = radians(angle - 90)) and sent
to the Gazebo `leg_position_controller`.

Commands (std_msgs/String on /recon/command) use the same letters as the firmware
web/serial protocol, plus a few extras:

    f  walk forward        b  walk backward      l  turn left        r  turn right
    g  run                 s  stand (home)       k  sit
    z  hello               p  push-up            u  up-down          q  front-back
    w  body right          t  body left          d  dance            x  twerk
    m  moonwalk            n  wheel mode         c  crab stand       i  sit-wiggle
    y  toggle neck scan (head sweeps left/right)

geometry_msgs/Twist on /cmd_vel also works (x>0 forward, x<0 back, z>0 left, z<0 right).
"""
import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String

# servo indices (Recon.h)
FR_HIP, FL_HIP, FR_LEG, FL_LEG, BR_HIP, BL_HIP, BR_LEG, BL_LEG = range(8)


class Oscillator:
    """Octosnake::Oscillator"""

    def __init__(self):
        self.period = 2000.0
        self.amplitude = 50.0
        self.phase = 0.0
        self.offset = 0.0
        self.ref_time = 0.0
        self.output = 90.0

    def refresh(self, now_ms):
        delta = (now_ms - self.ref_time) % self.period
        self.output = (self.amplitude * math.sin(delta * 2.0 * math.pi / self.period
                                                 + math.radians(self.phase)) + self.offset)
        return self.output


class Gait:
    """Recon::execute() - all 8 oscillators run continuously."""

    def __init__(self, period, amplitude, offset, phase):
        self.period = period
        self.amplitude = amplitude
        self.offset = offset
        self.phase = phase
        self.osc = [Oscillator() for _ in range(8)]

    def start(self, now_ms, last_angles):
        for i, o in enumerate(self.osc):
            o.period = float(self.period[i])
            o.amplitude = float(self.amplitude[i])
            o.phase = float(self.phase[i])
            o.offset = float(self.offset[i])
            o.ref_time = now_ms
            o.output = last_angles[i]

    def update(self, now_ms):
        return [o.refresh(now_ms) for o in self.osc]


class WalkGait(Gait):
    """Recon::walk() - hips run continuously, the legs are refreshed in diagonal pairs:
    FL+BR lift during the first half period, FR+BL during the second half."""

    def __init__(self, period, amplitude, offset, phase, T):
        super().__init__(period, amplitude, offset, phase)
        self.T = T
        self.t0 = 0.0

    def start(self, now_ms, last_angles):
        super().start(now_ms, last_angles)
        self.t0 = now_ms

    def update(self, now_ms):
        side = int((now_ms - self.t0) / (self.T / 2.0)) % 2
        for i in (FR_HIP, FL_HIP, BR_HIP, BL_HIP):
            self.osc[i].refresh(now_ms)
        if side == 0:
            self.osc[FL_LEG].refresh(now_ms)
            self.osc[BR_LEG].refresh(now_ms)
        else:
            self.osc[FR_LEG].refresh(now_ms)
            self.osc[BL_LEG].refresh(now_ms)
        return [o.output for o in self.osc]   # servos that were not refreshed hold their value


# ----------------------------------------------------------------------------------
# gait definitions (numbers copied from Recon.cpp)
# ----------------------------------------------------------------------------------
def _all(T):
    return [T] * 8


def home_pose(ap=20, hi=0):
    return [90 + ap, 90 - ap, 90 - hi, 90 + hi, 90 - ap, 90 + ap, 90 + hi, 90 - hi]


def static(pose):
    return Gait(_all(1000), [0] * 8, pose, [0] * 8)


def walk(direction, T):
    x_amp, z_amp, ap, hi, front_x = 15, 15, 15, 25, 5
    period = [T, T, T / 2, T / 2, T, T, T / 2, T / 2]
    amplitude = [x_amp, x_amp, z_amp, z_amp, x_amp, x_amp, z_amp, z_amp]
    offset = [90 + ap - front_x, 90 - ap + front_x, 90 - hi, 90 + hi,
              90 - ap - front_x, 90 + ap + front_x, 90 + hi, 90 - hi]
    phase = [270, 270, 90, 270, 90, 90, 270, 90]
    if direction == 0:  # backward
        phase[0] = phase[1] = 90
        phase[4] = phase[5] = 270
        offset[0] = 90 + ap + front_x
        offset[1] = 90 - ap - front_x
        offset[4] = 90 - ap + front_x
        offset[5] = 90 + ap - front_x
    return WalkGait(period, amplitude, offset, phase, T)


def run(direction, T):
    x_amp, z_amp, ap, hi = 15, 15, 15, 0
    amplitude = [x_amp, x_amp, z_amp, z_amp, x_amp, x_amp, z_amp, z_amp]
    offset = [90 + ap, 90 - ap, 90 - hi, 90 + hi, 90 - ap, 90 + ap, 90 + hi, 90 - hi]
    phase = [0, 0, 90, 90, 180, 180, 90, 90]
    if direction == 1:
        phase[0] = phase[1] = 180
        phase[4] = phase[5] = 0
    return Gait(_all(T), amplitude, offset, phase)


def turn(left, T):
    x_amp, z_amp, ap, hi = 15, 15, 15, 23
    amplitude = [x_amp, x_amp, z_amp, z_amp, x_amp, x_amp, z_amp, z_amp]
    offset = [90 + ap, 90 - ap, 90 - hi, 90 + hi, 90 - ap, 90 + ap, 90 + hi, 90 - hi]
    phase = [180, 0, 90, 90, 0, 180, 90, 90] if left else [0, 180, 90, 90, 180, 0, 90, 90]
    return Gait(_all(T), amplitude, offset, phase)


def front_back(T):
    x_amp, z_amp, ap, hi = 30, 25, 20, 0
    amplitude = [x_amp, x_amp, z_amp, z_amp, x_amp, x_amp, z_amp, z_amp]
    offset = [90 + ap, 90 - ap, 90 - hi, 90 + hi, 90 - ap, 90 + ap, 90 + hi, 90 - hi]
    return Gait(_all(T), amplitude, offset, [0, 180, 270, 90, 0, 180, 90, 270])


def up_down(T):
    z_amp, ap, hi = 35, 20, 25
    amplitude = [0, 0, z_amp, z_amp, 0, 0, z_amp, z_amp]
    offset = [90 + ap, 90 - ap, 90 - hi, 90 + hi, 90 - ap, 90 + ap, 90 + hi, 90 - hi]
    return Gait(_all(T), amplitude, offset, [0, 0, 90, 270, 180, 180, 270, 90])


def push_up(T):
    z_amp, x_amp, hi, ap = 40, 25, 30, 80
    amplitude = [0, 0, z_amp, z_amp, 0, 0, 0, 0]
    offset = [90 + x_amp, 90 - x_amp, 90 - hi, 90 + hi, 90 - ap, 90 + ap, 90 + hi, 90 - hi]
    return Gait(_all(T), amplitude, offset, [0, 90, 180, 0, 0, 0, 0, 0])


def dance(T):
    x_amp, z_amp, ap, hi = 0, 40, 30, 0
    amplitude = [x_amp, x_amp, z_amp, z_amp, x_amp, x_amp, z_amp, z_amp]
    offset = [90 + ap, 90 - ap, 90 - hi, 90 + hi, 90 - ap, 90 + ap, 90 + hi, 90 - hi]
    return Gait(_all(T), amplitude, offset, [0, 0, 0, 270, 0, 0, 90, 180])


def body_side(T):   # bodyLeft / bodyRight are identical in the firmware
    x_amp, ap, hi = 35, 15, 50
    amplitude = [0, 0, 0, 0, x_amp, x_amp, 0, 0]
    offset = [90 + ap, 90 - ap, 90 - hi, 90 + hi, 90 - ap, 90 + ap, 90 + hi, 90 - hi]
    return Gait(_all(T), amplitude, offset, [0, 0, 90, 270, 0, 0, 270, 90])


def twerk(T=350):
    z_amp, hi = 35, 50
    amplitude = [0, 0, 0, 0, 0, 0, z_amp, z_amp]
    offset = [90, 90, 90 - hi, 90 + hi, 0, 180, 90 + hi, 90 - hi]
    return Gait(_all(T), amplitude, offset, [0, 0, 90, 270, 90, 270, 270, 90])


def moonwalk(T):
    z_amp = 45
    amplitude = [0, 0, z_amp, z_amp, 0, 0, z_amp, z_amp]
    return Gait(_all(T), amplitude, [90] * 8, [0, 0, 0, 120, 0, 0, 180, 290])


def hello(T):
    ap, hi = 10, 70
    amplitude = [0, 50, 30, 0, 0, 0, 0, 0]
    offset = [90 + 15, 40, 90 - 90, 90, 90 + ap, 90 - ap, 90 + hi, 90 - hi]
    return Gait(_all(T), amplitude, offset, [0, 180, 90, 0, 0, 0, 0, 0])


def sit_wiggle(T=400):
    ap, hi, ti, z_amp = 90, 90, -20, 30
    amplitude = [0, 0, 0, 0, z_amp, z_amp, 0, 0]
    offset = [90 + ap, 90 - ap, 90 - ap, 90 + ap, 90 - hi, 90 + hi, 90 + ti, 90 - ti]
    return Gait(_all(T), amplitude, offset, [0, 0, 90, 180, 0, 0, 0, 0])


def build_gait(cmd, T):
    """Map a command letter to a Gait (T = period in ms, like `Speed` in the .ino)."""
    table = {
        'f': lambda: walk(0, T),      # gaits(): FORWARD  -> robot.walk(0, ...)
        'b': lambda: walk(1, T),      # gaits(): BACKWARD -> robot.walk(1, ...)
        'l': lambda: turn(True, max(T, 600)),
        'r': lambda: turn(False, max(T, 600)),
        'g': lambda: run(1, T),
        's': lambda: static(home_pose()),
        'k': lambda: static(home_pose(20, 50)),          # Recon::sit()
        'z': lambda: hello(T),
        'p': lambda: push_up(T),
        'u': lambda: up_down(T),
        'q': lambda: front_back(T),
        'w': lambda: body_side(T),
        't': lambda: body_side(T),
        'd': lambda: dance(T),
        'x': lambda: twerk(),
        'm': lambda: moonwalk(max(T, 2000)),
        'n': lambda: static([90 + 90, 90 - 90, 90 - 90, 90 + 90, 90 - 90, 90 + 90, 90 + 90, 90 - 90]),
        'c': lambda: static([90 + 90, 90 - 90, 90 - 90, 90 + 90, 90, 90, 90 + 90, 90 - 90]),
        'i': lambda: sit_wiggle(),
    }
    return table[cmd]() if cmd in table else None


class ReconGait(Node):
    def __init__(self):
        super().__init__('recon_gait')
        self.declare_parameter('period_ms', 500.0)     # `Speed` in the .ino
        self.declare_parameter('rate_hz', 100.0)       # firmware loop = delay(10)
        self.declare_parameter('slew_dps', 600.0)      # MG90S: 0.1 s / 60 deg
        self.declare_parameter('demo', False)
        self.declare_parameter('demo_loop', False)
        self.declare_parameter('start_command', 's')

        self.T = float(self.get_parameter('period_ms').value)
        rate = float(self.get_parameter('rate_hz').value)
        self.slew = float(self.get_parameter('slew_dps').value)
        self.dt = 1.0 / rate

        self.angles = home_pose() + [90.0]      # current servo angles (0..180), index 8 = neck
        self.angles[:8] = [90.0] * 8            # start from the calibration pose
        self.gait = None
        self.cmd = ''
        self.neck_scan = False
        self.neck_t0 = 0.0
        self.vel_driven = False

        self.pub = self.create_publisher(Float64MultiArray, '/recon/joint_commands', 10)
        self.create_subscription(String, '/recon/command', self.on_command, 10)
        self.create_subscription(Twist, '/cmd_vel', self.on_twist, 10)
        self.create_timer(self.dt, self.tick)

        # demo
        # (command, seconds).  Walk ~9 cm/s, turn ~58 deg/s with the default 500 ms period.
        self.demo_seq = [('s', 3.0), ('f', 8.0), ('s', 1.0),            # stand, walk forward ~70 cm
                         ('l', 1.6), ('f', 5.0), ('s', 1.0),            # turn left 90, walk
                         ('r', 3.1), ('f', 5.0), ('s', 1.0),            # turn right 180, walk back
                         ('b', 4.0), ('s', 1.0),                        # walk backwards
                         ('y', 0.1), ('s', 5.0), ('y', 0.1),            # head scan (neck servo)
                         ('k', 2.0), ('s', 2.0),                        # sit / stand
                         ('p', 4.0), ('s', 1.5),                        # push-ups
                         ('u', 4.0), ('s', 1.5),                        # up-down
                         ('z', 4.0), ('s', 2.0)]                        # hello
        self.demo_idx = -1
        self.demo_end = None
        self.demo_on = bool(self.get_parameter('demo').value)

        self.set_command(str(self.get_parameter('start_command').value) or 's')
        self.get_logger().info(
            f'recon_gait ready: period={self.T:.0f} ms, demo={self.demo_on}. '
            "Publish a letter on /recon/command (f b l r s g k z p u q w d ...) or use /cmd_vel.")

    # ---------------------------------------------------------------- time helpers
    def now_ms(self):
        return self.get_clock().now().nanoseconds / 1e6

    def now_s(self):
        return self.get_clock().now().nanoseconds / 1e9

    # ---------------------------------------------------------------- commands
    def set_command(self, c):
        c = c.strip()[:1]
        if not c:
            return
        if c == 'y':
            self.neck_scan = not self.neck_scan
            self.neck_t0 = self.now_s()
            self.get_logger().info(f'neck scan {"ON" if self.neck_scan else "OFF"}')
            return
        gait = build_gait(c, self.T)
        if gait is None:
            self.get_logger().warn(f"unknown command '{c}'")
            return
        gait.start(self.now_ms(), self.angles[:8])
        self.gait, self.cmd = gait, c
        self.get_logger().info(f'gait -> {c}')

    def on_command(self, msg):
        self.demo_on = False          # a manual command cancels the demo
        self.vel_driven = False
        self.set_command(msg.data)

    def on_twist(self, msg):
        self.demo_on = False
        lin, ang = msg.linear.x, msg.angular.z
        if abs(ang) > 0.05 and abs(ang) >= abs(lin):
            c = 'l' if ang > 0 else 'r'
        elif lin > 0.01:
            c = 'f'
        elif lin < -0.01:
            c = 'b'
        else:
            c = 's' if self.vel_driven else self.cmd
        self.vel_driven = True
        if c != self.cmd:
            self.set_command(c)

    # ---------------------------------------------------------------- demo
    def run_demo(self):
        t = self.now_s()
        if self.demo_end is None or t >= self.demo_end:
            self.demo_idx += 1
            if self.demo_idx >= len(self.demo_seq):
                if self.get_parameter('demo_loop').value:
                    self.demo_idx = 0
                else:
                    self.demo_on = False
                    self.get_logger().info('demo finished - waiting for commands')
                    return
            c, dur = self.demo_seq[self.demo_idx]
            self.demo_end = t + dur
            self.set_command(c)

    # ---------------------------------------------------------------- main loop
    def tick(self):
        if self.demo_on:
            self.run_demo()

        targets = list(self.angles[:8])
        if self.gait is not None:
            targets = self.gait.update(self.now_ms())

        if self.neck_scan:
            neck = 90.0 + 45.0 * math.sin(2 * math.pi * (self.now_s() - self.neck_t0) / 4.0)
        else:
            neck = 90.0
        targets = targets + [neck]

        step = self.slew * self.dt            # servo speed limit
        for i in range(9):
            tgt = min(180.0, max(0.0, targets[i]))
            d = tgt - self.angles[i]
            self.angles[i] += max(-step, min(step, d))

        msg = Float64MultiArray()
        msg.data = [math.radians(a - 90.0) for a in self.angles]
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = ReconGait()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
