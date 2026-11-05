#!/usr/bin/env python3
"""
Odometry for the simulated Recon (needed by SLAM).

A walking robot has no wheel encoders, so for the simulation the pose is taken from Gazebo
(/gazebo/model_states, "ground truth") and split into the usual 2D-navigation frames:

    odom --(x, y, yaw)--> base_footprint --(z, roll, pitch)--> base_link

The tilt of the body while walking therefore sits below base_footprint and does not bend the map.
Optional Gaussian noise on the planar motion (parameter `noise`) makes the odometry drift like a
real dead-reckoned one, so that SLAM has something to correct.
"""
import math
import random

import rclpy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


def quat_to_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def quat_mul(a, b):  # (x, y, z, w)
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


class ReconOdom(Node):
    def __init__(self):
        super().__init__('recon_odom')
        self.declare_parameter('model_name', 'recon')
        self.declare_parameter('noise', 0.0)       # fraction of the travelled distance / turned angle
        self.name = self.get_parameter('model_name').value
        self.noise = float(self.get_parameter('noise').value)
        self.br = TransformBroadcaster(self)
        self.pub = self.create_publisher(Odometry, '/odom', 10)
        self.create_subscription(ModelStates, '/gazebo/model_states', self.cb, 10)
        self.prev = None      # previous ground-truth (x, y, yaw)
        self.est = [0.0, 0.0, 0.0]  # estimated odom pose
        self.origin = None

    def cb(self, msg):
        if self.name not in msg.name:
            return
        p = msg.pose[msg.name.index(self.name)]
        yaw = quat_to_yaw(p.orientation)
        x, y = p.position.x, p.position.y
        if self.origin is None:
            self.origin = (x, y, yaw)
        if self.prev is not None:
            dx, dy = x - self.prev[0], y - self.prev[1]
            dyaw = math.atan2(math.sin(yaw - self.prev[2]), math.cos(yaw - self.prev[2]))
            # motion expressed in the previous robot frame, then re-applied to the estimate
            c, s = math.cos(self.prev[2]), math.sin(self.prev[2])
            fx, fy = c * dx + s * dy, -s * dx + c * dy
            if self.noise > 0.0:
                dist = math.hypot(fx, fy)
                fx += random.gauss(0.0, self.noise * dist)
                fy += random.gauss(0.0, self.noise * dist)
                dyaw += random.gauss(0.0, self.noise * abs(dyaw) + 0.1 * self.noise * dist)
            ce, se = math.cos(self.est[2]), math.sin(self.est[2])
            self.est[0] += ce * fx - se * fy
            self.est[1] += se * fx + ce * fy
            self.est[2] += dyaw
        self.prev = (x, y, yaw)

        now = self.get_clock().now().to_msg()
        ex, ey, eyaw = self.est
        qz, qw = math.sin(eyaw / 2.0), math.cos(eyaw / 2.0)

        t1 = TransformStamped()
        t1.header.stamp = now
        t1.header.frame_id = 'odom'
        t1.child_frame_id = 'base_footprint'
        t1.transform.translation.x, t1.transform.translation.y = ex, ey
        t1.transform.rotation.z, t1.transform.rotation.w = qz, qw

        # tilt of the body relative to the planar frame: q_rel = q_yaw^-1 * q_full
        qy = (0.0, 0.0, -math.sin(yaw / 2.0), math.cos(yaw / 2.0))
        qf = (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
        qr = quat_mul(qy, qf)
        t2 = TransformStamped()
        t2.header.stamp = now
        t2.header.frame_id = 'base_footprint'
        t2.child_frame_id = 'base_link'
        t2.transform.translation.z = p.position.z
        t2.transform.rotation.x, t2.transform.rotation.y = qr[0], qr[1]
        t2.transform.rotation.z, t2.transform.rotation.w = qr[2], qr[3]
        self.br.sendTransform([t1, t2])

        od = Odometry()
        od.header.stamp = now
        od.header.frame_id = 'odom'
        od.child_frame_id = 'base_footprint'
        od.pose.pose.position.x, od.pose.pose.position.y = ex, ey
        od.pose.pose.orientation.z, od.pose.pose.orientation.w = qz, qw
        self.pub.publish(od)


def main():
    rclpy.init()
    node = ReconOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
