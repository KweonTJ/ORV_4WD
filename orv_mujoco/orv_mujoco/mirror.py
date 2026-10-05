"""Read-only visual twin of the bridged vehicle telemetry. No motor publishers."""
import argparse
import math
import signal
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState

from .model import World, WHEELS, settings
from .view import open_viewer


class Mirror(Node):
    def __init__(self, world, prefix='/orv', viewer=True):
        super().__init__('orv_mujoco_mirror')
        self.world, self.viewer = world, open_viewer(world) if viewer else None
        self.position, self.quaternion, self.joints = np.zeros(3), np.array([1., 0, 0, 0]), np.zeros(4)
        self.odom_seen = self.joints_seen = 0.
        self.was_fresh = None
        self.create_subscription(Odometry, prefix+'/odom', self.on_odom, 10)
        self.create_subscription(JointState, prefix+'/joint_states', self.on_joints, 10)
        self.create_timer(1/30, self.update)
        self.get_logger().info('Read-only mirror: odometry is encoder dead reckoning, not measured absolute pose.')

    def on_odom(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        values = [p.x, p.y, p.z, q.w, q.x, q.y, q.z]
        if not all(math.isfinite(v) for v in values): return
        norm = np.linalg.norm(values[3:])
        if norm < 1e-6: return
        self.position, self.quaternion = np.array(values[:3]), np.array(values[3:])/norm
        self.odom_seen = time.monotonic()

    def on_joints(self, msg):
        lookup = dict(zip(msg.name, msg.position))
        names = [name+'_wheel_joint' for name in WHEELS]
        if not all(name in lookup and math.isfinite(lookup[name]) for name in names): return
        self.joints = np.array([lookup[name] for name in names])
        self.joints_seen = time.monotonic()

    def update(self):
        now = time.monotonic()
        fresh = now-self.odom_seen < .5 and now-self.joints_seen < .5
        if fresh != self.was_fresh:
            self.get_logger().info('Vehicle telemetry live' if fresh else 'Waiting/stale vehicle telemetry; model frozen')
            self.was_fresh = fresh
        if fresh: self.world.mirror(self.position, self.quaternion, self.joints)
        if self.viewer:
            if self.viewer.is_running(): self.viewer.sync()
            else: self.context.try_shutdown()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--physics-config')
    parser.add_argument('--prefix', default='/orv')
    parser.add_argument('--headless', action='store_true')
    args, ros_args = parser.parse_known_args()
    world = World(settings(args.physics_config))
    rclpy.init(args=ros_args)
    node = None
    try:
        node = Mirror(world, args.prefix.rstrip('/'), not args.headless)
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if node:
            if node.viewer: node.viewer.close()
            node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
