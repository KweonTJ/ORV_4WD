import argparse
import json
import math
import signal
import time

import mujoco
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import Trigger

from orv_4wd.node import Driver
from .model import World, settings
from .transport import MujocoTransport
from .view import open_viewer


class Simulator(Driver):
    def __init__(self, world, viewer=False):
        self.world = world
        self.viewer = None
        self.last_scene = self.last_metrics = 0.
        self.transport_instance = None
        def factory():
            self.transport_instance = MujocoTransport(world)
            return self.transport_instance
        super().__init__(simulation_factory=factory)
        if self.core.backend != 'mujoco':
            raise ValueError('Simulator requires mode:=mujoco')
        self.truth_pub = self.create_publisher(Odometry, '/orv/sim/ground_truth', 10)
        self.metrics_pub = self.create_publisher(String, '/orv/sim/metrics', 10)
        self.reset_service = self.create_service(Trigger, '/orv/sim/reset', self.on_reset_world)
        if viewer: self.viewer = open_viewer(world)
        self.get_logger().info('MuJoCo physics; inertia, friction and motor torque are provisional. No USB device is opened.')

    def on_reset_world(self, _, response):
        if self.core.armed or (self.core.telemetry and self.core.telemetry.armed):
            response.success, response.message = False, 'Stop and disarm before resetting the simulation'
        elif not self.core.ready:
            response.success, response.message = False, 'Wait for simulator readiness'
        else:
            self.transport_instance.reset_world()
            self.core.prev_ticks = [0]*4
            self.core.tick_origin = [0]*4
            self.core.prev_uptime = round(self.world.data.time*1000)
            self.core.positions = [0.]*4
            self.core.x = self.core.y = self.core.yaw = self.core.linear = self.core.angular = 0.
            response.success, response.message = True, 'Physics pose, wheel encoders and odometry reset'
        return response

    def tick(self):
        super().tick()
        now = time.monotonic()
        if now-self.last_metrics >= .05 and self.transport_instance:
            self.last_metrics = now
            pos, quat = self.world.pose()
            msg = Odometry()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id, msg.child_frame_id = 'sim_world', 'base_link'
            msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = map(float, pos)
            msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z = map(float, quat)
            velocity = np.zeros(6)
            mujoco.mj_objectVelocity(self.world.model, self.world.data, mujoco.mjtObj.mjOBJ_BODY,
                                    self.world.model.body('base_link').id, velocity, 1)
            msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z = map(float, velocity[3:])
            msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z = map(float, velocity[:3])
            self.truth_pub.publish(msg)
            age = max(.001, now-self.transport_instance.wall_start)
            metrics = {'simulation_time': self.world.data.time, 'contacts': int(self.world.data.ncon),
                       'real_time_factor': self.world.data.time/age,
                       'dropped_wall_time': self.transport_instance.dropped_wall_time,
                       'odometry_position_error_m': math.hypot(self.core.x-pos[0], self.core.y-pos[1]),
                       'total_mass_kg': self.world.cfg['total_mass'], 'parameters_calibrated': False}
            self.metrics_pub.publish(String(data=json.dumps(metrics)))
        if self.viewer and now-self.last_scene >= 1/30:
            self.last_scene = now
            if self.viewer.is_running(): self.viewer.sync()
            else:
                self.core.stop('viewer closed')
                self.context.try_shutdown()

    def close(self):
        super().close()
        if self.viewer:
            self.viewer.close()
            self.viewer = None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--physics-config')
    parser.add_argument('--viewer', action='store_true')
    parser.add_argument('--terrain', choices=('flat', 'course'), default=None)
    args, ros_args = parser.parse_known_args()
    cfg = settings(args.physics_config)
    if args.terrain: cfg['terrain'] = args.terrain
    world = World(cfg)
    rclpy.init(args=ros_args)
    node = None
    try:
        node = Simulator(world, args.viewer)
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if node: node.close(); node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
