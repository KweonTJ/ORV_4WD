"""ROS 2 Humble adapter for the independently tested control core."""
from dataclasses import fields
import json
import math
import os
import signal
from pathlib import Path
import tempfile
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from tf2_ros import TransformBroadcaster
import yaml

from .api import Api
from .config import Config
from .core import Controller
from .transport import MockTransport, SerialTransport

JOINT_NAMES = ['front_left_wheel_joint', 'rear_left_wheel_joint',
               'front_right_wheel_joint', 'rear_right_wheel_joint']


class Driver(Node):
    def __init__(self):
        super().__init__('orv_driver')
        self.declare_parameter('mode', 'mock')
        self.declare_parameter('port', '/dev/serial/by-id/SET_YOUR_UNO_DEVICE')
        self.declare_parameter('api_enabled', True)
        self.declare_parameter('api_bind', '127.0.0.1')
        self.declare_parameter('api_port', 8765)
        self.declare_parameter('profile_path', str(Path.home()/'.config/orv_4wd/profile.yaml'))
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', True)
        defaults = Config()
        self.config_names = {f.name for f in fields(Config)}
        for key, value in defaults.as_dict().items():
            self.declare_parameter(key, value)
        cfg = Config(**{key: self.get_parameter(key).value for key in self.config_names}).validate()
        mode = self.get_parameter('mode').value
        if mode not in ('mock', 'hardware'):
            raise ValueError('mode must be mock or hardware')
        port = self.get_parameter('port').value
        factory = MockTransport if mode == 'mock' else lambda: SerialTransport(port)
        self.core = Controller(factory, cfg, mock=mode == 'mock')
        self.add_on_set_parameters_callback(self.parameters_changed)
        self.status_pub = self.create_publisher(String, 'orv/status', 10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf = TransformBroadcaster(self)
        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.on_twist, 1)
        self.service_handles = [
            self.create_service(SetBool, 'orv/arm', self.on_arm),
            self.create_service(Trigger, 'orv/stop', self.on_stop),
            self.create_service(Trigger, 'orv/reset_odometry', self.on_reset),
            self.create_service(Trigger, 'orv/save_config', self.on_save),
        ]
        self.api = None
        if self.get_parameter('api_enabled').value:
            self.api = Api(self.get_parameter('api_bind').value,
                           self.get_parameter('api_port').value, os.environ.get('ORV_API_TOKEN', ''))
        self.last_publish = 0.0
        self.last_reason = ''
        self.last_device_stamp = None
        self.timer = self.create_timer(0.01, self.tick)
        self.get_logger().info(f'ORV 4WD mode={mode}; starts disarmed. M1..M4 configuration is provisional.')

    def parameters_changed(self, parameters):
        try:
            patch = {}
            for p in parameters:
                if p.name not in self.config_names:
                    raise ValueError(f'{p.name} is startup-only; restart bringup to change it')
                patch[p.name] = p.value
            self.core.configure(patch)
            return SetParametersResult(successful=True, reason='validated; device acknowledgement pending')
        except (ValueError, TypeError) as exc:
            return SetParametersResult(successful=False, reason=str(exc))

    def on_twist(self, msg):
        if not self.core.armed or self.core.source != 'ros':
            return
        try:
            values = [msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y, msg.angular.z]
            if not all(math.isfinite(v) for v in values) or any(abs(v) > 1e-9 for v in (values[1], values[2], values[3], values[4])):
                raise ValueError('differential drive accepts only linear.x and angular.z')
            self.core.twist(msg.linear.x, msg.angular.z, 'ros')
        except (ValueError, OSError) as exc:
            self.core.stop(str(exc))

    def on_arm(self, request, response):
        try:
            self.core.arm('ros') if request.data else self.core.stop()
            response.success, response.message = True, 'request sent; inspect /orv/status for device confirmation'
        except (ValueError, OSError) as exc:
            response.success, response.message = False, str(exc)
        return response

    def on_stop(self, _, response):
        self.core.stop('ROS stop')
        response.success, response.message = True, 'disarmed'
        return response

    def on_reset(self, _, response):
        try:
            self.core.reset_odometry()
            response.success, response.message = True, 'odometry and relative encoder origin reset'
        except ValueError as exc:
            response.success, response.message = False, str(exc)
        return response

    def save(self):
        if not self.core.ready or self.core.pending or self.core.armed or self.core.telemetry.armed:
            raise ValueError('save requires an acknowledged configuration and a disarmed device')
        path = Path(self.get_parameter('profile_path').value).expanduser()
        path.parent.mkdir(parents=True, exist_ok=True)
        data = {'orv_driver': {'ros__parameters': self.core.cfg.as_dict()}}
        with tempfile.NamedTemporaryFile('w', dir=path.parent, delete=False) as f:
            yaml.safe_dump(data, f, sort_keys=False)
            temporary = f.name
        os.replace(temporary, path)
        return str(path)

    def on_save(self, _, response):
        try:
            response.message = self.save()
            response.success = True
        except (ValueError, OSError) as exc:
            response.success, response.message = False, str(exc)
        return response

    def api_command(self, path, data):
        if path == '/api/arm':
            self.core.arm('gui')
        elif path == '/api/stop':
            self.core.stop('GUI stop')
        elif path == '/api/wheels':
            self.core.wheels(data['values'], data['mode'], 'gui')
        elif path == '/api/twist':
            self.core.twist(float(data['linear']), float(data['angular']), 'gui')
        elif path == '/api/config':
            self.core.cfg.updated(data)  # check unknown keys before creating ROS parameters
            result = self.set_parameters_atomically([Parameter(k, value=v) for k, v in data.items()])
            if not result.successful:
                raise ValueError(result.reason)
        elif path == '/api/reset':
            self.core.reset_odometry()
        elif path == '/api/save':
            return {'ok': True, 'path': self.save()}
        return {'ok': True, 'note': 'request accepted; inspect status for device acknowledgement'}

    def tick(self):
        if self.api:
            self.api.drain(self.api_command)
        self.core.step()
        now = time.monotonic()
        if now-self.last_publish < 0.05:
            return
        self.last_publish = now
        status = self.core.status()
        self.status_pub.publish(String(data=json.dumps(status, allow_nan=False)))
        if self.api:
            self.api.publish(status)
        if self.core.reason != self.last_reason:
            self.last_reason = self.core.reason
            self.get_logger().info(self.last_reason)
        state = self.core.telemetry
        if not self.core.ready or not state or state.uptime_ms == self.last_device_stamp:
            return
        self.last_device_stamp = state.uptime_ms
        stamp = self.get_clock().now().to_msg()
        joints = JointState()
        joints.header.stamp = stamp
        joints.name = JOINT_NAMES
        joints.position = self.core.positions[:]
        joints.velocity = [state.rpm[m-1]*2*math.pi/60 for m in self.core.cfg.motor_map]
        self.joint_pub.publish(joints)
        if self.core.cfg.wheel_radius <= 0 or self.core.cfg.track_width <= 0:
            return
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.get_parameter('odom_frame').value
        odom.child_frame_id = self.get_parameter('base_frame').value
        odom.pose.pose.position.x, odom.pose.pose.position.y = self.core.x, self.core.y
        odom.pose.pose.orientation.z = math.sin(self.core.yaw/2)
        odom.pose.pose.orientation.w = math.cos(self.core.yaw/2)
        odom.twist.twist.linear.x, odom.twist.twist.angular.z = self.core.linear, self.core.angular
        # Initial conservative covariance; skid steering requires physical calibration.
        for i, value in enumerate([0.05, 0.2, 1e6, 1e6, 1e6, 0.3]):
            odom.pose.covariance[i*7] = value
            odom.twist.covariance[i*7] = value
        self.odom_pub.publish(odom)
        if self.get_parameter('publish_tf').value:
            transform = TransformStamped()
            transform.header, transform.child_frame_id = odom.header, odom.child_frame_id
            transform.transform.translation.x = self.core.x
            transform.transform.translation.y = self.core.y
            transform.transform.rotation = odom.pose.pose.orientation
            self.tf.sendTransform(transform)

    def close(self):
        self.core.close()
        if self.api:
            self.api.close()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = Driver()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Ctrl-C reaches both launch and children; launch may forward it again.
        # Finish STOP and close the HTTP server even when that second SIGINT arrives.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if node:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
