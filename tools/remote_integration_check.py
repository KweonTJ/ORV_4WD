#!/usr/bin/env python3
"""Verify manual encoder telemetry through the actual ROS Driver, using mock I/O."""
import argparse
import json
import os
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from tf2_msgs.msg import TFMessage
from orv_4wd.domain_config import default_domain_config, domain_id, load_domains
from orv_4wd.node import Driver
from orv_4wd.transport import MockTransport


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--domain', required=True, type=domain_id)
    args = parser.parse_args()
    if args.domain in load_domains(default_domain_config()):
        parser.error('Use a separate, unused test domain')
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    rclpy.init(domain_id=args.domain, args=['--ros-args', '-p', 'mode:=mock',
                '-p', 'api_enabled:=false', '-p', 'wheel_radius:=0.044', '-p', 'track_width:=0.210'])
    executor = SingleThreadedExecutor()
    driver = Driver()
    probe = rclpy.create_node('orv_manual_telemetry_probe')
    executor.add_node(driver)
    executor.add_node(probe)
    received = {}
    for topic, msg_type in (('orv/status', String), ('odom', Odometry),
                            ('joint_states', JointState), ('tf', TFMessage)):
        probe.create_subscription(msg_type, topic, lambda msg, key=topic: received.__setitem__(key, msg), 10)
    command = probe.create_publisher(Twist, 'cmd_vel', 1)
    arm = probe.create_client(SetBool, 'orv/arm')
    stop = probe.create_client(Trigger, 'orv/stop')

    def spin_until(predicate, timeout=5, inject=None):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if inject: inject()
            executor.spin_once(timeout_sec=0.01)
            if predicate(): return
        raise AssertionError('ROS manual integration condition timed out')

    try:
        spin_until(lambda: driver.core.ready and all(t in received for t in ('odom', 'joint_states', 'tf')))
        device = driver.core.transport
        assert isinstance(device, MockTransport) and device.remote_enabled
        device.remote_connected = device.remote_active = device.armed = True
        device.targets = [10]*4

        def radio():
            device.last_command = time.monotonic()
            command.publish(Twist())  # PC commands cannot overwrite manual control

        spin_until(lambda: received['odom'].pose.pose.position.x > 0.02, inject=radio)
        status = json.loads(received['orv/status'].data)
        assert status['source'] == 'remote' and status['armed'] and not status['arm_requested']
        assert status['target'] == [10]*4 and all(v > 0 for v in received['joint_states'].velocity)
        assert not driver.core.armed
        future = arm.call_async(SetBool.Request(data=True))
        spin_until(future.done, inject=radio)
        assert not future.result().success
        future = stop.call_async(Trigger.Request())
        spin_until(future.done)  # no further simulated radio input
        assert future.result().success
        spin_until(lambda: not json.loads(received['orv/status'].data)['armed'])
        assert not device.remote_active
        print('PASS: manual encoder data -> ROS status/joint_states/odom/TF; host cmd ignored; ARM rejected; STOP works')
    finally:
        driver.close()
        executor.shutdown()
        driver.destroy_node()
        probe.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
