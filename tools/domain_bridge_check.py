#!/usr/bin/env python3
"""Check the real domain_bridge with a mock vehicle on two explicit test domains."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import SetBool
from tf2_msgs.msg import TFMessage
from orv_4wd.domain_config import default_domain_config, domain_id, load_domains


def stop(process):
    if process.poll() is None:
        # launch forwards SIGINT to its children; signaling the entire group
        # also can send a second SIGINT while rclcpp is still shutting down.
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=3)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--vehicle-domain', required=True, type=domain_id)
    parser.add_argument('--operator-domain', required=True, type=domain_id)
    args = parser.parse_args()
    domains = (args.vehicle_domain, args.operator_domain)
    if domains[0] == domains[1] or set(domains) & set(load_domains(default_domain_config())):
        parser.error('Choose two distinct test domains, separate from both configured domains')
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    directory = Path(tempfile.mkdtemp(prefix='orv_bridge_check_'))
    environment = dict(os.environ, ROS_LOG_DIR=str(directory / 'ros'))
    processes, logs, contexts, nodes, executors = [], [], [], [], []

    def launch(filename, arguments):
        log = (directory / (filename + '.log')).open('w')
        logs.append(log)
        process = subprocess.Popen(
            ['ros2', 'launch', 'orv_bringup', filename, *arguments], env=environment,
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True,
        )
        processes.append(process)
        return process

    def wait(predicate, label, timeout=15, publish=None):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if publish:
                publish()
            for executor in executors:
                executor.spin_once(timeout_sec=0.01)
            if predicate():
                return
            time.sleep(0.02)
        raise AssertionError('Timed out: ' + label)

    try:
        for name, domain in zip(('vehicle', 'operator'), domains):
            context = Context()
            rclpy.init(context=context, domain_id=domain)
            contexts.append(context)
            node = rclpy.create_node('orv_bridge_probe_' + name, context=context)
            nodes.append(node)
            executor = SingleThreadedExecutor(context=context)
            executor.add_node(node)
            executors.append(executor)
        vehicle, operator = nodes
        received = {}

        def record(key):
            return lambda message: received.__setitem__(key, message)

        vehicle.create_subscription(String, '/orv/status', record('vehicle_status'), 10)
        vehicle.create_subscription(Twist, '/cmd_vel', record('command'), 1)
        for topic, msg_type in (('odom', Odometry), ('joint_states', JointState),
                                ('status', String), ('tf', TFMessage)):
            operator.create_subscription(msg_type, '/orv/' + topic, record(topic), 10)
        operator.create_subscription(Odometry, '/odom', record('unwanted_raw_odom'), 10)
        publisher = operator.create_publisher(Twist, '/orv/cmd_vel', 1)
        arm = vehicle.create_client(SetBool, '/orv/arm')
        launch('bringup.launch.py', ['mode:=mock', 'api_enabled:=false',
               f'vehicle_domain_id:={domains[0]}', f'profile_path:={directory}/profile.yaml'])
        bridge = launch('domain_bridge.launch.py', [f'vehicle_domain_id:={domains[0]}',
                        f'operator_domain_id:={domains[1]}'])

        def vehicle_status():
            return json.loads(received['vehicle_status'].data) if 'vehicle_status' in received else {}

        wait(lambda: vehicle_status().get('ready') and 'status' in received,
             'vehicle ready and status bridged')
        assert vehicle_status()['mode'] == 'mock' and not vehicle_status()['armed']
        wait(lambda: all(topic in received for topic in ('odom', 'joint_states', 'tf')),
             'telemetry and dynamic TF')
        wait(lambda: publisher.get_subscription_count() > 0, 'command bridge discovery')

        # Subscribe only after the bridge has published/cached these startup samples.
        wait(lambda: operator.count_publishers('/orv/tf_static') > 0 and
             operator.count_publishers('/orv/robot_description') > 0, 'static bridge endpoints')
        time.sleep(0.5)
        latched = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        operator.create_subscription(TFMessage, '/orv/tf_static', record('static'), latched)
        operator.create_subscription(String, '/orv/robot_description', record('description'), latched)
        wait(lambda: 'static' in received and 'description' in received, 'late static subscriptions')
        assert '<robot' in received['description'].data
        assert len(received['static'].transforms) >= 5
        assert len(received['joint_states'].position) == 4
        print('PASS: status, odom, joint states, TF and late-joining URDF/static TF')

        assert arm.wait_for_service(timeout_sec=5)
        command = Twist()
        command.linear.x = 0.05
        send = lambda: publisher.publish(command)
        future = arm.call_async(SetBool.Request(data=True))
        wait(future.done, 'arm service', publish=send)
        assert future.result().success, future.result().message
        wait(lambda: received.get('command') is not None and
             received['odom'].pose.pose.position.x > 0.03, 'bridged command motion', publish=send)
        assert received['command'].linear.x == 0.05
        assert 'unwanted_raw_odom' not in received
        print('PASS: operator /orv/cmd_vel -> vehicle /cmd_vel -> encoder odometry; raw topics isolated')

        stop(bridge)
        # Continue publishing from the PC: vehicle must still disarm without its bridge.
        wait(lambda: not vehicle_status().get('armed', True), 'bridge loss watchdog',
             timeout=3, publish=send)
        print('PASS: bridge loss disarms the mock vehicle while PC commands continue')
    finally:
        for process in reversed(processes):
            stop(process)
        for executor in executors:
            executor.shutdown()
        for node in nodes:
            node.destroy_node()
        for context in contexts:
            context.shutdown()
        for log in logs:
            log.close()
        print('Logs:', directory)
    for log in logs:
        output = Path(log.name).read_text()
        assert 'Traceback' not in output and '[ERROR]' not in output, output[-6000:]
    print('PASS: clean shutdown')


if __name__ == '__main__':
    main()
