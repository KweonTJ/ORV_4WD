#!/usr/bin/env python3
"""Run a private mock launch, test ROS and HTTP paths, then stop that launch only.

Usage after sourcing ROS and the workspace:
  ROS_DOMAIN_ID=174 python3 src/tools/integration_check.py
"""
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import tempfile
import time
from urllib.request import Request, urlopen
from urllib.error import HTTPError, URLError
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool


def main():
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0)); port = sock.getsockname()[1]
    directory = Path(tempfile.mkdtemp(prefix='orv_integration_'))
    environment = dict(os.environ, ROS_LOG_DIR=str(directory/'ros'), ROS_LOCALHOST_ONLY='1')
    log = (directory/'launch.log').open('w')
    process = subprocess.Popen(['ros2', 'launch', 'orv_bringup', 'bringup.launch.py',
                                f'vehicle_domain_id:={os.environ.get("ROS_DOMAIN_ID", "0")}',
                                'mode:=mock', f'api_port:={port}', f'profile_path:={directory}/profile.yaml'], env=environment,
                               stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    node = None
    def api(path='/api/status', data=None, expected=200):
        req = Request(f'http://127.0.0.1:{port}'+path,
                      data=None if data is None else json.dumps(data).encode(),
                      headers={'Content-Type': 'application/json'})
        try:
            with urlopen(req, timeout=1) as response:
                assert response.status == expected
                return json.load(response)
        except HTTPError as exc:
            assert exc.code == expected
            return json.load(exc)
    def wait(predicate, timeout=5):
        end = time.monotonic()+timeout
        while time.monotonic() < end:
            if node:
                rclpy.spin_once(node, timeout_sec=0.02)
            try:
                if predicate(): return
            except (URLError, OSError): pass
            time.sleep(0.02)
        raise AssertionError('condition timed out')
    try:
        wait(lambda: api().get('ready'), 10)
        assert api()['mode'] == 'mock' and not api()['armed']
        rclpy.init()
        node = rclpy.create_node('orv_integration_probe')
        received = {'odom': [], 'joints': []}
        subs = [node.create_subscription(Odometry, 'odom', lambda m: received['odom'].append(m), 10),
                node.create_subscription(JointState, 'joint_states', lambda m: received['joints'].append(m), 10)]
        pub = node.create_publisher(Twist, 'cmd_vel', 1)
        arm = node.create_client(SetBool, 'orv/arm')
        assert arm.wait_for_service(timeout_sec=5)
        future = arm.call_async(SetBool.Request(data=True))
        wait(future.done)
        assert future.result().success, future.result().message
        wait(lambda: api()['armed'])
        for _ in range(20):
            msg = Twist(); msg.linear.x = 0.05
            pub.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.05)
            time.sleep(0.05)
        assert api()['odometry']['x'] > 0.03
        wait(lambda: bool(received['odom'] and received['joints']))
        assert len(received['joints'][-1].position) == 4
        wait(lambda: not api()['armed'], 2)
        assert api('/api/config', {'encoder_cpr': [0]*4}, expected=400)['ok'] is False
        time.sleep(0.7)
        assert api('/api/config', {'kp': [2.0]*4})['ok']
        wait(lambda: api()['config_applied'] and api()['config']['kp'] == [2.0]*4)
        assert api('/api/arm', {})['ok']
        wait(lambda: api()['armed'])
        for _ in range(10):
            assert api('/api/wheels', {'mode': 'rpm', 'values': [-10, -10, 10, 10]})['ok']
            time.sleep(0.1)
        assert api()['odometry']['yaw'] > 0.1
        assert api('/api/config', {'kp': [3.0]*4}, expected=400)['ok'] is False
        assert api('/api/stop', {})['ok']
        wait(lambda: not api()['armed'])
        result = api('/api/save', {})
        assert result['ok'] and Path(result['path']) == directory/'profile.yaml'
        import yaml
        saved = yaml.safe_load((directory/'profile.yaml').read_text())['orv_driver']['ros__parameters']
        assert saved['kp'] == [2.0]*4
        print('PASS: ROS cmd_vel / arm service / odom / joint_states / command timeout')
        print('PASS: HTTP tuning ACK / GUI motor command / rotation / stop / reject armed tuning / YAML save')
        print('Logs:', directory)
    finally:
        try:
            api('/api/stop', {})
        except Exception: pass
        if node:
            node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try: process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM); process.wait(timeout=3)
        log.close()
        if process.returncode not in (0, None):
            print((directory/'launch.log').read_text()[-6000:])
    output = (directory/'launch.log').read_text()
    assert 'Traceback' not in output and '[ERROR]' not in output, output[-6000:]
    print('PASS: clean shutdown, no launch errors')


if __name__ == '__main__': main()
