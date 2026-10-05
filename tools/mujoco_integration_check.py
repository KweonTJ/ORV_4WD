#!/usr/bin/env python3
"""Headless MuJoCo -> ROS/domain_bridge and HTTP end-to-end check on private domains."""
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import tempfile
import time
from urllib.request import Request, urlopen
from urllib.error import URLError

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import Trigger, SetBool
from ament_index_python.packages import get_package_share_directory
from orv_4wd.domain_config import default_domain_config, load_domains
from orv_mujoco.model import World
from orv_mujoco.mirror import Mirror


def main():
    # Test domains are supplied, never fall back to the operator's ambient domain.
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--simulation-domain', type=int, required=True)
    parser.add_argument('--operator-domain', type=int, required=True)
    args = parser.parse_args()
    domains = (args.simulation_domain, args.operator_domain)
    if len(set(domains)) != 2 or set(domains) & set(load_domains(default_domain_config())):
        parser.error('Choose two distinct unused test domains, outside the real vehicle/operator domains')
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0)); port = sock.getsockname()[1]
    directory = Path(tempfile.mkdtemp(prefix='orv_mujoco_check_'))
    env = dict(os.environ, ROS_DOMAIN_ID=str(domains[1]), ROS_LOCALHOST_ONLY='1', ROS_LOG_DIR=str(directory/'ros'))
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    logs, processes = [], []
    def launch(package, file, arguments):
        log = (directory/(package+'.log')).open('w'); logs.append(log)
        proc = subprocess.Popen(['ros2', 'launch', package, file, *arguments], env=env,
                 stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        processes.append(proc)
        return proc
    node = mirror = car_node = car_context = executor = car_executor = None
    def api(path='/api/status', data=None):
        request = Request(f'http://127.0.0.1:{port}'+path,
                  data=None if data is None else json.dumps(data).encode(), headers={'Content-Type':'application/json'})
        with urlopen(request, timeout=1) as response: return json.load(response)
    def wait(predicate, timeout=10):
        deadline = time.monotonic()+timeout
        while time.monotonic()<deadline:
            if executor: executor.spin_once(timeout_sec=.01)
            if car_executor: car_executor.spin_once(timeout_sec=.01)
            try:
                if predicate(): return
            except (URLError, ConnectionError): pass
            time.sleep(.02)
        raise AssertionError('MuJoCo integration timed out')
    try:
        launch('orv_mujoco', 'simulation.launch.py', ['viewer:=false', 'gui:=false',
               f'domain_id:={domains[0]}', f'api_port:={port}'])
        launch('orv_bringup', 'domain_bridge.launch.py', [f'vehicle_domain_id:={domains[0]}',
               f'operator_domain_id:={domains[1]}',
               'config:='+str(Path(get_package_share_directory('orv_mujoco'))/'config/domain_bridge.yaml')])
        wait(lambda: api().get('ready'), 20)
        assert api()['mode'] == 'mujoco' and not api()['armed']
        rclpy.init(domain_id=domains[1])
        node = rclpy.create_node('orv_mujoco_probe')
        mirror = Mirror(World(), prefix='/orv_sim', viewer=False)
        executor = SingleThreadedExecutor()
        executor.add_node(node); executor.add_node(mirror)
        car_context = Context(); rclpy.init(context=car_context, domain_id=domains[0])
        car_node = rclpy.create_node('orv_physics_probe', context=car_context)
        car_executor = SingleThreadedExecutor(context=car_context); car_executor.add_node(car_node)
        received = {}
        node.create_subscription(Odometry, '/orv_sim/odom', lambda m: received.__setitem__('odom',m), 10)
        node.create_subscription(Odometry, '/orv_sim/ground_truth', lambda m: received.__setitem__('truth',m), 10)
        node.create_subscription(String, '/orv_sim/status', lambda m: received.__setitem__('status',json.loads(m.data)), 10)
        wait(lambda: 'status' in received and 'odom' in received)
        assert api('/api/arm', {})['ok']
        for _ in range(35):
            assert api('/api/twist', {'linear':.06,'angular':0})['ok']
            executor.spin_once(timeout_sec=.01)
            time.sleep(.09)
        wait(lambda: received['odom'].pose.pose.position.x > .1)
        assert received['status']['mode'] == 'mujoco'
        wait(lambda: 'truth' in received and received['truth'].pose.pose.position.x > .1 and
             mirror.world.pose()[0][0] > .1)
        wait(lambda: not api()['armed'], 3)
        wait(lambda: max(abs(v) for v in api()['rpm']) < 2, 6)
        assert api('/api/config', {'kp':[1.5]*4,'max_pwm':[90]*4})['ok']
        wait(lambda: api()['config_applied'] and api()['config']['kp']==[1.5]*4)
        reset = car_node.create_client(Trigger, '/orv/sim/reset')
        wait(lambda: reset.service_is_ready())
        result = reset.call_async(Trigger.Request()); wait(result.done)
        assert result.result().success
        wait(lambda: abs(received['truth'].pose.pose.position.x) < .005 and
             abs(received['odom'].pose.pose.position.x) < .005)
        arm = car_node.create_client(SetBool, '/orv/arm')
        command = node.create_publisher(Twist, '/orv_sim/cmd_vel', 1)
        active = [True]
        msg = Twist(); msg.linear.x = .05
        timer = node.create_timer(.05, lambda: command.publish(msg) if active[0] else None)
        wait(lambda: arm.service_is_ready() and command.get_subscription_count() > 0)
        result = arm.call_async(SetBool.Request(data=True)); wait(result.done)
        assert result.result().success
        wait(lambda: received['truth'].pose.pose.position.x > .08)
        active[0] = False
        wait(lambda: not api()['armed'], 3)
        assert api()['ready']  # world reset must not break the telemetry handshake
        print('PASS: GUI and bridged ROS cmd_vel -> torque/contact physics -> encoder/ground-truth topics')
        print('PASS: read-only telemetry mirror, watchdog stop, PID tuning and physical/encoder reset')
    finally:
        try: api('/api/stop', {})
        except Exception: pass
        if executor: executor.shutdown()
        if car_executor: car_executor.shutdown()
        if node: node.destroy_node()
        if mirror: mirror.destroy_node()
        if car_node: car_node.destroy_node()
        if car_context: car_context.shutdown()
        if rclpy.ok(): rclpy.shutdown()
        for process in reversed(processes):
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try: process.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGTERM);process.wait(timeout=3)
        for log in logs: log.close()
        print('Logs:', directory)
    for log in logs:
        contents = Path(log.name).read_text()
        assert '[ERROR]' not in contents and 'Traceback' not in contents, contents[-5000:]
    print('PASS: clean shutdown')


if __name__ == '__main__': main()
