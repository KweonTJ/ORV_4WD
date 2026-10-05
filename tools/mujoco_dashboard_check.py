#!/usr/bin/env python3
"""Exercise the integrated Qt/ROS/physics loop without opening hardware or a desktop window."""
import os
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
os.environ.setdefault('MUJOCO_GL', 'egl')
os.environ.setdefault('ROS_LOCALHOST_ONLY', '1')

import argparse
from pathlib import Path
import time

parser = argparse.ArgumentParser()
parser.add_argument('--test-domain', type=int, required=True)
args = parser.parse_args()
from orv_4wd.domain_config import default_domain_config, load_domains, domain_id
from orv_mujoco.model import settings
cfg = settings()
assert domain_id(args.test_domain) not in (*load_domains(default_domain_config()), cfg['simulation_domain_id'])
os.environ['ROS_DOMAIN_ID'] = str(args.test_domain)

import rclpy
from rclpy.executors import SingleThreadedExecutor
from nav_msgs.msg import Odometry
from orv_mujoco.dashboard import Dashboard, QtCore, World, Simulator, application
try:
    from PySide6.QtTest import QTest
except ImportError:
    from PyQt5.QtTest import QTest

root = Path(__file__).resolve().parents[1]
app = application()
rclpy.init(args=['--ros-args', '--params-file', str(root/'orv_mujoco/config/driver.yaml'),
                 '-p', 'api_enabled:=false'])
node = Simulator(World(cfg))
executor = SingleThreadedExecutor(context=node.context)
executor.add_node(node)
window = Dashboard(node)
pump = QtCore.QTimer()
pump.timeout.connect(lambda: executor.spin_once(timeout_sec=0.))
pump.start(5)
odometry = []
subscription = node.create_subscription(Odometry, '/odom', odometry.append, 10)


def until(condition, timeout=3):
    end = time.monotonic()+timeout
    while time.monotonic() < end:
        QTest.qWait(10)
        assert not window.failed, 'dashboard rendering failed'
        if condition():
            return
    raise AssertionError(f'condition timed out: {window.worker.message}; {node.core.status()}')


try:
    window.resize(1366, 800)
    window.show()
    until(lambda: window.arm_button.isEnabled() and window.loaded and not window.scene.frame.isNull())
    assert sum(w.isVisible() for w in app.topLevelWidgets()) == 1
    assert window.table.visualItemRect(window.table.item(3, 4)).bottom() < window.table.viewport().height()
    assert window.wheel_hold.visibleRegion().boundingRect().height() == window.wheel_hold.height()
    assert window.collect() == node.core.cfg.as_dict()
    initial = node.world.pose()[0].copy()
    window.arm_button.click()
    until(lambda: node.core.armed and window.drive_buttons[0].isEnabled())
    QTest.mousePress(window.drive_buttons[0], QtCore.Qt.LeftButton)
    until(lambda: node.world.pose()[0][0]-initial[0] > .02)
    assert max(abs(v) for v in node.core.status()['rpm']) > 1
    assert max(abs(v) for v in node.core.status()['ticks']) > 0
    assert len(odometry) > 5
    QTest.mouseRelease(window.drive_buttons[0], QtCore.Qt.LeftButton)
    until(lambda: not node.core.status()['armed'] and not node.transport_instance.armed)
    assert not node.world.data.ctrl.any()
    until(lambda: window.arm_button.isEnabled() and max(map(abs, node.core.status()['rpm'])) <= 2, timeout=15)
    window.arm_button.click()
    until(lambda: node.core.armed and window.drive_buttons[0].isEnabled())
    QTest.mousePress(window.drive_buttons[0], QtCore.Qt.LeftButton)
    QTest.qWait(200)
    app.sendEvent(window, QtCore.QEvent(QtCore.QEvent.WindowDeactivate))
    until(lambda: not node.core.status()['armed'] and not node.transport_instance.armed)
    assert window.held is None
    window.reset_world()
    assert abs(node.core.x) < .001 and abs(node.world.pose()[0][0]) < .001
    until(lambda: max(map(abs, node.core.status()['rpm'])) == 0)
    window.tabs.setCurrentIndex(1)
    window.config_widgets['kp'][0].setValue(1.25)
    window.apply_config()
    until(lambda: node.core.ready and node.core.cfg.kp[0] == 1.25 and not node.core.pending)
    assert abs(node.transport_instance.kp[0]-1.25) < .001
    QTest.qWait(100)
    # Cell widgets must all be visible, including the fourth motor and direction columns.
    for boxes in window.config_widgets.values():
        for box in boxes:
            assert box.visibleRegion().boundingRect().width() >= 40
    assert window.grab().save('/tmp/orv_dashboard_settings.png')
    window.tabs.setCurrentIndex(0)
    QTest.qWait(100)
    assert window.grab().save('/tmp/orv_dashboard.png')
    until(lambda: window.arm_button.isEnabled() and max(map(abs, node.core.status()['rpm'])) <= 2, timeout=15)
    window.arm_button.click()
    until(lambda: node.core.armed and window.drive_buttons[0].isEnabled())
    QTest.mousePress(window.drive_buttons[0], QtCore.Qt.LeftButton)
    QTest.qWait(200)
    window.close()
    assert not node.transport_instance.armed and not node.world.data.ctrl.any()
    print('PASS: one window at 1366x800, live scene and all 4 rows, Qt hold/release and focus-loss stop,')
    print('      ROS odometry while driving, PID apply, world reset, armed close and renderer cleanup')
finally:
    pump.stop()
    window.close()
    executor.shutdown()
    node.close()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
