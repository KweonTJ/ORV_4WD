"""One Qt window owns the physics, ROS callbacks, camera and motor controls."""
import argparse
import os
import signal
import sys
import time

# Must precede the first MuJoCo import. EGL does not create a second GLFW window.
if sys.platform.startswith('linux'):
    os.environ.setdefault('MUJOCO_GL', 'egl')

import mujoco
import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from std_srvs.srv import Trigger

from orv_gui.app import Q, QtCore, QtGui, Window
from .model import World, settings
from .node import Simulator


def application(argv=None):
    # Qt 5 otherwise scales point fonts but leaves pixel widget heights unchanged.
    for name in ('AA_EnableHighDpiScaling', 'AA_UseHighDpiPixmaps'):
        attribute = getattr(QtCore.Qt, name, None)
        if attribute is not None:
            Q.QApplication.setAttribute(attribute)
    app = Q.QApplication(argv or [])
    app.setFont(QtGui.QFont('Noto Sans CJK KR', 10))
    return app


def fit_table(table):
    """Keep all four rows visible with the current font, style and monitor DPI."""
    table.ensurePolished()
    header = table.verticalHeader()
    row_height = max(30, table.fontMetrics().height()+10, header.minimumSectionSize())
    header.setDefaultSectionSize(row_height)
    table.setFixedHeight(sum(table.rowHeight(row) for row in range(table.rowCount())) +
                         table.horizontalHeader().sizeHint().height()+2*table.frameWidth()+2)


class LocalControls:
    """Use the existing validated command handlers on their ROS/Qt owner thread."""
    def __init__(self, node):
        self.node = node
        self.message = ''

    def snapshot(self):
        return self.node.core.status(), self.node.last_publish, self.message

    def submit(self, path, data=None):
        try:
            result = self.node.api_command(path, data or {})
            self.message = result.get('path', '')
        except (ValueError, TypeError, KeyError, OSError) as exc:
            self.message = str(exc)

    def hold(self, path, data):
        self.submit(path, data)

    def close(self):
        self.node.core.stop('dashboard closed')


class Scene(Q.QWidget):
    def __init__(self, world):
        super().__init__()
        self.world = world
        self.renderer = None
        self.frame = QtGui.QImage()
        self.camera = mujoco.MjvCamera()
        self.option = mujoco.MjvOption()
        self.option.geomgroup[3] = 0
        self.follow = True
        self.last_mouse = None
        self.setMinimumSize(360, 260)
        self.setSizePolicy(Q.QSizePolicy.Expanding, Q.QSizePolicy.Expanding)
        self.setToolTip('왼쪽 드래그: 회전 · 오른쪽 드래그: 이동 · 휠: 확대/축소 · 더블 클릭: 시점 초기화')
        self.home()

    def home(self):
        self.camera.type = mujoco.mjtCamera.mjCAMERA_FREE
        self.camera.distance = .70
        self.camera.azimuth = 135
        self.camera.elevation = -28
        self.camera.lookat[:] = self.world.pose()[0]
        self.follow = True

    def render_frame(self):
        # Match the widget aspect ratio, cap GPU work and stay within XML offscreen limits.
        ratio = min(1., 1200/max(1, self.width()), 800/max(1, self.height()))
        width, height = max(1, int(self.width()*ratio)), max(1, int(self.height()*ratio))
        if self.renderer is None or (self.renderer.width, self.renderer.height) != (width, height):
            self.close_renderer()
            self.renderer = mujoco.Renderer(self.world.model, height=height, width=width)
        if self.follow:
            self.camera.lookat[:] = self.world.pose()[0]
        self.renderer.update_scene(self.world.data, self.camera, scene_option=self.option)
        rgb = self.renderer.render()
        self.frame = QtGui.QImage(rgb.data, width, height, rgb.strides[0],
                                 QtGui.QImage.Format_RGB888).copy()
        self.update()

    def paintEvent(self, _):
        painter = QtGui.QPainter(self)
        painter.fillRect(self.rect(), QtGui.QColor('#17202d'))
        if not self.frame.isNull():
            painter.drawImage(self.rect(), self.frame)

    def mousePressEvent(self, event):
        self.last_mouse = event.pos()

    def mouseMoveEvent(self, event):
        if self.last_mouse is None or self.renderer is None:
            return
        delta = event.pos()-self.last_mouse
        self.last_mouse = event.pos()
        if event.buttons() & QtCore.Qt.RightButton:
            action = mujoco.mjtMouse.mjMOUSE_MOVE_V
            self.follow = False
        elif event.buttons() & QtCore.Qt.LeftButton:
            action = mujoco.mjtMouse.mjMOUSE_ROTATE_V
        else:
            return
        mujoco.mjv_moveCamera(self.world.model, action, delta.x()/self.height(),
                             delta.y()/self.height(), self.renderer.scene, self.camera)

    def mouseReleaseEvent(self, _):
        self.last_mouse = None

    def wheelEvent(self, event):
        self.camera.distance = max(.22, min(8., self.camera.distance *
                                           (0.85 ** (event.angleDelta().y()/120))))
        event.accept()

    def mouseDoubleClickEvent(self, _):
        self.home()

    def close_renderer(self):
        if self.renderer is not None:
            self.renderer.close()
            self.renderer = None


class Dashboard(Window):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.worker = LocalControls(node)
        self.setWindowTitle('ORV 4WD · MuJoCo 디지털 트윈')
        self.resize(1440, 900)
        self.setMinimumSize(1100, 730)
        old_root = self.takeCentralWidget()
        tabs = old_root.findChild(Q.QTabWidget)
        self.tabs = tabs
        self.scene = Scene(node.world)
        root = Q.QWidget()
        self.setCentralWidget(root)
        layout = Q.QVBoxLayout(root)
        layout.setContentsMargins(12, 10, 12, 10)
        header = Q.QHBoxLayout()
        title = Q.QLabel('ORV 4WD  /  DIGITAL TWIN')
        title.setStyleSheet('font-size:18px; font-weight:700; color:#142238;')
        header.addWidget(title)
        header.addStretch()
        header.addWidget(self.arm_button)
        header.addWidget(self.stop_button)
        layout.addLayout(header)
        self.banner.setStyleSheet('padding:6px; background:#e2e8f0; color:#142238;')
        layout.addWidget(self.banner)
        split = Q.QSplitter(QtCore.Qt.Horizontal)
        split.setChildrenCollapsible(False)
        layout.addWidget(split, 1)
        left, right = Q.QWidget(), Q.QWidget()
        left_layout, right_layout = Q.QVBoxLayout(left), Q.QVBoxLayout(right)
        left_layout.setContentsMargins(0, 0, 6, 0)
        right_layout.setContentsMargins(6, 0, 0, 0)
        view_tools = Q.QHBoxLayout()
        view_tools.addWidget(Q.QLabel('물리 시뮬레이션'))
        view_tools.addStretch()
        home = Q.QPushButton('차량 따라가기 / 시점 복원')
        home.clicked.connect(self.scene.home)
        view_tools.addWidget(home)
        self.world_reset = Q.QPushButton('차량 위치 초기화')
        self.world_reset.clicked.connect(self.reset_world)
        view_tools.addWidget(self.world_reset)
        left_layout.addLayout(view_tools)
        left_layout.addWidget(self.scene, 1)
        left_layout.addWidget(Q.QLabel('드래그: 회전  ·  오른쪽 드래그: 이동  ·  휠: 확대/축소'))
        self.metrics = Q.QLabel('시뮬레이션 준비 중')
        left_layout.addWidget(self.metrics)
        left_layout.addWidget(Q.QLabel('모터 회전수  /  RPM · 최근 30초'))
        self.plot.setMinimumHeight(130)
        self.plot.setMaximumHeight(200)
        left_layout.addWidget(self.plot)
        right_layout.addWidget(Q.QLabel('모터 / 엔코더 실시간 상태'))
        self.table.setHorizontalHeaderLabels(['누적 카운트', '상대 카운트', '목표 RPM/PWM', '실제 RPM', 'PWM'])
        fit_table(self.table)
        right_layout.addWidget(self.table)
        self.pose.setStyleSheet('padding:6px; background:#edf2f7; color:#142238;')
        right_layout.addWidget(self.pose)
        tabs.setTabText(0, '주행 / 모터 시험')
        tabs.setTabText(1, 'PID / 엔코더 설정')
        right_layout.addWidget(tabs, 1)
        right.setMinimumWidth(600)
        split.addWidget(left)
        split.addWidget(right)
        split.setStretchFactor(0, 3)
        split.setStretchFactor(1, 2)
        split.setSizes([820, 600])
        layout.addWidget(self.note)
        old_root.deleteLater()
        self.refresh()
        self.render_timer = QtCore.QTimer(self)
        self.render_timer.timeout.connect(self.render_scene)
        self.render_timer.start(50)
        self.failed = False

    def drive_tab(self):
        widget = Q.QWidget()
        layout = Q.QVBoxLayout(widget)
        layout.addWidget(Q.QLabel('구동 활성화 후 버튼을 누르고 있는 동안 주행합니다.'))
        speed_row = Q.QHBoxLayout()
        self.speed = self.double(0, .5, .05, 3)
        self.turn = self.double(0, 2, .3, 3)
        for label, box in [('속도 m/s', self.speed), ('회전 rad/s', self.turn)]:
            speed_row.addWidget(Q.QLabel(label))
            speed_row.addWidget(box)
        layout.addLayout(speed_row)
        pad = Q.QGridLayout()
        self.drive_buttons = []
        for label, direction, row, col in [('▲ 전진', (1, 0), 0, 1), ('▼ 후진', (-1, 0), 1, 1),
                                            ('◀ 좌회전', (0, 1), 1, 0), ('우회전 ▶', (0, -1), 1, 2)]:
            button = Q.QPushButton(label)
            button.setMinimumHeight(42)
            button.pressed.connect(lambda b=button, d=direction: self.begin_hold(b, d))
            button.released.connect(self.stop)
            pad.addWidget(button, row, col)
            self.drive_buttons.append(button)
        layout.addLayout(pad)
        layout.addSpacing(8)
        mode_row = Q.QHBoxLayout()
        mode_row.addWidget(Q.QLabel('개별 모터 시험  /  M1 → M4'))
        mode_row.addStretch()
        self.command_mode = Q.QComboBox()
        self.command_mode.addItems(['rpm', 'pwm'])
        mode_row.addWidget(self.command_mode)
        layout.addLayout(mode_row)
        wheel_row = Q.QGridLayout()
        self.wheel_inputs = [self.double(-255, 255, 0) for _ in range(4)]
        for i, box in enumerate(self.wheel_inputs):
            wheel_row.addWidget(Q.QLabel(f'M{i+1}'), 0, i)
            wheel_row.addWidget(box, 1, i)
        layout.addLayout(wheel_row)
        self.wheel_hold = Q.QPushButton('누르는 동안 모터 시험')
        self.wheel_hold.setMinimumHeight(36)
        self.wheel_hold.pressed.connect(lambda: self.begin_hold(self.wheel_hold, 'wheels'))
        self.wheel_hold.released.connect(self.stop)
        layout.addWidget(self.wheel_hold)
        reset = Q.QPushButton('오도메트리 / 상대 카운트 영점')
        reset.clicked.connect(lambda: self.send('/api/reset'))
        layout.addWidget(reset)
        self.pose = Q.QLabel('x 0 · y 0 · yaw 0')
        layout.addStretch()
        return widget

    def settings_tab(self):
        # Reuse the tuner's exact fields and validation while rearranging its wide rows.
        original = super().settings_tab()
        buttons = original.findChildren(Q.QPushButton)
        widget = Q.QWidget()
        layout = Q.QVBoxLayout(widget)
        dimensions = Q.QGridLayout()
        for i, (label, box) in enumerate([('바퀴 반경 mm', self.radius), ('좌우 간격 mm', self.track),
                                         ('최대 RPM', self.max_rpm), ('가속 RPM/s', self.accel)]):
            dimensions.addWidget(Q.QLabel(label), i//2, (i % 2)*2)
            dimensions.addWidget(box, i//2, (i % 2)*2+1)
        layout.addLayout(dimensions)
        self.config_table.setHorizontalHeaderLabels(['CPR', 'Kp', 'Ki', 'Kd', 'PWM 최대', 'PWM 최소', '모터 ±', '엔코더 ±'])
        self.config_table.setFont(QtGui.QFont('Noto Sans CJK KR', 9))
        for boxes in self.config_widgets.values():
            for box in boxes:
                box.setFont(self.config_table.font())
                if isinstance(box, Q.QDoubleSpinBox):
                    box.setButtonSymbols(Q.QAbstractSpinBox.NoButtons)
        fit_table(self.config_table)
        layout.addWidget(self.config_table)
        map_row = Q.QHBoxLayout()
        map_row.addWidget(Q.QLabel('FL / RL / FR / RR → M번호'))
        for box in self.maps:
            map_row.addWidget(box)
        layout.addLayout(map_row)
        # These fields stay in the saved profile, but the physics backend has no receiver.
        for field in (self.remote_enabled, self.remote_limit, self.confirm):
            field.setParent(widget)
            field.hide()
        actions = Q.QGridLayout()
        for i, button in enumerate(buttons):
            if button.text() == 'Pi에 설정 저장':
                button.setText('시뮬레이션 설정 저장')
            actions.addWidget(button, i//3, i % 3)
        layout.addLayout(actions)
        hint = Q.QLabel('정지 후 적용 · 저장된 PID/엔코더 설정은 시뮬레이션 프로필에 보관됩니다.')
        hint.setWordWrap(True)
        layout.addWidget(hint)
        layout.addStretch()
        original.deleteLater()
        return widget

    def reset_world(self):
        response = self.node.on_reset_world(Trigger.Request(), Trigger.Response())
        self.worker.message = ('차량 위치와 엔코더를 초기화했습니다.' if response.success else response.message)
        if response.success:
            self.scene.home()
            self.plot.samples.clear()
        self.refresh()

    def render_scene(self):
        try:
            self.scene.render_frame()
            world, transport = self.node.world, self.node.transport_instance
            pos, _ = world.pose()
            rtf = world.data.time/max(.001, time.monotonic()-transport.wall_start)
            self.metrics.setText(f'실제 위치 x {pos[0]:.3f} · y {pos[1]:.3f} · z {pos[2]:.3f} m'
                                 f'  |  실시간 배율 {rtf:.2f}×')
            self.world_reset.setEnabled(self.node.core.ready and not self.node.core.armed and
                                       not (self.node.core.telemetry and self.node.core.telemetry.armed))
        except Exception as exc:
            self.failed = True
            self.stop()
            self.render_timer.stop()
            self.node.get_logger().error(f'Dashboard rendering failed: {exc}')
            self.close()

    def closeEvent(self, event):
        self.timer.stop()
        self.render_timer.stop()
        super().closeEvent(event)
        self.scene.close_renderer()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--physics-config')
    parser.add_argument('--terrain', choices=('flat', 'course'), default=None)
    args, ros_args = parser.parse_known_args()
    cfg = settings(args.physics_config)
    if args.terrain:
        cfg['terrain'] = args.terrain
    app = application([sys.argv[0]])
    world = World(cfg)
    rclpy.init(args=ros_args)
    node = window = executor = None
    try:
        node = Simulator(world)
        executor = SingleThreadedExecutor(context=node.context)
        executor.add_node(node)
        window = Dashboard(node)
        pump = QtCore.QTimer(window)

        def spin():
            try:
                if rclpy.ok():
                    executor.spin_once(timeout_sec=0.)
                else:
                    app.quit()
            except ExternalShutdownException:
                app.quit()
            except Exception as exc:
                window.failed = True
                node.get_logger().error(f'Dashboard ROS callback failed: {exc}')
                app.quit()

        pump.timeout.connect(spin)
        pump.start(5)
        signal.signal(signal.SIGINT, lambda *_: app.quit())
        signal.signal(signal.SIGTERM, lambda *_: app.quit())
        window.showMaximized()
        app.exec() if hasattr(app, 'exec') else app.exec_()
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if window:
            window.close()
        if executor:
            executor.shutdown()
        if node:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return int(bool(window and window.failed))
