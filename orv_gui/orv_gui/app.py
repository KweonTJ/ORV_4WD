import json
import os
from pathlib import Path
import sys
import time
from collections import deque
try:
    from PySide6 import QtCore, QtGui, QtWidgets
except ImportError:
    from PyQt5 import QtCore, QtGui, QtWidgets
from .client import Client, Worker

Q = QtWidgets


class RpmPlot(Q.QWidget):
    def __init__(self):
        super().__init__()
        self.samples = deque(maxlen=300)
        self.setMinimumHeight(170)

    def append(self, values):
        self.samples.append(values)
        self.update()

    def paintEvent(self, _):
        painter = QtGui.QPainter(self)
        painter.fillRect(self.rect(), QtGui.QColor('#17202d'))
        w, h = self.width(), self.height()
        scale = max(10, max((abs(v) for sample in self.samples for v in sample), default=10)) * 1.1
        painter.setPen(QtGui.QColor('#718096'))
        painter.drawLine(45, h//2, w-10, h//2)
        painter.drawText(8, 20, f'{scale:.0f}')
        painter.drawText(8, h-12, f'-{scale:.0f}')
        colors = ['#60a5fa', '#4ade80', '#fbbf24', '#f472b6']
        for ch, color in enumerate(colors):
            painter.setPen(QtGui.QPen(QtGui.QColor(color), 2))
            painter.drawText(65+ch*80, 20, f'M{ch+1}')
            points = [QtCore.QPointF(45+i*(w-60)/299, h/2-v[ch]*(h-50)/(2*scale)) for i, v in enumerate(self.samples)]
            if len(points) > 1:
                painter.drawPolyline(QtGui.QPolygonF(points))


class Window(Q.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('ORV 4WD · 모터 / 엔코더 튜너')
        self.resize(1180, 860)
        self.worker = None
        self.loaded = False
        self.last_status = None
        self.held = None
        root = Q.QWidget()
        self.setCentralWidget(root)
        layout = Q.QVBoxLayout(root)
        connection = Q.QHBoxLayout()
        self.address = Q.QLineEdit(os.environ.get('ORV_API_URL', 'http://127.0.0.1:8765'))
        self.token = Q.QLineEdit(os.environ.get('ORV_API_TOKEN', ''))
        self.token.setEchoMode(Q.QLineEdit.Password)
        self.token.setPlaceholderText('원격 접속 토큰')
        connect = Q.QPushButton('연결 / 다시 연결')
        connect.clicked.connect(self.connect_server)
        connection.addWidget(Q.QLabel('차량 / 시뮬레이터'))
        connection.addWidget(self.address, 2)
        connection.addWidget(self.token, 1)
        connection.addWidget(connect)
        layout.addLayout(connection)
        self.banner = Q.QLabel('연결 전 · 모터 비활성')
        self.banner.setStyleSheet('font-size: 17px; padding: 8px; background: #e2e8f0; color: #142238;')
        layout.addWidget(self.banner)
        actions = Q.QHBoxLayout()
        self.arm_button = Q.QPushButton('GUI 구동 활성화')
        self.arm_button.clicked.connect(lambda: self.send('/api/arm'))
        self.stop_button = Q.QPushButton('전체 정지 / 비활성')
        self.stop_button.setStyleSheet('background:#b91c1c;color:white;font-weight:bold;padding:10px;')
        self.stop_button.clicked.connect(self.stop)
        actions.addWidget(self.arm_button)
        actions.addWidget(self.stop_button, 1)
        layout.addLayout(actions)
        self.table = Q.QTableWidget(4, 5)
        self.table.setHorizontalHeaderLabels(['누적 엔코더 카운트', '상대 카운트', '목표 (RPM/PWM)', '실제 RPM', '출력 PWM'])
        self.table.setVerticalHeaderLabels(['M1', 'M2', 'M3', 'M4'])
        self.table.horizontalHeader().setSectionResizeMode(Q.QHeaderView.Stretch)
        self.table.setEditTriggers(Q.QAbstractItemView.NoEditTriggers)
        self.table.setMaximumHeight(170)
        layout.addWidget(self.table)
        self.plot = RpmPlot()
        layout.addWidget(self.plot)
        tabs = Q.QTabWidget()
        layout.addWidget(tabs, 1)
        tabs.addTab(self.drive_tab(), '주행 / 개별 모터 시험')
        tabs.addTab(self.settings_tab(), 'PID / 엔코더 / 차체 설정')
        self.note = Q.QLabel('버튼을 누르는 동안만 명령 전송 · 놓거나 창이 비활성화되면 정지 · 설정은 정지 후 적용')
        self.note.setWordWrap(True)
        layout.addWidget(self.note)
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.refresh()
        self.timer.start(100)

    def double(self, low, high, value, decimals=2):
        box = Q.QDoubleSpinBox()
        box.setRange(low, high)
        box.setDecimals(decimals)
        box.setValue(value)
        return box

    def drive_tab(self):
        widget = Q.QWidget()
        layout = Q.QVBoxLayout(widget)
        row = Q.QHBoxLayout()
        self.command_mode = Q.QComboBox()
        self.command_mode.addItems(['rpm', 'pwm'])
        row.addWidget(Q.QLabel('M1 → M4 시험 명령'))
        row.addWidget(self.command_mode)
        self.wheel_inputs = [self.double(-255, 255, 0) for _ in range(4)]
        for box in self.wheel_inputs:
            row.addWidget(box)
        self.wheel_hold = Q.QPushButton('누르는 동안 모터 시험')
        self.wheel_hold.pressed.connect(lambda: self.begin_hold(self.wheel_hold, 'wheels'))
        self.wheel_hold.released.connect(self.stop)
        row.addWidget(self.wheel_hold)
        layout.addLayout(row)
        row = Q.QHBoxLayout()
        self.speed = self.double(0, 0.5, 0.05, 3)
        self.turn = self.double(0, 2, 0.3, 3)
        row.addWidget(Q.QLabel('전진 m/s'))
        row.addWidget(self.speed)
        row.addWidget(Q.QLabel('회전 rad/s'))
        row.addWidget(self.turn)
        self.drive_buttons = []
        for label, direction in [('전진', (1, 0)), ('후진', (-1, 0)), ('좌회전', (0, 1)), ('우회전', (0, -1))]:
            button = Q.QPushButton(label)
            button.pressed.connect(lambda b=button, d=direction: self.begin_hold(b, d))
            button.released.connect(self.stop)
            row.addWidget(button)
            self.drive_buttons.append(button)
        layout.addLayout(row)
        row = Q.QHBoxLayout()
        reset = Q.QPushButton('오도메트리 / 상대 카운트 영점')
        reset.clicked.connect(lambda: self.send('/api/reset'))
        row.addWidget(reset)
        self.pose = Q.QLabel('x 0 · y 0 · yaw 0')
        row.addWidget(self.pose)
        layout.addLayout(row)
        layout.addStretch()
        return widget

    def settings_tab(self):
        widget = Q.QWidget()
        layout = Q.QVBoxLayout(widget)
        row = Q.QHBoxLayout()
        self.radius = self.double(0, 1000, 0, 2)
        self.track = self.double(0, 3000, 0, 2)
        self.max_rpm = self.double(1, 150, 30)
        self.accel = self.double(1, 300, 30)
        for label, box in [('바퀴 반경 mm', self.radius), ('좌우 중심 간격 mm', self.track), ('최대 RPM', self.max_rpm), ('가속 RPM/s', self.accel)]:
            row.addWidget(Q.QLabel(label)); row.addWidget(box)
        layout.addLayout(row)
        self.config_table = Q.QTableWidget(4, 8)
        remote_row = Q.QHBoxLayout()
        self.remote_enabled = Q.QCheckBox('동봉 무선 조종기 사용')
        self.remote_limit = self.double(1, 150, 15)
        remote_row.addWidget(self.remote_enabled)
        remote_row.addWidget(Q.QLabel('조종기 최대 RPM'))
        remote_row.addWidget(self.remote_limit)
        layout.addLayout(remote_row)
        self.config_table.setHorizontalHeaderLabels(['CPR', 'Kp', 'Ki', 'Kd', 'PWM 최대', 'PWM 최소', '모터 방향', '엔코더 방향'])
        self.config_table.setVerticalHeaderLabels(['M1', 'M2', 'M3', 'M4'])
        self.config_table.horizontalHeader().setSectionResizeMode(Q.QHeaderView.Stretch)
        self.config_widgets = {}
        for col, (key, lo, hi, value, decimals) in enumerate([
            ('encoder_cpr', 1, 60000, 4320, 0), ('kp', 0, 100, 1, 2),
            ('ki', 0, 100, 0.5, 2), ('kd', 0, 100, 0, 2),
            ('max_pwm', 1, 255, 100, 0), ('min_pwm', 0, 254, 0, 0),
            ('motor_sign', -1, 1, 1, 0), ('encoder_sign', -1, 1, 1, 0)]):
            boxes = []
            for i in range(4):
                if key.endswith('_sign'):
                    box = Q.QComboBox(); box.addItems(['+1', '-1'])
                else:
                    box = self.double(lo, hi, value, decimals)
                self.config_table.setCellWidget(i, col, box)
                boxes.append(box)
            self.config_widgets[key] = boxes
        layout.addWidget(self.config_table)
        row = Q.QHBoxLayout()
        row.addWidget(Q.QLabel('바퀴 순서 FL / RL / FR / RR → M번호'))
        self.maps = []
        for i in range(4):
            box = Q.QSpinBox(); box.setRange(1, 4); box.setValue(i+1)
            self.maps.append(box); row.addWidget(box)
        self.confirm = Q.QCheckBox('실물 실드·배선·모터 전원 확인')
        row.addWidget(self.confirm)
        layout.addLayout(row)
        row = Q.QHBoxLayout()
        for label, handler in [('현재 설정 읽기', self.reload_config), ('설정 적용', self.apply_config),
                               ('Pi에 설정 저장', lambda: self.send('/api/save')),
                               ('JSON 내보내기', self.export_config), ('JSON 불러오기', self.import_config)]:
            button = Q.QPushButton(label); button.clicked.connect(handler); row.addWidget(button)
        layout.addLayout(row)
        return widget

    def connect_server(self):
        self.stop()
        if self.worker:
            self.worker.close()
        self.worker = None
        try:
            self.worker = Worker(Client(self.address.text(), self.token.text()))
            self.worker.start()
            self.loaded = False
        except ValueError as exc:
            self.note.setText(str(exc))

    def send(self, path, data=None):
        if self.worker:
            self.worker.submit(path, data)

    def begin_hold(self, button, mode):
        self.held = (button, mode)
        self.refresh()

    def stop(self):
        if self.held:
            self.held[0].setDown(False)
        self.held = None
        self.send('/api/stop')

    def event(self, event):
        if event.type() == QtCore.QEvent.WindowDeactivate and hasattr(self, 'worker'):
            self.stop()
        return super().event(event)

    def load_fields(self, config):
        self.radius.setValue(config['wheel_radius']*1000)
        self.track.setValue(config['track_width']*1000)
        self.max_rpm.setValue(config['max_rpm']); self.accel.setValue(config['accel_rpm_s'])
        self.confirm.setChecked(config['hardware_confirmed'])
        self.remote_enabled.setChecked(config.get('remote_enabled', True))
        self.remote_limit.setValue(config.get('remote_max_rpm', 15.0))
        for box, value in zip(self.maps, config['motor_map']): box.setValue(value)
        for key, boxes in self.config_widgets.items():
            for box, value in zip(boxes, config[key]):
                box.setCurrentIndex(0 if value == 1 else 1) if key.endswith('_sign') else box.setValue(value)

    def reload_config(self):
        if self.last_status:
            self.load_fields(self.last_status['config'])

    def collect(self):
        cfg = dict(self.last_status['config']) if self.last_status else {}
        cfg.update(wheel_radius=self.radius.value()/1000, track_width=self.track.value()/1000,
                   max_rpm=self.max_rpm.value(), accel_rpm_s=self.accel.value(),
                   hardware_confirmed=self.confirm.isChecked(), motor_map=[b.value() for b in self.maps])
        cfg.update(remote_enabled=self.remote_enabled.isChecked(), remote_max_rpm=self.remote_limit.value())
        for key, boxes in self.config_widgets.items():
            cfg[key] = [int(b.currentText()) if key.endswith('_sign') else
                        int(b.value()) if key in ('encoder_cpr', 'max_pwm', 'min_pwm') else b.value() for b in boxes]
        return cfg

    def apply_config(self):
        self.send('/api/config', self.collect())

    def export_config(self):
        if not self.last_status or not self.last_status.get('config_applied'):
            self.note.setText('UNO에서 적용 확인된 설정이 있어야 내보낼 수 있습니다.')
            return
        path, _ = Q.QFileDialog.getSaveFileName(self, '적용된 설정 내보내기', 'orv_profile.json', 'JSON (*.json)')
        if path:
            try:
                Path(path).write_text(json.dumps(self.last_status['config'], indent=2), encoding='utf-8')
            except OSError as exc:
                self.note.setText(str(exc))

    def import_config(self):
        path, _ = Q.QFileDialog.getOpenFileName(self, '설정 불러오기 (적용 버튼으로 전송)', '', 'JSON (*.json)')
        if path:
            try:
                self.load_fields(json.loads(Path(path).read_text(encoding='utf-8')))
            except (OSError, ValueError, KeyError, TypeError) as exc:
                self.note.setText(str(exc))

    def refresh(self):
        status, stamp, message = self.worker.snapshot() if self.worker else (None, 0, '')
        fresh = bool(status and time.monotonic()-stamp < 0.4)
        armed = bool(fresh and status.get('armed') and status.get('source') == 'gui')
        self.arm_button.setEnabled(bool(fresh and status.get('ready') and
                                        not status.get('armed') and not status.get('arm_requested')))
        self.wheel_hold.setEnabled(armed)
        for button in self.drive_buttons:
            button.setEnabled(bool(armed and status.get('geometry_valid')))
        if self.held:
            button, mode = self.held
            if not armed or not button.isDown():
                self.stop()
            elif mode == 'wheels':
                self.worker.hold('/api/wheels', {'mode': self.command_mode.currentText(), 'values': [b.value() for b in self.wheel_inputs]})
            else:
                self.worker.hold('/api/twist', {'linear': mode[0]*self.speed.value(), 'angular': mode[1]*self.turn.value()})
        if not fresh:
            self.banner.setText('연결 대기 / 상태 만료 · 구동 불가')
            if message: self.note.setText(message)
            return
        self.last_status = status
        owner = {'remote': '무선 조종기', 'ros': 'ROS', 'gui': 'GUI'}.get(status.get('source'), '없음')
        device = {'mock': '모의 장치 — 실제 로봇 아님', 'mujoco': 'MuJoCo 물리 시뮬레이션',
                  'hardware': '실제 UNO'}.get(status['mode'], status['mode'])
        self.banner.setText(f"{device} · "
                            f"{'구동 활성' if status.get('armed') else '구동 비활성'} · 제어: {owner} · {status['reason']}")
        if not self.loaded and status.get('config'):
            self.load_fields(status['config']); self.loaded = True
        for row in range(4):
            for col, key in enumerate(['ticks', 'relative_ticks', 'target', 'rpm', 'pwm']):
                value = status[key][row]
                item = Q.QTableWidgetItem(f'{value:.2f}' if key in ('target', 'rpm') else str(value))
                self.table.setItem(row, col, item)
        self.plot.append(status['rpm'])
        pose = status['odometry']
        self.pose.setText(f"x {pose['x']:.3f} m · y {pose['y']:.3f} m · yaw {pose['yaw']:.3f} rad"
                         if status.get('geometry_valid') else '바퀴 반경·좌우 중심 간격을 입력해야 위치를 계산합니다.')
        self.note.setText(message or '버튼을 놓거나 창이 비활성화되면 정지합니다. 설정 변경은 정지 후 가능합니다.')

    def closeEvent(self, event):
        self.stop()
        if self.worker:
            self.worker.close()
        event.accept()


def main():
    app = Q.QApplication(sys.argv)
    window = Window()
    window.show()
    if os.environ.get('ORV_AUTO_CONNECT') == '1':
        QtCore.QTimer.singleShot(500, window.connect_server)
    return app.exec() if hasattr(app, 'exec') else app.exec_()


if __name__ == '__main__':
    main()
