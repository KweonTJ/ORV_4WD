#!/usr/bin/env python3
"""Offscreen widget check; no network or motors. Uses PySide6 or PyQt5."""
import os
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'orv_gui'))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'orv_4wd'))
from orv_gui.app import Q, Window
from orv_4wd.config import Config
from orv_4wd.core import Controller
from orv_4wd.transport import MockTransport
import time

app = Q.QApplication([])
window = Window()
cfg = Config(wheel_radius=0.05, track_width=0.30)
window.last_status = {'config': cfg.as_dict()}
window.load_fields(cfg.as_dict())
assert window.collect() == cfg.as_dict()
state = Controller(MockTransport, cfg, mock=True).status()
state.update(ready=True, armed=True, arm_requested=True, source='gui', config_applied=True, reason='armed')
state['rpm'] = [12.0, 11.8, 12.2, 12.1]
class FakeWorker:
    motion = None
    commands = []
    def snapshot(self): return state, time.monotonic(), ''
    def hold(self, path, data): self.motion = (path, data)
    def submit(self, path, data=None): self.commands.append(path); self.motion = None
    def close(self): pass
fake = FakeWorker()
window.worker = fake
window.show()
app.processEvents()
window.refresh()
window.wheel_inputs[0].setValue(10)
window.wheel_hold.setDown(True)
window.begin_hold(window.wheel_hold, 'wheels')
assert fake.motion == ('/api/wheels', {'mode':'rpm', 'values':[10.0,0.0,0.0,0.0]})
window.wheel_hold.released.emit()
assert fake.commands[-1] == '/api/stop' and window.held is None
state['armed'] = state['arm_requested'] = False
state['reason'] = 'ready; disarmed'
window.refresh()
assert not window.wheel_hold.isEnabled()
path = Path('/tmp/orv_gui_preview.png')
assert window.grab().save(str(path))
window.config_table.parentWidget().parentWidget().parentWidget().setCurrentIndex(1)
app.processEvents()
assert window.grab().save('/tmp/orv_gui_settings_preview.png')
window.close()
print('PASS: GUI settings round-trip, hold/release stop, disarmed gating and offscreen render', path)
