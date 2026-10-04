import json
import threading
import time
from collections import deque
from urllib.request import Request, urlopen
from urllib.error import HTTPError
from urllib.parse import urlparse


class Client:
    def __init__(self, url, token=''):
        parsed = urlparse(url)
        if parsed.scheme not in ('http', 'https') or not parsed.netloc or parsed.username or parsed.password:
            raise ValueError('enter an http(s) URL without embedded credentials')
        if not token.isascii():
            raise ValueError('토큰에는 영문·숫자 등 ASCII 문자만 사용하세요.')
        self.url, self.token = url.rstrip('/'), token

    def request(self, path, data=None):
        body = None if data is None else json.dumps(data, allow_nan=False).encode()
        headers = {'Content-Type': 'application/json'}
        if self.token:
            headers['Authorization'] = 'Bearer '+self.token
        req = Request(self.url+path, data=body, headers=headers)
        try:
            with urlopen(req, timeout=0.4) as response:
                return json.loads(response.read(65536))
        except HTTPError as exc:
            result = json.loads(exc.read(65536))
            raise ValueError(result.get('error', str(exc))) from exc


class Worker(threading.Thread):
    def __init__(self, client):
        super().__init__(daemon=True)
        self.client = client
        self.lock = threading.Lock()
        self.tasks = deque()
        self.motion = None
        self.status = None
        self.status_time = 0
        self.message = ''
        self.quit_event = threading.Event()

    def submit(self, path, data=None):
        with self.lock:
            if path == '/api/stop':
                self.tasks.clear()
                self.motion = None
                self.tasks.appendleft((path, data or {}))
            elif len(self.tasks) < 16:
                self.tasks.append((path, data or {}))

    def hold(self, path, data):
        with self.lock:
            self.motion = (time.monotonic(), path, data)

    def snapshot(self):
        with self.lock:
            return self.status, self.status_time, self.message

    def run(self):
        while not self.quit_event.is_set():
            start = time.monotonic()
            with self.lock:
                task = self.tasks.popleft() if self.tasks else None
                motion = self.motion
            try:
                if task:
                    result = self.client.request(*task)
                    with self.lock:
                        self.message = result.get('path', result.get('note', '완료'))
                elif motion:
                    updated, path, data = motion
                    if start-updated < 0.25:
                        self.client.request(path, data)
                    else:
                        self.client.request('/api/stop', {})
                        with self.lock:
                            self.motion = None
                status = self.client.request('/api/status')
                with self.lock:
                    self.status, self.status_time = status, time.monotonic()
            except Exception as exc:
                with self.lock:
                    self.message = str(exc)
                    self.motion = None
                    # Failed queued motion/arm actions must never be replayed on reconnect.
                    self.tasks.clear()
                    self.status_time = 0
            self.quit_event.wait(max(0, 0.1-(time.monotonic()-start)))

    def close(self):
        self.quit_event.set()
        self.join(timeout=1)
        try:
            self.client.request('/api/stop', {})
        except Exception:
            pass
