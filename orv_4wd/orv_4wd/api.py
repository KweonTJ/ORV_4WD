"""Bounded HTTP command queue; all robot operations run on the ROS owner thread."""
import hmac
import json
import queue
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer


class Api:
    def __init__(self, host, port, token=''):
        if not token.isascii():
            raise ValueError('ORV_API_TOKEN must contain only ASCII characters')
        if host not in ('127.0.0.1', 'localhost', '::1') and len(token) < 16:
            raise ValueError('remote API requires ORV_API_TOKEN of at least 16 characters')
        self.requests = queue.Queue(maxsize=32)
        self.snapshot = {}
        self.lock = threading.Lock()
        outer = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *_):
                pass

            def respond(self, code, data):
                body = json.dumps(data, allow_nan=False).encode()
                self.send_response(code)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Content-Length', str(len(body)))
                self.send_header('Cache-Control', 'no-store')
                self.end_headers()
                try:
                    self.wfile.write(body)
                except (BrokenPipeError, ConnectionResetError):
                    pass

            def authorized(self):
                if self.headers.get('Origin'):
                    self.respond(403, {'error': 'browser-origin requests are not accepted'})
                    return False
                if token and not hmac.compare_digest(self.headers.get('Authorization', ''), 'Bearer '+token):
                    self.respond(401, {'error': 'invalid API token'})
                    return False
                return True

            def do_GET(self):
                if not self.authorized():
                    return
                if self.path != '/api/status':
                    self.respond(404, {'error': 'unknown endpoint'})
                    return
                with outer.lock:
                    state = outer.snapshot
                self.respond(200, state)

            def do_POST(self):
                if not self.authorized():
                    return
                if self.path not in ('/api/arm', '/api/stop', '/api/wheels', '/api/twist', '/api/config', '/api/reset', '/api/save'):
                    self.respond(404, {'error': 'unknown endpoint'})
                    return
                try:
                    length = int(self.headers.get('Content-Length', '0'))
                    if not 0 < length <= 16384:
                        raise ValueError('body length must be 1..16384 bytes')
                    self.connection.settimeout(1.0)
                    data = json.loads(self.rfile.read(length), parse_constant=lambda _: (_ for _ in ()).throw(ValueError('non-finite JSON')))
                    if not isinstance(data, dict):
                        raise ValueError('request must be an object')
                    event = threading.Event()
                    result = {}
                    outer.requests.put_nowait((time.monotonic()+0.5, self.path, data, event, result))
                    if not event.wait(0.7):
                        self.respond(504, {'error': 'driver did not process request before deadline'})
                        return
                    self.respond(200 if result.get('ok') else 400, result)
                except (ValueError, OSError, queue.Full) as exc:
                    self.respond(400, {'error': str(exc)})

        self.server = ThreadingHTTPServer((host, port), Handler)
        self.server.daemon_threads = True
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()

    def drain(self, callback):
        for _ in range(16):
            try:
                deadline, path, data, event, result = self.requests.get_nowait()
            except queue.Empty:
                break
            try:
                if time.monotonic() > deadline:
                    raise ValueError('request expired without being applied')
                result.update(callback(path, data))
            except (ValueError, TypeError, KeyError, OSError) as exc:
                result.update(ok=False, error=str(exc))
            finally:
                event.set()

    def publish(self, status):
        with self.lock:
            self.snapshot = status

    def close(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(timeout=1)
