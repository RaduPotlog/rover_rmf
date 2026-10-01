# Copyright 2026 Rover A1 contributors
# Licensed under the Apache License, Version 2.0.

"""
The rover control API the dashboard's Rover card uses (http://<host>:8030).

    GET  /api/robots                     {"robots": [RobotSession.snapshot(), ...]}
    POST /api/robots/<name>/drive_mode   {"mode": "MANUAL" | "AUTOMATIC"}
         -> 200 {"ok": bool, "message": str, "robot": snapshot} once the rover answered (or the
            request timed out); 400 for a bad mode or an offline rover; 404 for an unknown robot

Published on the VPN address only (docker-compose.server.yml), like the sites page; CORS is open
because the dashboard is served from another port.
"""

from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import re
import threading
import time
from typing import Callable, Mapping
from urllib.parse import unquote

from ..application.ports import Log
from ..domain.model import DomainError

DRIVE_MODE_PATH = re.compile(r'/api/robots/([^/]+)/drive_mode')


def make_server(sessions: Mapping[str, object], host: str, port: int, log: Log,
                clock: Callable[[], float] = time.monotonic,
                sleep: Callable[[float], None] = time.sleep,
                poll: float = 0.1) -> ThreadingHTTPServer:
    """A server for these sessions (RobotSession by robot name); serve_forever() to run it."""

    class Handler(BaseHTTPRequestHandler):

        def log_message(self, fmt, *args):  # noqa: D102 quiet: the adapter logs requests itself
            pass

        def do_OPTIONS(self):  # noqa: N802
            self._send(204, None)

        def do_GET(self):  # noqa: N802
            if self.path.split('?')[0] != '/api/robots':
                return self._send(404, {'error': 'not found'})
            self._send(200, {'robots': [s.snapshot() for s in sessions.values()]})

        def do_POST(self):  # noqa: N802
            match = DRIVE_MODE_PATH.fullmatch(self.path)
            if not match:
                return self._send(404, {'error': 'not found'})
            name = unquote(match.group(1))
            session = sessions.get(name)
            if session is None:
                return self._send(404, {'error': f'no robot {name!r}'})
            try:
                length = int(self.headers.get('Content-Length') or 0)
                mode = json.loads(self.rfile.read(length) or b'{}').get('mode')
                action_id = session.request_drive_mode(str(mode or ''), clock())
            except (ValueError, AttributeError) as e:
                return self._send(400, {'error': f'bad request: {e}'})
            except DomainError as e:
                return self._send(400, {'error': str(e)})
            while (result := session.drive_mode_result(action_id, clock())) is None:
                sleep(poll)
            ok, message = result
            log.info(f'[{name}] control API: drive mode {mode}: '
                     f'{"ok" if ok else "failed"}: {message}')
            self._send(200, {'ok': ok, 'message': message, 'robot': session.snapshot()})

        def _send(self, code: int, body) -> None:
            data = b'' if body is None else json.dumps(body).encode()
            self.send_response(code)
            self.send_header('Access-Control-Allow-Origin', '*')
            self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
            self.send_header('Access-Control-Allow-Headers', 'Content-Type')
            if body is not None:
                self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(data)))
            self.end_headers()
            self.wfile.write(data)

    server = ThreadingHTTPServer((host, port), Handler)
    server.daemon_threads = True
    return server


def start(sessions: Mapping[str, object], host: str, port: int, log: Log) -> ThreadingHTTPServer:
    """Serve in a daemon thread."""
    server = make_server(sessions, host, port, log)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    log.info(f'rover control API on http://{host}:{port}/api/robots')
    return server
