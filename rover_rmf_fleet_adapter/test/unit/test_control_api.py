# Copyright 2026 Rover A1 contributors
# Licensed under the Apache License, Version 2.0.

import json
import threading
import urllib.error
import urllib.request

import pytest

from rover_rmf_fleet_adapter.application.ports import Log
from rover_rmf_fleet_adapter.domain.model import DomainError
from rover_rmf_fleet_adapter.presentation.control_api import make_server


class QuietLog(Log):
    def info(self, message):
        pass

    def warning(self, message):
        pass

    def error(self, message):
        pass


class FakeSession:
    """Answers a drive mode request after `polls` drive_mode_result() calls."""

    def __init__(self, result=(True, 'Drive mode Manual.'), polls=2):
        self.requests = []
        self.result = result
        self.polls = polls

    def snapshot(self):
        return {'name': 'rover_a1', 'operating_mode': 'AUTOMATIC'}

    def request_drive_mode(self, mode, now):
        if mode not in ('MANUAL', 'AUTOMATIC'):
            raise DomainError(f'bad mode {mode!r}')
        self.requests.append(mode)
        return 'a1'

    def drive_mode_result(self, action_id, now):
        self.polls -= 1
        return self.result if self.polls <= 0 else None


@pytest.fixture
def api():
    session = FakeSession()
    server = make_server({'rover_a1': session}, '127.0.0.1', 0, QuietLog(), sleep=lambda s: None)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    yield f'http://127.0.0.1:{server.server_address[1]}', session
    server.shutdown()


def call(url, method='GET', body=None):
    data = None if body is None else json.dumps(body).encode()
    request = urllib.request.Request(url, data=data, method=method,
                                     headers={'Content-Type': 'application/json'})
    try:
        with urllib.request.urlopen(request, timeout=5) as response:
            return response.status, json.loads(response.read() or b'null'), response.headers
    except urllib.error.HTTPError as e:
        return e.code, json.loads(e.read() or b'null'), e.headers


def test_lists_the_robots_with_cors(api):
    base, _ = api
    status, body, headers = call(f'{base}/api/robots')
    assert status == 200 and body == {'robots': [{'name': 'rover_a1',
                                                  'operating_mode': 'AUTOMATIC'}]}
    assert headers['Access-Control-Allow-Origin'] == '*'
    status, _, headers = call(f'{base}/api/robots/rover_a1/drive_mode', 'OPTIONS')
    assert status == 204 and 'POST' in headers['Access-Control-Allow-Methods']


def test_drive_mode_waits_for_the_rovers_answer(api):
    base, session = api
    status, body, _ = call(f'{base}/api/robots/rover_a1/drive_mode', 'POST', {'mode': 'MANUAL'})
    assert status == 200 and body['ok'] is True and body['message'] == 'Drive mode Manual.'
    assert session.requests == ['MANUAL'] and session.polls <= 0
    assert body['robot']['name'] == 'rover_a1'


def test_drive_mode_errors(api):
    base, session = api
    assert call(f'{base}/api/robots/nobody/drive_mode', 'POST', {'mode': 'MANUAL'})[0] == 404
    status, body, _ = call(f'{base}/api/robots/rover_a1/drive_mode', 'POST', {'mode': 'X'})
    assert status == 400 and 'bad mode' in body['error']
    assert call(f'{base}/api/nothing')[0] == 404
    assert session.requests == []
