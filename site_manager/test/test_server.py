# Copyright 2026 Rover A1 contributors
# Licensed under the Apache License, Version 2.0.

from http.server import ThreadingHTTPServer
import json
import os
import sys
import threading
import urllib.error
import urllib.request

import pytest

HERE = os.path.dirname(__file__)
REPO = os.path.abspath(os.path.join(HERE, '..', '..'))
sys.path.insert(0, os.path.join(REPO, 'site_manager'))
sys.path.insert(0, os.path.join(REPO, 'rover_rmf_maps', 'test'))

import server  # noqa: E402
from site_io import DirMapSource  # noqa: E402
from test_rover_map_import import write_map  # noqa: E402
import yaml  # noqa: E402

# Stands in for building_map_generator nav <building> <out_dir>: writes out_dir/0.yaml.
FAKE_NAV_GRAPH = ['python3', '-c',
                  'import os, sys; os.makedirs(sys.argv[2], exist_ok=True); '
                  'open(os.path.join(sys.argv[2], "0.yaml"), "w").write("levels: {}\\n")']
FAILING_NAV_GRAPH = ['python3', '-c', 'import sys; print("no lanes"); sys.exit(1)']


class Clock:
    def __init__(self):
        self.now = 1000.0

    def __call__(self):
        return self.now


@pytest.fixture
def manager(tmp_path):
    maps = tmp_path / 'maps'
    (maps / 'lab').mkdir(parents=True)
    write_map(maps / 'lab')
    return server.SiteManager(
        sites_dir=str(tmp_path / 'sites'),
        builtin_maps=os.path.join(REPO, 'rover_rmf_maps', 'maps'),
        template_path=os.path.join(REPO, 'rover_rmf_bringup', 'config',
                                   'fleet_rover_world.yaml'),
        source=DirMapSource(str(maps)), broker=('192.168.1.201', 1883),
        nav_graph_cmd=FAKE_NAV_GRAPH, clock=Clock())


def test_builtin_sites_are_listed(manager):
    sites = {s['name']: s for s in manager.sites()}
    world = sites['rover_world']
    assert world['builtin'] and world['charger'] == 'rover_a1_charger'
    assert len(world['waypoints']) == 9 and world['lanes'] == 11


def test_import_writes_a_complete_site(manager):
    summary = manager.import_map({'name': 'lab', 'charger': 'dock', 'clearance': 0.3})
    assert summary['charger'] == 'dock' and len(summary['waypoints']) == 5
    site_dir = os.path.join(manager.sites_dir, 'lab')
    assert sorted(os.listdir(site_dir)) == [
        'fleet_lab.yaml', 'import.json', 'lab.building.yaml', 'lab.png', 'nav_graphs',
        'preview.png']
    with open(os.path.join(site_dir, 'fleet_lab.yaml')) as f:
        config = yaml.safe_load(f)
    assert config['vda5050']['broker'] == {'host': '192.168.1.201', 'port': 1883,
                                           'username': '', 'password': ''}
    lab = next(s for s in manager.sites() if s['name'] == 'lab')
    assert not lab['builtin'] and lab['charger'] == 'dock' and lab['import']['options'][
        'clearance'] == 0.3


def test_import_errors_are_request_errors(manager):
    with pytest.raises(server.MapImportError, match="no place named 'charger'"):
        manager.import_map({'name': 'lab', 'charger': 'charger', 'clearance': 0.3})
    with pytest.raises(server.MapImportError, match="no map named 'nope'"):
        manager.import_map({'name': 'nope'})
    with pytest.raises(server.RequestError, match='not a map name'):
        manager.import_map({'name': '../etc'})
    with pytest.raises(server.RequestError, match='Gazebo site'):
        manager.import_map({'name': 'rover_world'})
    with pytest.raises(server.RequestError, match='must be > 0'):
        manager.import_map({'name': 'lab', 'charger': 'dock', 'clearance': 0})


def test_failed_reimport_keeps_the_previous_site(manager):
    manager.import_map({'name': 'lab', 'charger': 'dock', 'clearance': 0.3})
    manager.nav_graph_cmd = FAILING_NAV_GRAPH
    with pytest.raises(server.MapImportError, match='no lanes'):
        manager.import_map({'name': 'lab', 'charger': 'dock', 'clearance': 0.3})
    site_dir = os.path.join(manager.sites_dir, 'lab')
    assert os.path.isfile(os.path.join(site_dir, 'nav_graphs', '0.yaml'))
    assert not any(n.startswith('.') for n in os.listdir(manager.sites_dir) if n != 'lab')


def test_activate_and_delete(manager):
    manager.import_map({'name': 'lab', 'charger': 'dock', 'clearance': 0.3})
    assert manager.activate({'name': 'lab'}) == {'active': 'lab'}
    with open(os.path.join(manager.sites_dir, 'active')) as f:
        assert f.read() == 'lab\n'
    with pytest.raises(server.RequestError, match='RMF runs'):
        manager.delete('lab')
    manager.activate({'name': 'rover_world'})
    with pytest.raises(server.RequestError, match='not an imported site'):
        manager.delete('rover_world')
    assert manager.delete('lab') == {'deleted': 'lab'}
    with pytest.raises(server.RequestError, match='import it first'):
        manager.activate({'name': 'lab'})


def test_rover_access_is_checked_at_most_every_15_s(manager):
    calls = []
    manager.source.check = lambda: calls.append(1) or ('ok', '')
    manager.access()
    manager.access()
    manager.clock.now += 16
    manager.access()
    assert len(calls) == 2


@pytest.fixture
def http(manager):
    httpd = ThreadingHTTPServer(('127.0.0.1', 0), server.make_handler(
        manager, os.path.join(REPO, 'site_manager', 'static')))
    thread = threading.Thread(target=httpd.serve_forever, daemon=True)
    thread.start()
    yield f'http://127.0.0.1:{httpd.server_address[1]}'
    httpd.shutdown()


def request(url, method='GET', body=None):
    data = json.dumps(body).encode() if body is not None else None
    req = urllib.request.Request(url, data=data, method=method,
                                 headers={'Content-Type': 'application/json'})
    try:
        with urllib.request.urlopen(req) as res:
            return res.status, res.headers.get('Content-Type'), res.read()
    except urllib.error.HTTPError as e:
        return e.code, e.headers.get('Content-Type'), e.read()


def test_http_end_to_end(http):
    code, kind, body = request(f'{http}/')
    assert code == 200 and kind.startswith('text/html') and b'RMF sites' in body

    code, _, body = request(f'{http}/api/rover/maps')
    assert code == 200 and json.loads(body)[0]['name'] == 'lab'

    code, _, body = request(f'{http}/api/import', 'POST', {'name': 'lab', 'clearance': 0.3})
    assert code == 200 and json.loads(body)['charger'] == 'dock'

    code, kind, body = request(f'{http}/sites/lab/preview.png')
    assert code == 200 and kind == 'image/png' and body[:4] == b'\x89PNG'
    code, _, _ = request(f'{http}/sites/rover_world/floorplan.png')
    assert code == 200

    code, _, body = request(f'{http}/api/import', 'POST', {'name': 'lab', 'charger': 'x'})
    assert code == 400 and "no place named 'x'" in json.loads(body)['error']

    assert request(f'{http}/api/activate', 'POST', {'name': 'lab'})[0] == 200
    status = json.loads(request(f'{http}/api/status')[2])
    assert status['active'] == 'lab' and status['access']['state'] == 'ok'
    assert {s['name'] for s in status['sites']} >= {'lab', 'rover_world'}

    assert request(f'{http}/api/sites/lab', 'DELETE')[0] == 400      # active
    assert request(f'{http}/sites/..%2F..%2Fetc/preview.png')[0] == 404
    assert request(f'{http}/api/nothing')[0] == 404
