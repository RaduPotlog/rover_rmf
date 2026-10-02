#!/usr/bin/env python3
# Copyright 2026 Rover A1 contributors
# Licensed under the Apache License, Version 2.0.

"""
RMF site manager: a small web page that turns maps saved on the rover into RMF sites.

    http://localhost:8020

It lists the maps rover_indoor_nav_manager saved on the rover. Import turns one into a site in
the rmf-sites volume: floor plan, building, nav graph, fleet config and a preview with the
lanes. Activate writes the site's name to <sites>/active; docker/run_rmf.sh, which runs RMF,
restarts RMF on it. Built-in sites (rover_world, and CLI imports in rover_rmf_maps) can be
activated too.

Environment:
    SITES_DIR       the rmf-sites volume (default /sites)
    BUILTIN_MAPS    rover_rmf_maps' installed maps directory
    FLEET_TEMPLATE  fleet config the imported sites copy (fleet_rover_world.yaml)
    ROVER_SSH       user@host of the rover's orchestrator sshd (default root@192.168.1.201)
    ROVER_SSH_PORT  default 24
    ROVER_BROKER    host[:port] written into imported sites (default the ROVER_SSH host, 1883)
    MAPS_DIR        read maps from this directory instead of ssh (the sim's --maps-dir)
    PORT            default 8020
"""

from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile
import threading
import time
from typing import Callable, Optional, Sequence
from urllib.parse import unquote

HERE = os.path.dirname(os.path.abspath(__file__))
for candidate in (os.path.join(HERE, '..', 'rover_rmf_maps', 'scripts'),
                  '/opt/rover_rmf_maps_scripts'):
    if os.path.isdir(candidate):
        sys.path.insert(0, candidate)

from rover_map_import import import_site, MapImportError  # noqa: E402
from site_io import DirMapSource, render_preview, SshMapSource  # noqa: E402
from site_io import write_site  # noqa: E402
import yaml  # noqa: E402

NAME = re.compile(r'[A-Za-z0-9_-]{1,64}')
NAV_GRAPH_CMD = ('ros2', 'run', 'rmf_building_map_tools', 'building_map_generator', 'nav')


class RequestError(ValueError):
    """Bad request: the message goes back to the page as is."""


def _valid_name(name: str) -> str:
    if not isinstance(name, str) or not NAME.fullmatch(name):
        raise RequestError(f'{name!r} is not a map name ([A-Za-z0-9_-], up to 64 characters)')
    return name


class SiteManager:

    def __init__(self, sites_dir: str, builtin_maps: str, template_path: str, source,
                 broker: tuple, nav_graph_cmd: Sequence[str] = NAV_GRAPH_CMD,
                 clock: Callable[[], float] = time.time):
        self.sites_dir = sites_dir
        self.builtin_maps = builtin_maps
        self.template_path = template_path
        self.source = source
        self.broker = broker
        self.nav_graph_cmd = list(nav_graph_cmd)
        self.clock = clock
        self._lock = threading.Lock()          # one import/activate/delete at a time
        self._access = ('unknown', '', 0.0)    # cached rover access check
        os.makedirs(sites_dir, exist_ok=True)

    # --- sites -----------------------------------------------------------------------------

    def _site_dir(self, name: str, builtin: bool) -> str:
        return os.path.join(self.builtin_maps if builtin else self.sites_dir, name)

    def _site_info(self, name: str, builtin: bool) -> Optional[dict]:
        directory = self._site_dir(name, builtin)
        building = os.path.join(directory, f'{name}.building.yaml')
        if not os.path.isfile(building):
            return None
        info = {'name': name, 'builtin': builtin, 'waypoints': [], 'lanes': 0, 'charger': None,
                'modified': os.path.getmtime(building),
                'nav_graph': os.path.isfile(os.path.join(directory, 'nav_graphs', '0.yaml'))}
        try:
            with open(building) as f:
                level = (yaml.safe_load(f).get('levels') or {}).get(name) or {}
            for v in level.get('vertices') or []:
                if len(v) > 3 and v[3]:
                    info['waypoints'].append(str(v[3]))
                    if len(v) > 4 and (v[4] or {}).get('is_charger', [0, False])[1]:
                        info['charger'] = str(v[3])
            info['lanes'] = len(level.get('lanes') or [])
        except (OSError, yaml.YAMLError, AttributeError):
            pass
        meta_path = os.path.join(directory, 'import.json')
        if not builtin and os.path.isfile(meta_path):
            with open(meta_path) as f:
                info['import'] = json.load(f)
        return info

    def sites(self) -> list:
        found = {}
        if os.path.isdir(self.builtin_maps):
            for name in sorted(os.listdir(self.builtin_maps)):
                info = self._site_info(name, builtin=True)
                if info:
                    found[name] = info
        for name in sorted(os.listdir(self.sites_dir)):
            info = self._site_info(name, builtin=False)
            if info:
                info['overrides_builtin'] = name in found
                found[name] = info       # an imported site wins, as in run_rmf.sh
        return sorted(found.values(), key=lambda s: (s['builtin'], s['name']))

    def active(self) -> str:
        try:
            with open(os.path.join(self.sites_dir, 'active')) as f:
                return f.read().strip()
        except OSError:
            return ''

    def rmf_status(self) -> dict:
        try:
            with open(os.path.join(self.sites_dir, 'rmf_status.json')) as f:
                return json.load(f)
        except (OSError, ValueError):
            return {}

    def access(self, max_age: float = 15.0) -> dict:
        state, detail, checked = self._access
        if self.clock() - checked > max_age:
            state, detail = self.source.check()
            self._access = (state, detail, self.clock())
        return {'state': state, 'detail': detail, 'source': self.source.describe()}

    def status(self) -> dict:
        return {'active': self.active(), 'rmf': self.rmf_status(), 'sites': self.sites(),
                'access': self.access(), 'public_key': public_key(),
                'broker': f'{self.broker[0]}:{self.broker[1]}'}

    def rover_maps(self) -> list:
        return self.source.list_maps()

    # --- actions ---------------------------------------------------------------------------

    def import_map(self, request: dict) -> dict:
        name = _valid_name(request.get('name'))
        if name == 'rover_world':
            raise RequestError('rover_world is the Gazebo site; save the map under another name')
        charger = str(request.get('charger') or 'dock')
        try:
            clearance = float(request.get('clearance', 0.8))
            max_lane = float(request.get('max_lane', 15.0))
            neighbours = int(request.get('neighbours', 3))
        except (TypeError, ValueError):
            raise RequestError('clearance, max_lane and neighbours must be numbers')
        if not (clearance > 0 and max_lane > 0 and neighbours >= 0):
            raise RequestError('clearance and max_lane must be > 0, neighbours >= 0')

        with self._lock, tempfile.TemporaryDirectory() as tmp:
            map_dir = self.source.fetch(name, tmp)
            site = import_site(name, map_dir, charger, clearance, max_lane, neighbours)
            # Build the new site next to the old one, swap only once the nav graph exists: a
            # failed import leaves the previous version (maybe the one RMF runs) intact.
            staging = os.path.join(self.sites_dir, f'.{name}.new')
            shutil.rmtree(staging, ignore_errors=True)
            try:
                charger_name = self._build(site, staging)
            except BaseException:  # noqa: B902 cleanup, then re-raised
                shutil.rmtree(staging, ignore_errors=True)
                raise
            summary = {
                'name': name, 'charger': charger_name, 'imported': self.clock(),
                'source': self.source.describe(),
                'size_px': [site.width, site.height], 'resolution': site.resolution,
                'translation': list(site.translation),
                'waypoints': [w.name for w in site.waypoints],
                'lanes': [list(lane) for lane in site.lanes], 'warnings': site.warnings,
                'options': {'clearance': clearance, 'max_lane': max_lane,
                            'neighbours': neighbours}}
            with open(os.path.join(staging, 'import.json'), 'w') as f:
                json.dump(summary, f, indent=1)
            final = os.path.join(self.sites_dir, name)
            shutil.rmtree(final, ignore_errors=True)
            os.rename(staging, final)
        summary['active'] = self.active() == name
        return summary

    def _build(self, site, directory: str) -> str:
        with open(self.template_path) as f:
            template = yaml.safe_load(f)
        charger = write_site(site, directory, os.path.join(directory, f'fleet_{site.name}.yaml'),
                             template, self.broker[0], self.broker[1],
                             origin='the RMF site manager')
        self._nav_graph(os.path.join(directory, f'{site.name}.building.yaml'),
                        os.path.join(directory, 'nav_graphs'))
        render_preview(site, os.path.join(directory, 'preview.png'))
        return charger

    def _nav_graph(self, building: str, out_dir: str) -> None:
        result = subprocess.run(self.nav_graph_cmd + [building, out_dir],
                                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=120)
        if result.returncode != 0 or not os.path.isfile(os.path.join(out_dir, '0.yaml')):
            output = result.stdout.decode(errors='replace').strip().splitlines()[-5:]
            raise MapImportError('building_map_generator could not make the nav graph: '
                                 + ' | '.join(output))

    def activate(self, request: dict) -> dict:
        name = _valid_name(request.get('name'))
        if not any(s['name'] == name for s in self.sites()):
            raise RequestError(f'no site {name!r}; import it first')
        with self._lock:
            path = os.path.join(self.sites_dir, 'active')
            with open(path + '.tmp', 'w') as f:
                f.write(name + '\n')
            os.replace(path + '.tmp', path)
        return {'active': name}

    def delete(self, name: str) -> dict:
        name = _valid_name(name)
        directory = self._site_dir(name, builtin=False)
        if not os.path.isfile(os.path.join(directory, f'{name}.building.yaml')):
            raise RequestError(f'{name!r} is not an imported site (built-in sites stay)')
        if self.active() == name or self.rmf_status().get('site') == name:
            raise RequestError(f'RMF runs {name!r}; activate another site first')
        with self._lock:
            shutil.rmtree(directory)
        return {'deleted': name}

    def image(self, name: str, which: str) -> Optional[str]:
        """preview.png of an imported site, or a site's plain floor plan."""
        name = _valid_name(name)
        for builtin in (False, True):
            directory = self._site_dir(name, builtin)
            path = os.path.join(directory, 'preview.png' if which == 'preview'
                                else f'{name}.png')
            if os.path.isfile(path):
                return path
        return None


def public_key() -> str:
    path = os.path.expanduser('~/.ssh/id_ed25519.pub')
    try:
        with open(path) as f:
            return f.read().strip()
    except OSError:
        return ''


def ensure_key() -> None:
    """The site manager's own ssh key, made once (it lives in the site-manager-ssh volume)."""
    key = os.path.expanduser('~/.ssh/id_ed25519')
    if not os.path.isfile(key):
        os.makedirs(os.path.dirname(key), mode=0o700, exist_ok=True)
        subprocess.run(['ssh-keygen', '-q', '-t', 'ed25519', '-N', '', '-C',
                        'rover-rmf-site-manager', '-f', key], check=True)


# --- HTTP ---------------------------------------------------------------------------------------

def make_handler(manager: SiteManager, static_dir: str):

    class Handler(BaseHTTPRequestHandler):
        server_version = 'RoverRmfSites/1'

        def log_message(self, fmt, *args):
            if not self.path.startswith('/api/status'):
                sys.stderr.write('site-manager: ' + fmt % args + '\n')

        def _send(self, code: int, body: bytes, content_type: str):
            self.send_response(code)
            self.send_header('Content-Type', content_type)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(body)

        def _json(self, code: int, data) -> None:
            self._send(code, json.dumps(data).encode(), 'application/json')

        def _call(self, action):
            try:
                self._json(200, action())
            except (RequestError, MapImportError) as e:
                self._json(400, {'error': str(e)})
            except Exception as e:  # noqa: B902 the page shows it; the server keeps running
                self._json(500, {'error': f'{type(e).__name__}: {e}'})

        def _body(self) -> dict:
            length = int(self.headers.get('Content-Length') or 0)
            try:
                data = json.loads(self.rfile.read(length) or b'{}')
            except ValueError:
                raise RequestError('the request body is not JSON')
            if not isinstance(data, dict):
                raise RequestError('the request body must be a JSON object')
            return data

        def do_GET(self):  # noqa: N802
            path = unquote(self.path.split('?')[0])
            if path in ('/', '/index.html'):
                with open(os.path.join(static_dir, 'index.html'), 'rb') as f:
                    return self._send(200, f.read(), 'text/html; charset=utf-8')
            if path == '/api/status':
                return self._call(manager.status)
            if path == '/api/rover/maps':
                return self._call(manager.rover_maps)
            match = re.fullmatch(r'/sites/([^/]+)/(preview|floorplan)\.png', path)
            if match:
                try:
                    image = manager.image(match.group(1), match.group(2))
                except RequestError:
                    image = None
                if image:
                    with open(image, 'rb') as f:
                        return self._send(200, f.read(), 'image/png')
            self._json(404, {'error': 'not found'})

        def do_POST(self):  # noqa: N802
            if self.path == '/api/import':
                return self._call(lambda: manager.import_map(self._body()))
            if self.path == '/api/activate':
                return self._call(lambda: manager.activate(self._body()))
            self._json(404, {'error': 'not found'})

        def do_DELETE(self):  # noqa: N802
            match = re.fullmatch(r'/api/sites/([^/]+)', unquote(self.path))
            if match:
                return self._call(lambda: manager.delete(match.group(1)))
            self._json(404, {'error': 'not found'})

    return Handler


def main():
    ssh_target = os.environ.get('ROVER_SSH', 'root@192.168.1.201')
    host = ssh_target.split('@')[-1]
    broker = os.environ.get('ROVER_BROKER', f'{host}:1883')
    broker_host, _, broker_port = broker.partition(':')
    if os.environ.get('MAPS_DIR'):
        source = DirMapSource(os.environ['MAPS_DIR'])
    else:
        ensure_key()
        source = SshMapSource(ssh_target, int(os.environ.get('ROVER_SSH_PORT', '24')))
    manager = SiteManager(
        sites_dir=os.environ.get('SITES_DIR', '/sites'),
        builtin_maps=os.environ.get(
            'BUILTIN_MAPS', '/rmf_ws/install/rover_rmf_maps/share/rover_rmf_maps/maps'),
        template_path=os.environ.get(
            'FLEET_TEMPLATE',
            '/rmf_ws/install/rover_rmf_bringup/share/rover_rmf_bringup/config/'
            'fleet_rover_world.yaml'),
        source=source, broker=(broker_host, int(broker_port or 1883)))
    port = int(os.environ.get('PORT', '8020'))
    server = ThreadingHTTPServer(('0.0.0.0', port),
                                 make_handler(manager, os.path.join(HERE, 'static')))
    print(f'RMF site manager on :{port}; maps from {source.describe()}, sites in '
          f'{manager.sites_dir}', flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
