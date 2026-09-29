# Copyright 2026 Rover A1 contributors
# Licensed under the Apache License, Version 2.0.

"""
Where rover maps come from, and where RMF sites go.

- DirMapSource / SshMapSource list the maps rover_indoor_nav_manager saved (/maps/<name>/
  map.yaml, map.pgm, places.yaml) and fetch one. The rover side is the orchestrator container's
  sshd (port 24); one ssh connection per call, tar instead of scp.
- write_site() writes an imported Site as an RMF site (floor plan, building, fleet config).
- render_preview() draws the floor plan with its waypoints and lanes, for the site manager page.

Shared by scripts/import_rover_map.py (CLI, writes into the repo) and site_manager/ (web page,
writes into the rmf-sites volume). Pure Python + PyYAML.
"""

import io
import os
import shutil
import subprocess
import tarfile
from typing import Dict, List, Sequence, Tuple

from building_writer import write_png
from rover_map_import import fleet_config, MapImportError, Site
import yaml

MAP_FILES = ('map.yaml', 'map.pgm', 'places.yaml')


class MapSourceError(MapImportError):
    """The rover (or directory) could not provide the map; the message says why."""


def _describe(files: Dict[str, Tuple[bytes, float]], active: str) -> List[dict]:
    """{'<map>/map.yaml': (content, mtime), ...} -> one entry per map directory, sorted."""
    maps: Dict[str, dict] = {}
    for path, (content, mtime) in files.items():
        name, _, file = path.partition('/')
        entry = maps.setdefault(name, {'name': name, 'has_map': False, 'places': [],
                                       'modified': 0.0, 'active': name == active})
        if file == 'map.yaml':
            entry['has_map'] = True
            entry['modified'] = max(entry['modified'], mtime)
        elif file == 'places.yaml':
            try:
                data = yaml.safe_load(content) or {}
                entry['places'] = [str(p['name']) for p in data.get('places') or []
                                   if isinstance(p, dict) and 'name' in p]
            except yaml.YAMLError:
                entry['places'] = []
    return sorted(maps.values(), key=lambda m: m['name'])


class DirMapSource:
    """Maps in a local directory laid out like the rover's /maps (the sim's --maps-dir)."""

    def __init__(self, path: str):
        self.path = os.path.expanduser(path)

    def describe(self) -> str:
        return f'directory {self.path}'

    def check(self) -> Tuple[str, str]:
        return ('ok', '') if os.path.isdir(self.path) else ('unreachable', f'no {self.path}')

    def list_maps(self) -> List[dict]:
        if not os.path.isdir(self.path):
            raise MapSourceError(f'no maps directory {self.path}')
        files = {}
        for name in os.listdir(self.path):
            for file in ('map.yaml', 'places.yaml'):
                path = os.path.join(self.path, name, file)
                if os.path.isfile(path):
                    with open(path, 'rb') as f:
                        # Whole seconds, like the tar the ssh source reads.
                        mtime = float(int(os.path.getmtime(path)))
                        files[f'{name}/{file}'] = (f.read(), mtime)
        return _describe(files, self._active())

    def _active(self) -> str:
        try:
            with open(os.path.join(self.path, 'active')) as f:
                return f.read().strip()
        except OSError:
            return ''

    def fetch(self, name: str, into: str) -> str:
        source = os.path.join(self.path, name)
        if not os.path.isfile(os.path.join(source, 'map.yaml')):
            names = ', '.join(m['name'] for m in self.list_maps()) or 'none'
            raise MapSourceError(f'no map named {name!r} in {self.path} (maps: {names})')
        target = os.path.join(into, name)
        os.makedirs(target)
        for file in MAP_FILES:
            if os.path.isfile(os.path.join(source, file)):
                shutil.copy(os.path.join(source, file), target)
        return target


# Run on the rover. {dir} and {name} are filled in; names were validated by the caller.
LIST_SCRIPT = """
cd {dir} 2>/dev/null || exit 3
find . -mindepth 1 -maxdepth 2 \\( -name map.yaml -o -name places.yaml -o -name active \\) \\
    | tar cf - -T -
"""
FETCH_SCRIPT = """
cd {dir}/{name} 2>/dev/null || {{
    echo "maps on the rover: $(cd {dir} 2>/dev/null && ls -d */ | tr -d / | tr '\\n' ' ')" >&2
    exit 3; }}
[ -f map.yaml ] && [ -f map.pgm ] || {{ echo "{dir}/{name} has no map.yaml/map.pgm" >&2; exit 4; }}
tar cf - map.yaml map.pgm $([ -f places.yaml ] && echo places.yaml)
"""


class SshMapSource:
    """
    Maps on the rover, over ssh to its orchestrator container.

    batch=True (the site manager) never prompts: no key accepted means an error, reported as
    'needs_key' by check(). batch=False (the CLI) lets ssh ask for the password.
    """

    def __init__(self, target: str, port: int = 24, remote_dir: str = '/maps',
                 batch: bool = True, ssh: Sequence[str] = ('ssh',), timeout: float = 30.0):
        self.target = target
        self.port = port
        self.remote_dir = remote_dir
        self.batch = batch
        self.ssh = list(ssh)
        self.timeout = timeout

    def describe(self) -> str:
        return f'{self.target} (ssh port {self.port})'

    def _run(self, script: str) -> subprocess.CompletedProcess:
        options = ['-o', 'ConnectTimeout=10', '-o', 'StrictHostKeyChecking=accept-new']
        if self.batch:
            options += ['-o', 'BatchMode=yes']
        cmd = self.ssh + ['-p', str(self.port)] + options + [self.target, script]
        try:
            return subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                  timeout=self.timeout)
        except subprocess.TimeoutExpired:
            raise MapSourceError(f'{self.describe()} did not answer within {self.timeout:.0f} s')

    @staticmethod
    def _error(result: subprocess.CompletedProcess) -> str:
        lines = [line for line in result.stderr.decode(errors='replace').splitlines()
                 if line.strip() and not line.startswith('Warning: Permanently added')]
        return ' '.join(lines)

    def check(self) -> Tuple[str, str]:
        """('ok' | 'needs_key' | 'unreachable', detail) without ever prompting."""
        options = ['-o', 'ConnectTimeout=5', '-o', 'StrictHostKeyChecking=accept-new',
                   '-o', 'BatchMode=yes']
        try:
            result = subprocess.run(
                self.ssh + ['-p', str(self.port)] + options + [self.target, 'true'],
                stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=15)
        except subprocess.TimeoutExpired:
            return 'unreachable', 'no answer within 15 s'
        if result.returncode == 0:
            return 'ok', ''
        error = self._error(result)
        # A new rover release forgets our key and changes its host key; rover-authorize fixes both.
        if 'Permission denied' in error or 'REMOTE HOST IDENTIFICATION HAS CHANGED' in error:
            return 'needs_key', error
        return 'unreachable', error

    def list_maps(self) -> List[dict]:
        result = self._run(LIST_SCRIPT.format(dir=self.remote_dir))
        if result.returncode == 3:
            raise MapSourceError(f'{self.remote_dir} does not exist on {self.describe()}')
        if result.returncode != 0 and not result.stdout:
            raise MapSourceError(f'ssh to {self.describe()} failed: {self._error(result)}')
        files, active = {}, ''
        if result.stdout:
            with tarfile.open(fileobj=io.BytesIO(result.stdout)) as archive:
                for member in archive.getmembers():
                    if not member.isfile():
                        continue
                    content = archive.extractfile(member).read()
                    path = member.name[2:] if member.name.startswith('./') else member.name
                    if path == 'active':
                        active = content.decode(errors='replace').strip()
                    else:
                        files[path] = (content, float(member.mtime))
        return _describe(files, active)

    def fetch(self, name: str, into: str) -> str:
        result = self._run(FETCH_SCRIPT.format(dir=self.remote_dir, name=name))
        error = self._error(result)
        if result.returncode == 3:
            raise MapSourceError(f'no map named {name!r} on the rover. {error}')
        if result.returncode == 4:
            raise MapSourceError(f'{error}. Save the map in the drive UI first.')
        if result.returncode != 0:
            raise MapSourceError(f'ssh to {self.describe()} failed ({result.returncode}): {error}')
        target = os.path.join(into, name)
        os.makedirs(target)
        with tarfile.open(fileobj=io.BytesIO(result.stdout)) as archive:
            archive.extractall(target, filter='data')
        return target


def write_site(site: Site, out_dir: str, config_path: str, template: dict, broker_host: str,
               broker_port: int, origin: str) -> str:
    """Floor plan + building into out_dir, fleet config to config_path. Returns the charger."""
    os.makedirs(out_dir, exist_ok=True)
    write_png(os.path.join(out_dir, f'{site.name}.png'), site.floor_plan)
    with open(os.path.join(out_dir, f'{site.name}.building.yaml'), 'w') as f:
        f.write(site.building())
    charger = next(w.name for w in site.waypoints if w.charger)
    config = fleet_config(template, site, charger, broker_host, broker_port)
    with open(config_path, 'w') as f:
        f.write(f'# Generated by {origin} for the rover map {site.name!r}.\n'
                '# Same values as fleet_rover_world.yaml (see its comments for sources), except\n'
                '# transforms, vda5050.map_name, the charger and the broker. Re-importing the\n'
                '# map overwrites this file.\n')
        yaml.safe_dump(config, f, sort_keys=False, default_flow_style=None)
    return charger


LANE = (40, 110, 220)
WAYPOINT = (240, 140, 20)
CHARGER = (30, 170, 70)


def render_preview(site: Site, path: str, crop: bool = True) -> None:
    """
    The floor plan in colour with lanes (blue), waypoints (orange) and the charger (green).

    With crop, the image is cut to the waypoints plus a 3 m margin, so a small site in a big
    SLAM map stays readable.
    """
    xs = [w.px for w in site.waypoints]
    ys = [w.py for w in site.waypoints]
    margin = max(40, int(3.0 / site.resolution))
    x0, x1 = max(0, int(min(xs)) - margin), min(site.width, int(max(xs)) + margin + 1)
    y0, y1 = max(0, int(min(ys)) - margin), min(site.height, int(max(ys)) + margin + 1)
    if not crop:
        x0, x1, y0, y1 = 0, site.width, 0, site.height
    image = [[v for v in site.floor_plan[r][x0:x1] for _ in range(3)] for r in range(y0, y1)]
    width, height = x1 - x0, y1 - y0

    def dot(cx, cy, radius, colour):
        for r in range(int(cy - radius), int(cy + radius) + 1):
            for c in range(int(cx - radius), int(cx + radius) + 1):
                if 0 <= r < height and 0 <= c < width and \
                        (c - cx) ** 2 + (r - cy) ** 2 <= radius ** 2:
                    image[r][3 * c:3 * c + 3] = colour

    points = {w.name: (w.px - x0, w.py - y0) for w in site.waypoints}
    for a, b in site.lanes:
        (ax, ay), (bx, by) = points[a], points[b]
        steps = max(1, int(max(abs(bx - ax), abs(by - ay))))
        for i in range(steps + 1):
            dot(ax + (bx - ax) * i / steps, ay + (by - ay) * i / steps, 1.5, LANE)
    for w in site.waypoints:
        x, y = points[w.name]
        dot(x, y, 5, CHARGER if w.charger else WAYPOINT)
    write_png(path, image, rgb=True)
