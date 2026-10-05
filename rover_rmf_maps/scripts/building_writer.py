# Copyright 2026 Mechatronics Academy
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Write RMF building maps (traffic-editor format) and their floor plans, stdlib only.

Shared by generate_rover_world.py (the Gazebo world) and ../../scripts/import_rover_map.py
(maps saved on the real rover). Buildings use the reference_image coordinate system: vertices in
pixels of the floor plan, y down, with one measurement across the image width that sets the
scale. building_map_generator then gives RMF coordinates

    RMF (x, y) = (px * resolution, -py * resolution)
"""

from dataclasses import dataclass
import json
import re
import struct
from typing import Iterable, List, Sequence, Tuple
import zlib


@dataclass(frozen=True)
class Waypoint:
    name: str
    px: float           # floor plan pixel column
    py: float           # floor plan pixel row (y down)
    charger: bool = False


def _yaml_name(name: str) -> str:
    # Place names come from the drive UI and may hold spaces or YAML syntax; quote those.
    return name if re.fullmatch(r'[A-Za-z0-9_-]+', name) else json.dumps(name)


def write_png(path: str, image: Sequence[Sequence[int]], rgb: bool = False) -> None:
    """
    8-bit PNG from rows of values.

    Greyscale by default (one 0..255 value per pixel). With rgb=True each row holds r, g, b
    values in turn, three per pixel.
    """
    raw = b''.join(b'\x00' + bytes(row) for row in image)

    def chunk(kind, data):
        body = kind + data
        return struct.pack('>I', len(data)) + body + struct.pack('>I', zlib.crc32(body))

    width = len(image[0]) // 3 if rgb else len(image[0])
    header = struct.pack('>IIBBBBB', width, len(image), 8, 2 if rgb else 0, 0, 0, 0)
    with open(path, 'wb') as f:
        f.write(b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', header)
                + chunk(b'IDAT', zlib.compress(raw, 9)) + chunk(b'IEND', b''))


def building_yaml(building: str, level: str, image_name: str, image_width_px: int,
                  image_width_m: float, waypoints: List[Waypoint],
                  lanes: Iterable[Tuple[str, str]], comments: Iterable[str] = ()) -> str:
    """
    One level, its floor plan, waypoints and two-way lanes (graph 0).

    The charger waypoint is also a parking spot; every waypoint is a holding point, so RMF may
    leave the rover waiting there.
    """
    names = [w.name for w in waypoints]
    if len(set(names)) != len(names):
        raise ValueError(f'duplicate waypoint names: {names}')
    lines = [f'# {c}' for c in comments] + [
        'coordinate_system: reference_image',
        'lifts: {}',
        f'name: {building}',
        'levels:',
        f'  {level}:',
        '    drawing:',
        f'      filename: {image_name}',
        '    elevation: 0',
        '    measurements:',
        # vertices 0 and 1 span the image width: that sets the level scale in m/px.
        f'      - [0, 1, {{distance: [3, {image_width_m}]}}]',
        '    vertices:',
        '      - [0, 0, 0, ""]',
        f'      - [{image_width_px}, 0, 0, ""]',
    ]
    for w in waypoints:
        if w.charger:
            params = ('{is_charger: [4, true], is_holding_point: [4, true], '
                      'is_parking_spot: [4, true]}')
        else:
            params = '{is_holding_point: [4, true], is_parking_spot: [4, false]}'
        lines.append(f'      - [{w.px:.1f}, {w.py:.1f}, 0, {_yaml_name(w.name)}, {params}]')
    lines.append('    lanes:')
    for a, b in lanes:
        ia, ib = names.index(a) + 2, names.index(b) + 2  # +2: the measurement vertices
        lines.append(
            f'      - [{ia}, {ib}, {{bidirectional: [4, true], graph_idx: [2, 0], '
            f'mutex: [1, ""], orientation: [1, ""], speed_limit: [3, 0]}}]')
    return '\n'.join(lines) + '\n'
