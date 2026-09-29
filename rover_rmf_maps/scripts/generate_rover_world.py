#!/usr/bin/env python3
# Copyright 2026 Rover A1 contributors
# Licensed under the Apache License, Version 2.0.

"""
Generate the RMF building map for rover_gazebo's world from its SDF.

    generate_rover_world.py [--sdf .../rover_world/world/rover_world.sdf] [--out maps/rover_world]

Writes rover_world.png (floor plan: every box collision of the world, 1 m grid) and
rover_world.building.yaml (traffic-editor format, level L1, the waypoints and lanes below).
Both are committed; re-run after changing the world or the graph. traffic-editor can open the
building file too, but edits made there are lost on the next run of this script.

Coordinates. The building uses traffic-editor's reference_image system: pixels, y down, scaled
by a measurement. The image covers the 25 x 25 m ground plane at RESOLUTION m/px with its
top-left corner at world (-12.5, 12.5), so

    RMF (x, y) = world (x + 12.5, y - 12.5)

rover_rmf_bringup/config/rover_fleet.yaml's `transforms` then maps RMF to the rover's frame.
"""

import argparse
import math
import os
import struct
import xml.etree.ElementTree as ET
import zlib

HALF_SIZE = 12.5          # m, half the ground plane (rover_world.sdf: plane 25 x 25)
RESOLUTION = 0.05         # m per pixel
PIXELS = int(round(2 * HALF_SIZE / RESOLUTION))

# World coordinates (m). Kept >= 1.5 m clear of obstacles and >= 2 m from the walls at +/-11 m:
# the rover's footprint is 0.913 x 0.803 m (circumscribed radius 0.61 m).
CHARGER = 'rover_a1_charger'
WAYPOINTS = {
    CHARGER: (0.0, -2.0),     # the spawn pose (rover_gazebo simulate_robot.launch.py x/y)
    'south': (0.0, -7.0),
    'southeast': (7.0, -8.0),
    'east': (9.0, 0.0),
    'northeast': (9.0, 9.0),
    'north': (0.0, 8.0),
    'northwest': (-9.0, 9.0),
    'west': (-9.0, 0.0),
    'southwest': (-8.0, -8.0),
}
# A ring around the yard plus three spokes from the charger; all lanes two-way.
LANES = [
    ('south', 'southeast'), ('southeast', 'east'), ('east', 'northeast'),
    ('northeast', 'north'), ('north', 'northwest'), ('northwest', 'west'),
    ('west', 'southwest'), ('southwest', 'south'),
    (CHARGER, 'south'), (CHARGER, 'east'), (CHARGER, 'west'),
]


def world_to_pixel(x, y):
    return (x + HALF_SIZE) / RESOLUTION, (HALF_SIZE - y) / RESOLUTION


def parse_pose(text):
    values = [float(v) for v in (text or '').split()] + [0.0] * 6
    return values[0], values[1], values[5]


def boxes(sdf_path):
    """(x, y, yaw, size_x, size_y) of every box collision, in world coordinates."""
    root = ET.parse(sdf_path).getroot()
    found = []
    for model in root.iter('model'):
        mx, my, myaw = parse_pose(model.findtext('pose'))
        for link in model.findall('link'):
            lx, ly, lyaw = parse_pose(link.findtext('pose'))
            for collision in link.findall('collision'):
                size = collision.findtext('geometry/box/size')
                if not size:
                    continue
                cx, cy, cyaw = parse_pose(collision.findtext('pose'))
                sx, sy, _ = (float(v) for v in size.split())
                # Compose model * link * collision (planar).
                x, y, yaw = mx, my, myaw
                for dx, dy, dyaw in ((lx, ly, lyaw), (cx, cy, cyaw)):
                    x += dx * math.cos(yaw) - dy * math.sin(yaw)
                    y += dx * math.sin(yaw) + dy * math.cos(yaw)
                    yaw += dyaw
                found.append((x, y, yaw, sx, sy))
    return found


def render(obstacles):
    free, grid, grid_major, obstacle = 250, 228, 200, 70
    image = [[free] * PIXELS for _ in range(PIXELS)]
    per_metre = int(round(1.0 / RESOLUTION))
    for i in range(PIXELS):
        on_major = (i % (5 * per_metre)) == 0
        if i % per_metre == 0:
            shade = grid_major if on_major else grid
            for j in range(PIXELS):
                image[i][j] = min(image[i][j], shade)
                image[j][i] = min(image[j][i], shade)

    for x, y, yaw, sx, sy in obstacles:
        c, s = math.cos(yaw), math.sin(yaw)
        reach = math.hypot(sx, sy) / 2
        u0, v0 = world_to_pixel(x - reach, y + reach)
        u1, v1 = world_to_pixel(x + reach, y - reach)
        for row in range(max(0, int(v0)), min(PIXELS, int(v1) + 1)):
            for col in range(max(0, int(u0)), min(PIXELS, int(u1) + 1)):
                wx = (col + 0.5) * RESOLUTION - HALF_SIZE - x
                wy = HALF_SIZE - (row + 0.5) * RESOLUTION - y
                if abs(wx * c + wy * s) <= sx / 2 and abs(-wx * s + wy * c) <= sy / 2:
                    image[row][col] = obstacle
    return image


def write_png(path, image):
    """8-bit greyscale PNG, stdlib only."""
    raw = b''.join(b'\x00' + bytes(row) for row in image)

    def chunk(kind, data):
        body = kind + data
        return struct.pack('>I', len(data)) + body + struct.pack('>I', zlib.crc32(body))

    header = struct.pack('>IIBBBBB', len(image[0]), len(image), 8, 0, 0, 0, 0)
    with open(path, 'wb') as f:
        f.write(b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', header)
                + chunk(b'IDAT', zlib.compress(raw, 9)) + chunk(b'IEND', b''))


def building_yaml(image_name):
    names = list(WAYPOINTS)
    lines = [
        '# Generated by rover_rmf_maps/scripts/generate_rover_world.py from rover_world.sdf.',
        '# Edit the script (WAYPOINTS / LANES) and re-run it instead of editing this file.',
        'coordinate_system: reference_image',
        'lifts: {}',
        'name: rover_world',
        'levels:',
        '  L1:',
        '    drawing:',
        f'      filename: {image_name}',
        '    elevation: 0',
        '    measurements:',
        # vertices 0 and 1 span the image width: the level scale is RESOLUTION m/px.
        f'      - [0, 1, {{distance: [3, {2 * HALF_SIZE}]}}]',
        '    vertices:',
        '      - [0, 0, 0, ""]',
        f'      - [{PIXELS}, 0, 0, ""]',
    ]
    for name in names:
        u, v = world_to_pixel(*WAYPOINTS[name])
        if name == CHARGER:
            params = ('{is_charger: [4, true], is_holding_point: [4, true], '
                      'is_parking_spot: [4, true]}')
        else:
            params = '{is_holding_point: [4, true], is_parking_spot: [4, false]}'
        lines.append(f'      - [{u:.1f}, {v:.1f}, 0, {name}, {params}]')
    lines.append('    lanes:')
    for a, b in LANES:
        ia, ib = names.index(a) + 2, names.index(b) + 2  # +2: the measurement vertices
        lines.append(
            f'      - [{ia}, {ib}, {{bidirectional: [4, true], graph_idx: [2, 0], '
            f'mutex: [1, ""], orientation: [1, ""], speed_limit: [3, 0]}}]')
    return '\n'.join(lines) + '\n'


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--sdf', default=os.path.normpath(os.path.join(
        here, '../../../rover_ros/rover_world/world/rover_world.sdf')))
    parser.add_argument('--out', default=os.path.join(here, '../maps/rover_world'))
    args = parser.parse_args()

    os.makedirs(args.out, exist_ok=True)
    obstacles = boxes(args.sdf)
    write_png(os.path.join(args.out, 'rover_world.png'), render(obstacles))
    with open(os.path.join(args.out, 'rover_world.building.yaml'), 'w') as f:
        f.write(building_yaml('rover_world.png'))
    print(f'{len(obstacles)} obstacles, {len(WAYPOINTS)} waypoints, {len(LANES)} lanes '
          f'-> {os.path.normpath(args.out)}')


if __name__ == '__main__':
    main()
