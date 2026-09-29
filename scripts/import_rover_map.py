#!/usr/bin/env python3
# Copyright 2026 Rover A1 contributors
# Licensed under the Apache License, Version 2.0.

"""
Import a map saved on the rover as an Open-RMF site.

    import_rover_map.py lab --from-rover root@192.168.1.201     # scp from the rover
    import_rover_map.py lab --dir ~/rover_maps/lab               # a local copy

The map is one saved with the drive UI in indoor mode (rover_indoor_nav_manager:
/maps/<name>/map.yaml, map.pgm, places.yaml). Every place becomes an RMF waypoint; the place
named --charger (default "dock") is where the rover parks and charges. Writes:

    rover_rmf_maps/maps/<name>/<name>.png, <name>.building.yaml   floor plan, waypoints, lanes
    rover_rmf_bringup/config/fleet_<name>.yaml                    transform, map name, broker

The RMF level is named <name>, like the rover's map, because the rover reports that name as its
VDA 5050 mapId. Then rebuild and start RMF for the site:

    RMF_SITE=<name> RMF_BROKER_HOST=192.168.1.201 \\
        docker compose -f docker/docker-compose.yml up -d --build
"""

import argparse
import os
import re
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(REPO, 'rover_rmf_maps', 'scripts'))

from building_writer import write_png  # noqa: E402
from rover_map_import import fleet_config, import_site, MapImportError  # noqa: E402
import yaml  # noqa: E402

TEMPLATE = os.path.join(REPO, 'rover_rmf_bringup', 'config', 'fleet_rover_world.yaml')


def fetch(name: str, ssh_target: str, ssh_port: int, into: str) -> str:
    """Copy /maps/<name>/ from the rover's orchestrator container (sshd on port 24)."""
    target = os.path.join(into, name)
    os.makedirs(target)
    for file in ('map.yaml', 'map.pgm', 'places.yaml'):
        cmd = ['scp', '-q', '-P', str(ssh_port), '-o', 'ConnectTimeout=10',
               f'{ssh_target}:/maps/{name}/{file}', target]
        if subprocess.run(cmd).returncode != 0:
            if file == 'places.yaml':
                continue  # reported below as "no places"
            sys.exit(f'Could not copy /maps/{name}/{file} from {ssh_target} (port {ssh_port}). '
                     'Is the map saved, and the rover reachable?')
    return target


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('name', help='map name on the rover (becomes the RMF site and level)')
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument('--from-rover', metavar='USER@HOST', help='scp the map from the rover')
    source.add_argument('--dir', help='local directory with map.yaml, map.pgm, places.yaml')
    parser.add_argument('--ssh-port', type=int, default=24,
                        help="the orchestrator container's sshd (default 24)")
    parser.add_argument('--charger', default='dock', help='place where the rover parks')
    parser.add_argument('--clearance', type=float, default=0.8,
                        help='m kept from obstacles along lanes (footprint radius 0.61 + margin)')
    parser.add_argument('--max-lane', type=float, default=15.0, help='longest lane, m')
    parser.add_argument('--neighbours', type=int, default=3,
                        help='shortest lanes kept per place, on top of a spanning tree')
    parser.add_argument('--broker-host', default='192.168.1.201',
                        help="MQTT broker of the rover's VDA 5050 connector")
    parser.add_argument('--broker-port', type=int, default=1883)
    args = parser.parse_args()

    if not re.fullmatch(r'[A-Za-z0-9_-]{1,64}', args.name):
        sys.exit(f'{args.name!r} is not a rover map name ([A-Za-z0-9_-], up to 64)')
    if args.name == 'rover_world':
        sys.exit('rover_world is the Gazebo site; pick another name')

    with tempfile.TemporaryDirectory() as tmp:
        map_dir = (fetch(args.name, args.from_rover, args.ssh_port, tmp) if args.from_rover
                   else os.path.expanduser(args.dir))
        try:
            site = import_site(args.name, map_dir, args.charger, args.clearance,
                               args.max_lane, args.neighbours)
        except MapImportError as e:
            sys.exit(f'Import failed: {e}')

    out = os.path.join(REPO, 'rover_rmf_maps', 'maps', args.name)
    os.makedirs(out, exist_ok=True)
    write_png(os.path.join(out, f'{args.name}.png'), site.floor_plan)
    with open(os.path.join(out, f'{args.name}.building.yaml'), 'w') as f:
        f.write(site.building())

    with open(TEMPLATE) as f:
        template = yaml.safe_load(f)
    charger = next(w.name for w in site.waypoints if w.charger)
    config = fleet_config(template, site, charger, args.broker_host, args.broker_port)
    config_path = os.path.join(REPO, 'rover_rmf_bringup', 'config', f'fleet_{args.name}.yaml')
    with open(config_path, 'w') as f:
        f.write(f'# Generated by scripts/import_rover_map.py for the rover map {args.name!r}.\n'
                '# Same values as fleet_rover_world.yaml (see its comments for sources), except\n'
                '# transforms, vda5050.map_name, the charger and the broker. Re-running the\n'
                '# importer overwrites this file.\n')
        yaml.safe_dump(config, f, sort_keys=False, default_flow_style=None)

    tx, ty = site.translation
    print(f'Site {args.name}: {site.width}x{site.height} px at {site.resolution} m/px, '
          f'{len(site.waypoints)} waypoints, {len(site.lanes)} lanes, charger {charger!r}')
    print(f'  robot = RMF + ({tx:.3f}, {ty:.3f})')
    for a, b in site.lanes:
        print(f'  lane {a} <-> {b}')
    for w in site.warnings:
        print(f'  WARNING: {w}')
    print(f'Wrote {os.path.relpath(out, REPO)}/ and {os.path.relpath(config_path, REPO)}')
    print(f'Next: RMF_SITE={args.name} RMF_BROKER_HOST={args.broker_host} '
          'docker compose -f docker/docker-compose.yml up -d --build')


if __name__ == '__main__':
    main()
