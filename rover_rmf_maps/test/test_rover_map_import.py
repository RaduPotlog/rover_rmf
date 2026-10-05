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

import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))

from rover_map_import import (  # noqa: E402
    classify, fleet_config, FREE, import_site, load_map_yaml, MapImportError, OCCUPIED,
    read_pgm, UNKNOWN)
import yaml  # noqa: E402

# A 6 x 4 m map at 0.1 m/px, origin (-1, -2): a wall 3 px thick at columns 29..31 with a 1.1 m
# gap in rows 15..25. map_server pixels: 254 free, 0 occupied, 205 unknown.
W, H, RES, ORIGIN = 60, 40, 0.1, (-1.0, -2.0)


def world(px, py):
    return ORIGIN[0] + px * RES, ORIGIN[1] + (H - py) * RES


PLACES = {                      # name: pixel (px, py)
    'dock': (10, 20),
    'left_top': (10, 6),
    'gap': (30, 20),
    'right': (50, 20),
    'right_top': (50, 6),
}


def write_map(directory, places=PLACES, gap=True, negate=False, unknown_block=None):
    rows = []
    for py in range(H):
        row = []
        for px in range(W):
            value = 254
            if 29 <= px <= 31 and not (gap and 15 <= py <= 25):
                value = 0
            if unknown_block and unknown_block[0] <= px <= unknown_block[1] and py >= 34:
                value = 205
            row.append(255 - value if negate else value)
        rows.append(bytes(row))
    with open(os.path.join(directory, 'map.pgm'), 'wb') as f:
        f.write(f'P5\n# CREATOR: test\n{W} {H}\n255\n'.encode() + b''.join(rows))
    with open(os.path.join(directory, 'map.yaml'), 'w') as f:
        yaml.safe_dump({'image': 'map.pgm', 'mode': 'trinary', 'resolution': RES,
                        'origin': [ORIGIN[0], ORIGIN[1], 0.0], 'negate': int(negate),
                        'occupied_thresh': 0.65, 'free_thresh': 0.196}, f)  # map_saver's
    with open(os.path.join(directory, 'places.yaml'), 'w') as f:
        yaml.safe_dump({'places': [
            {'id': str(i), 'name': n, 'x': world(*p)[0], 'y': world(*p)[1], 'theta': 0.0}
            for i, (n, p) in enumerate(places.items())]}, f)
    return str(directory)


def lane_set(site):
    return {frozenset(lane) for lane in site.lanes}


def test_rmf_to_robot_transform_round_trips_every_place(tmp_path):
    site = import_site('lab', write_map(tmp_path), 'dock', clearance=0.3)
    tx, ty = site.translation
    for w in site.waypoints:
        rmf = (w.px * RES, -w.py * RES)          # building_map_generator's convention
        robot = (rmf[0] + tx, rmf[1] + ty)       # EasyFullControl's transform
        assert robot == pytest.approx(world(*PLACES[w.name]))


def test_lanes_go_through_the_gap_not_the_wall(tmp_path):
    site = import_site('lab', write_map(tmp_path), 'dock', clearance=0.3, neighbours=10)
    lanes = lane_set(site)
    assert frozenset(('left_top', 'right_top')) not in lanes     # straight through the wall
    assert frozenset(('dock', 'gap')) in lanes
    assert frozenset(('gap', 'right')) in lanes


def test_graph_is_a_connected_spanning_set(tmp_path):
    site = import_site('lab', write_map(tmp_path), 'dock', clearance=0.3, neighbours=0)
    # neighbours=0 leaves only the spanning tree: n - 1 lanes joining all places.
    assert len(site.lanes) == len(PLACES) - 1
    reached, frontier = {'dock'}, ['dock']
    while frontier:
        n = frontier.pop()
        for a, b in site.lanes:
            for x, y in ((a, b), (b, a)):
                if x == n and y not in reached:
                    reached.add(y)
                    frontier.append(y)
    assert reached == set(PLACES)


def test_closed_wall_is_reported_not_silently_split(tmp_path):
    with pytest.raises(MapImportError, match='cannot all be joined') as e:
        import_site('lab', write_map(tmp_path, gap=False), 'dock', clearance=0.3)
    assert 'lower the clearance' not in str(e.value)   # no clearance gets through a wall


def test_narrow_gap_names_the_closest_link_and_a_clearance_that_works(tmp_path):
    # Without the place in the gap, the 1.1 m gap leaves dock - right ~0.6 m, under 0.8.
    places = {n: p for n, p in PLACES.items() if n != 'gap'}
    directory = write_map(tmp_path, places=places)
    with pytest.raises(MapImportError, match='cannot all be joined') as e:
        import_site('lab', directory, 'dock', clearance=0.8)
    message = str(e.value)
    assert 'dock - right' in message
    suggested = float(message.split('lower the clearance to ')[1].split(' m')[0])
    assert 0.5 <= suggested < 0.8
    site = import_site('lab', directory, 'dock', clearance=suggested)
    assert len(site.waypoints) == len(places)


def test_place_close_to_a_wall_warns_but_stays_reachable(tmp_path):
    places = dict(PLACES, near_wall=(27, 6))    # 0.2 m from the wall, clearance 0.3
    site = import_site('lab', write_map(tmp_path, places=places), 'dock', clearance=0.3,
                       neighbours=10)
    assert any('near_wall' in w for w in site.warnings)
    assert any('near_wall' in lane for lane in site.lanes)
    # ...but never through the wall to the other side.
    assert frozenset(('near_wall', 'right_top')) not in lane_set(site)


def test_unknown_cells_block_lanes_like_obstacles(tmp_path):
    # An unexplored strip along the bottom right: lanes must not cut through it.
    places = dict(PLACES, bottom_left=(10, 37), bottom_right=(50, 37))
    directory = write_map(tmp_path, places=places, unknown_block=(20, 40))
    site = import_site('lab', directory, 'dock', clearance=0.3, neighbours=10)
    assert frozenset(('bottom_left', 'bottom_right')) not in lane_set(site)


def test_negated_pgm_reads_the_same(tmp_path):
    plain, negated = tmp_path / 'plain', tmp_path / 'negated'
    plain.mkdir()
    negated.mkdir()
    grids = []
    for directory, negate in ((plain, False), (negated, True)):
        write_map(directory, negate=negate)
        meta = load_map_yaml(str(directory / 'map.yaml'))
        _, _, rows = read_pgm(str(directory / 'map.pgm'))
        grids.append(classify(rows, meta))
    assert grids[0] == grids[1]
    assert grids[0][0][30] == OCCUPIED and grids[0][20][30] == FREE


def test_trinary_thresholds():
    meta = load_map_yaml.__globals__['MapMeta'](image='x', resolution=0.1, origin=(0, 0, 0))
    assert classify([[254, 0, 205]], meta) == [[FREE, OCCUPIED, UNKNOWN]]


def test_missing_charger_names_the_places(tmp_path):
    with pytest.raises(MapImportError, match='no place named .charger.*dock'):
        import_site('lab', write_map(tmp_path), 'charger', clearance=0.3)


def test_rotated_map_is_refused(tmp_path):
    directory = write_map(tmp_path)
    path = os.path.join(directory, 'map.yaml')
    with open(path) as f:
        data = yaml.safe_load(f)
    data['origin'][2] = 0.5
    with open(path, 'w') as f:
        yaml.safe_dump(data, f)
    with pytest.raises(MapImportError, match='yaw'):
        import_site('lab', directory, 'dock')


def test_building_yaml_is_valid_and_names_the_level_after_the_map(tmp_path):
    places = dict(PLACES)
    places['Loading bay: 2'] = places.pop('right_top')   # a drive-UI name needing quotes
    site = import_site('lab', write_map(tmp_path, places=places), 'dock', clearance=0.3)
    building = yaml.safe_load(site.building())
    assert building['name'] == 'lab' and list(building['levels']) == ['lab']
    level = building['levels']['lab']
    assert level['drawing']['filename'] == 'lab.png'
    assert level['measurements'][0][2]['distance'][1] == pytest.approx(W * RES)
    names = [v[3] for v in level['vertices']]
    assert 'Loading bay: 2' in names
    charger = next(v for v in level['vertices'] if v[3] == 'dock')
    assert charger[4]['is_charger'] == [4, True]


def test_fleet_config_points_at_the_site_and_the_rover_broker(tmp_path):
    site = import_site('lab', write_map(tmp_path), 'dock', clearance=0.3)
    template = {
        'rmf_fleet': {'name': 'rover_fleet', 'robots': {'rover_a1': {'charger': 'x'}},
                      'transforms': {'rover_world': {'translation': [1, 2]}}},
        'vda5050': {'map_name': 'rover_world', 'broker': {'host': 'mosquitto', 'port': 1883}},
    }
    config = fleet_config(template, site, 'dock', '192.168.1.201', 1883)
    assert config['rmf_fleet']['transforms'] == {
        'lab': {'rotation': 0.0, 'scale': 1.0, 'translation': list(site.translation)}}
    assert config['rmf_fleet']['robots']['rover_a1']['charger'] == 'dock'
    assert config['vda5050']['map_name'] == 'lab'
    assert config['vda5050']['broker'] == {'host': '192.168.1.201', 'port': 1883}
    assert template['vda5050']['map_name'] == 'rover_world'   # template untouched
