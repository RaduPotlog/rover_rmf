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

from datetime import datetime, timedelta, timezone
import json
import math

import pytest

from rover_rmf_fleet_adapter.domain.model import DomainError, Pose2D, VdaIdentity
from rover_rmf_fleet_adapter.domain.vda_messages import (
    cancel_order_action, HeaderCounter, instant_actions_body, NavigationOrder,
    set_drive_mode_action, timestamp, with_header)

IDENTITY = VdaIdentity('uagv', 'MechatronicsAcademy', 'rover_a1')


def test_topics_follow_interface_major_manufacturer_serial():
    # Same prefix as rover_vda5050_bringup/config/connector.yaml.
    assert IDENTITY.topic('order') == 'uagv/v2/MechatronicsAcademy/rover_a1/order'


@pytest.mark.parametrize('bad', ['', 'a/b', 'a+b', 'a#', '$sys'])
def test_identity_rejects_invalid_topic_levels(bad):
    with pytest.raises(DomainError):
        VdaIdentity('uagv', 'MechatronicsAcademy', bad)


def test_pose_wraps_theta_and_rejects_nan():
    assert Pose2D(0, 0, 3 * math.pi).theta == pytest.approx(math.pi)
    with pytest.raises(DomainError):
        Pose2D(float('nan'), 0)


def test_timestamp_is_utc_milliseconds_with_z():
    local = datetime(2026, 9, 29, 12, 0, 0, 123456, tzinfo=timezone(timedelta(hours=3)))
    assert timestamp(local) == '2026-09-29T09:00:00.123Z'


def test_header_ids_count_per_topic():
    counter = HeaderCounter()
    ids = [counter.next_id('order'), counter.next_id('order'), counter.next_id('instantActions')]
    assert ids == [1, 2, 1]


def test_header_carries_identity_and_version():
    now = datetime(2026, 9, 29, tzinfo=timezone.utc)
    message = with_header(IDENTITY, 7, now, {'orderId': 'o'})
    assert message == {
        'headerId': 7, 'timestamp': '2026-09-29T00:00:00.000Z', 'version': '2.0.0',
        'manufacturer': 'MechatronicsAcademy', 'serialNumber': 'rover_a1', 'orderId': 'o'}


def test_navigation_order_is_start_edge_goal_all_released():
    order = NavigationOrder('o1', Pose2D(1, 2, 0.1), Pose2D(5, 6, 1.0), map_id='', speed_limit=0.5)
    body = order.body()
    json.dumps(body)  # serializable, no NaN

    assert body['orderId'] == 'o1' and body['orderUpdateId'] == 0
    start, goal = body['nodes']
    assert (start['nodeId'], start['sequenceId']) == ('o1-start', 0)
    assert (goal['nodeId'], goal['sequenceId']) == ('o1-goal', 2)
    assert goal['nodePosition'] == {'x': 5, 'y': 6, 'theta': 1.0, 'mapId': ''}
    assert all(n['released'] for n in body['nodes'])

    (edge,) = body['edges']
    assert edge == {
        'edgeId': 'o1-start-o1-goal', 'sequenceId': 1, 'released': True,
        'startNodeId': 'o1-start', 'endNodeId': 'o1-goal', 'actions': [], 'maxSpeed': 0.5}


def test_edge_has_no_max_speed_without_a_limit():
    body = NavigationOrder('o1', Pose2D(0, 0), Pose2D(1, 0), map_id='').body()
    assert 'maxSpeed' not in body['edges'][0]


def test_cancel_order_instant_action():
    body = instant_actions_body([cancel_order_action()])
    (action,) = body['actions']
    assert action['actionType'] == 'cancelOrder'
    assert action['blockingType'] == 'HARD'
    assert action['actionId']


def test_set_drive_mode_action():
    action = set_drive_mode_action('AUTOMATIC', 'a1')
    assert action == {'actionType': 'setDriveMode', 'actionId': 'a1', 'blockingType': 'HARD',
                      'actionParameters': [{'key': 'mode', 'value': 'AUTOMATIC'}]}
    assert set_drive_mode_action('MANUAL')['actionId']  # a fresh uuid
    with pytest.raises(DomainError):
        set_drive_mode_action('ASSISTED')
