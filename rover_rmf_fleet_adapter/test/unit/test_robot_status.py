# Copyright 2026 Rover A1 contributors
# Licensed under the Apache License, Version 2.0.

import pytest

from rover_rmf_fleet_adapter.domain.robot_status import (
    in_fleet, parse_battery, parse_position, parse_state)


def rover_state(**overrides):
    """A state as the rover's connector publishes it (VDA 5050 2.0, InOrbit mqtt_bridge)."""
    state = {
        'headerId': 12, 'orderId': 'o1', 'orderUpdateId': 0, 'lastNodeId': 'o1-start',
        'lastNodeSequenceId': 0, 'driving': True, 'paused': False,
        'operatingMode': 'AUTOMATIC',
        'nodeStates': [{'nodeId': 'o1-goal', 'sequenceId': 2, 'released': True}],
        'edgeStates': [{'edgeId': 'o1-start-o1-goal', 'sequenceId': 1, 'released': True}],
        'agvPosition': {'x': 1.5, 'y': -2.0, 'theta': 0.3, 'mapId': '',
                        'positionInitialized': True},
        'batteryState': {'batteryCharge': 80.0, 'batteryVoltage': 25.1, 'charging': False},
        'errors': [],
    }
    state.update(overrides)
    return state


def test_parses_a_driving_rover():
    status = parse_state(rover_state())
    assert (status.pose.x, status.pose.y, status.pose.theta) == (1.5, -2.0, 0.3)
    assert status.battery_soc == pytest.approx(0.8)
    assert status.has_active_order
    assert status.node_ids == ('o1-goal',)
    assert status.order_id == 'o1'
    assert status.operating_mode == 'AUTOMATIC'


def test_idle_rover_has_no_active_order():
    status = parse_state(rover_state(nodeStates=[], edgeStates=[], lastNodeId='o1-goal'))
    assert not status.has_active_order
    assert status.last_node_id == 'o1-goal'


def test_uninitialized_position_is_unknown():
    pose, map_id = parse_position({'x': 0, 'y': 0, 'positionInitialized': False, 'mapId': 'm'})
    assert pose is None and map_id == 'm'


def test_non_finite_position_is_unknown():
    pose, _ = parse_position({'x': float('inf'), 'y': 0, 'positionInitialized': True})
    assert pose is None


def test_zero_charge_and_voltage_means_no_battery_reading():
    # rover_vda5050_adapter's batteryFor() reports 0 % / 0 V without a reading (Gazebo).
    assert parse_battery({'batteryCharge': 0.0, 'batteryVoltage': 0.0}) == (None, False)
    assert parse_battery({'batteryCharge': 0.0, 'batteryVoltage': 22.0}) == (0.0, False)
    assert parse_battery({'batteryCharge': 150.0, 'batteryVoltage': 1.0})[0] == 1.0


def test_malformed_fields_read_as_unknown():
    status = parse_state({'agvPosition': 'x', 'batteryState': None, 'nodeStates': 5,
                          'errors': [None, {'errorType': 'e'}]})
    assert status.pose is None and status.battery_soc is None
    assert not status.has_active_order
    assert [e.error_type for e in status.errors] == ['e']


def test_errors_match_order_by_reference_whatever_the_key_spelling():
    status = parse_state(rover_state(errors=[
        {'errorType': 'orderUpdateError', 'errorLevel': 'WARNING',
         'errorReferences': [{'referenceKey': 'order_id', 'referenceValue': 'o2'}]},
        {'errorType': 'noRouteError', 'errorLevel': 'FATAL',
         'errorReferences': [{'referenceKey': 'nodeId', 'referenceValue': 'o1-goal'}]},
        {'errorType': 'noOrderToCancel', 'errorLevel': 'WARNING',
         'errorReferences': [{'referenceKey': 'action_id', 'referenceValue': 'o1'}]},
    ]))
    assert [e.error_type for e in status.errors_for_order('o2')] == ['orderUpdateError']
    assert [e.error_type for e in status.errors_for_order('o1', ('o1-start', 'o1-goal'))] == \
        ['noRouteError']


def test_blocked_reason_reads_the_current_mode_and_motion_lock():
    def state(mode, *error_types):
        return parse_state({'operatingMode': mode,
                            'errors': [{'errorType': t, 'errorLevel': 'WARNING'}
                                       for t in error_types]})

    assert state('AUTOMATIC').blocked_reason() is None
    assert state('').blocked_reason() is None  # unknown: don't hold
    for mode in ('MANUAL', 'SERVICE', 'TEACHIN'):
        assert mode in state(mode).blocked_reason()
    assert 'motionLocked' in state('AUTOMATIC', 'motionLocked').blocked_reason()
    # Why the last order was refused, not the rover's state now.
    assert state('AUTOMATIC', 'missionRefused').blocked_reason() is None


def test_action_states_are_parsed():
    status = parse_state({'actionStates': [
        {'actionId': 'a1', 'actionType': 'setDriveMode', 'actionStatus': 'FINISHED',
         'resultDescription': 'Drive mode Manual.'},
        {'actionId': 'a2', 'actionType': 'startPause', 'actionStatus': 'RUNNING'},
        'junk']})
    assert [a.action_id for a in status.action_states] == ['a1', 'a2']
    assert status.action_state('a1').finished
    assert status.action_state('a1').result_description == 'Drive mode Manual.'
    assert not status.action_state('a2').finished
    assert status.action_state('nope') is None
    assert parse_state({}).action_states == ()


def test_in_fleet_follows_the_operating_mode():
    assert in_fleet('AUTOMATIC') is True
    assert in_fleet('SEMIAUTOMATIC') is True
    for mode in ('MANUAL', 'SERVICE', 'TEACHIN'):
        assert in_fleet(mode) is False
    assert in_fleet('') is None
