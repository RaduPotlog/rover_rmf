# Copyright 2026 Rover A1 contributors
# Licensed under the Apache License, Version 2.0.

import itertools

import pytest

from rover_rmf_fleet_adapter.application.actions import ActionHandler
from rover_rmf_fleet_adapter.application.ports import FleetLink, Log, RmfCommands
from rover_rmf_fleet_adapter.application.robot_session import RobotSession
from rover_rmf_fleet_adapter.domain.command_tracker import CommandTracker
from rover_rmf_fleet_adapter.domain.model import DomainError, Pose2D
from rover_rmf_fleet_adapter.domain.robot_status import ActionState, RobotStatus, VdaError


class FakeLink(FleetLink):
    def __init__(self):
        self.sent = []

    def send(self, topic, body):
        self.sent.append((topic, body))


class FakeRmf(RmfCommands):
    def __init__(self):
        self.finished_executions = []
        self.replans = 0
        self.session = None
        self.lock_free = []
        self.fleet = []
        self.offline = []

    def finished(self, execution):
        # RMF may call navigate() from inside finished(): the session lock must be free.
        self.lock_free.append(self.session._lock.acquire(blocking=False))
        self.session._lock.release()
        self.finished_executions.append(execution)

    def replan(self):
        self.replans += 1

    def set_in_fleet(self, in_fleet):
        self.fleet.append(in_fleet)

    def set_offline(self, offline):
        self.offline.append(offline)


class QuietLog(Log):
    def __init__(self):
        self.errors = []
        self.warnings = []

    def info(self, message):
        pass

    def warning(self, message):
        self.warnings.append(message)

    def error(self, message):
        self.errors.append(message)


@pytest.fixture
def wired():
    ids = (f'o{i}' for i in itertools.count(1))
    link, rmf = FakeLink(), FakeRmf()
    session = RobotSession('rover_a1', link, rmf, QuietLog(), CommandTracker(lambda: next(ids)),
                           default_map='L1', unknown_battery_soc=1.0)
    rmf.session = session
    session.on_connection('ONLINE')
    return session, link, rmf


def at(x, y, **kw):
    return RobotStatus(pose=Pose2D(x, y), **kw)


def test_rmf_state_uses_default_map_and_battery_when_unknown(wired):
    session, _, _ = wired
    assert session.rmf_state() is None  # nothing heard yet
    session.on_state(at(1.0, 2.0), 0.0)
    state = session.rmf_state()
    assert state.map_name == 'L1' and state.battery_soc == 1.0
    assert state.position == (1.0, 2.0, 0.0)

    session.on_state(at(1.0, 2.0, map_id='hall', battery_soc=0.4), 1.0)
    assert (session.rmf_state().map_name, session.rmf_state().battery_soc) == ('hall', 0.4)


def test_offline_rover_is_not_reported(wired):
    session, _, _ = wired
    session.on_state(at(0, 0), 0.0)
    session.on_connection('CONNECTIONBROKEN')
    assert session.rmf_state() is None


def test_visualization_refreshes_the_position_between_states(wired):
    session, _, _ = wired
    session.on_state(at(0, 0), 0.0)
    session.on_visualization(Pose2D(0.4, 0.1), '')
    assert session.rmf_state().position[:2] == (0.4, 0.1)


def test_navigate_to_arrival_finishes_the_execution_outside_the_lock(wired):
    session, link, rmf = wired
    session.on_state(at(0, 0), 0.0)
    session.navigate('exec-1', Pose2D(3, 0), None, 0.0)
    ((topic, body),) = link.sent
    assert topic == 'order' and body['orderId'] == 'o1'
    assert session.execution == 'exec-1'

    session.on_state(at(1, 0, order_id='o1', node_ids=('o1-goal',)), 1.0)
    session.on_state(at(3, 0, order_id='o1', last_node_id='o1-goal'), 5.0)
    assert rmf.finished_executions == ['exec-1']
    assert rmf.lock_free == [True]
    assert session.execution is None


def test_failure_asks_rmf_to_replan(wired):
    session, _, rmf = wired
    session.on_state(at(0, 0), 0.0)
    session.navigate('exec-1', Pose2D(3, 0), None, 0.0)
    session.on_state(at(0, 0, order_id='o1', node_ids=('o1-goal',)), 1.0)
    session.on_state(at(0, 0, order_id='o1', last_node_id='o1-start'), 2.0)
    assert rmf.replans == 0  # not at once: after the backoff
    session.tick(2.9)
    assert rmf.replans == 0
    session.tick(3.0)
    assert rmf.replans == 1 and rmf.finished_executions == []
    session.tick(10.0)
    assert rmf.replans == 1


def test_stop_sends_cancel_order_and_drops_the_execution(wired):
    session, link, rmf = wired
    session.on_state(at(0, 0), 0.0)
    session.navigate('exec-1', Pose2D(3, 0), None, 0.0)
    session.on_state(at(1, 0, order_id='o1', node_ids=('o1-goal',)), 1.0)
    session.stop(2.0)
    topic, body = link.sent[-1]
    assert topic == 'instantActions' and body['actions'][0]['actionType'] == 'cancelOrder'
    assert session.execution is None
    session.on_state(at(1, 0, order_id='o1'), 3.0)
    assert rmf.finished_executions == [] and rmf.replans == 0


def test_wait_action_holds_the_execution_until_its_time_is_up(wired):
    session, link, rmf = wired
    session.on_state(at(0, 0), 0.0)
    session.perform_action('wait', {'duration_sec': 5}, 'exec-w', 10.0)
    assert session.execution == 'exec-w'
    session.tick(14.9)
    assert rmf.finished_executions == []
    session.tick(15.0)
    assert rmf.finished_executions == ['exec-w']
    assert rmf.lock_free == [True]
    assert session.execution is None
    assert link.sent == []  # the rover is not told anything: it just stays put


def test_unknown_or_bad_action_is_skipped_with_an_error(wired):
    session, _, rmf = wired
    session.perform_action('dance', {}, 'exec-1', 0.0)
    session.perform_action('wait', {'duration_sec': -3}, 'exec-2', 0.0)
    assert rmf.finished_executions == ['exec-1', 'exec-2']
    assert rmf.lock_free == [True, True]
    assert len(session._log.errors) == 2
    assert session.execution is None


class RecordingAction(ActionHandler):
    def __init__(self):
        self.cancelled = False

    def tick(self, now):
        return False

    def cancel(self):
        self.cancelled = True


def test_stop_and_navigate_cancel_a_running_action():
    started = []

    def start(description, now, link):
        started.append(RecordingAction())
        return started[-1]

    rmf = FakeRmf()
    session = RobotSession('rover_a1', FakeLink(), rmf, QuietLog(),
                           CommandTracker(lambda: 'o1'), default_map='L1',
                           unknown_battery_soc=1.0, actions={'hold': start})
    rmf.session = session
    session.on_connection('ONLINE')
    session.on_state(at(0, 0), 0.0)

    session.perform_action('hold', {}, 'exec-1', 0.0)
    session.stop(1.0)
    assert started[0].cancelled and session.execution is None

    session.perform_action('hold', {}, 'exec-2', 2.0)
    session.navigate('exec-3', Pose2D(3, 0), None, 3.0)
    assert started[1].cancelled and session.execution == 'exec-3'
    session.tick(4.0)
    assert rmf.finished_executions == []


def fail_once(session, execution, order, now):
    """navigate, then the rover refuses the order at once (as the real rover did 2026-10-01)."""
    session.navigate(execution, Pose2D(3, 0), None, now)
    refusal = VdaError('noRouteError', 'FATAL', 'Failed to reach current node.',
                       references=(('orderId', order),))
    session.on_state(at(0, 0, errors=(refusal,)), now + 0.5)


def test_replan_backoff_doubles_and_resets_on_arrival(wired):
    session, _, rmf = wired
    session.on_state(at(0, 0), 0.0)
    replan_times = []
    t = 0.0
    for i in range(1, 9):
        fail_once(session, f'e{i}', f'o{i}', t)
        while rmf.replans < i:
            t += 0.5
            session.tick(t)
        replan_times.append(t)
    gaps = [b - a for a, b in zip(replan_times, replan_times[1:])]
    # Each gap is the next backoff plus the 0.5 s the refusal took: 2, 4, 8, ... capped at 60.
    assert gaps == [2.5, 4.5, 8.5, 16.5, 32.5, 60.5, 60.5]

    # An arrival resets the backoff.
    session.navigate('ok', Pose2D(3, 0), None, t)
    session.on_state(at(1, 0, order_id='o9', node_ids=('o9-goal',)), t + 1)
    session.on_state(at(3, 0, order_id='o9', last_node_id='o9-goal'), t + 2)
    assert rmf.finished_executions == ['ok']
    fail_once(session, 'again', 'o10', t + 3)
    session.tick(t + 4.5)
    assert rmf.replans == 9


def test_a_new_command_drops_the_pending_replan(wired):
    session, _, rmf = wired
    session.on_state(at(0, 0), 0.0)
    fail_once(session, 'e1', 'o1', 0.0)
    session.navigate('e2', Pose2D(3, 0), None, 0.7)
    session.tick(5.0)
    assert rmf.replans == 0


def manual():
    return {'operating_mode': 'MANUAL'}


def locked():
    return {'operating_mode': 'AUTOMATIC',
            'errors': (VdaError('motionLocked', 'WARNING', 'Motion is locked'),)}


@pytest.mark.parametrize('blocked', [manual(), locked()])
def test_blocked_rover_gets_no_order_until_it_is_available(wired, blocked):
    session, link, rmf = wired
    log = session._log
    session.on_state(at(0, 0, **blocked), 0.0)
    session.navigate('e1', Pose2D(3, 0), None, 0.0)
    for t in range(1, 600):  # ten minutes: no order, no failure, no replan, one warning
        session.on_state(at(0, 0, **blocked), float(t))
        session.tick(float(t))
    assert link.sent == [] and rmf.replans == 0 and rmf.finished_executions == []
    assert len(log.warnings) == 1 and 'no order sent' in log.warnings[0]
    assert session.execution == 'e1'

    session.on_state(at(0, 0, operating_mode='AUTOMATIC'), 600.0)
    ((topic, body),) = link.sent
    assert topic == 'order' and body['orderId'] == 'o1'


def test_a_stale_mission_refusal_does_not_block(wired):
    session, link, _ = wired
    refused = (VdaError('missionRefused', 'WARNING', 'Drive mode is not AUTOMATIC'),)
    session.on_state(at(0, 0, operating_mode='AUTOMATIC', errors=refused), 0.0)
    session.navigate('e1', Pose2D(3, 0), None, 0.0)
    assert [topic for topic, _ in link.sent] == ['order']


def test_drive_mode_decides_fleet_membership(wired):
    session, _, rmf = wired
    session.on_state(at(0, 0), 0.0)  # mode unknown: no change
    assert rmf.fleet == []
    session.on_state(at(0, 0, operating_mode='AUTOMATIC'), 1.0)
    session.on_state(at(0, 0, operating_mode='AUTOMATIC'), 2.0)  # edges only
    session.on_state(at(0, 0, operating_mode='MANUAL'), 3.0)
    session.on_state(at(0, 0, operating_mode='SERVICE'), 4.0)
    session.on_state(at(0, 0, operating_mode='AUTOMATIC'), 5.0)
    assert rmf.fleet == [True, False, True]
    assert session.snapshot()['in_fleet'] is True


def test_connection_marks_the_robot_offline(wired):
    session, _, rmf = wired  # the fixture connected it: ONLINE
    session.on_connection('ONLINE')
    session.on_connection('CONNECTIONBROKEN')
    session.on_connection('CONNECTIONBROKEN')
    session.on_connection('ONLINE')
    assert rmf.offline == [False, True, False]


def test_an_offline_rover_leaves_the_fleet_until_it_reports_automatic(wired):
    session, _, rmf = wired
    session.on_state(at(0, 0, operating_mode='AUTOMATIC'), 0.0)
    session.on_connection('CONNECTIONBROKEN')
    session.on_connection('ONLINE')  # back, but no state yet: stays out
    assert rmf.fleet == [True, False]
    session.on_state(at(0, 0, operating_mode='AUTOMATIC'), 1.0)
    assert rmf.fleet == [True, False, True]


def action_states(action_id, status, description=''):
    return (ActionState(action_id, 'setDriveMode', status, description),)


def test_drive_mode_request_reports_the_rovers_answer(wired):
    session, link, _ = wired
    action_id = session.request_drive_mode('MANUAL', 0.0)
    ((topic, body),) = link.sent
    (action,) = body['actions']
    assert topic == 'instantActions' and action['actionType'] == 'setDriveMode'
    assert action['actionParameters'] == [{'key': 'mode', 'value': 'MANUAL'}]
    assert action['actionId'] == action_id

    assert session.drive_mode_result(action_id, 0.5) is None
    session.on_state(at(0, 0, action_states=action_states(action_id, 'RUNNING')), 0.6)
    assert session.drive_mode_result(action_id, 0.7) is None
    session.on_state(at(0, 0, operating_mode='MANUAL', action_states=action_states(
        action_id, 'FINISHED', 'Drive mode Manual.')), 1.0)
    assert session.drive_mode_result(action_id, 1.1) == (True, 'Drive mode Manual.')
    snapshot = session.snapshot()
    assert snapshot['drive_mode_request'] == {
        'mode': 'MANUAL', 'done': True, 'ok': True, 'message': 'Drive mode Manual.'}
    assert snapshot['operating_mode'] == 'MANUAL' and snapshot['in_fleet'] is False


def test_drive_mode_refusal_and_timeout(wired):
    session, _, _ = wired
    refused = session.request_drive_mode('AUTOMATIC', 0.0)
    session.on_state(at(0, 0, action_states=action_states(
        refused, 'FAILED', 'Automatic refused: no mission manager')), 1.0)
    assert session.drive_mode_result(refused, 1.0) == (
        False, 'Automatic refused: no mission manager')

    unanswered = session.request_drive_mode('MANUAL', 2.0)
    assert session.drive_mode_result(refused, 2.0)[0] is False  # superseded
    assert session.drive_mode_result(unanswered, 11.0) is None
    ok, message = session.drive_mode_result(unanswered, 12.5)
    assert not ok and 'did not answer' in message


def test_drive_mode_needs_a_known_mode_and_a_connected_rover(wired):
    session, link, _ = wired
    with pytest.raises(DomainError):
        session.request_drive_mode('ASSISTED', 0.0)
    session.on_connection('CONNECTIONBROKEN')
    with pytest.raises(DomainError):
        session.request_drive_mode('MANUAL', 0.0)
    assert link.sent == []
