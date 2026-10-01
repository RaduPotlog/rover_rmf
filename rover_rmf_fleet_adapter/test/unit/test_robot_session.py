# Copyright 2026 Rover A1 contributors
# Licensed under the Apache License, Version 2.0.

import itertools

import pytest

from rover_rmf_fleet_adapter.application.actions import ActionHandler
from rover_rmf_fleet_adapter.application.ports import FleetLink, Log, RmfCommands
from rover_rmf_fleet_adapter.application.robot_session import RobotSession
from rover_rmf_fleet_adapter.domain.command_tracker import CommandTracker
from rover_rmf_fleet_adapter.domain.model import Pose2D
from rover_rmf_fleet_adapter.domain.robot_status import RobotStatus


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

    def finished(self, execution):
        # RMF may call navigate() from inside finished(): the session lock must be free.
        self.lock_free.append(self.session._lock.acquire(blocking=False))
        self.session._lock.release()
        self.finished_executions.append(execution)

    def replan(self):
        self.replans += 1


class QuietLog(Log):
    def __init__(self):
        self.errors = []

    def info(self, message):
        pass

    def warning(self, message):
        pass

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
    assert rmf.replans == 1 and rmf.finished_executions == []


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
