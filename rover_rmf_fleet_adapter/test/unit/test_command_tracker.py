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

import itertools

from rover_rmf_fleet_adapter.domain.command_tracker import (
    CommandTracker, Goal, Phase, Result, SendCancel, SendOrder)
from rover_rmf_fleet_adapter.domain.model import Pose2D
from rover_rmf_fleet_adapter.domain.robot_status import RobotStatus, VdaError

HERE = Pose2D(0.0, 0.0, 0.0)
GOAL = Goal(Pose2D(4.0, 1.0, 0.5), speed_limit=None)


def tracker(**kwargs):
    ids = (f'o{i}' for i in itertools.count(1))
    return CommandTracker(lambda: next(ids), **kwargs)


def idle(order_id='', last_node='', pose=HERE, errors=()):
    return RobotStatus(pose=pose, order_id=order_id, last_node_id=last_node, errors=errors)


def running(order_id, last_node=None):
    return RobotStatus(pose=HERE, order_id=order_id,
                       last_node_id=last_node or f'{order_id}-start',
                       node_ids=(f'{order_id}-goal',),
                       edge_ids=(f'{order_id}-start-{order_id}-goal',))


def test_idle_rover_gets_the_order_at_once():
    t = tracker()
    step = t.navigate(GOAL, idle(), now=0.0)
    (effect,) = step.effects
    assert isinstance(effect, SendOrder)
    assert effect.order.start == HERE and effect.order.goal == GOAL.pose
    assert t.phase == Phase.SENT


def test_full_trip_sent_active_arrived():
    t = tracker()
    t.navigate(GOAL, idle(), 0.0)
    assert t.on_status(running('o1'), 1.0).result == Result.NONE
    assert t.phase == Phase.ACTIVE
    assert t.on_status(running('o1'), 5.0).result == Result.NONE
    step = t.on_status(idle('o1', 'o1-goal'), 9.0)
    assert step.result == Result.ARRIVED
    assert t.phase == Phase.ARRIVED


def test_short_trip_can_finish_before_a_state_shows_it_running():
    t = tracker()
    t.navigate(GOAL, idle(), 0.0)
    assert t.on_status(idle('o1', 'o1-goal'), 1.0).result == Result.ARRIVED


def test_busy_rover_is_cancelled_first_then_ordered_from_where_it_stopped():
    t = tracker()
    step = t.navigate(GOAL, running('foreign'), 0.0)
    assert step.effects == [SendCancel()]
    assert t.phase == Phase.CANCELLING
    assert t.on_status(running('foreign'), 1.0).effects == []

    stopped_at = Pose2D(2.0, 0.5, 0.0)
    step = t.on_status(idle('foreign', pose=stopped_at), 3.0)
    (effect,) = step.effects
    assert isinstance(effect, SendOrder)
    assert effect.order.order_id == 'o1'
    assert effect.order.start == stopped_at
    assert t.phase == Phase.SENT


def test_cancel_that_never_completes_fails():
    t = tracker(cancel_timeout=20.0)
    t.navigate(GOAL, running('foreign'), 0.0)
    assert t.on_status(running('foreign'), 19.0).result == Result.NONE
    step = t.on_status(running('foreign'), 21.0)
    assert step.result == Result.FAILED and 'cancelOrder' in step.reason


def test_rejected_order_fails():
    t = tracker()
    t.navigate(GOAL, idle(), 0.0)
    rejection = VdaError('orderUpdateError', 'WARNING', 'There is an active running order.',
                         (('order_id', 'o1'), ('order_update_id', '0')))
    step = t.on_status(idle('previous', errors=(rejection,)), 1.0)
    assert step.result == Result.FAILED and 'orderUpdateError' in step.reason


def test_stale_errors_about_other_orders_are_ignored():
    t = tracker()
    t.navigate(GOAL, idle(), 0.0)
    old = VdaError('noRouteError', 'FATAL', 'Failed to reach current node.',
                   (('node_id', 'o0-goal'),))
    assert t.on_status(RobotStatus(pose=HERE, order_id='o1', node_ids=('o1-goal',),
                                   errors=(old,)), 1.0).result == Result.NONE
    assert t.phase == Phase.ACTIVE


def test_navigation_failure_on_our_node_fails():
    t = tracker()
    t.navigate(GOAL, idle(), 0.0)
    t.on_status(running('o1'), 1.0)
    failure = VdaError('noRouteError', 'FATAL', 'Failed to reach current node.',
                       (('node_id', 'o1-goal'),))
    status = RobotStatus(pose=HERE, order_id='o1', node_ids=('o1-goal',), errors=(failure,))
    assert t.on_status(status, 4.0).result == Result.FAILED


def test_order_not_accepted_in_time_fails():
    t = tracker(accept_timeout=10.0)
    t.navigate(GOAL, idle('previous'), 0.0)
    assert t.on_status(idle('previous'), 9.0).result == Result.NONE
    step = t.on_status(idle('previous'), 11.0)
    assert step.result == Result.FAILED and 'not accepted' in step.reason


def test_order_ending_away_from_goal_fails():
    # e.g. cancelled on the rover's web UI, or drive mode left AUTOMATIC.
    t = tracker()
    t.navigate(GOAL, idle(), 0.0)
    t.on_status(running('o1'), 1.0)
    step = t.on_status(idle('o1', 'o1-start'), 3.0)
    assert step.result == Result.FAILED


def test_another_master_taking_over_fails():
    t = tracker()
    t.navigate(GOAL, idle(), 0.0)
    t.on_status(running('o1'), 1.0)
    assert t.on_status(running('other'), 2.0).result == Result.FAILED


def test_unlocalized_rover_is_waited_for_not_failed_at_once():
    # Failing at once made RMF replan and re-navigate in a tight loop (Gazebo, 2026-09-29).
    t = tracker(accept_timeout=10.0)
    step = t.navigate(GOAL, idle(pose=None), 0.0)
    assert step.result == Result.NONE and step.effects == []
    assert t.phase == Phase.WAITING
    assert t.on_status(idle(pose=None), 5.0).effects == []
    (effect,) = t.on_status(idle(), 6.0).effects
    assert isinstance(effect, SendOrder) and effect.order.start == HERE


def test_no_state_yet_waits_for_the_first_one():
    t = tracker()
    assert t.navigate(GOAL, None, 0.0).effects == []
    (effect,) = t.on_status(idle(), 1.0).effects
    assert isinstance(effect, SendOrder)


def test_waiting_cancels_a_running_order_once_localized():
    t = tracker()
    t.navigate(GOAL, None, 0.0)
    assert t.on_status(running('foreign'), 1.0).effects == [SendCancel()]
    assert t.phase == Phase.CANCELLING


def test_rover_that_never_localizes_fails_after_the_accept_timeout():
    t = tracker(accept_timeout=10.0)
    t.navigate(GOAL, idle(pose=None), 0.0)
    assert t.on_status(idle(pose=None), 9.0).result == Result.NONE
    step = t.on_status(idle(pose=None), 11.0)
    assert step.result == Result.FAILED and 'positionInitialized' in step.reason


def test_losing_position_during_a_cancel_waits_too():
    t = tracker()
    t.navigate(GOAL, running('foreign'), 0.0)
    assert t.on_status(idle('foreign', pose=None), 2.0).effects == []
    assert t.phase == Phase.WAITING
    (effect,) = t.on_status(idle('foreign'), 3.0).effects
    assert isinstance(effect, SendOrder)


def test_stop_cancels_a_running_order_once():
    t = tracker()
    t.navigate(GOAL, idle(), 0.0)
    t.on_status(running('o1'), 1.0)
    assert t.stop(running('o1'), 2.0).effects == [SendCancel()]
    assert t.phase == Phase.IDLE
    # Idle after stop: later states change nothing.
    assert t.on_status(running('o1'), 3.0).effects == []


def test_stop_on_an_idle_rover_sends_nothing():
    t = tracker()
    assert t.stop(idle(), 0.0).effects == []


def test_stop_while_cancelling_does_not_cancel_twice():
    t = tracker()
    t.navigate(GOAL, running('foreign'), 0.0)
    assert t.stop(running('foreign'), 1.0).effects == []
    # And the order that was waiting for the cancel is never sent.
    assert t.on_status(idle('foreign'), 2.0).effects == []


def test_new_command_replaces_a_running_one():
    t = tracker()
    t.navigate(GOAL, idle(), 0.0)
    t.on_status(running('o1'), 1.0)
    step = t.navigate(Goal(Pose2D(-3.0, 2.0)), running('o1'), 2.0)
    assert step.effects == [SendCancel()]
    step = t.on_status(idle('o1', 'o1-start'), 4.0)  # the cancelled order must not fail us
    (effect,) = step.effects
    assert effect.order.order_id == 'o2' and effect.order.goal == Pose2D(-3.0, 2.0)


def test_stalled_order_fails_after_the_stall_timeout():
    t = tracker(stall_timeout=60.0)
    t.navigate(GOAL, idle(), 0.0)
    t.on_status(running('o1'), 1.0)
    # Small wobbles (< 0.3 m, < 0.5 rad) are not progress.
    wobble = RobotStatus(pose=Pose2D(0.1, 0.0, 0.2), order_id='o1', node_ids=('o1-goal',))
    assert t.on_status(wobble, 60.0).result == Result.NONE
    step = t.on_status(wobble, 62.0)
    assert step.result == Result.FAILED and 'no progress' in step.reason


def test_moving_or_turning_resets_the_stall_clock():
    t = tracker(stall_timeout=60.0)
    t.navigate(GOAL, idle(), 0.0)
    t.on_status(running('o1'), 1.0)
    moved = RobotStatus(pose=Pose2D(1.0, 0.0), order_id='o1', node_ids=('o1-goal',))
    assert t.on_status(moved, 50.0).result == Result.NONE
    turned = RobotStatus(pose=Pose2D(1.0, 0.0, 1.0), order_id='o1', node_ids=('o1-goal',))
    assert t.on_status(turned, 100.0).result == Result.NONE
    assert t.on_status(turned, 155.0).result == Result.NONE
    assert t.on_status(turned, 161.0).result == Result.FAILED


def test_paused_rover_is_not_stalled():
    t = tracker(stall_timeout=60.0)
    t.navigate(GOAL, idle(), 0.0)
    t.on_status(running('o1'), 1.0)
    paused = RobotStatus(pose=HERE, order_id='o1', node_ids=('o1-goal',), paused=True)
    assert t.on_status(paused, 200.0).result == Result.NONE
    still = RobotStatus(pose=HERE, order_id='o1', node_ids=('o1-goal',))
    assert t.on_status(still, 250.0).result == Result.NONE
    assert t.on_status(still, 261.0).result == Result.FAILED


def test_zero_stall_timeout_disables_the_watchdog():
    t = tracker(stall_timeout=0.0)
    t.navigate(GOAL, idle(), 0.0)
    t.on_status(running('o1'), 1.0)
    assert t.on_status(running('o1'), 10000.0).result == Result.NONE
