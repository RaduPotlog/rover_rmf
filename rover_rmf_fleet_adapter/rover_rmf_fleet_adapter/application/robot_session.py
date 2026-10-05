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
One rover under RMF control: RMF's callbacks in, VDA 5050 messages out, and the state RMF polls.

Called from three threads (MQTT callbacks, RMF's command callbacks, the update loop), so every
entry point takes the session lock. Effects are published while holding it (FleetLink.send only
queues the message in paho); RMF is told about a finished or failed command after the lock is
released, because RMF may answer finished() with the next navigate() call.

RMF's commands are a navigate or a perform-action (application/actions.py); at most one runs,
and `execution` is its RMF handle. An action advances on tick(), from the update loop.

The rover's drive mode decides whether it is in the fleet: out (decommissioned) while an operator
has it (operatingMode MANUAL, SERVICE) or while it is offline, back in on its next AUTOMATIC
state. Fleet control can switch the mode with the rover's setDriveMode action
(request_drive_mode), and the result comes back in the state's actionStates.

A failed navigate asks RMF to replan after a backoff that doubles with each failure in a row
(replan_backoff, up to replan_backoff_max) and resets on arrival: a goal the rover refuses at once
must not become a 1 Hz loop of orders over the rover's 4G link.
"""

from dataclasses import dataclass, field
import functools
import threading
from typing import Callable, Mapping, Optional, Tuple

from .actions import ACTION_HANDLERS, ActionFactory, ActionHandler
from .ports import FleetLink, Log, RmfCommands
from ..domain.command_tracker import CommandTracker, Goal, Result, SendCancel, SendOrder, Step
from ..domain.model import DomainError, Pose2D
from ..domain.robot_status import in_fleet, RobotStatus
from ..domain.vda_messages import (
    cancel_order_action, instant_actions_body, INSTANT_ACTIONS_TOPIC, ORDER_TOPIC,
    set_drive_mode_action)

ONLINE = 'ONLINE'


@dataclass(frozen=True)
class RmfRobotState:
    """What RMF's EasyFullControl RobotState needs: map name, [x, y, yaw], battery 0..1."""

    map_name: str
    position: Tuple[float, float, float]
    battery_soc: float


@dataclass
class DriveModeRequest:
    """A setDriveMode sent to the rover; result is (ok, message) once it ended or timed out."""

    action_id: str
    mode: str
    sent_at: float
    result: Optional[Tuple[bool, str]] = field(default=None)


class RobotSession:

    def __init__(self, name: str, link: FleetLink, rmf: RmfCommands, log: Log,
                 tracker: CommandTracker, default_map: str, unknown_battery_soc: float,
                 actions: Mapping[str, ActionFactory] = ACTION_HANDLERS,
                 replan_backoff: float = 1.0, replan_backoff_max: float = 60.0,
                 drive_mode_timeout: float = 10.0):
        """
        Wire one rover.

        default_map: the RMF map (level) name used when the rover's mapId is empty, which it is
        unless rover_indoor_nav_manager names its map. unknown_battery_soc: reported to RMF when
        the rover has no battery reading (Gazebo); RMF refuses tasks below recharge_threshold.
        """
        self._name = name
        self._link = link
        self._rmf = rmf
        self._log = log
        self._tracker = tracker
        self._default_map = default_map
        self._unknown_battery_soc = unknown_battery_soc
        self._lock = threading.Lock()
        self._status: Optional[RobotStatus] = None
        self._connection = ''
        self._execution: Optional[object] = None
        self._actions = actions
        self._action: Optional[ActionHandler] = None
        self._replan_backoff = replan_backoff
        self._replan_backoff_max = replan_backoff_max
        self._failures = 0
        self._replan_at: Optional[float] = None
        self._logged_hold: Optional[str] = None
        self._in_fleet: Optional[bool] = None
        self._offline: Optional[bool] = None
        self._drive_mode_timeout = drive_mode_timeout
        self._drive_request: Optional[DriveModeRequest] = None

    # --- rover side -------------------------------------------------------------------------

    def on_state(self, status: RobotStatus, now: float) -> None:
        with self._lock:
            self._status = status
            report = self._apply(self._tracker.on_status(status, now), now)
            fleet = self._sync_fleet(status)
            self._settle_drive_request(status)
        report()
        fleet()

    def on_visualization(self, pose: Optional[Pose2D], map_id: str) -> None:
        """Position-only updates at 2 Hz, between the 1 Hz states."""
        with self._lock:
            if self._status is not None and pose is not None:
                self._status = self._status.with_pose(pose, map_id)

    def on_connection(self, connection_state: str) -> None:
        with self._lock:
            if connection_state != self._connection:
                self._log.info(f'[{self._name}] connection: {connection_state}')
            self._connection = connection_state
            offline = connection_state != ONLINE
            changed = offline != self._offline
            self._offline = offline
            # Leaving the fleet now; rejoining waits for a fresh state (the mode may have changed
            # while the rover was away).
            fleet = self._sync_fleet(self._status) if offline else _nothing
        if changed:
            self._rmf.set_offline(offline)
        fleet()

    # --- RMF side ---------------------------------------------------------------------------

    def navigate(self, execution: object, goal: Pose2D, speed_limit: Optional[float],
                 now: float) -> None:
        with self._lock:
            self._drop_action()
            self._replan_at = None
            self._execution = execution
            self._log.info(
                f'[{self._name}] navigate to ({goal.x:.2f}, {goal.y:.2f}, {goal.theta:.2f})')
            report = self._apply(
                self._tracker.navigate(Goal(goal, speed_limit), self._status, now), now)
        report()

    def stop(self, now: float) -> None:
        with self._lock:
            self._drop_action()
            self._replan_at = None
            self._execution = None
            self._log.info(f'[{self._name}] stop')
            report = self._apply(self._tracker.stop(self._status, now), now)
        report()

    def perform_action(self, category: str, description: object, execution: object,
                       now: float) -> None:
        with self._lock:
            self._drop_action()
            self._replan_at = None
            self._execution = None
            factory = self._actions.get(category)
            try:
                if factory is None:
                    raise DomainError('this adapter has no handler for it')
                action = factory(description, now, self._link)
            except DomainError as e:
                # Finish rather than hang the task; RMF goes on with the next step.
                self._log.error(f'[{self._name}] perform-action {category!r} skipped: {e}')
                report = functools.partial(self._rmf.finished, execution)
            else:
                self._log.info(f'[{self._name}] perform-action {category!r}: {description}')
                self._execution, self._action = execution, action
                report = self._advance_action(now)
        report()

    def tick(self, now: float) -> None:
        """Advance the running perform-action; ask for a replan once its backoff is over."""
        with self._lock:
            report = self._advance_action(now)
            if self._replan_at is not None and now >= self._replan_at:
                self._replan_at = None
                report = self._rmf.replan
        report()

    # --- fleet control (the control API) ---------------------------------------------------

    def request_drive_mode(self, mode: str, now: float) -> str:
        """
        Send setDriveMode (MANUAL or AUTOMATIC); returns its actionId for drive_mode_result().

        Raises DomainError for another mode or while the rover is offline.
        """
        with self._lock:
            if self._connection != ONLINE:
                raise DomainError(f'the rover is not connected ({self._connection or "no news"})')
            action = set_drive_mode_action(mode)
            self._log.info(f'[{self._name}] setDriveMode {mode} ({action["actionId"]})')
            self._link.send(INSTANT_ACTIONS_TOPIC, instant_actions_body([action]))
            self._drive_request = DriveModeRequest(action['actionId'], mode, now)
            return action['actionId']

    def drive_mode_result(self, action_id: str, now: float) -> Optional[Tuple[bool, str]]:
        """(ok, message) once the rover finished or failed the request; None while it runs."""
        with self._lock:
            request = self._drive_request
            if request is None or request.action_id != action_id:
                return (False, 'superseded by a newer drive mode request')
            if request.result is None and now - request.sent_at > self._drive_mode_timeout:
                request.result = (False, 'the rover did not answer within '
                                  f'{self._drive_mode_timeout:.0f} s (is its connector new '
                                  'enough for setDriveMode?)')
            return request.result

    def snapshot(self) -> dict:
        """What the control API shows about this rover (JSON-ready)."""
        with self._lock:
            status = self._status
            request = self._drive_request
            return {
                'name': self._name,
                'connection': self._connection or 'UNKNOWN',
                'operating_mode': status.operating_mode if status else '',
                'in_fleet': self._in_fleet,
                'blocked_reason': status.blocked_reason() if status else None,
                'localized': status is not None and status.pose is not None,
                'battery_soc': status.battery_soc if status else None,
                'drive_mode_request': None if request is None else {
                    'mode': request.mode,
                    'done': request.result is not None,
                    'ok': request.result[0] if request.result else None,
                    'message': request.result[1] if request.result else '',
                },
            }

    @property
    def execution(self) -> Optional[object]:
        with self._lock:
            return self._execution

    def rmf_state(self) -> Optional[RmfRobotState]:
        """None while the rover is offline or not localized: RMF then gets no update."""
        with self._lock:
            status = self._status
            if self._connection != ONLINE or status is None or status.pose is None:
                return None
            battery = status.battery_soc
            return RmfRobotState(
                map_name=status.map_id or self._default_map,
                position=(status.pose.x, status.pose.y, status.pose.theta),
                battery_soc=self._unknown_battery_soc if battery is None else battery)

    # --- internals (lock held) --------------------------------------------------------------

    def _apply(self, step: Step, now: float) -> Callable[[], None]:
        """Send the step's messages; return what to tell RMF once the lock is released."""
        for effect in step.effects:
            if isinstance(effect, SendOrder):
                order = effect.order
                self._log.info(
                    f'[{self._name}] order {order.order_id}: ({order.start.x:.2f}, '
                    f'{order.start.y:.2f}) -> ({order.goal.x:.2f}, {order.goal.y:.2f})')
                self._link.send(ORDER_TOPIC, order.body())
            elif isinstance(effect, SendCancel):
                self._log.info(f'[{self._name}] cancelOrder')
                self._link.send(INSTANT_ACTIONS_TOPIC,
                                instant_actions_body([cancel_order_action()]))

        held = self._tracker.held_reason
        if held != self._logged_hold:
            if held is not None:
                self._log.warning(f'[{self._name}] holding the command, no order sent: {held}')
            elif self._logged_hold is not None:
                self._log.info(f'[{self._name}] rover available again')
            self._logged_hold = held

        if step.result == Result.NONE or self._execution is None or self._action is not None:
            return _nothing
        execution, self._execution = self._execution, None
        if step.result == Result.ARRIVED:
            self._log.info(f'[{self._name}] arrived')
            self._failures = 0
            return lambda: self._rmf.finished(execution)
        self._failures += 1
        delay = min(self._replan_backoff * 2 ** (self._failures - 1), self._replan_backoff_max)
        self._log.error(f'[{self._name}] navigation failed: {step.reason}; replanning in '
                        f'{delay:.0f} s (failure {self._failures} in a row)')
        self._replan_at = now + delay
        return _nothing

    def _advance_action(self, now: float) -> Callable[[], None]:
        if self._action is None or not self._action.tick(now):
            return _nothing
        execution, self._execution, self._action = self._execution, None, None
        self._log.info(f'[{self._name}] perform-action done')
        return lambda: self._rmf.finished(execution)

    def _sync_fleet(self, status: Optional[RobotStatus]) -> Callable[[], None]:
        """Out of the fleet while offline or an operator has the rover, back in on Automatic."""
        if self._connection != ONLINE:
            wanted, why = False, f'connection {self._connection or "unknown"}'
        elif status is None:
            return _nothing
        else:
            wanted, why = in_fleet(status.operating_mode), f'operatingMode {status.operating_mode}'
        if wanted is None or wanted == self._in_fleet:
            return _nothing
        self._in_fleet = wanted
        self._log.info(f'[{self._name}] {"in" if wanted else "out of"} the fleet ({why})')
        return functools.partial(self._rmf.set_in_fleet, wanted)

    def _settle_drive_request(self, status: RobotStatus) -> None:
        request = self._drive_request
        if request is None or request.result is not None:
            return
        state = status.action_state(request.action_id)
        if state is not None and state.finished:
            ok = state.status == 'FINISHED'
            message = state.result_description
            if not message:
                # A connector without the setDriveMode handler fails it with no description.
                message = 'done' if ok else ('the rover failed setDriveMode without a reason; '
                                             'its VDA 5050 connector may predate setDriveMode')
            request.result = (ok, message)
            self._log.info(f'[{self._name}] setDriveMode {request.mode}: {state.status} '
                           f'{state.result_description}')

    def _drop_action(self) -> None:
        if self._action is not None:
            self._log.info(f'[{self._name}] perform-action cancelled')
            self._action.cancel()
            self._action = None


def _nothing() -> None:
    pass
