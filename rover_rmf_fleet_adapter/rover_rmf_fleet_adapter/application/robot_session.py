# Copyright 2026 Rover A1 contributors
# Licensed under the Apache License, Version 2.0.

"""
One rover under RMF control: RMF's callbacks in, VDA 5050 messages out, and the state RMF polls.

Called from three threads (MQTT callbacks, RMF's command callbacks, the update loop), so every
entry point takes the session lock. Effects are published while holding it (FleetLink.send only
queues the message in paho); RMF is told about a finished or failed command after the lock is
released, because RMF may answer finished() with the next navigate() call.

RMF's commands are a navigate or a perform-action (application/actions.py); at most one runs,
and `execution` is its RMF handle. An action advances on tick(), from the update loop.
"""

from dataclasses import dataclass
import functools
import threading
from typing import Callable, Mapping, Optional, Tuple

from .actions import ACTION_HANDLERS, ActionFactory, ActionHandler
from .ports import FleetLink, Log, RmfCommands
from ..domain.command_tracker import CommandTracker, Goal, Result, SendCancel, SendOrder, Step
from ..domain.model import DomainError, Pose2D
from ..domain.robot_status import RobotStatus
from ..domain.vda_messages import (
    cancel_order_action, instant_actions_body, INSTANT_ACTIONS_TOPIC, ORDER_TOPIC)

ONLINE = 'ONLINE'


@dataclass(frozen=True)
class RmfRobotState:
    """What RMF's EasyFullControl RobotState needs: map name, [x, y, yaw], battery 0..1."""

    map_name: str
    position: Tuple[float, float, float]
    battery_soc: float


class RobotSession:

    def __init__(self, name: str, link: FleetLink, rmf: RmfCommands, log: Log,
                 tracker: CommandTracker, default_map: str, unknown_battery_soc: float,
                 actions: Mapping[str, ActionFactory] = ACTION_HANDLERS):
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

    # --- rover side -------------------------------------------------------------------------

    def on_state(self, status: RobotStatus, now: float) -> None:
        with self._lock:
            self._status = status
            report = self._apply(self._tracker.on_status(status, now))
        report()

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

    # --- RMF side ---------------------------------------------------------------------------

    def navigate(self, execution: object, goal: Pose2D, speed_limit: Optional[float],
                 now: float) -> None:
        with self._lock:
            self._drop_action()
            self._execution = execution
            self._log.info(
                f'[{self._name}] navigate to ({goal.x:.2f}, {goal.y:.2f}, {goal.theta:.2f})')
            report = self._apply(
                self._tracker.navigate(Goal(goal, speed_limit), self._status, now))
        report()

    def stop(self, now: float) -> None:
        with self._lock:
            self._drop_action()
            self._execution = None
            self._log.info(f'[{self._name}] stop')
            report = self._apply(self._tracker.stop(self._status, now))
        report()

    def perform_action(self, category: str, description: object, execution: object,
                       now: float) -> None:
        with self._lock:
            self._drop_action()
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
        """Advance the running perform-action, if any (update loop)."""
        with self._lock:
            report = self._advance_action(now)
        report()

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

    def _apply(self, step: Step) -> Callable[[], None]:
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

        if step.result == Result.NONE or self._execution is None or self._action is not None:
            return _nothing
        execution, self._execution = self._execution, None
        if step.result == Result.ARRIVED:
            self._log.info(f'[{self._name}] arrived')
            return lambda: self._rmf.finished(execution)
        self._log.error(f'[{self._name}] navigation failed: {step.reason}; replanning')
        return self._rmf.replan

    def _advance_action(self, now: float) -> Callable[[], None]:
        if self._action is None or not self._action.tick(now):
            return _nothing
        execution, self._execution, self._action = self._execution, None, None
        self._log.info(f'[{self._name}] perform-action done')
        return lambda: self._rmf.finished(execution)

    def _drop_action(self) -> None:
        if self._action is not None:
            self._log.info(f'[{self._name}] perform-action cancelled')
            self._action.cancel()
            self._action = None


def _nothing() -> None:
    pass
