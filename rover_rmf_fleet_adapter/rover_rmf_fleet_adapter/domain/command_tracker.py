# Copyright 2026 Rover A1 contributors
# Licensed under the Apache License, Version 2.0.

"""
Follows one RMF navigate command through the rover's VDA 5050 connector.

RMF's EasyFullControl sends one destination at a time; each becomes a two-node order. The
connector (vda5050_controller.py process_order) rejects a new orderId while another order still
has nodes/edges, so a command that arrives mid-order first cancels it and waits for the rover to
go idle. The tracker is pure: it returns the messages to send (effects) and whether the command
ended, and never touches MQTT or RMF itself.

    IDLE --navigate--> SENT --state names our order--> ACTIVE --nodes done at goal--> ARRIVED
      \\--(order active)--> CANCELLING --rover idle--> SENT
      \\--(no state / not localized)--> WAITING --rover localized--> SENT (or CANCELLING)
    WAITING --still not localized after accept_timeout--> FAILED
    SENT/ACTIVE/CANCELLING --error on our order | timeout | order ended elsewhere--> FAILED
    ACTIVE --no progress for stall_timeout--> FAILED

The stall watchdog exists because Nav 2 does not give up on its own: its behavior tree keeps
replanning while the skid-steer rover sits near a goal it cannot turn onto slowly (seen in
Gazebo 2026-09-29). A failed command makes RMF replan, and the replacing order restarts Nav 2.
"""

from dataclasses import dataclass, field
from enum import Enum
from typing import Callable, List, Optional

from .model import normalize_angle, Pose2D
from .robot_status import RobotStatus
from .vda_messages import NavigationOrder


class Phase(Enum):
    IDLE = 'idle'
    WAITING = 'waiting'
    CANCELLING = 'cancelling'
    SENT = 'sent'
    ACTIVE = 'active'
    ARRIVED = 'arrived'
    FAILED = 'failed'


class Result(Enum):
    NONE = 'none'
    ARRIVED = 'arrived'
    FAILED = 'failed'


@dataclass(frozen=True)
class SendOrder:
    order: NavigationOrder


@dataclass(frozen=True)
class SendCancel:
    pass


@dataclass(frozen=True)
class Step:
    effects: List[object] = field(default_factory=list)
    result: Result = Result.NONE
    reason: str = ''


@dataclass(frozen=True)
class Goal:
    pose: Pose2D
    speed_limit: Optional[float] = None


class CommandTracker:

    def __init__(self, new_order_id: Callable[[], str],
                 accept_timeout: float = 10.0, cancel_timeout: float = 20.0,
                 stall_timeout: float = 60.0, progress_distance: float = 0.3,
                 progress_angle: float = 0.5):
        """
        Track navigate commands.

        accept_timeout: seconds for the rover's state to name a sent order (state is published
        every 1 s and on events). cancel_timeout: seconds for a cancelOrder to leave the rover
        idle; rover_mission_manager waits for Nav 2 to drain a cancelled goal first.
        stall_timeout: seconds an active, unpaused order may go without the rover moving
        progress_distance (m) or turning progress_angle (rad); 0 disables the watchdog.
        """
        self._new_order_id = new_order_id
        self._accept_timeout = accept_timeout
        self._cancel_timeout = cancel_timeout
        self._stall_timeout = stall_timeout
        self._progress_distance = progress_distance
        self._progress_angle = progress_angle
        self._progress_pose: Optional[Pose2D] = None
        self._progress_at = 0.0
        self._phase = Phase.IDLE
        self._goal: Optional[Goal] = None
        self._order: Optional[NavigationOrder] = None
        self._since = 0.0

    @property
    def phase(self) -> Phase:
        return self._phase

    @property
    def order(self) -> Optional[NavigationOrder]:
        return self._order

    @property
    def busy(self) -> bool:
        return self._phase in (Phase.WAITING, Phase.CANCELLING, Phase.SENT, Phase.ACTIVE)

    def navigate(self, goal: Goal, status: Optional[RobotStatus], now: float) -> Step:
        """Start a new command, replacing (and cancelling) whatever runs."""
        self._goal = goal
        self._order = None
        # Failing at once here would spin: RMF answers a failure with an immediate replan and
        # a new navigate. Wait for the rover instead, and fail after accept_timeout.
        self._enter(Phase.WAITING, now)
        if status is None:
            return Step()
        return self._start(status, now)

    def stop(self, status: Optional[RobotStatus], now: float) -> Step:
        """Drop the command; cancel the rover's order if it still runs one."""
        cancelling = self._phase == Phase.CANCELLING
        self._goal = None
        self._order = None
        self._enter(Phase.IDLE, now)
        if cancelling or status is None or not status.has_active_order:
            return Step()
        return Step([SendCancel()])

    def on_status(self, status: RobotStatus, now: float) -> Step:
        if self._phase == Phase.WAITING:
            if now - self._since > self._accept_timeout:
                return self._fail(
                    'the rover reports no position (positionInitialized false) for '
                    f'{self._accept_timeout:.0f} s')
            return self._start(status, now)

        if self._phase == Phase.CANCELLING:
            if not status.has_active_order:
                return self._send(status, now)
            if now - self._since > self._cancel_timeout:
                return self._fail(
                    f'the rover still runs order {status.order_id!r} '
                    f'{self._cancel_timeout:.0f} s after cancelOrder')
            return Step()

        if self._phase not in (Phase.SENT, Phase.ACTIVE):
            return Step()

        order = self._order
        errors = status.errors_for_order(order.order_id, (order.start_node_id, order.goal_node_id))
        if errors:
            e = errors[0]
            return self._fail(f'{e.error_type} ({e.level}): {e.description}')

        ours = status.order_id == order.order_id
        if self._phase == Phase.SENT:
            if ours and status.has_active_order:
                self._enter(Phase.ACTIVE, now)
                self._mark_progress(status, now)
                return Step()
            if ours and status.last_node_id == order.goal_node_id:
                return self._arrive()
            if now - self._since > self._accept_timeout:
                return self._fail(
                    f'order {order.order_id} not accepted within {self._accept_timeout:.0f} s '
                    f'(the rover reports order {status.order_id!r})')
            return Step()

        # ACTIVE
        if not ours:
            return self._fail(f'the rover switched to order {status.order_id!r}')
        if status.has_active_order:
            return self._check_progress(status, now)
        if status.last_node_id == order.goal_node_id:
            return self._arrive()
        return self._fail(f'order ended at node {status.last_node_id!r}, not at the goal')

    def _start(self, status: RobotStatus, now: float) -> Step:
        """From WAITING: cancel whatever runs, or send the order once the rover is localized."""
        if status.has_active_order:
            self._enter(Phase.CANCELLING, now)
            return Step([SendCancel()])
        return self._send(status, now)

    def _send(self, status: RobotStatus, now: float) -> Step:
        if status.pose is None:
            if self._phase != Phase.WAITING:
                self._enter(Phase.WAITING, now)
            return Step()
        goal = self._goal
        self._order = NavigationOrder(
            order_id=self._new_order_id(),
            start=status.pose,
            goal=goal.pose,
            # Orders go in the frame the rover reports; its mapId is '' unless the indoor
            # manager names the map, and the connector compares mapIds as strings.
            map_id=status.map_id,
            speed_limit=goal.speed_limit)
        self._enter(Phase.SENT, now)
        return Step([SendOrder(self._order)])

    def _mark_progress(self, status: RobotStatus, now: float):
        self._progress_pose = status.pose
        self._progress_at = now

    def _check_progress(self, status: RobotStatus, now: float) -> Step:
        last, pose = self._progress_pose, status.pose
        if status.paused or last is None or pose is None:
            # A paused rover is not stalled; an unknown pose restarts the clock.
            self._mark_progress(status, now)
            return Step()
        turned = abs(normalize_angle(pose.theta - last.theta))
        if pose.distance_to(last) >= self._progress_distance or turned >= self._progress_angle:
            self._mark_progress(status, now)
            return Step()
        if self._stall_timeout > 0.0 and now - self._progress_at > self._stall_timeout:
            return self._fail(
                f'no progress for {self._stall_timeout:.0f} s at ({pose.x:.2f}, {pose.y:.2f})')
        return Step()

    def _arrive(self) -> Step:
        self._enter(Phase.ARRIVED, self._since)
        return Step(result=Result.ARRIVED)

    def _fail(self, reason: str) -> Step:
        self._enter(Phase.FAILED, self._since)
        return Step(result=Result.FAILED, reason=reason)

    def _enter(self, phase: Phase, now: float):
        self._phase = phase
        self._since = now
