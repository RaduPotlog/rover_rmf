# Copyright 2026 Rover A1 contributors
# Licensed under the Apache License, Version 2.0.

"""
What the fleet adapter needs to know from the rover's VDA 5050 2.0 `state` message.

Field names are the connector's (InOrbit's mqtt_bridge converts its ROS messages to dromedary
camelCase). Anything missing or malformed reads as "unknown" rather than raising: a state from a
connector that is still starting up must not take the adapter down.
"""

from dataclasses import dataclass, field, replace
import math
from typing import Any, Mapping, Optional, Tuple

from .model import DomainError, Pose2D


# operatingMode values in which an operator drives, so the rover takes no orders
# (rover_vda5050_adapter: drive modes ASSISTED and MANUAL -> MANUAL, no drive-mode manager ->
# SERVICE).
MANUAL_OPERATING_MODES = ('MANUAL', 'SERVICE', 'TEACHIN')
# operatingMode values in which fleet control drives (VDA 5050: orders are accepted).
FLEET_OPERATING_MODES = ('AUTOMATIC', 'SEMIAUTOMATIC')
# Errors that describe the rover now and stop it from driving. Not missionRefused: that is why the
# LAST order was refused, and it stays until an order is accepted.
BLOCKING_ERRORS = ('motionLocked',)


def _reference_key(key: str) -> str:
    # The connector fills referenceKey values in snake_case (order_id, node_id) and only the
    # JSON keys get camelCased on the way out, so compare keys spelling-insensitively.
    return key.replace('_', '').lower()


@dataclass(frozen=True)
class VdaError:
    error_type: str
    level: str = ''
    description: str = ''
    references: Tuple[Tuple[str, str], ...] = ()

    def references_value(self, key: str, value: str) -> bool:
        wanted = _reference_key(key)
        return any(_reference_key(k) == wanted and v == value for k, v in self.references)


@dataclass(frozen=True)
class ActionState:
    """One entry of the state's actionStates: an action the rover runs or has run."""

    action_id: str
    action_type: str = ''
    status: str = ''
    result_description: str = ''

    @property
    def finished(self) -> bool:
        return self.status in ('FINISHED', 'FAILED')


def in_fleet(operating_mode: str) -> Optional[bool]:
    """
    Whether RMF may give the rover tasks in this operatingMode; None when it is unknown ('').

    Automatic: yes. Manual, Assisted (reported as MANUAL), Service, Teach-in: an operator has the
    rover, so it is out of the fleet until it is back in Automatic.
    """
    if not operating_mode:
        return None
    return operating_mode in FLEET_OPERATING_MODES


@dataclass(frozen=True)
class RobotStatus:
    pose: Optional[Pose2D] = None
    map_id: str = ''
    # Fraction 0..1, or None when the rover does not know (the connector then reports 0 %).
    battery_soc: Optional[float] = None
    charging: bool = False
    driving: bool = False
    paused: bool = False
    order_id: str = ''
    last_node_id: str = ''
    node_ids: Tuple[str, ...] = ()
    edge_ids: Tuple[str, ...] = ()
    operating_mode: str = ''
    errors: Tuple[VdaError, ...] = field(default_factory=tuple)
    action_states: Tuple[ActionState, ...] = field(default_factory=tuple)

    def action_state(self, action_id: str) -> Optional[ActionState]:
        return next((a for a in self.action_states if a.action_id == action_id), None)

    @property
    def has_active_order(self) -> bool:
        """Nodes or edges left to traverse: the connector will reject a new orderId."""
        return bool(self.node_ids) or bool(self.edge_ids)

    def blocked_reason(self) -> Optional[str]:
        """Why the rover cannot take an order right now, or None."""
        if self.operating_mode in MANUAL_OPERATING_MODES:
            return f'operatingMode {self.operating_mode}: the rover is not in Automatic'
        for e in self.errors:
            if e.error_type in BLOCKING_ERRORS:
                return f'{e.error_type}: {e.description}' if e.description else e.error_type
        return None

    def errors_for_order(self, order_id: str, node_ids: Tuple[str, ...] = ()):
        """Errors that name this order, directly (orderId) or through one of its nodes."""
        return tuple(
            e for e in self.errors
            if e.references_value('orderId', order_id)
            or any(e.references_value('nodeId', n) for n in node_ids))

    def with_pose(self, pose: Optional[Pose2D], map_id: Optional[str] = None) -> 'RobotStatus':
        return replace(self, pose=pose, map_id=self.map_id if map_id is None else map_id)


def _float(value: Any) -> Optional[float]:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def parse_position(agv_position: Any) -> Tuple[Optional[Pose2D], str]:
    """agvPosition -> (pose or None when not initialized, mapId)."""
    if not isinstance(agv_position, Mapping):
        return None, ''
    map_id = str(agv_position.get('mapId') or '')
    if not agv_position.get('positionInitialized', False):
        return None, map_id
    x, y = _float(agv_position.get('x')), _float(agv_position.get('y'))
    theta = _float(agv_position.get('theta'))
    if x is None or y is None:
        return None, map_id
    try:
        return Pose2D(x, y, theta or 0.0), map_id
    except DomainError:
        return None, map_id


def parse_battery(battery_state: Any) -> Tuple[Optional[float], bool]:
    """batteryState -> (state of charge 0..1 or None if unknown, charging)."""
    if not isinstance(battery_state, Mapping):
        return None, False
    charging = bool(battery_state.get('charging', False))
    charge = _float(battery_state.get('batteryCharge'))
    voltage = _float(battery_state.get('batteryVoltage')) or 0.0
    # rover_vda5050_adapter reports 0 % and 0 V when it has no battery reading (e.g. Gazebo).
    if charge is None or (charge <= 0.0 and voltage <= 0.0):
        return None, charging
    return min(max(charge / 100.0, 0.0), 1.0), charging


def _ids(items: Any, key: str) -> Tuple[str, ...]:
    if not isinstance(items, list):
        return ()
    return tuple(str(i.get(key, '')) for i in items if isinstance(i, Mapping))


def parse_errors(errors: Any) -> Tuple[VdaError, ...]:
    if not isinstance(errors, list):
        return ()
    parsed = []
    for error in errors:
        if not isinstance(error, Mapping):
            continue
        references = tuple(
            (str(r.get('referenceKey', '')), str(r.get('referenceValue', '')))
            for r in error.get('errorReferences') or [] if isinstance(r, Mapping))
        parsed.append(VdaError(
            error_type=str(error.get('errorType', '')),
            level=str(error.get('errorLevel', '')),
            description=str(error.get('errorDescription', '')),
            references=references))
    return tuple(parsed)


def parse_action_states(action_states: Any) -> Tuple[ActionState, ...]:
    if not isinstance(action_states, list):
        return ()
    return tuple(
        ActionState(
            action_id=str(a.get('actionId', '')),
            action_type=str(a.get('actionType', '')),
            status=str(a.get('actionStatus', '')),
            result_description=str(a.get('resultDescription') or ''))
        for a in action_states if isinstance(a, Mapping))


def parse_state(state: Mapping) -> RobotStatus:
    pose, map_id = parse_position(state.get('agvPosition'))
    battery_soc, charging = parse_battery(state.get('batteryState'))
    return RobotStatus(
        pose=pose,
        map_id=map_id,
        battery_soc=battery_soc,
        charging=charging,
        driving=bool(state.get('driving', False)),
        paused=bool(state.get('paused', False)),
        order_id=str(state.get('orderId') or ''),
        last_node_id=str(state.get('lastNodeId') or ''),
        node_ids=_ids(state.get('nodeStates'), 'nodeId'),
        edge_ids=_ids(state.get('edgeStates'), 'edgeId'),
        operating_mode=str(state.get('operatingMode') or ''),
        errors=parse_errors(state.get('errors')),
        action_states=parse_action_states(state.get('actionStates')),
    )
