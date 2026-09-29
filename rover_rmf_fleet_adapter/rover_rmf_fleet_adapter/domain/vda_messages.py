# Copyright 2026 Rover A1 contributors
# Licensed under the Apache License, Version 2.0.

"""
VDA 5050 master-control messages, as plain dicts ready for json.dumps.

The rover's connector speaks VDA 5050 2.0.0 (rover_vda5050, InOrbit's connector), so these
follow 2.0 field names, not the v3 ones in .claude/rules/vda5050_messages.md. The order shape
is the one rover_vda5050_bringup/scripts/fake_master.py verified in Gazebo: the first node is
the rover's current position, every node and edge is released, edge ids are '<start>-<end>'.
"""

from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Dict, List, Optional
import uuid

from .model import Pose2D, VdaIdentity

ORDER_TOPIC = 'order'
INSTANT_ACTIONS_TOPIC = 'instantActions'


def timestamp(now: datetime) -> str:
    """ISO 8601 UTC with milliseconds and a Z suffix (VDA 5050 header format)."""
    return now.astimezone(timezone.utc).isoformat(timespec='milliseconds').replace('+00:00', 'Z')


class HeaderCounter:
    """headerId is a per-topic counter, incremented on every message sent on that topic."""

    def __init__(self):
        self._ids: Dict[str, int] = {}

    def next_id(self, topic: str) -> int:
        self._ids[topic] = self._ids.get(topic, 0) + 1
        return self._ids[topic]


def with_header(identity: VdaIdentity, header_id: int, now: datetime, body: dict) -> dict:
    return {
        'headerId': header_id,
        'timestamp': timestamp(now),
        'version': identity.version,
        'manufacturer': identity.manufacturer,
        'serialNumber': identity.serial_number,
        **body,
    }


def action(action_type: str, blocking_type: str = 'HARD',
           parameters: Optional[List[dict]] = None, action_id: Optional[str] = None) -> dict:
    return {
        'actionType': action_type,
        'actionId': action_id or str(uuid.uuid4()),
        'blockingType': blocking_type,
        'actionParameters': parameters or [],
    }


def node(node_id: str, sequence_id: int, pose: Pose2D, map_id: str) -> dict:
    return {
        'nodeId': node_id,
        'sequenceId': sequence_id,
        'released': True,
        'nodePosition': {'x': pose.x, 'y': pose.y, 'theta': pose.theta, 'mapId': map_id},
        'actions': [],
    }


def edge(sequence_id: int, start: dict, end: dict, max_speed: Optional[float] = None) -> dict:
    body = {
        'edgeId': f'{start["nodeId"]}-{end["nodeId"]}',
        'sequenceId': sequence_id,
        'released': True,
        'startNodeId': start['nodeId'],
        'endNodeId': end['nodeId'],
        'actions': [],
    }
    if max_speed is not None and max_speed > 0.0:
        body['maxSpeed'] = max_speed
    return body


@dataclass(frozen=True)
class NavigationOrder:
    """One RMF navigate command as a two-node VDA 5050 order: here -> goal."""

    order_id: str
    start: Pose2D
    goal: Pose2D
    map_id: str
    speed_limit: Optional[float] = None

    @property
    def start_node_id(self) -> str:
        # Node ids carry the order id so a noRouteError's nodeId reference names the order.
        return f'{self.order_id}-start'

    @property
    def goal_node_id(self) -> str:
        return f'{self.order_id}-goal'

    def owns_node(self, node_id: str) -> bool:
        return node_id in (self.start_node_id, self.goal_node_id)

    def body(self) -> dict:
        start = node(self.start_node_id, 0, self.start, self.map_id)
        goal = node(self.goal_node_id, 2, self.goal, self.map_id)
        return {
            'orderId': self.order_id,
            'orderUpdateId': 0,
            'nodes': [start, goal],
            'edges': [edge(1, start, goal, self.speed_limit)],
        }


def instant_actions_body(actions: List[dict]) -> dict:
    return {'actions': actions}


def cancel_order_action() -> dict:
    return action('cancelOrder', 'HARD')
