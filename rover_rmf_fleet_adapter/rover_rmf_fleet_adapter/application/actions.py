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
The perform-actions this adapter can run, by RMF action category.

A fleet config may list an action under `rmf_fleet.actions` only if ACTION_HANDLERS has it
(fleet_adapter checks at startup). To add one: a description in domain/actions.py, an
ActionHandler here, an ACTION_HANDLERS entry, and the category in the fleet config template
(rover_rmf_bringup/config/fleet_rover_world.yaml). A handler that has to move or switch
something on the rover sends VDA 5050 messages through the FleetLink it is started with.
"""

from abc import ABC, abstractmethod
from typing import Callable, Dict

from .ports import FleetLink
from ..domain.actions import Wait


class ActionHandler(ABC):
    """One running perform-action. Called with the session lock held: never block."""

    @abstractmethod
    def tick(self, now: float) -> bool:
        """Advance; True once the action is done (RMF then moves on)."""

    def cancel(self) -> None:
        """RMF stopped the action. Undo or abort whatever the rover is still doing."""


# (description, now, link) -> a started handler. Raises DomainError for a bad description.
ActionFactory = Callable[[object, float, FleetLink], ActionHandler]


class WaitHandler(ActionHandler):
    """The rover stays where it is; done once the time is up."""

    def __init__(self, wait: Wait, now: float):
        self.duration_sec = wait.duration_sec
        self._deadline = now + wait.duration_sec

    @classmethod
    def start(cls, description: object, now: float, link: FleetLink) -> 'WaitHandler':
        return cls(Wait.parse(description), now)

    def tick(self, now: float) -> bool:
        return now >= self._deadline


ACTION_HANDLERS: Dict[str, ActionFactory] = {
    'wait': WaitHandler.start,
}
