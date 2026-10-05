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

"""Ports the application layer drives; infrastructure implements them."""

from abc import ABC, abstractmethod


class FleetLink(ABC):
    """The VDA 5050 master-control side of the MQTT link to one rover."""

    @abstractmethod
    def send(self, topic: str, body: dict) -> None:
        """Publish body (without header) on <prefix>/<topic>; the link adds the header."""


class RmfCommands(ABC):
    """What the session reports back to RMF about the command RMF gave it."""

    @abstractmethod
    def finished(self, execution: object) -> None:
        """The command behind this execution handle is done (RMF moves on)."""

    @abstractmethod
    def replan(self) -> None:
        """The command failed; ask RMF to plan again from where the rover is."""

    @abstractmethod
    def set_in_fleet(self, in_fleet: bool) -> None:
        """Commission the robot (RMF gives it tasks) or decommission it (RMF gives it none)."""

    @abstractmethod
    def set_offline(self, offline: bool) -> None:
        """Show the robot as offline in RMF's robot state, or let RMF report it again."""


class Log(ABC):

    @abstractmethod
    def info(self, message: str) -> None: ...

    @abstractmethod
    def warning(self, message: str) -> None: ...

    @abstractmethod
    def error(self, message: str) -> None: ...
