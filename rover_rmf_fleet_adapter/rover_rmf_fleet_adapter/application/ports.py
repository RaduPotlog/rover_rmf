# Copyright 2026 Rover A1 contributors
# Licensed under the Apache License, Version 2.0.

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


class Log(ABC):

    @abstractmethod
    def info(self, message: str) -> None: ...

    @abstractmethod
    def warning(self, message: str) -> None: ...

    @abstractmethod
    def error(self, message: str) -> None: ...
