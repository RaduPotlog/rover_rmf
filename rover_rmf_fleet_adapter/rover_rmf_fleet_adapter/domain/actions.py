# Copyright 2026 Rover A1 contributors
# Licensed under the Apache License, Version 2.0.

"""
Descriptions of the perform-actions RMF can hand the rover (a compose task's `perform_action`).

RMF passes the event's `description` through as parsed JSON; each action validates its own here.
"""

from dataclasses import dataclass
import math

from .model import DomainError

# A pause longer than an hour is a typo, not a patrol.
MAX_WAIT_SEC = 3600.0


@dataclass(frozen=True)
class Wait:
    """Stay where the rover is for duration_sec: {"duration_sec": N}, 0 < N <= MAX_WAIT_SEC."""

    duration_sec: float

    def __post_init__(self):
        if not math.isfinite(self.duration_sec) or not 0.0 < self.duration_sec <= MAX_WAIT_SEC:
            raise DomainError(
                f'wait duration_sec must be in (0, {MAX_WAIT_SEC:.0f}], got {self.duration_sec}')

    @staticmethod
    def parse(description: object) -> 'Wait':
        if not isinstance(description, dict):
            raise DomainError(f'wait needs {{"duration_sec": N}}, got {description!r}')
        value = description.get('duration_sec')
        # bool is an int in Python; true is not a duration.
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise DomainError(f'wait duration_sec must be a number, got {value!r}')
        return Wait(float(value))
