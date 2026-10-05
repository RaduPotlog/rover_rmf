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

"""Value objects shared by the fleet adapter's domain."""

from dataclasses import dataclass
import math


class DomainError(ValueError):
    """A value the domain refuses (bad identity, NaN pose, ...)."""


def normalize_angle(theta: float) -> float:
    """Wrap an angle to [-pi, pi], as VDA 5050 requires for theta."""
    return math.atan2(math.sin(theta), math.cos(theta))


@dataclass(frozen=True)
class Pose2D:
    """A planar pose in the robot's map frame (m, m, rad)."""

    x: float
    y: float
    theta: float = 0.0

    def __post_init__(self):
        if not all(math.isfinite(v) for v in (self.x, self.y, self.theta)):
            # VDA 5050 forbids NaN/Infinity on the wire.
            raise DomainError(f'pose must be finite, got ({self.x}, {self.y}, {self.theta})')
        object.__setattr__(self, 'theta', normalize_angle(self.theta))

    def distance_to(self, other: 'Pose2D') -> float:
        return math.hypot(self.x - other.x, self.y - other.y)


@dataclass(frozen=True)
class VdaIdentity:
    """Who we talk to: the MQTT topic levels and the header fields of every message."""

    interface_name: str
    manufacturer: str
    serial_number: str
    version: str = '2.0.0'

    def __post_init__(self):
        for name in ('interface_name', 'manufacturer', 'serial_number'):
            value = getattr(self, name)
            # '/' separates topic levels and + # $ are MQTT wildcards / reserved.
            if not value or any(c in value for c in '/+#$'):
                raise DomainError(f'{name} {value!r} is not a valid MQTT topic level')
        major = self.version.split('.')[0]
        if not major.isdigit():
            raise DomainError(f'version {self.version!r} is not [Major].[Minor].[Patch]')

    @property
    def topic_prefix(self) -> str:
        major = self.version.split('.')[0]
        return f'{self.interface_name}/v{major}/{self.manufacturer}/{self.serial_number}'

    def topic(self, name: str) -> str:
        return f'{self.topic_prefix}/{name}'
