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

"""Where the adapter's MQTT broker is, and how it logs in.

The fleet YAML gives the defaults. A deployment overrides them for every site: host and port
from the command line (run_rmf.sh), user and password from the environment, so the password
never shows in a process list or in a committed config.
"""

from dataclasses import dataclass
from typing import Mapping

USERNAME_ENV = 'RMF_BROKER_USERNAME'
PASSWORD_ENV = 'RMF_BROKER_PASSWORD'


@dataclass(frozen=True)
class BrokerSettings:
    host: str
    port: int
    username: str
    password: str


def resolve_broker(broker_cfg: Mapping, host_override: str = '', port_override: int = 0,
                   env: Mapping[str, str] = None) -> BrokerSettings:
    """Merge the YAML `vda5050.broker` block with the deployment's overrides."""
    env = env or {}
    username = env.get(USERNAME_ENV) or broker_cfg.get('username', '') or ''
    password = broker_cfg.get('password', '') or ''
    if env.get(USERNAME_ENV):
        # A user from the environment never pairs with the YAML's password.
        password = env.get(PASSWORD_ENV, '')
    return BrokerSettings(
        host=host_override or broker_cfg.get('host', 'localhost'),
        port=port_override or int(broker_cfg.get('port', 1883)),
        username=username,
        password=password)
