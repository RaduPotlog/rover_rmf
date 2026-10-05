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

from rover_rmf_fleet_adapter.application.broker_settings import BrokerSettings, resolve_broker

YAML = {'host': '192.168.1.201', 'port': 1883, 'username': 'yaml_user', 'password': 'yaml_pw'}


def test_yaml_only():
    assert resolve_broker(YAML) == BrokerSettings('192.168.1.201', 1883, 'yaml_user', 'yaml_pw')


def test_defaults_for_an_empty_block():
    assert resolve_broker({}) == BrokerSettings('localhost', 1883, '', '')


def test_host_and_port_overrides():
    settings = resolve_broker(YAML, host_override='mosquitto', port_override=1884)
    assert (settings.host, settings.port) == ('mosquitto', 1884)


def test_env_credentials_replace_the_yaml_pair():
    env = {'RMF_BROKER_USERNAME': 'rmf', 'RMF_BROKER_PASSWORD': 'env_pw'}
    settings = resolve_broker(YAML, env=env)
    assert (settings.username, settings.password) == ('rmf', 'env_pw')


def test_env_user_without_password_does_not_borrow_the_yaml_password():
    settings = resolve_broker(YAML, env={'RMF_BROKER_USERNAME': 'rmf'})
    assert (settings.username, settings.password) == ('rmf', '')


def test_empty_env_user_keeps_the_yaml_pair():
    env = {'RMF_BROKER_USERNAME': '', 'RMF_BROKER_PASSWORD': 'ignored'}
    assert resolve_broker(YAML, env=env).username == 'yaml_user'
    assert resolve_broker(YAML, env=env).password == 'yaml_pw'
