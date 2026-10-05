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

# rmf-web api-server settings for the rover_rmf compose stack (RMF_API_SERVER_CONFIG).
# Like upstream's sqlite_local_config.py, but on wall time: RMF here never sees a /clock.
import os

from api_server.default_config import config

config.update(
    {
        'host': '0.0.0.0',
        'port': 8000,
        # The browser (dashboard) reaches the server through the published port (compose sets
        # RMF_API_PUBLIC_URL from RMF_API_PORT; see docker-compose.yml).
        'public_url': os.environ.get('RMF_API_PUBLIC_URL', 'http://localhost:8010'),
        'db_url': 'sqlite:///ws/run/db.sqlite3',
        'cache_directory': '/ws/run/cache',
        'ros_args': [],
        'log_level': 'INFO',
        'timezone': 'UTC',
    }
)
