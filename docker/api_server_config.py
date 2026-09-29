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
