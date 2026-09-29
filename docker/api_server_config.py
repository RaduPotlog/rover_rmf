# rmf-web api-server settings for the rover_rmf compose stack (RMF_API_SERVER_CONFIG).
# Like upstream's sqlite_local_config.py, but on wall time: RMF here never sees a /clock.
from api_server.default_config import config

config.update(
    {
        'host': '0.0.0.0',
        'port': 8000,
        # The browser (dashboard) reaches the server through the published port.
        'public_url': 'http://localhost:8000',
        'db_url': 'sqlite:///ws/run/db.sqlite3',
        'cache_directory': '/ws/run/cache',
        'ros_args': [],
        'log_level': 'INFO',
        'timezone': 'UTC',
    }
)
