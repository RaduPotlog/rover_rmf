# Copyright 2026 Rover A1 contributors
# Licensed under the Apache License, Version 2.0.

"""
paho-mqtt client acting as VDA 5050 master control for one rover.

Subscribes to state / visualization / connection / factsheet and hands them to the session;
publishes order / instantActions with the VDA 5050 header. QoS as VDA 5050 prescribes: 0 for
everything but connection (1). The connector's connection message is retained, so a restarted
adapter learns at once whether the rover is ONLINE.
"""

from datetime import datetime, timezone
import json
import threading
import time
from typing import Optional

from paho.mqtt import client as mqtt

from ..application.ports import FleetLink, Log
from ..application.robot_session import RobotSession
from ..domain.model import VdaIdentity
from ..domain.robot_status import parse_position, parse_state
from ..domain.vda_messages import HeaderCounter, with_header


class MqttVdaLink(FleetLink):

    def __init__(self, identity: VdaIdentity, host: str, port: int, log: Log,
                 username: str = '', password: str = '', client_id: str = ''):
        self._identity = identity
        self._host = host
        self._port = port
        self._log = log
        self._headers = HeaderCounter()
        self._headers_lock = threading.Lock()
        self._session: Optional[RobotSession] = None

        client_id = client_id or f'rmf-{identity.serial_number}'
        # paho 2.x (the rover's WSL host) and 1.6 (Ubuntu Noble, the Jazzy image) differ in the
        # constructor and the connect/disconnect callback signatures; both land in _connected /
        # _disconnected.
        if hasattr(mqtt, 'CallbackAPIVersion'):
            self._client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=client_id)
            self._client.on_connect = (
                lambda client, userdata, flags, reason_code, properties:
                    self._connected(client, not reason_code.is_failure, str(reason_code)))
            self._client.on_disconnect = (
                lambda client, userdata, flags, reason_code, properties:
                    self._disconnected(str(reason_code)))
        else:
            self._client = mqtt.Client(client_id=client_id)
            self._client.on_connect = (
                lambda client, userdata, flags, rc:
                    self._connected(client, rc == 0, mqtt.connack_string(rc)))
            self._client.on_disconnect = (
                lambda client, userdata, rc: self._disconnected(mqtt.error_string(rc)))
        if username:
            self._client.username_pw_set(username, password or None)
        self._client.on_message = self._on_message
        self._client.reconnect_delay_set(min_delay=1, max_delay=10)

    def attach(self, session: RobotSession) -> None:
        self._session = session

    def start(self) -> None:
        # connect_async + loop_start: keeps retrying in the background while the broker is down.
        self._client.connect_async(self._host, self._port, keepalive=30)
        self._client.loop_start()

    def close(self) -> None:
        self._client.loop_stop()
        self._client.disconnect()

    # --- FleetLink ----------------------------------------------------------------------------

    def send(self, topic: str, body: dict) -> None:
        with self._headers_lock:
            header_id = self._headers.next_id(topic)
        message = with_header(self._identity, header_id, datetime.now(timezone.utc), body)
        info = self._client.publish(self._identity.topic(topic), json.dumps(message), qos=0)
        if info.rc != mqtt.MQTT_ERR_SUCCESS:
            self._log.error(f'MQTT publish on {topic} failed: {mqtt.error_string(info.rc)}')

    # --- paho callbacks (paho's network thread) -------------------------------------------------

    def _connected(self, client, ok: bool, reason: str):
        if not ok:
            self._log.error(f'MQTT connect to {self._host}:{self._port} refused: {reason}')
            return
        self._log.info(f'MQTT connected to {self._host}:{self._port}, '
                       f'master control for {self._identity.topic_prefix}')
        client.subscribe([
            (self._identity.topic('state'), 0),
            (self._identity.topic('visualization'), 0),
            (self._identity.topic('connection'), 1),
            (self._identity.topic('factsheet'), 0),
        ])

    def _disconnected(self, reason: str):
        self._log.warning(f'MQTT disconnected ({reason}); reconnecting')
        if self._session is not None:
            # Without the broker we know nothing about the rover: stop feeding RMF positions.
            self._session.on_connection('BROKER_UNREACHABLE')

    def _on_message(self, client, userdata, msg):
        session = self._session
        if session is None or not msg.payload:
            # An empty payload clears a retained message; there is nothing to read.
            return
        try:
            payload = json.loads(msg.payload)
        except (UnicodeDecodeError, json.JSONDecodeError) as e:
            self._log.warning(f'Ignoring malformed JSON on {msg.topic}: {e}')
            return
        if not isinstance(payload, dict):
            return
        topic = msg.topic.rsplit('/', 1)[-1]
        if topic == 'state':
            session.on_state(parse_state(payload), time.monotonic())
        elif topic == 'visualization':
            pose, map_id = parse_position(payload.get('agvPosition'))
            session.on_visualization(pose, map_id)
        elif topic == 'connection':
            session.on_connection(str(payload.get('connectionState', '')))
        elif topic == 'factsheet':
            series = (payload.get('typeSpecification') or {}).get('seriesName', '?')
            self._log.info(f'factsheet from {self._identity.serial_number}: {series}')
