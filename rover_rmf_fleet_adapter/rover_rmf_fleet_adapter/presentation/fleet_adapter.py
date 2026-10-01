# Copyright 2026 Rover A1 contributors
# Licensed under the Apache License, Version 2.0.

"""
Entry point: an Open-RMF EasyFullControl fleet adapter that drives rovers over VDA 5050 / MQTT.

    fleet_adapter -c fleet_<site>.yaml -n nav_graphs/0.yaml [--broker-host H] [--broker-port P]
                  [-sim] --ros-args -p server_uri:=ws://api-server:8000/_internal

Same arguments as rmf_demos_fleet_adapter, plus the broker. The fleet config holds RMF's
`rmf_fleet` section and this adapter's `vda5050` section (identity per robot, timeouts).
RMF_BROKER_USERNAME / RMF_BROKER_PASSWORD in the environment override the broker login.
vda5050.control_api_port (8030) serves the rover control API (presentation/control_api.py).
"""

import argparse
import os
import sys
import threading
import time
import uuid

import rclpy
import rclpy.node
from rclpy.parameter import Parameter
import rmf_adapter
from rmf_adapter import Adapter
import rmf_adapter.easy_full_control as rmf_easy
import yaml

from ..application.actions import ACTION_HANDLERS
from ..application.broker_settings import resolve_broker
from ..application.ports import Log
from ..application.robot_session import RobotSession
from ..domain.command_tracker import CommandTracker
from ..domain.model import VdaIdentity
from ..infrastructure.mqtt_vda_link import MqttVdaLink
from ..infrastructure.rmf_robot import RmfRobot
from . import control_api


class RclpyLog(Log):

    def __init__(self, logger):
        self._logger = logger

    def info(self, message: str) -> None:
        self._logger.info(message)

    def warning(self, message: str) -> None:
        self._logger.warning(message)

    def error(self, message: str) -> None:
        self._logger.error(message)


def order_id_factory(robot: str):
    counter = iter(range(1, 1 << 31))
    run = uuid.uuid4().hex[:6]
    # Unique across adapter restarts (run), readable in logs (counter).
    return lambda: f'rmf-{robot}-{run}-{next(counter)}'


def parse_args(argv):
    parser = argparse.ArgumentParser(prog='fleet_adapter', description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('-c', '--config_file', required=True, help='Fleet config yaml')
    parser.add_argument('-n', '--nav_graph', required=True, help='Nav graph yaml')
    parser.add_argument('-sim', '--use_sim_time', action='store_true',
                        help='Use /clock (RMF and the rover must then share one clock)')
    parser.add_argument('--broker-host', default='', help='Overrides vda5050.broker.host')
    parser.add_argument('--broker-port', type=int, default=0, help='Overrides vda5050.broker.port')
    return parser.parse_args(argv)


def main(argv=None):
    argv = sys.argv if argv is None else argv
    rclpy.init(args=argv)
    rmf_adapter.init_rclcpp()
    args = parse_args(rclpy.utilities.remove_ros_args(argv)[1:])

    fleet_config = rmf_easy.FleetConfiguration.from_config_files(args.config_file, args.nav_graph)
    if not fleet_config:
        sys.exit(f'Failed to parse fleet config [{args.config_file}]')
    with open(args.config_file) as f:
        config = yaml.safe_load(f)
    vda = config.get('vda5050') or {}
    broker = resolve_broker(vda.get('broker') or {}, args.broker_host, args.broker_port,
                            os.environ)

    fleet_name = fleet_config.fleet_name
    node = rclpy.node.Node(f'{fleet_name}_command_handle')
    log = RclpyLog(node.get_logger())
    adapter = Adapter.make(f'{fleet_name}_fleet_adapter')
    if not adapter:
        sys.exit('Unable to initialize the fleet adapter: is rmf_traffic_schedule running?')
    if args.use_sim_time:
        node.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        adapter.node.use_sim_time()
    adapter.start()
    time.sleep(1.0)

    node.declare_parameter('server_uri', '')
    server_uri = node.get_parameter('server_uri').get_parameter_value().string_value
    fleet_config.server_uri = server_uri or None
    fleet_handle = adapter.add_easy_fleet(fleet_config)
    for action in config['rmf_fleet'].get('actions') or []:
        if action not in ACTION_HANDLERS:
            # RMF would dispatch it and the session would skip it: say so up front.
            log.error(f'fleet config lists action {action!r}, which this adapter cannot run '
                      f'(it runs {sorted(ACTION_HANDLERS)})')

    robots, links, sessions = {}, [], {}
    robot_vda = vda.get('robots') or {}
    for name in fleet_config.known_robots:
        robot_cfg = robot_vda.get(name) or {}
        identity = VdaIdentity(
            interface_name=robot_cfg.get('interface_name', vda.get('interface_name', 'uagv')),
            manufacturer=robot_cfg.get('manufacturer', vda.get('manufacturer', '')),
            serial_number=robot_cfg.get('serial_number', name),
            version=vda.get('version', '2.0.0'))
        link = MqttVdaLink(
            identity,
            host=broker.host,
            port=broker.port,
            log=log,
            username=broker.username,
            password=broker.password,
            client_id=f'rmf-{fleet_name}-{name}')
        rmf_robot = RmfRobot(name, fleet_handle,
                             fleet_config.get_known_robot_configuration(name), log)
        tracker = CommandTracker(order_id_factory(name),
                                 accept_timeout=float(vda.get('accept_timeout', 10.0)),
                                 cancel_timeout=float(vda.get('cancel_timeout', 20.0)),
                                 stall_timeout=float(vda.get('stall_timeout', 60.0)))
        session = RobotSession(name, link, rmf_robot, log, tracker,
                               default_map=robot_cfg.get('map_name', vda.get('map_name', 'L1')),
                               unknown_battery_soc=float(vda.get('unknown_battery_soc', 1.0)))
        link.attach(session)
        rmf_robot.attach(session)
        link.start()
        robots[name] = rmf_robot
        sessions[name] = session
        links.append(link)

    # The dashboard's Rover card: drive mode and fleet membership. 0 turns it off.
    control_port = int(vda.get('control_api_port', 8030))
    if control_port:
        control_api.start(sessions, '0.0.0.0', control_port, log)

    period = 1.0 / float(vda.get('update_frequency', 10.0))
    reassign_interval = float(config['rmf_fleet'].get('reassign_task_interval', 60.0))

    def update_loop():
        last_reassign = time.monotonic()
        while rclpy.ok():
            started = time.monotonic()
            for robot in robots.values():
                try:
                    robot.update()
                except Exception as e:  # noqa: B902 keep the loop alive for the other robots
                    log.error(f'update failed: {e!r}')
            if started - last_reassign > reassign_interval:
                fleet_handle.more().reassign_dispatched_tasks()
                last_reassign = started
            time.sleep(max(0.0, period - (time.monotonic() - started)))

    threading.Thread(target=update_loop, daemon=True).start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        for link in links:
            link.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
