# Copyright 2026 Rover A1 contributors
# Licensed under the Apache License, Version 2.0.

"""
Open-RMF EasyFullControl binding for one rover (rmf_fleet_adapter_python, Jazzy).

Positions cross this boundary in the ROBOT's frame: EasyFullControl applies the fleet config's
`transforms` (RMF map -> robot) to destinations and its inverse to the positions we report.
"""

import time
from typing import Optional

import rmf_adapter.easy_full_control as rmf_easy

from ..application.ports import Log, RmfCommands
from ..application.robot_session import RobotSession
from ..domain.model import DomainError, Pose2D


class RmfRobot(RmfCommands):

    def __init__(self, name: str, fleet_handle, configuration, log: Log):
        self._name = name
        self._fleet_handle = fleet_handle
        self._configuration = configuration
        self._log = log
        self._session: Optional[RobotSession] = None
        self._update_handle = None
        self._refused_map: Optional[str] = None

    def attach(self, session: RobotSession) -> None:
        self._session = session

    # --- RmfCommands --------------------------------------------------------------------------

    def finished(self, execution: object) -> None:
        execution.finished()

    def replan(self) -> None:
        if self._update_handle is not None:
            self._update_handle.more().replan()

    # --- update loop --------------------------------------------------------------------------

    def update(self) -> None:
        """Push the rover's state to RMF; the first valid state registers the robot."""
        self._session.tick(time.monotonic())
        state = self._session.rmf_state()
        if state is None:
            return
        rmf_state = rmf_easy.RobotState(state.map_name, list(state.position), state.battery_soc)
        if self._update_handle is None:
            handle = self._fleet_handle.add_robot(
                self._name, rmf_state, self._configuration, self._callbacks())
            if handle is None:
                # RMF refuses a robot whose map is not in the nav graph, e.g. the rover has
                # another saved map loaded than the site RMF runs. Say so once per map name;
                # keep retrying quietly, it registers as soon as the maps agree.
                if self._refused_map != state.map_name:
                    self._refused_map = state.map_name
                    self._log.error(
                        f'[{self._name}] RMF refused the robot: it is on map '
                        f"{state.map_name!r}, which this site's nav graph does not have. Load "
                        'the matching map on the rover, or start RMF with RMF_SITE=<that map> '
                        '(imported with scripts/import_rover_map.py).')
                return
            self._update_handle = handle
            self._refused_map = None
            self._log.info(f'[{self._name}] registered with RMF on {state.map_name} at '
                           f'({state.position[0]:.2f}, {state.position[1]:.2f})')
            return
        execution = self._session.execution
        activity = execution.identifier if execution is not None else None
        self._update_handle.update(rmf_state, activity)

    # --- RMF callbacks ------------------------------------------------------------------------

    def _callbacks(self):
        return rmf_easy.RobotCallbacks(
            lambda destination, execution: self._navigate(destination, execution),
            lambda activity: self._stop(activity),
            lambda category, description, execution: self._execute_action(
                category, description, execution))

    def _navigate(self, destination, execution) -> None:
        x, y, yaw = (float(v) for v in destination.position)
        try:
            goal = Pose2D(x, y, yaw)
        except DomainError as e:
            self._log.error(f'[{self._name}] unusable destination: {e}')
            return
        if destination.dock is not None:
            # No docking on the rover yet: drive to the dock's waypoint like any other.
            self._log.warning(f'[{self._name}] dock {destination.dock} treated as a waypoint')
        self._session.navigate(execution, goal, destination.speed_limit, time.monotonic())

    def _stop(self, activity) -> None:
        execution = self._session.execution
        if execution is not None and execution.identifier.is_same(activity):
            self._session.stop(time.monotonic())

    def _execute_action(self, category: str, description: dict, execution) -> None:
        self._session.perform_action(category, description, execution, time.monotonic())
