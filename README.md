# rover_rmf — Open-RMF fleet management for Rover A1 over MQTT

Open-RMF plans, schedules and dispatches tasks for Rover A1. The only link between the two is
**VDA 5050 2.0 over MQTT**: RMF's fleet adapter acts as VDA 5050 master control, and the rover
keeps its existing connector (`rover_vda5050`) unchanged. RMF runs off-robot in a **ROS 2 Jazzy**
container, because RMF's fleet adapter, dispatcher and websocket have no Lyrical release. The two
ROS distros never share a graph.

```
 Browser ── rmf-web dashboard :3000 ── api-server :8000
                                          │ ROS 2 Jazzy, rmw_cyclonedds, compose network only
 ┌─ docker compose (docker/) ─────────────┴────────────────────────────────────────┐
 │ rmf_traffic_schedule · blockade · rmf_task_dispatcher · building_map_server      │
 │ schedule visualizer (:8006) · rover_rmf_fleet_adapter (EasyFullControl + paho)   │
 │ mosquitto (host :1884)                                                           │
 └──────────────────────────────┬──────────────────────────────────────────────────┘
      MQTT uagv/v2/MechatronicsAcademy/rover_a1/{order, instantActions ↓ · state, connection, visualization ↑}
 Host (Lyrical, rmw_zenoh)      ▼
   rover_vda5050 connector → rover_mission_manager → Nav 2 → Gazebo (rover_gazebo)
```

## Packages

| Package | What |
|---------|------|
| `rover_rmf_fleet_adapter` | EasyFullControl fleet adapter. `domain/` holds the VDA 5050 messages, the state parsing and the command tracker. `application/` holds `RobotSession`. `infrastructure/` holds the paho link and the RMF binding. `presentation/` is the entry point. Only `infrastructure/` and `presentation/` import RMF, ROS or paho, and `scripts/check_domain_purity.sh` enforces it. |
| `rover_rmf_maps` | The Gazebo world `rover_world.sdf` as an RMF building (level `L1`): the floor plan, 9 waypoints (`rover_a1_charger`, `north`, `northeast`, …) and 11 two-way lanes. The nav graph is generated at build time. |
| `rover_rmf_bringup` | `rover_rmf.launch.xml` (RMF core and the adapter) and `config/rover_fleet.yaml` (RMF fleet config plus the adapter's `vda5050` section). |
| `docker/` | `Dockerfile.rmf` (Jazzy, RMF debs, builds and tests the packages), `docker-compose.yml` (mosquitto, rmf, rmf-web api-server and dashboard), broker and api-server configs. |
| `scripts/start_sim_rover.sh` | Starts the rover's side in one terminal: Nav 2, drive mode, mission manager and the VDA 5050 connector on `:1884`. It then sets the drive mode to AUTOMATIC. |

## How a command reaches the rover

RMF's EasyFullControl hands the adapter one destination at a time. Each destination becomes a
two-node VDA order, from the rover's current pose to the destination, in the format
`fake_master.py` verified. The node ids carry the order id (`<order>-start`, `<order>-goal`).

- **Arrival:** the order counts as arrived when the rover reports it with no nodes or edges left
  and `lastNodeId == <order>-goal`. The adapter then calls `execution.finished()`.
- **Failure:** the command fails in any of these cases, and the adapter asks RMF to
  `replan()`:
  - an error references the order or one of its nodes, for example `orderUpdateError`, or the
    `FATAL` `noRouteError` that follows a refused or failed mission;
  - the order ends away from the goal, for example because it was cancelled on the web UI or
    the rover left AUTOMATIC;
  - the rover switches to another order;
  - the order is not accepted within 10 s.
- **Replacing a running order:** the connector rejects a new `orderId` while an order still has
  nodes or edges. So when an order is running, the adapter sends `cancelOrder`, waits until the
  rover is idle (up to 20 s), and then sends the new order from where the rover stopped.
- **Rover not localized:** a command that arrives while the rover reports no position
  (`positionInitialized: false`) or no state waits up to 10 s for one, then fails. It does not
  fail at once, because RMF answers a failure with an immediate replan and the two would spin.
- **Stall watchdog:** an active order that makes no progress (0.3 m or 0.5 rad) for 60 s
  (`vda5050.stall_timeout`) fails, and RMF replans. Nav 2 never gives up on its own when the
  skid steer stalls near a goal.
- **`stop()`:** sends `cancelOrder`.
- **Reported state:** position comes from `state` (1 Hz) and `visualization` (2 Hz). The map is
  `mapId`, or `L1` while that is empty. The battery comes from `batteryState`; when the rover
  reports 0 % and 0 V (no reading, as in Gazebo), the adapter reports `unknown_battery_soc`
  (1.0) instead. Nothing is reported while `connection` is not `ONLINE`.

**Coordinates:** RMF (x, y) = world (x + 12.5, y − 12.5). The simulated rover's SLAM map starts
at its spawn pose, world (0, −2), so `rover_fleet.yaml` `transforms.L1.translation` is
`[-12.5, 14.5]`. EasyFullControl applies that transform to destinations and its inverse to
reported positions. A real site needs its own survey here.

## Running the simulation

Terminal 1, Gazebo:

```bash
~/ros2_ws/rover_a1/src/rover_ros/rover_gazebo/scripts/rover_sim.sh
```

Terminal 2, RMF (Docker Desktop):

```bash
cd ~/ros2_ws/rover_a1/src/rover_rmf
docker compose -f docker/docker-compose.yml up --build
```

Terminal 3, the rover stack:

```bash
~/ros2_ws/rover_a1/src/rover_rmf/scripts/start_sim_rover.sh
```

Then:

- **Dashboard:** open <http://localhost:3000>. `rover_a1` appears at `rover_a1_charger` on L1.
- **Patrol task from the CLI:**

  ```bash
  docker compose -f docker/docker-compose.yml exec rmf \
      /entrypoint.sh ros2 run rmf_demos_tasks dispatch_patrol -p north east -n 2
  ```

  The dashboard's task panel can create the same task.
- **Watching the VDA traffic:** `mosquitto_sub -p 1884 -t 'uagv/v2/#' -v`

**Zenoh:** `start_sim_rover.sh` runs everything except Nav 2 as zenoh clients. A peer that
exits, such as a restarted connector, stalls `slam_toolbox`'s links, and the map frame
disappears.

**Ports:** the broker is published on **1884**, because the host may run its own mosquitto on
1883. WSL 2 distros share the Docker Desktop VM's network, so the host-side connector reaches it
on `127.0.0.1:1884`.

## Tests

```bash
colcon build --packages-select rover_rmf_fleet_adapter --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select rover_rmf_fleet_adapter && colcon test-result --verbose
```

These unit tests run on the host without RMF. The image build runs them again under Jazzy.

## Regenerating the map

Edit `WAYPOINTS` / `LANES` in `rover_rmf_maps/scripts/generate_rover_world.py`, then run it. It
reads `rover_world.sdf` and rewrites the PNG and the building file.

## Known limitations

- **Stops at every waypoint.** Each RMF waypoint is its own order, so the rover stops at every
  graph waypoint. Keep the lane graph sparse. The fix would be order stitching (same `orderId`,
  `orderUpdateId + 1`).
- **Final heading.** The rover arrives facing along the path (the connector's
  `orientation_mode: path`), not at the yaw RMF asked for.
- **Missing features:** no docking, doors, lifts or perform-actions, and no battery model in
  simulation.
- **Real rover not calibrated.** Its transform and battery parameters are still ASSUMPTION
  values in `rover_fleet.yaml`.
