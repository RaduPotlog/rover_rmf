# rover_rmf — Open-RMF fleet management for Rover A1 over MQTT

Open-RMF plans, schedules and dispatches tasks for Rover A1. The only link between the two is
**VDA 5050 2.0 over MQTT**: RMF's fleet adapter acts as VDA 5050 master control, and the rover
keeps its existing connector (`rover_vda5050`) unchanged. RMF runs off-robot in a **ROS 2 Jazzy**
container, because RMF's fleet adapter, dispatcher and websocket have no Lyrical release. The two
ROS distros never share a graph.

```
 Browser ── rmf-web dashboard :3000 ── api-server :8010
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
| `rover_rmf_maps` | One RMF site per directory in `maps/<site>/`: a floor plan and a building whose level is named `<site>`. `rover_world` is the Gazebo world (9 waypoints such as `rover_a1_charger` and `north`, 11 two-way lanes, generated from `rover_world.sdf`). Real sites are imported from the rover's saved maps. Nav graphs are generated at build time. `scripts/building_writer.py` and `scripts/rover_map_import.py` hold the shared code. |
| `rover_rmf_bringup` | `rover_rmf.launch.xml` (RMF core and the adapter; `site:=<site>`) and `config/fleet_<site>.yaml` (RMF fleet config plus the adapter's `vda5050` section). |
| `docker/` | `Dockerfile.rmf` (Jazzy, RMF debs, builds and tests the packages), `docker-compose.yml` (mosquitto, rmf, rmf-web api-server and dashboard), broker and api-server configs. `Dockerfile.dashboard` points the prebuilt dashboard at the api-server's published port. |
| `scripts/start_sim_rover.sh` | Starts the simulated rover's side in one terminal: Nav 2, drive mode, mission manager and the VDA 5050 connector on `:1884`. It then sets the drive mode to AUTOMATIC. `--localization indoor` behaves like the real rover. |
| `scripts/import_rover_map.py` | Turns a map saved on the rover (drive UI, indoor mode) into an RMF site. See "Real rover". |

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
  `mapId` (the rover's indoor map name), or `vda5050.map_name` while that is empty. The battery comes from `batteryState`; when the rover
  reports 0 % and 0 V (no reading, as in Gazebo), the adapter reports `unknown_battery_soc`
  (1.0) instead. Nothing is reported while `connection` is not `ONLINE`.

**Coordinates:** each site's fleet config has `transforms.<site>` from RMF coordinates to the
rover's map frame. EasyFullControl applies it to destinations and its inverse to reported
positions.
- **Gazebo:** RMF (x, y) = world (x + 12.5, y − 12.5). The simulated rover's SLAM map starts at
  its spawn pose, world (0, −2), so the translation is `[-12.5, 14.5]`.
- **Imported sites:** the floor plan is the rover's own map image, so the translation is exact:
  `(origin_x, origin_y + height · resolution)` from `map.yaml`. Nothing is surveyed.

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

- **Dashboard:** open <http://localhost:3000>. `rover_a1` appears at `rover_a1_charger` on the `rover_world` map.
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

The api-server is published on **8010** (`RMF_API_PORT`), not upstream's 8000. A Windows service
(`Manager.exe`) holds `0.0.0.0:8000` on this box. Docker Desktop then silently publishes nothing,
and the dashboard's LOGIN goes to that service and hangs. The dashboard image is rebuilt with the
same port. To pick another port, run
`RMF_API_PORT=<port> docker compose -f docker/docker-compose.yml up -d --build`.

## Real rover

RMF runs on the laptop. It talks to the rover's own broker, `192.168.1.201:1883`, which the
rover's `rover-a1-vda5050` container runs. The rover localizes indoors on a map saved with the
drive UI, and that map becomes the RMF site.

1. **Deploy.** Push the VDA 5050 and drive-mode work (rover_vda5050 `master`, rover_orchestrator,
   rover_ros, rover_drive_interface, rover_docker), then `balena push`.
2. **balenaCloud variables, on all services:**
   - `ROVER_LOCALIZATION_SOURCE=indoor`
   - `ROVER_START_MISSION_MANAGER=true`
   - `ROVER_START_VDA5050=true`
   - `ROVER_VDA5050_LOCAL_BROKER=true` is already the default.
3. **Map the area** in the drive UI.
   - Start mapping, drive the area, and save the map (e.g. `lab`). Then load it.
   - Save places. `dock` is where the rover parks; it becomes RMF's charger and parking spot.
   - Add 4–8 destinations at corridor junctions and ends, about 1 m from obstacles.
   - RMF drives only between places, along straight lanes the importer finds free in the map.
4. **Check the rover on its own** from the laptop, on the rover LAN (`.184` wired, or the
   ROVER-A1 AP):
   ```bash
   mosquitto_sub -h 192.168.1.201 -t 'uagv/v2/#' -v   # connection ONLINE, state mapId "lab"
   ros2 run rover_vda5050_bringup fake_master.py --host 192.168.1.201 order <x>,<y>   # a short hop
   ```
5. **Import the map and run RMF:**
   ```bash
   scripts/import_rover_map.py lab --from-rover root@192.168.1.201   # scp from port 24
   RMF_SITE=lab RMF_BROKER_HOST=192.168.1.201 \
       docker compose -f docker/docker-compose.yml up -d --build
   ```
   - The importer prints the lanes and the transform, and warns about places too close to
     obstacles.
   - If the places can't all be joined by clear straight lanes, it fails and names the groups.
     Add a place where the corridors meet.
   - Re-run it after changing the map or its places, then rebuild.
6. **On the dashboard** (http://localhost:3000), `rover_a1` appears at `dock` on the `lab` map.
   Start with a patrol between two places.

**First runs, safety:**
- Keep the RC transmitter / E-stop at hand and use a clear area.
- RMF only drives in AUTOMATIC. Moving the web joystick takes over to ASSISTED and cancels the
  order, and RMF then replans.
- **Known rover issues:**
  - The wheel PID overshoots goals.
  - The skid steer turns slowly in place; the 60 s stall watchdog makes RMF replan.
  - Nav 2 bringup aborts if the RS16 lidar is silent at boot, so check that the rover is
    localized (`positionInitialized` true) before dispatching.
- `rover_fleet` limits (0.5 m/s) only shape RMF's schedule. Nav 2's own limits drive the rover.

## Tests

```bash
colcon build --packages-select rover_rmf_fleet_adapter --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select rover_rmf_fleet_adapter && colcon test-result --verbose
cd rover_rmf_maps && python3 -m pytest test   # the map importer (rover_rmf_maps needs RMF's
                                              # building map tools to build with colcon)
```

These unit tests run on the host without RMF. The image build runs them again under Jazzy.

## Regenerating the Gazebo map

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
- **Battery and mechanics are estimates.** They are ASSUMPTION values in the fleet configs.
  They only shape RMF's planning; the real rover reports its own battery.
- **Straight lanes between places.** Imported sites get only these, so a place hidden behind a
  corner needs an intermediate place.
