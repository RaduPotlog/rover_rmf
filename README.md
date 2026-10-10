<p align="center">
  <img src="icons/Logo-Arm-WhiteOrange-372x372-1.png" alt="Mechatronics Academy" width="140">
</p>

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
| `dashboard/` | Our rmf-web dashboard app: Mechatronics Academy branding, the **ROVER TASK** dialog (`tasks/`) and the **Rover** window (`rover/`: drive mode, in or out of the fleet). See [Dashboard](#dashboard). |
| `docker/` | `Dockerfile.rmf` (Jazzy, RMF debs, builds and tests the packages), `docker-compose.yml` (mosquitto, rmf, rmf-web api-server and dashboard), broker and api-server configs. `Dockerfile.dashboard` builds `dashboard/` inside a pinned rmf-web checkout. |
| `scripts/start_sim_rover.sh` | Starts the simulated rover's side in one terminal: Nav 2, drive mode, mission manager and the VDA 5050 connector on `:1884`. It then sets the drive mode to AUTOMATIC. `--localization indoor` behaves like the real rover. |
| `site_manager/` | The RMF sites page (<http://localhost:8020>, compose service `site-manager`): lists the maps saved on the rover, imports them as sites into the `rmf-sites` volume, and activates one. `docker/run_rmf.sh` runs RMF for the active site and restarts it on a switch. |
| `scripts/import_rover_map.py` | The same import from the command line, into `rover_rmf_maps/maps/` (git-ignored, built into the local image). |

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
- **Rover not available:** while the rover reports `operatingMode` `MANUAL`/`SERVICE` (drive mode
  not Automatic) or a `motionLocked` error (e-stop, lock), the adapter sends no order. RMF's
  command waits, with no timeout, and goes out once the rover is back in Automatic and unlocked.
  The log says `holding the command` once. (`missionRefused` doesn't hold: it is why the *last*
  order was refused and stays until the next one is accepted.)
- **Replan backoff:** a failed command asks RMF to replan after 1 s, then 2, 4 … up to 60 s for
  failures in a row; an arrival resets it. On 2026-10-01 the real rover refused every order at
  once, and without the backoff RMF and the rover exchanged an order and a refusal every second
  over 4G.
- **`stop()`:** sends `cancelOrder`, and cancels a running perform-action.
- **In or out of the fleet:** the adapter decommissions the rover in RMF while it is offline or
  an operator has it (`operatingMode` `MANUAL`/`SERVICE`: drive mode Manual or Assisted). It
  recommissions it on its next `AUTOMATIC` state. Decommissioned, RMF gives it no new tasks and a
  task sent anyway fails at dispatch ("No fleet adapters offered a bid"). Queued tasks are kept.
  Because this is derived from the rover's state, it holds whoever switched the mode and survives
  RMF restarts. A decommission from rmf-web's robot dialog lasts until the next mode change. While
  the connection is down, RMF also shows the robot `offline`. RMF never removes a robot
  otherwise; it stays registered for the adapter's lifetime.
- **Drive mode from RMF:** the adapter's **rover control API**
  (`presentation/control_api.py`, port `vda5050.control_api_port`, 8030) serves the dashboard's
  Rover window. `POST /api/robots/<name>/drive_mode {"mode": "MANUAL"|"AUTOMATIC"}` sends the
  rover's custom VDA 5050 instant action `setDriveMode` (rover_vda5050). The request answers once
  the rover reports the action `FINISHED` or `FAILED` in `actionStates`, or after 10 s.
  `GET /api/robots` returns each rover's connection, mode, fleet membership and last request.
- **Idle rover:** `finishing_request: "nothing"` and `responsive_wait: false`, so the rover stays
  where its last task ended and starting RMF never moves it by itself. "park" would send it to its
  charger after every task and on startup. A responsive wait would drive it to the nearest
  waypoint.
- **Perform-actions:** a compose task's `perform_action` runs in the adapter
  (`application/actions.py`), by category. The only one so far is `wait`
  (`{"duration_sec": N}`, up to 3600 s): the rover stays where it is and the adapter finishes
  the action when the time is up. A category the fleet config lists under `actions` but the
  adapter has no handler for is logged at startup. If such an action is dispatched anyway, the
  adapter skips it with an error.
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
  **ROVER TASK** → *Patrol with pause* → stops `north` and `east`, pause 10 s, 2 rounds. The `rmf`
  log then shows `perform-action 'wait'` after each arrival.
- **Patrol task from the CLI:**

  ```bash
  docker compose -f docker/docker-compose.yml exec rmf \
      /entrypoint.sh ros2 run rmf_demos_tasks dispatch_patrol -p north east -n 2
  ```

  The dashboard's task panel can create the same task.
- **Watching the VDA traffic:** `mosquitto_sub -p 1884 -t 'uagv/v2/#' -v`
- **The whole real-rover flow in the sim:** run `start_sim_rover.sh --localization indoor`,
  then map and save the map and its places with the indoor manager's services.
  - Start the stack with `-f docker/docker-compose.sim-maps.yml` added, so the sites page reads
    `~/rover_maps` instead of the rover's `/maps`.
  - On <http://localhost:8020>, import and activate the map, as on the real rover.

**Zenoh:** `start_sim_rover.sh` runs everything except Nav 2 as zenoh clients. A peer that
exits, such as a restarted connector, stalls `slam_toolbox`'s links, and the map frame
disappears.

**Ports:** the broker is published on **1884**, because the host may run its own mosquitto on
1883. WSL 2 distros share the Docker Desktop VM's network, so the host-side connector reaches it
on `127.0.0.1:1884`.

The api-server is published on **8010** (`RMF_API_PORT`), not upstream's 8000. A Windows service
(`Manager.exe`) holds `0.0.0.0:8000` on this box. Docker Desktop then silently publishes nothing,
and the dashboard's LOGIN goes to that service and hangs. The dashboard is built with the same
port. To pick another port, run
`RMF_API_PORT=<port> docker compose -f docker/docker-compose.yml up -d --build`.

## Real rover

RMF runs on the laptop. It talks to the rover's own broker, `192.168.1.201:1883`, which the
rover's `rover-a1-vda5050` container runs. The rover localizes indoors on a map saved with the
drive UI, and that map becomes the RMF site.

1. **Deploy.** Push the VDA 5050 and drive-mode work (rover_vda5050 `master`, rover_orchestrator,
   rover_ros, rover_drive_interface, rover_docker), then `balena push`.
2. **balenaCloud variables, on all services:**
   - `ROVER_ORCH_LOCALIZATION_SOURCE=indoor`
   - `ROVER_ORCH_MISSION_MANAGER=true`
   - `ROVER_VDA5050_ENABLE=true`
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
5. **Start the stack and let it read the rover's maps.**
   ```bash
   docker compose -f docker/docker-compose.yml up -d --build
   docker compose -f docker/docker-compose.yml exec site-manager rover-authorize   # once; password root
   ```
   `rover-authorize` puts the sites page's own ssh key on the rover, which asks for its
   password once. A new rover release forgets the key; the page then says so, and you run the
   command again.
6. **Import and activate the map** on the RMF sites page, <http://localhost:8020>.
   - **Maps on the rover** lists the saved maps with their places. Pick the charger place
     (default `dock`) and press **Import**.
   - The site appears under **RMF sites** with a preview: lanes in blue, places in orange, the
     charger in green. Warnings name places too close to obstacles.
   - If the places can't all be joined by clear straight lanes, the import fails and names the
     groups. Add a place where the corridors meet, then import again.
   - Press **Activate**: RMF restarts on the site within a few seconds, with no rebuild.
     Importing the running site again restarts RMF on the new version.
7. **On the dashboard** (<http://localhost:3000>, reload it after a site switch), `rover_a1`
   appears at `dock` on the `lab` map. Start with a patrol between two places.

Sites imported on the page live in the `rmf-sites` Docker volume. The CLI,
`scripts/import_rover_map.py lab --from-rover root@192.168.1.201`, does the same import into
`rover_rmf_maps/maps/lab/` and `config/fleet_lab.yaml`. It uses ssh port 24 and asks for the
password once. Those files are built into the image made on this machine, but git ignores them:
maps imported from the rover are not committed; only the Gazebo site `rover_world` is. A page
import with the same name overrides a built-in site.

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

## Dashboard

`dashboard/` is our app on rmf-web's dashboard framework (`rmf-dashboard-framework`), the same
way rmf-web's own demo dashboard is built. `docker/Dockerfile.dashboard` downloads rmf-web at a
pinned commit (`RMF_WEB_REF`, the `jazzy` branch the jazzy-nightly images come from) and copies
`dashboard/` in as `examples/rover`. It runs the task model tests and a type check, then builds.
The API and trajectory URLs are compiled in from the `API_URL` / `TRAJECTORY_URL` build args.

What it changes from the stock dashboard:
- **Branding:** the logo (`public/resources/logo.png`, the round crop of
  `icons/Logo-Arm-WhiteOrange-372x372-1.png`), the tab title, the favicon and the orange theme
  (`theme.ts`).
- **Task list:** NEW TASK keeps only Patrol and Custom Compose. The fleet refuses Delivery
  and Clean.
- **Rover window** (Map and Robots tabs, also addable on Custom): connection, drive mode, in or out
  of the fleet, RMF status and battery, with **Manual** and **Automatic** buttons. Manual takes the
  rover out of the fleet. Automatic asks first whether the area is clear, because the rover
  rejoins the fleet and resumes its queued tasks. It polls the fleet adapter's rover control API
  every 2 s; the URL is compiled in from the `ROVER_API_URL` build arg. The buttons need the rover's
  connector with `setDriveMode` (rover_vda5050 `feature/remote-drive-mode` or later). An older
  connector fails the action, and the window shows that.
- **ROVER TASK:** a button in the app bar opens our own dialog with the rover's task types. It
  dispatches to "any robot" or to a chosen one.
  - *Patrol with pause* visits stops in order and waits at each (0 = drive through), for a
    number of rounds.
  - It is sent as a `compose` task with one phase per stop: `go_to_place`, then a
    `perform_action` `wait`. Jazzy compose tasks have no wait event of their own.

**Adding a rover task:**
1. Add `dashboard/tasks/<name>/` with a `RoverTaskType`: a pure model (`model.ts` with
   `makeDefault`, `validate`, `toRequest`, plus a `model.test.ts`) and a form (`form.tsx`).
2. Add one line to `tasks/registry.ts`.
3. If the robot has to *do* something other than drive (switch an aux output, dock …):
   - add a handler to `rover_rmf_fleet_adapter/application/actions.py`, with its description
     in `domain/actions.py`;
   - add its category to `actions` in `rover_rmf_bringup/config/fleet_rover_world.yaml`.

**Sites imported before an `actions` change** keep their old fleet config, which
`site_io.fleet_config` copied from that template. Re-import them on the sites page, or RMF
refuses tasks that use the new action ("Fleet not configured to perform this action").

## Server deployment (WireGuard)

For permanent use, RMF and the fleet's MQTT broker run on a server: `rover-a1-server`, reached at
`10.8.0.1` over WireGuard. The rover's connector dials out to that broker, so RMF keeps working
while the laptop is off. Every port is published on the WireGuard address only.

| Port on 10.8.0.1 | Service |
|---|---|
| 1883 | Mosquitto, with logins: `rmf` (this stack) and `rover_a1` (only its own `uagv/v2/MechatronicsAcademy/rover_a1/#`) |
| 3000 | Dashboard |
| 8010 | api-server |
| 8006 | Trajectory websocket (dashboard map) |
| 8030 | Rover control API (fleet adapter): the dashboard's Rover window |
| 8020 | Sites page |

**On the server**, once:

1. Install Docker Engine with the compose plugin, and start it after `wg-quick@wg0`. Ports bound
   to 10.8.0.1 fail if `wg0` isn't up yet.
2. Check out this repo and run:
   ```bash
   docker/rmf-server.sh init            # docker/server/.env + broker logins (git-ignored)
   docker/rmf-server.sh up -d --build
   docker/rmf-server.sh exec site-manager rover-authorize   # once per rover release
   ```

`docker/rmf-server.sh` wraps `docker compose` with `docker-compose.server.yml` and
`server/.env`. That overlay does three things:
- moves every port to `RMF_BIND_IP`;
- switches Mosquitto to `mosquitto.server.conf` + `mosquitto.acl`;
- points RMF at that broker for every site, logging in with `RMF_BROKER_USERNAME` /
  `RMF_BROKER_PASSWORD`.

**On the rover**, set these balenaCloud variables on `rover-a1-vda5050`. The password comes from
`docker/rmf-server.sh rover-password`.

| Variable | Value |
|---|---|
| `ROVER_VDA5050_ENABLE` | `true` |
| `ROVER_VDA5050_LOCAL_BROKER` | `false` |
| `ROVER_VDA5050_BROKER_HOST` | `10.8.0.1` |
| `ROVER_VDA5050_BROKER_PORT` | `1883` |
| `ROVER_VDA5050_BROKER_USER` | `rover_a1` |
| `ROVER_VDA5050_BROKER_PASSWORD` | the password from `rover-password` |
| `ROVER_VDA5050_BROKER_TLS` | `false`; WireGuard already encrypts the link |

The rover reaches 10.8.0.1 through the RUTX11's tunnel. The sites page reaches the rover's ssh
(`192.168.1.201:24`) the other way, which needs a RUTX11 firewall rule for `10.8.0.1`.
Over 4G, VDA `state` (1 Hz) and `visualization` (2 Hz) are the steady load.

## Tests

```bash
colcon build --packages-select rover_rmf_fleet_adapter --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select rover_rmf_fleet_adapter && colcon test-result --verbose
(cd rover_rmf_maps && python3 -m pytest test)  # map importer, map sources, preview (colcon
                                               # needs RMF's building map tools for this one)
python3 -m pytest site_manager/test            # the sites page's server and API
```

These unit tests run on the host without RMF. The image build runs them again under Jazzy. The
dashboard's task model tests (`dashboard/tasks/**/*.test.ts`, vitest) and its type check run in
`docker compose -f docker/docker-compose.yml build dashboard`.

## Regenerating the Gazebo map

Edit `WAYPOINTS` / `LANES` in `rover_rmf_maps/scripts/generate_rover_world.py`, then run it. It
reads `rover_world.sdf` and rewrites the PNG and the building file.

## Known limitations

- **Stops at every waypoint.** Each RMF waypoint is its own order, so the rover stops at every
  graph waypoint. Keep the lane graph sparse. The fix would be order stitching (same `orderId`,
  `orderUpdateId + 1`).
- **Final heading.** The rover arrives facing along the path (the connector's
  `orientation_mode: path`), not at the yaw RMF asked for.
- **Missing features:** no docking, doors or lifts, and no battery model in simulation. The
  only perform-action is `wait`.
- **Battery and mechanics are estimates.** They are ASSUMPTION values in the fleet configs.
  They only shape RMF's planning; the real rover reports its own battery.
- **Straight lanes between places.** Imported sites get only these, so a place hidden behind a
  corner needs an intermediate place.

## License

Apache-2.0 (`LICENSE`). `dashboard/main.tsx` is derived from rmf-web's demo dashboard
([open-rmf/rmf-web](https://github.com/open-rmf/rmf-web), Apache-2.0); its header names the
upstream file and commit.
