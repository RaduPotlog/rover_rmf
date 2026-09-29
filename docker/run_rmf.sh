#!/bin/bash
# Copyright 2026 Rover A1 contributors
# Licensed under the Apache License, Version 2.0.
#
# Runs RMF (rover_rmf.launch.xml) for the active site and restarts it when the site changes.
#
#   run_rmf.sh [launch args...]     e.g. server_uri:=ws://api-server:8000/_internal
#
# RMF_BROKER_HOST / RMF_BROKER_PORT, when set, override the broker in the site's fleet config.
#
# The site is the name in $SITES_DIR/active (written by the site manager page), else $RMF_SITE,
# else rover_world. An imported site lives in $SITES_DIR/<site>/ and wins over a built-in one in
# rover_rmf_maps. RMF is restarted when the active name changes or the active imported site is
# imported again. Status for the page: $SITES_DIR/rmf_status.json.
set -m

SITES_DIR="${SITES_DIR:-/sites}"
MAPS_SHARE="$(ros2 pkg prefix rover_rmf_maps)/share/rover_rmf_maps/maps"
CONFIG_SHARE="$(ros2 pkg prefix rover_rmf_bringup)/share/rover_rmf_bringup/config"
STATUS="$SITES_DIR/rmf_status.json"
mkdir -p "$SITES_DIR"

# ros2 launch refuses an empty 'name:=', so overrides are only passed when set.
OVERRIDES=()
[ -n "$RMF_BROKER_HOST" ] && OVERRIDES+=("broker_host:=$RMF_BROKER_HOST")
[ -n "$RMF_BROKER_PORT" ] && OVERRIDES+=("broker_port:=$RMF_BROKER_PORT")

LAUNCH_PID=""
SITE=""
ERROR=""
STARTED=0

requested_site() {
    local name=""
    [ -s "$SITES_DIR/active" ] && name="$(tr -d '[:space:]' < "$SITES_DIR/active")"
    echo "${name:-${RMF_SITE:-rover_world}}"
}

# Sets BUILDING, NAV_GRAPH, CONFIG for site $1; fails when neither place has it.
resolve() {
    local site="$1"
    if [ -f "$SITES_DIR/$site/$site.building.yaml" ]; then
        BUILDING="$SITES_DIR/$site/$site.building.yaml"
        NAV_GRAPH="$SITES_DIR/$site/nav_graphs/0.yaml"
        CONFIG="$SITES_DIR/$site/fleet_$site.yaml"
    else
        BUILDING="$MAPS_SHARE/$site/$site.building.yaml"
        NAV_GRAPH="$MAPS_SHARE/$site/nav_graphs/0.yaml"
        CONFIG="$CONFIG_SHARE/fleet_$site.yaml"
    fi
    [ -f "$BUILDING" ] && [ -f "$NAV_GRAPH" ] && [ -f "$CONFIG" ]
}

# What a restart depends on: the requested name, and the imported version of it (if any).
fingerprint() {
    local site
    site="$(requested_site)"
    echo "$site $(stat -c %Y "$SITES_DIR/$site/$site.building.yaml" 2>/dev/null)"
}

write_status() {  # state
    local tmp="$STATUS.tmp"
    printf '{"site": "%s", "state": "%s", "pid": %s, "started": %s, "error": "%s"}\n' \
        "$SITE" "$1" "${LAUNCH_PID:-0}" "$STARTED" "$ERROR" > "$tmp" && mv "$tmp" "$STATUS"
}

stop_rmf() {
    [ -n "$LAUNCH_PID" ] || return 0
    # SIGINT the whole launch group: ros2 launch orphans its nodes on SIGTERM.
    kill -INT -- "-$LAUNCH_PID" 2> /dev/null
    for _ in $(seq 30); do
        kill -0 -- "-$LAUNCH_PID" 2> /dev/null || break
        sleep 0.5
    done
    kill -KILL -- "-$LAUNCH_PID" 2> /dev/null
    wait "$LAUNCH_PID" 2> /dev/null
    LAUNCH_PID=""
}

shutdown() {
    echo "run_rmf: stopping"
    stop_rmf
    write_status stopped
    exit 0
}
trap shutdown INT TERM

while true; do
    WANTED="$(fingerprint)"
    SITE="$(requested_site)"
    ERROR=""
    if ! resolve "$SITE"; then
        ERROR="site $SITE not found (no building, nav graph or fleet config); running rover_world"
        echo "run_rmf: $ERROR" >&2
        SITE=rover_world
        resolve "$SITE"
    fi
    echo "run_rmf: starting RMF on site $SITE"
    ros2 launch rover_rmf_bringup rover_rmf.launch.xml site:="$SITE" \
        building_file:="$BUILDING" nav_graph_file:="$NAV_GRAPH" config_file:="$CONFIG" \
        initial_map:="$SITE" "${OVERRIDES[@]}" "$@" &
    LAUNCH_PID=$!
    STARTED="$(date +%s)"
    write_status running

    while kill -0 "$LAUNCH_PID" 2> /dev/null; do
        sleep 2
        if [ "$(fingerprint)" != "$WANTED" ]; then
            echo "run_rmf: site changed to $(requested_site); restarting RMF"
            write_status restarting
            stop_rmf
            break
        fi
    done
    if [ -n "$LAUNCH_PID" ]; then
        wait "$LAUNCH_PID" 2> /dev/null
        ERROR="RMF exited ($?); restarting in 5 s"
        echo "run_rmf: $ERROR" >&2
        LAUNCH_PID=""
        write_status restarting
        sleep 5
    fi
done
