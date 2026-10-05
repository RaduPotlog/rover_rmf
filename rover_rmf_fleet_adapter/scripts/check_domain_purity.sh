#!/usr/bin/env bash
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
#
# Fails if domain/ or application/ imports ROS, RMF, MQTT, a *_msgs package, or an outer layer
# of this package (.claude/rules/clean_architecture.md). Keeps the unit tests runnable on a host
# without Open-RMF.
set -euo pipefail

if [[ $# -ne 1 ]]; then
    echo "usage: $0 <package_source_dir>" >&2
    exit 2
fi

PKG="$1/rover_rmf_fleet_adapter"
PATTERN='^\s*(import|from)\s+(rclpy|rmf_adapter|paho|launch|[a-z0-9_]*_msgs|\.\.?(infrastructure|presentation))\b'
PATTERN_ABS='^\s*(import|from)\s+rover_rmf_fleet_adapter\.(infrastructure|presentation)\b'

status=0
for layer in domain application; do
    if grep -rnE "$PATTERN|$PATTERN_ABS" "$PKG/$layer" --include='*.py'; then
        echo "ERROR: $layer/ must not import ROS/RMF/MQTT or outer layers (see lines above)" >&2
        status=1
    fi
done
# application may only reach into domain
if grep -rnE '^\s*from\s+\.\.(infrastructure|presentation)' "$PKG/application" --include='*.py'; then
    status=1
fi
exit $status
