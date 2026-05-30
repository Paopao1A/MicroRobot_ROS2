#!/usr/bin/env bash
set -eo pipefail

MAP_NAME="${1:-micro_robot_map}"
MAP_DIR="${2:-$HOME/MicroRos/maps}"

mkdir -p "$MAP_DIR"

set +u
source /opt/ros/humble/setup.bash
if [ -f "$HOME/MicroRos/install/setup.bash" ]; then
  source "$HOME/MicroRos/install/setup.bash"
fi
set -u

ros2 run nav2_map_server map_saver_cli -f "$MAP_DIR/$MAP_NAME"
echo "Saved map to $MAP_DIR/$MAP_NAME.yaml and $MAP_DIR/$MAP_NAME.pgm"
