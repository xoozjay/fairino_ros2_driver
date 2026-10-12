#!/usr/bin/env bash
# Check of this repository (see AGENTS.md): build every package, then run its tests.
# Run from anywhere; the workspace root is derived from this script's location.
set -euo pipefail

repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ws="$(cd "$repo/../.." && pwd)"
if [ ! -d "$ws/src" ]; then
  echo "check.sh: $ws is not a colcon workspace (no src/)" >&2
  exit 2
fi

# The ROS environment is set up by the devcontainer, but only in interactive
# shells; colcon itself is on PATH anyway, so check AMENT_PREFIX_PATH.
if [ -z "${AMENT_PREFIX_PATH:-}" ]; then
  set +u
  # shellcheck disable=SC1090
  . "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
  set -u
fi

packages=(fr_client_library fr_description fr_robot_driver gripper_dahuan_pgi140)

cd "$ws"
colcon build --packages-up-to "${packages[@]}"

# The tests look up installed package shares (pluginlib, xacro), so the overlay
# has to be in the environment even when this script runs non-interactively.
if [ -f "$ws/install/setup.bash" ]; then
  set +u
  # shellcheck disable=SC1091
  . "$ws/install/setup.bash"
  set -u
fi

status=0
colcon test --packages-up-to "${packages[@]}" --return-code-on-test-failure || status=$?
colcon test-result --verbose || status=$?
exit "$status"
