#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_DISTRO="${ROS_DISTRO:-lyrical}"

if [[ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  printf 'ERROR: ROS 2 %s is not installed under /opt/ros/%s.\n' \
    "${ROS_DISTRO}" "${ROS_DISTRO}" >&2
  exit 1
fi

sudo apt-get update
sudo apt-get install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-serial \
  "ros-${ROS_DISTRO}-robot-state-publisher" \
  "ros-${ROS_DISTRO}-rviz2" \
  "ros-${ROS_DISTRO}-slam-toolbox" \
  "ros-${ROS_DISTRO}-xacro"

if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  sudo rosdep init
fi
rosdep update

bash "${ROOT_DIR}/scripts/apply_vendor_patches.sh"

# Navigation2 is pinned in src/navigation2 because its Lyrical metapackages
# are not currently published in the Ubuntu Resolute ARM64 apt repository.
# shellcheck disable=SC1090
set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u
rosdep install \
  --from-paths "${ROOT_DIR}/src" \
  --ignore-src \
  --rosdistro "${ROS_DISTRO}" \
  --reinstall \
  -r \
  -y

cd "${ROOT_DIR}"
colcon build \
  --symlink-install \
  --packages-up-to kiwi_bringup

printf '\nBootstrap complete. In each new terminal run:\n'
printf '  source /opt/ros/%s/setup.bash\n' "${ROS_DISTRO}"
printf '  source %s/install/setup.bash\n' "${ROOT_DIR}"
