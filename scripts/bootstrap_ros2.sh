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

# Pi RAM cannot compile slam_toolbox + Nav2 in parallel. Use apt slam_toolbox
# and compile remaining C++ packages one at a time.
swap_kb="$(awk '/SwapTotal:/ { print $2 }' /proc/meminfo)"
swap_kb="${swap_kb:-0}"
if [[ "${swap_kb}" -lt 2000000 ]]; then
  printf 'Low swap (%s kB). Creating a temporary 4 GiB swapfile...\n' "${swap_kb}"
  if [[ ! -f /swapfile ]]; then
    sudo fallocate -l 4G /swapfile || sudo dd if=/dev/zero of=/swapfile bs=1M count=4096 status=progress
    sudo chmod 600 /swapfile
    sudo mkswap /swapfile
  fi
  sudo swapon /swapfile || true
fi

export MAKEFLAGS="${MAKEFLAGS:--j1}"
colcon build \
  --symlink-install \
  --executor sequential \
  --parallel-workers 1 \
  --cmake-args -DBUILD_TESTING=OFF \
  --packages-up-to kiwi_bringup \
  --packages-skip \
    slam_toolbox \
    nav2_bringup \
    nav2_system_tests \
    nav2_minimal_tb3_sim \
    nav2_minimal_tb4_sim \
    nav2_minimal_tb4_description

printf '\nBootstrap complete. In each new terminal run:\n'
printf '  source /opt/ros/%s/setup.bash\n' "${ROS_DISTRO}"
printf '  source %s/install/setup.bash\n' "${ROOT_DIR}"
