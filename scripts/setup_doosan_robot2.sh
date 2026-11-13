#!/bin/bash
# Clone the official Doosan ROS 2 driver (humble) at the tested commit, apply the local safety patch, build.
# Prerequisites: ROS 2 Humble, Docker (virtual mode), and
#   sudo apt-get install -y libpoco-dev ros-humble-ros2-controllers dbus-x11
set -e
WS="$(cd "$(dirname "$0")/.." && pwd)"
COMMIT=8653b04   # DoosanRobotics/doosan-robot2 humble, tested 2026-11-13
cd "$WS/src"
if [ ! -d doosan-robot2 ]; then
  git clone -b humble https://github.com/DoosanRobotics/doosan-robot2.git
  git -C doosan-robot2 checkout -q "$COMMIT"
fi
if git -C doosan-robot2 apply --check "$WS/patches/dsr_controller2_safe_activation.patch" 2>/dev/null; then
  git -C doosan-robot2 apply "$WS/patches/dsr_controller2_safe_activation.patch"
  echo "patch applied: dsr_moveit_controller starts from the measured joint state"
else
  echo "patch already applied (or does not apply) - check dsr_controller2/config/dsr_controller2.yaml"
fi
docker pull doosanrobot/dsr_emulator:3.0.1
cd "$WS"
source /opt/ros/humble/setup.bash
colcon build --packages-up-to dsr_msgs2 dsr_common2 dsr_description2 dsr_hardware2 dsr_controller2 dsr_bringup2 \
  dsr_moveit_config_m0609 m0609_mirror --cmake-args -DDRCF_VER=2 -DCMAKE_BUILD_TYPE=Release
"$WS/tools/build.sh"
echo "done: source $WS/install/setup.bash"
