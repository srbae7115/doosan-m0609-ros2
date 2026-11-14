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
# dsr_controller2.cpp alone needs ~6 GB RAM to compile (measured 2026-11-14, -O3 and -O0 alike).
# Without enough RAM+swap the compiler is killed: "c++: fatal error: Killed signal terminated program cc1plus".
if [ ! -f install/dsr_controller2/share/dsr_controller2/package.xml ] || [ src/doosan-robot2/dsr_controller2/src/dsr_controller2.cpp -nt install/dsr_controller2/share/dsr_controller2/package.xml ]; then
  AVAIL_MB=$(awk '/MemAvailable|SwapFree/ {s+=$2} END {print int(s/1024)}' /proc/meminfo)
  if [ "$AVAIL_MB" -lt 7000 ] && [ -z "$FORCE" ]; then
    echo "메모리 부족: 사용 가능 RAM+스왑 ${AVAIL_MB} MB < 7000 MB (dsr_controller2 컴파일에 약 6 GB 필요)"
    echo "  [WSL2] Windows의 %UserProfile%\.wslconfig 에 아래를 넣고 PowerShell에서 wsl --shutdown 후 다시 실행:"
    echo "         [wsl2]"
    echo "         memory=8GB"
    echo "         swap=8GB"
    echo "  [순수 Ubuntu] 스왑 파일 8 GB 추가 후 다시 실행:"
    echo "         sudo fallocate -l 8G /swapfile2 && sudo chmod 600 /swapfile2 && sudo mkswap /swapfile2 && sudo swapon /swapfile2"
    echo "  (그래도 시도하려면 FORCE=1 $0)"
    exit 1
  fi
fi
# one package at a time: other packages compiling next to dsr_controller2 add to the peak memory
colcon build --parallel-workers 1 --packages-up-to dsr_msgs2 dsr_common2 dsr_description2 dsr_hardware2 dsr_controller2 dsr_bringup2 \
  dsr_moveit_config_m0609 m0609_mirror --cmake-args -DDRCF_VER=2 -DCMAKE_BUILD_TYPE=Release
"$WS/tools/build.sh"
echo "done: source $WS/install/setup.bash"
