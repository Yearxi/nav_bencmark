#!/bin/bash
# 云服务器环境检查脚本：BARN + DDR-opt
# 用法: bash check_server_env.sh

set -e
echo "========== 系统 =========="
lsb_release -a 2>/dev/null || true
echo ""
echo "========== ROS =========="
for d in noetic melodic; do
  if [ -f /opt/ros/$d/setup.bash ]; then
    source /opt/ros/$d/setup.bash
    echo "ROS_DISTRO: $ROS_DISTRO"
    rosversion -d
    break
  fi
done
echo ""
echo "========== Gazebo =========="
gzserver --version 2>/dev/null || echo "gzserver not found (install ros-${ROS_DISTRO}-gazebo-ros-pkgs)"
echo ""
echo "========== Python =========="
python3 --version
python3 -c "import rospkg; import numpy; print('rospkg, numpy OK')" 2>/dev/null || echo "pip3 install rospkg numpy defusedxml netifaces"
echo ""
echo "========== OSQP (DDR-opt) =========="
pkg-config --exists osqp 2>/dev/null && echo "OSQP found" || echo "OSQP not found (install from source)"
echo ""
echo "========== Workspace =========="
if [ -n "$ROS_DISTRO" ] && [ -d ~/jackal_ws/devel ]; then
  source ~/jackal_ws/devel/setup.bash
  rospack find barn_ddr_bridge 2>/dev/null && echo "barn_ddr_bridge OK" || true
  rospack find plan_manager 2>/dev/null && echo "plan_manager OK" || true
else
  echo "Set up workspace and run: source devel/setup.bash"
fi
echo ""
echo "Done."
