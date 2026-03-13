#!/usr/bin/env bash
set -euo pipefail

# path: tools/run_backend_terminals.sh
# modify this script to run the desired ROS2 nodes in separate terminals, with the appropriate environment setup
ROS_DISTRO="${ROS_DISTRO:-rolling}"
WS="$(realpath "${1:-$HOME/Scrivania/backendProvas/backend_ros2}")"
VRPN_SERVER="${2:-192.168.1.100}"
VRPN_PORT="${3:-3883}"
MOCAP_YAML="$(realpath "${4:-$WS/src/mocap_bridge_ros2/config/mocap.yaml}")"
CF_YAML="$(realpath "${5:-$WS/src/cf_bridge/config/cf_bridge.yaml}")"

CMD_ENV="source /opt/ros/$ROS_DISTRO/setup.bash; source $WS/install/setup.bash;"

#gnome-terminal -- bash -lc "$CMD_ENV ros2 run vrpn_mocap client_node --ros-args -p server:=$VRPN_SERVER -p port:=$VRPN_PORT; exec bash"
#gnome-terminal -- bash -lc "$CMD_ENV ros2 launch mocap_bridge_ros2 mocap_bridge.launch.py config_path:=$MOCAP_YAML; exec bash"
gnome-terminal -- bash -lc "$CMD_ENV ros2 launch cf_bridge cf_bridge.launch.py config_path:=$CF_YAML; exec bash"

#to run open a terminal, go into project directory and run: chmod +x script/run_backend_terminals.sh
# and then run: ./script/run_backend_terminals.sh