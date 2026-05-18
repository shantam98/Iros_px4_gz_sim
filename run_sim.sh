#!/usr/bin/env bash
# run_sim.sh — Start the full PX4 SITL + ROS2 stack.
# Run setup.sh ONCE before using this script on a new machine.
#
# Opens 4 terminal tabs (gnome-terminal).
# Each tab runs one layer of the stack in the correct order.
#
# Stack order:
#   T1: PX4 SITL + Gazebo (blocks — keep open)
#   T2: Micro XRCE-DDS bridge (ROS2 ↔ PX4)
#   T3: Gazebo ↔ ROS2 sensor bridge
#   T4: UAV planner stack (depth fusion + local planner + control)

set -euo pipefail

PX4_DIR="${PX4_DIR:-$HOME/irobot/px4_autopilot/PX4-Autopilot}"
DDS_AGENT="${DDS_AGENT:-$HOME/irobot/px4_autopilot/Micro-XRCE-DDS-Agent/build/MicroXRCEAgent}"
BRIDGE_YAML="${BRIDGE_YAML:-$HOME/irobot/px4_sim/sensor_bridge.yaml}"
PLANNER_WS="${PLANNER_WS:-$HOME/irobot/planner_ws}"
PX4_ROS2_WS="${PX4_ROS2_WS:-$HOME/irobot/px4_ros2_ws}"

# Cloud source for mp_node: 'd415' (forward RGBD) or 'fusion' (5 ring ToFs merged).
# Toggle per ablation run.
SENSOR_SOURCE="${SENSOR_SOURCE:-fusion}"

# PX4 world to load. Override per scenario for the ablation.
PX4_GZ_WORLD="${PX4_GZ_WORLD:-indoor_obstacle}"

ROS2_SETUP="/opt/ros/humble/setup.bash"

# ── Validate ──────────────────────────────────────────────────────────────────
missing=0
[ ! -d "$PX4_DIR" ]        && echo "MISSING: PX4_DIR=$PX4_DIR"        && missing=1
[ ! -f "$DDS_AGENT" ]      && echo "MISSING: DDS_AGENT=$DDS_AGENT"    && missing=1
[ ! -f "$BRIDGE_YAML" ]    && echo "MISSING: BRIDGE_YAML=$BRIDGE_YAML" && missing=1
[ ! -d "$PLANNER_WS" ]     && echo "MISSING: PLANNER_WS=$PLANNER_WS"  && missing=1
[ $missing -eq 1 ] && echo "Fix paths above (or export env vars) and re-run." && exit 1

echo "Starting full UAV stack..."
echo "  PX4         : $PX4_DIR"
echo "  DDS         : $DDS_AGENT"
echo "  Bridge      : $BRIDGE_YAML"
echo "  Planner     : $PLANNER_WS"
echo "  World       : $PX4_GZ_WORLD"
echo "  SensorSource: $SENSOR_SOURCE"
echo ""

# ── Open 4 gnome-terminal tabs ────────────────────────────────────────────────
gnome-terminal \
  --tab --title="T1: PX4 SITL ($PX4_GZ_WORLD)" \
  --command="bash -c 'cd $PX4_DIR && HEADLESS=${HEADLESS:-0} PX4_GZ_WORLD=$PX4_GZ_WORLD make px4_sitl gz_f450; exec bash'" \
  \
  --tab --title="T2: DDS Agent" \
  --command="bash -c 'sleep 10 && $DDS_AGENT udp4 -p 8888; exec bash'" \
  \
  --tab --title="T3: Sensor Bridge" \
  --command="bash -c 'sleep 20 && source $ROS2_SETUP && source $PX4_ROS2_WS/install/setup.bash && ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=$BRIDGE_YAML; exec bash'" \
  \
  --tab --title="T4: Planner ($SENSOR_SOURCE)" \
  --command="bash -c 'sleep 25 && source $ROS2_SETUP && source $PX4_ROS2_WS/install/setup.bash && source $PLANNER_WS/install/setup.bash && ros2 launch uav_bringup full_stack.launch.py with_global_planner:=false sensor_source:=$SENSOR_SOURCE; exec bash'"

echo "All terminals launched."
echo ""
echo "Once airborne, send a waypoint with:"
echo "  ros2 topic pub --once /uav/current_waypoint geometry_msgs/msg/PointStamped \\"
echo "    '{header: {frame_id: map}, point: {x: 5.0, y: 0.0, z: 1.5}}'"
