#!/usr/bin/env bash
# run_server.sh — Launch full UAV stack on NUS server (RHEL + Singularity)
#
# Usage:  ./run_server.sh
# Logs:   /scratch/<id>/irobot/logs/
#
# Override any path with env vars before running, e.g.:
#   PX4_DIR=/custom/path ./run_server.sh

set -euo pipefail

# ── Paths (auto-detect user ID via whoami) ────────────────────────────────────
BASE="/scratch/$(whoami)/irobot"
SIF="${SIF:-$BASE/uav_stack.sif}"
PX4_DIR="${PX4_DIR:-$BASE/px4_autopilot}"
DDS_AGENT="${DDS_AGENT:-$BASE/Micro-XRCE-DDS-Agent/build/MicroXRCEAgent}"
BRIDGE_YAML="${BRIDGE_YAML:-$BASE/px4_sim/sensor_bridge.yaml}"
PX4_ROS2_WS="${PX4_ROS2_WS:-$BASE/px4_ros2_ws}"
PLANNER_WS="${PLANNER_WS:-$BASE/planner_ws}"
LOG_DIR="${LOG_DIR:-$BASE/logs}"

# ── Validate ──────────────────────────────────────────────────────────────────
missing=0
[ ! -f "$SIF" ]         && echo "MISSING: SIF=$SIF"               && missing=1
[ ! -d "$PX4_DIR" ]     && echo "MISSING: PX4_DIR=$PX4_DIR"       && missing=1
[ ! -f "$DDS_AGENT" ]   && echo "MISSING: DDS_AGENT=$DDS_AGENT"   && missing=1
[ ! -f "$BRIDGE_YAML" ] && echo "MISSING: BRIDGE_YAML=$BRIDGE_YAML" && missing=1
[ $missing -eq 1 ] && exit 1

mkdir -p "$LOG_DIR"

echo "========================================"
echo " UAV Stack — NUS Server"
echo " SIF      : $SIF"
echo " PX4      : $PX4_DIR"
echo " Logs     : $LOG_DIR"
echo "========================================"

# ── Shared setup strings ──────────────────────────────────────────────────────
ROS="source /opt/ros/humble/setup.bash"
WS="source $PX4_ROS2_WS/install/setup.bash && source $PLANNER_WS/install/setup.bash"
SING="singularity exec --nvccli $SIF"

# ── Launch terminals (xterm) ──────────────────────────────────────────────────
xterm -title "T1: PX4 + Gazebo" -e bash -c \
    "$SING bash -c 'cd $PX4_DIR && PX4_GZ_WORLD=indoor_obstacle make px4_sitl gz_f450' \
     2>&1 | tee $LOG_DIR/px4_gazebo.log; exec bash" &

sleep 10
xterm -title "T2: DDS Agent" -e bash -c \
    "$SING $DDS_AGENT udp4 -p 8888 2>&1 | tee $LOG_DIR/dds_agent.log; exec bash" &

sleep 5
xterm -title "T3: Sensor Bridge" -e bash -c \
    "$SING bash -c '$ROS && source $PX4_ROS2_WS/install/setup.bash && \
      ros2 run ros_gz_bridge parameter_bridge \
        --ros-args --params-file $BRIDGE_YAML' \
     2>&1 | tee $LOG_DIR/sensor_bridge.log; exec bash" &

sleep 5
xterm -title "T4: Planner Stack" -e bash -c \
    "$SING bash -c '$ROS && $WS && \
      ros2 launch uav_bringup full_stack.launch.py with_global_planner:=false' \
     2>&1 | tee $LOG_DIR/planner.log; exec bash" &

echo ""
echo "All terminals launched. Logs → $LOG_DIR/"
echo ""
echo "Send a waypoint with:"
echo "  $SING bash -c \"$ROS && $WS && \\"
echo "    ros2 topic pub --once /uav/current_waypoint geometry_msgs/msg/PointStamped \\"
echo "    '{header: {frame_id: map}, point: {x: 5.0, y: 0.0, z: 1.5}}'\""
