#!/usr/bin/env bash
# run_server.sh — Launch full UAV stack in 4 xterm windows inside Apptainer
#
# Usage:  ./run_server.sh [user_id]
#   user_id  — scratch user ID (default: whoami)
# Logs:   /scratch/<id>/irobot/logs/
# Override any path:  PX4_DIR=/custom/path ./run_server.sh

set -euo pipefail

# ── Paths ─────────────────────────────────────────────────────────────────────
USER_ID="${1:-$(whoami)}"
BASE="/scratch/$USER_ID/irobot"
SIF="${SIF:-$BASE/uav_stack.sif}"
PX4_DIR="${PX4_DIR:-$BASE/px4_autopilot}"
DDS_AGENT="${DDS_AGENT:-$BASE/Micro-XRCE-DDS-Agent/build/MicroXRCEAgent}"
BRIDGE_YAML="${BRIDGE_YAML:-$BASE/px4_sim/sensor_bridge.yaml}"
PX4_ROS_WS="${PX4_ROS_WS:-$BASE/px4_ros2_ws}"
PLANNER_WS="${PLANNER_WS:-$BASE/planner_ws}"
LOG_DIR="${LOG_DIR:-$BASE/logs}"

# ── Validate ──────────────────────────────────────────────────────────────────
missing=0
[ ! -f "$SIF" ]         && echo "MISSING: $SIF"         && missing=1
[ ! -f "$DDS_AGENT" ]   && echo "MISSING: $DDS_AGENT"   && missing=1
[ ! -f "$BRIDGE_YAML" ] && echo "MISSING: $BRIDGE_YAML" && missing=1
[ ! -d "$PX4_DIR" ]     && echo "MISSING: $PX4_DIR"     && missing=1
[ ! -d "$PX4_ROS_WS" ]  && echo "MISSING: $PX4_ROS_WS"  && missing=1
[ ! -d "$PLANNER_WS" ]  && echo "MISSING: $PLANNER_WS"  && missing=1
[ $missing -eq 1 ] && exit 1

mkdir -p "$LOG_DIR"

echo "========================================"
echo " UAV Stack — NUS Server"
echo " SIF      : $SIF"
echo " PX4      : $PX4_DIR"
echo " Logs     : $LOG_DIR"
echo "========================================"

# ── Shared strings ────────────────────────────────────────────────────────────
# Sourced in every terminal — base ROS2 + px4_ros_ws + planner_ws
SRC="source /opt/ros/humble/setup.bash \
  && source $PX4_ROS_WS/install/setup.bash \
  && source $PLANNER_WS/install/setup.bash"

APT="singularity exec --nv $SIF"

# ── T1: PX4 SITL + Gazebo ─────────────────────────────────────────────────────
echo "[T1] Launching PX4 + Gazebo..."
xterm -title "T1: PX4 + Gazebo" -e bash -c \
  "$APT bash -c '$SRC && cd $PX4_DIR && PX4_GZ_WORLD=indoor_obstacle make px4_sitl gz_f450 2>&1 | tee $LOG_DIR/px4_gazebo.log'; exec bash" &

echo "     Waiting 30 s for PX4 + Gazebo to initialise..."
sleep 30

# ── T2: MicroXRCE-DDS Agent ───────────────────────────────────────────────────
echo "[T2] Launching MicroXRCE-DDS Agent..."
xterm -title "T2: DDS Agent" -e bash -c \
  "$APT bash -c '$SRC && $DDS_AGENT udp4 -p 8888 2>&1 | tee $LOG_DIR/dds_agent.log'; exec bash" &

echo "     Waiting 20 s for DDS agent to connect..."
sleep 20

# ── T3: ROS-GZ Sensor Bridge ─────────────────────────────────────────────────
echo "[T3] Launching Sensor Bridge..."
xterm -title "T3: Sensor Bridge" -e bash -c \
  "$APT bash -c '$SRC && ros2 run ros_gz_bridge parameter_bridge \
    --ros-args -p config_file:=$BRIDGE_YAML \
    2>&1 | tee $LOG_DIR/sensor_bridge.log'; exec bash" &

echo "     Waiting 5 s for bridge to come up..."
sleep 5

# ── T4: UAV Bringup (planner stack) ──────────────────────────────────────────
echo "[T4] Launching UAV Bringup..."
xterm -title "T4: UAV Bringup" -e bash -c \
  "$APT bash -c '$SRC && ros2 launch uav_bringup full_stack.launch.py \
    with_global_planner:=false \
    2>&1 | tee $LOG_DIR/planner.log'; exec bash" &

# ── T5: Interactive shell (waypoint / bag commands) ──────────────────────────
xterm -title "T5: UAV Shell" -e bash -c \
  "$APT bash -c '$SRC && exec bash'" &

echo ""
echo "========================================"
echo " All 5 terminals launched. Logs → $LOG_DIR/"
echo ""
echo " When ready — send a waypoint:"
echo "   $APT bash -c \"$SRC && \\"
echo "     ros2 action send_goal /uav/navigate_to_goal \\"
echo "     uav_planner_interface/action/NavigateToGoal \\"
echo "     '{target_pose: {header: {frame_id: map}, pose: {position: {x: 5.0, y: 0.0, z: 1.5}}}}'\" "
echo ""
echo " Record a bag:"
echo "   $APT bash -c \"$SRC && \\"
echo "     ros2 bag record -o $LOG_DIR/bag_\$(date +%Y%m%d_%H%M%S) \\"
echo "     /drone/odom /drone/tof_merged/points /uav/cmd_vel \\"
echo "     /uav/current_waypoint /uav/global_path /uav/mission_complete \\"
echo "     /uav/mp_diag /uav/vfh_status\""
echo "========================================"
