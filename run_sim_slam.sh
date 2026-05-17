#!/usr/bin/env bash
# run_sim_slam.sh — Local SITL + cuVSLAM + nvblox.
#
# Sibling of run_sim.sh. Same 4 native tabs, plus a 5th tab that brings up
# cuVSLAM + nvblox inside isaac_vslam.sif. Planner stack runs natively;
# only the Isaac ROS bits are containerised.
#
# Stack order (with sleeps to enforce):
#   T1 (+0 s)   PX4 SITL + Gazebo
#   T2 (+10 s)  Micro XRCE-DDS bridge
#   T3 (+20 s)  Gazebo ↔ ROS2 sensor bridge
#   T4 (+25 s)  Planner stack (with_vslam:=true)
#   T5 (+60 s)  cuVSLAM + nvblox (apptainer exec on isaac_vslam.sif)
#
# Prereq: planner_ws built, PX4 SITL built, isaac_vslam.sif present locally,
# Apptainer + NVIDIA Container Toolkit installed.

set -euo pipefail

PX4_DIR="${PX4_DIR:-$HOME/irobot/px4_autopilot/PX4-Autopilot}"
DDS_AGENT="${DDS_AGENT:-$HOME/irobot/px4_autopilot/Micro-XRCE-DDS-Agent/build/MicroXRCEAgent}"
BRIDGE_YAML="${BRIDGE_YAML:-$HOME/irobot/px4_sim/sensor_bridge.yaml}"
PLANNER_WS="${PLANNER_WS:-$HOME/irobot/planner_ws}"
PX4_ROS2_WS="${PX4_ROS2_WS:-$HOME/irobot/px4_ros2_ws}"
ISAAC_SIF="${ISAAC_SIF:-$HOME/irobot/isaac_vslam.sif}"

ROS2_SETUP="/opt/ros/humble/setup.bash"

# Share DDS domain between native planner and Apptainer cuVSLAM
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

# ── Performance tuning ─────────────────────────────────────────────────────
# Cap BLAS / OpenMP / MKL thread counts. Many Gazebo plugins, PCL and
# OpenCV link against these and default to one thread per core, which
# starves the renderer and PX4 SITL of CPU under load. 4 threads is a
# good balance on an 8-core laptop.
export OPENBLAS_NUM_THREADS="${OPENBLAS_NUM_THREADS:-4}"
export OMP_NUM_THREADS="${OMP_NUM_THREADS:-4}"
export MKL_NUM_THREADS="${MKL_NUM_THREADS:-4}"

# Force the discrete NVIDIA GPU for OpenGL apps on NVIDIA Optimus laptops.
# Without these, Gazebo can silently fall back to the integrated GPU and
# stall hard. Has no effect on desktops or non-Optimus laptops.
export __NV_PRIME_RENDER_OFFLOAD="${__NV_PRIME_RENDER_OFFLOAD:-1}"
export __GLX_VENDOR_LIBRARY_NAME="${__GLX_VENDOR_LIBRARY_NAME:-nvidia}"
export __VK_LAYER_NV_optimus="${__VK_LAYER_NV_optimus:-NVIDIA_only}"

# ── Validate ──────────────────────────────────────────────────────────────────
missing=0
[ ! -d "$PX4_DIR" ]        && echo "MISSING: PX4_DIR=$PX4_DIR"           && missing=1
[ ! -f "$DDS_AGENT" ]      && echo "MISSING: DDS_AGENT=$DDS_AGENT"       && missing=1
[ ! -f "$BRIDGE_YAML" ]    && echo "MISSING: BRIDGE_YAML=$BRIDGE_YAML"   && missing=1
[ ! -d "$PLANNER_WS" ]     && echo "MISSING: PLANNER_WS=$PLANNER_WS"     && missing=1
[ ! -d "$PX4_ROS2_WS" ]    && echo "MISSING: PX4_ROS2_WS=$PX4_ROS2_WS"   && missing=1
[ ! -f "$ISAAC_SIF" ]      && echo "MISSING: ISAAC_SIF=$ISAAC_SIF"       && missing=1
[ $missing -eq 1 ] && echo "Fix paths above (or export env vars) and re-run." && exit 1

if ! command -v apptainer &>/dev/null; then
  echo "MISSING: apptainer not installed. See setup notes." && exit 1
fi

echo "================================================================"
echo " UAV Stack — Local SITL + cuVSLAM + nvblox"
echo "  PX4         : $PX4_DIR"
echo "  DDS Agent   : $DDS_AGENT"
echo "  Bridge yaml : $BRIDGE_YAML"
echo "  Planner WS  : $PLANNER_WS"
echo "  Isaac SIF   : $ISAAC_SIF"
echo "  DOMAIN_ID   : $ROS_DOMAIN_ID"
echo "================================================================"

# ── Open gnome-terminal tabs ──────────────────────────────────────────────────
# T1 (PX4 SITL + Gazebo) is run MANUALLY in a separate terminal:
#   cd $PX4_DIR && PX4_GZ_WORLD=indoor_obstacle make px4_sitl gz_f450
# Reason: gnome-terminal --command was launching make in an env that
# prevented Gazebo's GUI from popping; running manually is reliable.
# T2-T5 still launch automatically below.
gnome-terminal \
  --tab --title="T2: DDS Agent" \
  --command="bash -c 'sleep 10 && $DDS_AGENT udp4 -p 8888; exec bash'" \
  \
  --tab --title="T3: Sensor Bridge" \
  --command="bash -c 'sleep 20 && source $ROS2_SETUP && source $PX4_ROS2_WS/install/setup.bash && ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=$BRIDGE_YAML; exec bash'" \
  \
  --tab --title="T4: Planner (with_vslam, mp baseline)" \
  --command="bash -c 'sleep 25 && source $ROS2_SETUP && source $PX4_ROS2_WS/install/setup.bash && source $PLANNER_WS/install/setup.bash && ros2 launch uav_bringup full_stack.launch.py with_global_planner:=false with_vslam:=true planner_backend:=mp; exec bash'" \
  \
  --tab --title="T5: cuVSLAM + nvblox" \
  --command="bash -c 'sleep 60 && ROS_DOMAIN_ID=$ROS_DOMAIN_ID apptainer exec --nv $ISAAC_SIF bash -c \"source /opt/ros/humble/setup.bash && source $PLANNER_WS/install/setup.bash && ros2 launch uav_bringup vslam.launch.py\"; exec bash'"

echo ""
echo "4 terminals launched (T2–T5)."
echo " - T2 (+10s), T3 (+20s), T4 (+25s), T5 (+60s) — staggered for dependencies"
echo ""
echo "⚠  T1 (PX4 SITL) is NOT auto-launched. Start it manually BEFORE running this script:"
echo "     cd $PX4_DIR && PX4_GZ_WORLD=indoor_obstacle make px4_sitl gz_f450"
echo ""
echo "Watch T5 for cuVSLAM tracking. Sanity checks after ~90s:"
echo "  ros2 topic hz /visual_slam/tracking/vo_pose"
echo "  ros2 run tf2_ros tf2_echo map base_link"
echo "  ros2 topic hz /nvblox_node/static_esdf_pointcloud"
echo ""
echo "Send a waypoint once airborne:"
echo "  ros2 topic pub --once /uav/current_waypoint geometry_msgs/msg/PointStamped \\"
echo "    '{header: {frame_id: map}, point: {x: 5.0, y: 0.0, z: 1.5}}'"
