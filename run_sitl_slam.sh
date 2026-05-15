#!/usr/bin/env bash
# run_sitl_slam.sh — SITL launcher with cuVSLAM/nvblox variants.
#
# Wraps run_server.sh (planner stack inside uav_stack.sif) and adds a second
# Singularity invocation against isaac_vslam.sif for cuVSLAM + nvblox. Both
# containers share ROS_DOMAIN_ID so DDS connects them transparently.
#
# Pick exactly ONE variant block below by un-commenting it. Two variants are
# commented out by default.
#
# Usage:  ./run_sitl_slam.sh [user_id]
#         user_id — scratch user ID (default: whoami)

set -euo pipefail

# ── Paths ─────────────────────────────────────────────────────────────────────
USER_ID="${1:-$(whoami)}"
BASE="/scratch/$USER_ID/irobot"
RUN_SERVER="${RUN_SERVER:-$BASE/px4_sim/run_server.sh}"
ISAAC_SIF="${ISAAC_SIF:-$BASE/isaac_vslam.sif}"
PLANNER_WS="${PLANNER_WS:-$BASE/planner_ws}"
LOG_DIR="${LOG_DIR:-$BASE/logs}"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

# ── Validate ──────────────────────────────────────────────────────────────────
missing=0
[ ! -x "$RUN_SERVER" ] && echo "MISSING (or not executable): $RUN_SERVER" && missing=1
[ ! -f "$ISAAC_SIF" ]  && echo "MISSING: $ISAAC_SIF (only needed for Variants 1 & 2)" && missing=1
[ ! -d "$PLANNER_WS" ] && echo "MISSING: $PLANNER_WS" && missing=1
[ $missing -eq 1 ] && exit 1
mkdir -p "$LOG_DIR"

VSLAM_CONFIG_DIR="$PLANNER_WS/install/uav_bringup/share/uav_bringup/config"

# Helper: launch cuVSLAM (+/- nvblox) inside isaac_vslam.sif.
# Args: $1 = path to vslam config yaml
launch_vslam_container() {
    local cfg="$1"
    xterm -title "T6: cuVSLAM + nvblox" -e bash -c \
      "singularity exec --nv $ISAAC_SIF bash -c \
         'source /opt/ros/humble/setup.bash && \
          source $PLANNER_WS/install/setup.bash && \
          ROS_DOMAIN_ID=$ROS_DOMAIN_ID ros2 launch uav_bringup vslam.launch.py \
            vslam_config:=$cfg \
            2>&1 | tee $LOG_DIR/vslam.log'; exec bash" &
}

echo "================================================================"
echo " SITL + Visual SLAM launcher"
echo " ROS_DOMAIN_ID = $ROS_DOMAIN_ID"
echo " RUN_SERVER    = $RUN_SERVER"
echo " ISAAC_SIF     = $ISAAC_SIF"
echo " VSLAM configs = $VSLAM_CONFIG_DIR"
echo "================================================================"

# ════════════════════════════════════════════════════════════════════════════
# Variant 1 (ACTIVE) — cuVSLAM standalone, no nvblox.
# Use this to validate tracking (vo_state, drift, TF) before adding mapping.
# full_stack runs with with_vslam:=true → static map→odom gated, OctoMap off.
# ════════════════════════════════════════════════════════════════════════════
WITH_VSLAM=true "$RUN_SERVER" "$USER_ID" &
echo "[Variant 1] Waiting 75 s for PX4/Gazebo/DDS/planner stack to settle..."
sleep 75
launch_vslam_container "$VSLAM_CONFIG_DIR/vslam_cuvslam_only.yaml"

# ════════════════════════════════════════════════════════════════════════════
# Variant 2 (commented) — cuVSLAM + nvblox (full Phase 2 path).
# nvblox builds the ESDF; downstream planner consumes it instead of OctoMap.
# ════════════════════════════════════════════════════════════════════════════
# WITH_VSLAM=true "$RUN_SERVER" "$USER_ID" &
# echo "[Variant 2] Waiting 75 s for PX4/Gazebo/DDS/planner stack to settle..."
# sleep 75
# launch_vslam_container "$VSLAM_CONFIG_DIR/vslam.yaml"

# ════════════════════════════════════════════════════════════════════════════
# Variant 3 (commented) — stock stack, no cuVSLAM, no nvblox.
# Identical to running run_server.sh directly: OctoMap mapping, EKF2 odom only.
# ════════════════════════════════════════════════════════════════════════════
# exec "$RUN_SERVER" "$USER_ID"

echo ""
echo "================================================================"
echo " Active variant launched. Logs → $LOG_DIR/"
echo ""
echo " Sanity checks (run in T5 shell):"
echo "   ros2 topic echo visual_slam/status --once    # vo_state should = 1"
echo "   ros2 run tf2_tools view_frames               # map→odom from cuVSLAM"
echo "   ros2 topic hz nvblox_node/static_esdf_pointcloud   # Variant 2 only"
echo "================================================================"
