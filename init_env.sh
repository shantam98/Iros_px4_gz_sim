# init_env.sh — one-time environment setup for an ablation shell.
#
# Usage:
#   source ~/irobot/px4_sim/init_env.sh
#
# Source once per shell; ablation wrapper scripts assume these vars are set.

# ── Paths ───────────────────────────────────────────────────────────────────
export PX4_DIR="${PX4_DIR:-$HOME/irobot/px4_autopilot/PX4-Autopilot}"
export DDS_AGENT="${DDS_AGENT:-$HOME/irobot/px4_autopilot/Micro-XRCE-DDS-Agent/build/MicroXRCEAgent}"
export BRIDGE_YAML="${BRIDGE_YAML:-$HOME/irobot/px4_sim/sensor_bridge.yaml}"
export PLANNER_WS="${PLANNER_WS:-$HOME/irobot/planner_ws}"
export PX4_ROS2_WS="${PX4_ROS2_WS:-$HOME/irobot/px4_ros2_ws}"
export ABLATION_BAGS="${ABLATION_BAGS:-$PLANNER_WS/bags}"

# ── ROS / workspaces ────────────────────────────────────────────────────────
source /opt/ros/humble/setup.bash
[ -f "$PX4_ROS2_WS/install/setup.bash" ] && source "$PX4_ROS2_WS/install/setup.bash"
[ -f "$PLANNER_WS/install/setup.bash" ]  && source "$PLANNER_WS/install/setup.bash"

# ── DDS domain (match cuVSLAM/nvblox if/when enabled) ───────────────────────
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

echo "[init_env] sourced — PX4=$PX4_DIR  PLANNER_WS=$PLANNER_WS  DOMAIN=$ROS_DOMAIN_ID"
echo "[init_env] next: run PX4 SITL manually in another terminal:"
echo "  cd $PX4_DIR && PX4_GZ_WORLD=<world> make px4_sitl gz_f450"
echo "  (world ∈ scenario_1_pole | scenario_2_slalom | scenario_3_corner | scenario_4_gap)"
echo "[init_env] then: ~/irobot/px4_sim/ablation/run_one.sh <scenario> <sensor>"
