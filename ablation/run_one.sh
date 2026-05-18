#!/usr/bin/env bash
# run_one.sh — drive a single ablation run end-to-end given that PX4 is
# already running manually in another terminal.
#
# What it does:
#   1. Starts T2 (DDS agent), T3 (sensor bridge), T4 (planner stack) in the
#      background, each logging to /tmp/ablation_<tag>.log.
#   2. Waits until /uav/vfh_status reports IDLE (planner has odom + cloud).
#   3. Calls start_run.sh to publish the waypoint, record the bag, poll for
#      goal-reach, and write meta.json.
#   4. Calls analyze_run.py on the bag dir to produce metrics.json.
#   5. Tears down T2/T3/T4 (PX4 left alone — user manages T1).
#
# Pre-conditions:
#   - Sourced init_env.sh in this shell.
#   - PX4 SITL is running with the matching scenario world (manually).
#
# Usage:
#   ~/irobot/px4_sim/ablation/run_one.sh <scenario> <sensor> [seed]
#   ~/irobot/px4_sim/ablation/run_one.sh scenario_1_pole fusion 0

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

scenario="${1:?usage: run_one.sh <scenario> <sensor> [seed]}"
sensor="${2:?usage: run_one.sh <scenario> <sensor> [seed]}"
seed="${3:-0}"

# ── Pre-flight: env must be sourced ────────────────────────────────────────
: "${PX4_DIR:?source ~/irobot/px4_sim/init_env.sh first}"
: "${DDS_AGENT:?source ~/irobot/px4_sim/init_env.sh first}"
: "${BRIDGE_YAML:?source ~/irobot/px4_sim/init_env.sh first}"
: "${PLANNER_WS:?source ~/irobot/px4_sim/init_env.sh first}"

LOG_DIR="${LOG_DIR:-/tmp/ablation_logs}"
mkdir -p "$LOG_DIR"
tag="${scenario}_${sensor}_seed${seed}_$(date +%H%M%S)"

# ── Verify PX4 + Gazebo are up (user started them manually) ─────────────────
# Process-based check — DDS bridge isn't up yet, so topic-based check would
# always fail. PX4 SITL spawns 'px4' and 'gz sim' / 'ruby gz-sim' processes.
if ! pgrep -af 'px4_sitl_default/bin/px4|px4 -i' >/dev/null && ! pgrep -f 'gz sim' >/dev/null; then
    echo "ERROR: no PX4 SITL process found — is PX4 + Gazebo running?"
    echo "       Open another terminal and run:"
    echo "       cd $PX4_DIR && PX4_GZ_WORLD=$scenario make px4_sitl gz_f450"
    exit 1
fi

# ── Cleanup any prior T2/T3/T4 from a previous run ──────────────────────────
echo "[run_one] cleaning prior T2/T3/T4 processes..."
pkill -f MicroXRCEAgent 2>/dev/null || true
pkill -f "ros_gz_bridge" 2>/dev/null || true
pkill -f "uav_bringup full_stack" 2>/dev/null || true
sleep 2

# ── PID tracking + teardown trap ────────────────────────────────────────────
T2_PID=""; T3_PID=""; T4_PID=""

cleanup() {
    echo "[run_one] tearing down T2/T3/T4..."
    [ -n "$T4_PID" ] && kill -INT "$T4_PID" 2>/dev/null || true
    [ -n "$T3_PID" ] && kill -INT "$T3_PID" 2>/dev/null || true
    [ -n "$T2_PID" ] && kill -INT "$T2_PID" 2>/dev/null || true
    sleep 1
    # Belt and suspenders for stragglers
    pkill -f MicroXRCEAgent 2>/dev/null || true
    pkill -f "ros_gz_bridge" 2>/dev/null || true
    pkill -f "uav_bringup full_stack" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

# ── T2 — DDS Agent ──────────────────────────────────────────────────────────
echo "[run_one] T2: DDS Agent..."
"$DDS_AGENT" udp4 -p 8888 > "$LOG_DIR/t2_dds_${tag}.log" 2>&1 &
T2_PID=$!
sleep 5

# ── T3 — Sensor Bridge ──────────────────────────────────────────────────────
echo "[run_one] T3: Sensor Bridge..."
ros2 run ros_gz_bridge parameter_bridge \
    --ros-args -p config_file:="$BRIDGE_YAML" \
    > "$LOG_DIR/t3_bridge_${tag}.log" 2>&1 &
T3_PID=$!
sleep 5

# ── T4 — Planner (sensor_source picked from arg) ────────────────────────────
echo "[run_one] T4: Planner (sensor_source=$sensor)..."
ros2 launch uav_bringup full_stack.launch.py \
    with_global_planner:=false \
    sensor_source:="$sensor" \
    > "$LOG_DIR/t4_planner_${tag}.log" 2>&1 &
T4_PID=$!

# ── Wait for planner ready (vfh_status == IDLE for 3s) ──────────────────────
echo -n "[run_one] waiting for /uav/vfh_status..."
for i in {1..60}; do
    status=$(timeout 2 ros2 topic echo /uav/vfh_status --once 2>/dev/null \
             | awk -F'data: ' '/data:/ {print $2; exit}' || true)
    if [[ "$status" == "IDLE" || "$status" == "NOMINAL" ]]; then
        echo " ready (status=$status, ${i}s)"
        break
    fi
    echo -n "."
    sleep 1
done
if [[ "$status" != "IDLE" && "$status" != "NOMINAL" ]]; then
    echo
    echo "ERROR: planner never reported IDLE/NOMINAL. status='${status:-<none>}'."
    echo "       Check $LOG_DIR/t4_planner_${tag}.log"
    exit 1
fi

# ── Wait for the drone to actually take off (z > 1.0 m in ENU) ──────────────
# /uav/vfh_status reports IDLE as soon as mp_node has odom, which can fire
# while the drone is still on the ground or in TAKEOFF. Without this check
# we'd send the waypoint mid-takeoff and setpoint_publisher would consume it
# before transitioning to AUTONOMOUS.
echo -n "[run_one] waiting for takeoff (z > 1.0 m)..."
for i in {1..60}; do
    z=$(timeout 5 ros2 topic echo /drone/odom --once --field pose.pose.position.z 2>/dev/null \
        | grep -E '^-?[0-9]+\.?[0-9]*([eE]-?[0-9]+)?$' | head -1)
    if [[ -n "${z:-}" ]] && python3 -c "import sys; sys.exit(0 if float('$z') > 1.0 else 1)" 2>/dev/null; then
        echo " airborne (z=${z}m, ${i}s)"
        break
    fi
    echo -n "."
    sleep 1
done
if [[ -z "${z:-}" ]] || ! python3 -c "import sys; sys.exit(0 if float('$z') > 1.0 else 1)" 2>/dev/null; then
    echo
    echo "ERROR: drone never reached z > 1.0 m. last z='${z:-<none>}'."
    echo "       Check $LOG_DIR/t4_planner_${tag}.log for setpoint_publisher state."
    exit 1
fi

# Tiny settle pause so the HOVER → AUTONOMOUS transition can fire on the
# first cmd_vel from mp_node (it needs one fresh cmd within cmd_timeout_s).
sleep 2

# ── Hand off to the existing harness ────────────────────────────────────────
echo "[run_one] running start_run.sh..."
"$SCRIPT_DIR/start_run.sh" "$scenario" "$sensor" "$seed"

# Find the run dir we just produced (most recent matching prefix).
run_dir=$(ls -td "$ABLATION_BAGS"/"${scenario}__${sensor}__seed${seed}__"* | head -1)
echo "[run_one] analysing $run_dir..."
"$SCRIPT_DIR/analyze_run.py" "$run_dir"

echo "[run_one] done. logs at $LOG_DIR"
# cleanup() runs via trap on exit.
