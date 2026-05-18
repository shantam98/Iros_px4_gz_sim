#!/usr/bin/env bash
# start_run.sh — drive one ablation run.
#
# Assumes:
#   T1 (PX4 SITL + Gazebo with the right world) already running.
#   T4 (planner stack with the right sensor_source) already running and at
#       /uav/vfh_status == NOMINAL (i.e. mp_node has odom + a cloud).
#   T5 (cuVSLAM + nvblox) is optional for this MP-D415-vs-MP-fusion ablation
#       and not required.
#
# What it does:
#   1. Reads goal from waypoints.yaml.
#   2. Starts a rosbag recording the topics we care about.
#   3. Publishes the waypoint to /uav/current_waypoint.
#   4. Polls /drone/odom until XY-distance < 0.5 m of goal, or timeout.
#   5. Stops the bag and prints the bag dir.
#
# Usage:
#   ./start_run.sh <scenario_name> <sensor_source> [seed]
#   ./start_run.sh scenario_1_pole d415 0

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

scenario="${1:?usage: start_run.sh <scenario_name> <sensor_source> [seed]}"
sensor="${2:?usage: start_run.sh <scenario_name> <sensor_source> [seed]}"
seed="${3:-0}"

# ── Parse waypoint from YAML (avoid yq dep, use python) ─────────────────────
read -r wp_x wp_y wp_z timeout_s < <(
  python3 - "$SCRIPT_DIR/waypoints.yaml" "$scenario" <<'PY'
import sys, yaml
with open(sys.argv[1]) as f:
    d = yaml.safe_load(f)
s = d[sys.argv[2]]
print(s["x"], s["y"], s["z"], s["timeout_s"])
PY
)

# ── Output dir ──────────────────────────────────────────────────────────────
bag_root="${ABLATION_BAGS:-$HOME/irobot/planner_ws/bags}"
run_dir="$bag_root/${scenario}__${sensor}__seed${seed}__$(date +%Y%m%d_%H%M%S)"
mkdir -p "$run_dir"
echo "================================================================"
echo " ABLATION RUN"
echo "  scenario     : $scenario"
echo "  sensor       : $sensor"
echo "  seed         : $seed"
echo "  waypoint     : ($wp_x, $wp_y, $wp_z)"
echo "  timeout      : ${timeout_s}s"
echo "  bag dir      : $run_dir"
echo "================================================================"

# ── Confirm planner is alive ────────────────────────────────────────────────
status=$(timeout 3 ros2 topic echo /uav/vfh_status --once 2>/dev/null \
         | awk -F'data: ' '/data:/ {print $2; exit}' || true)
# IDLE = planner is up but no waypoint yet (expected pre-run state).
# NOMINAL = planner is up and actively tracking a waypoint.
# Anything else = something's wrong.
case "${status:-}" in
    NOMINAL|IDLE)
        ;;
    *)
        echo "WARN: /uav/vfh_status='${status:-<no message>}'."
        echo "      Expected NOMINAL or IDLE. Make sure T1 + T4 are up and"
        echo "      the drone has reached HOVER. Continuing anyway in 3s..."
        sleep 3
        ;;
esac

# ── Start bag recorder in background ────────────────────────────────────────
ros2 bag record -o "$run_dir/bag" \
    /drone/odom \
    /uav/cmd_vel \
    /uav/mp_diag \
    /uav/vfh_status \
    /uav/current_waypoint \
    /uav/mission_complete \
    /fmu/in/trajectory_setpoint \
    /fmu/out/vehicle_local_position \
    > "$run_dir/bag.log" 2>&1 &
bag_pid=$!
sleep 2   # let bag set up subscribers

# ── Send waypoint via Python (avoids 'ros2 topic pub --once' discovery race) ─
echo "[$(date +%H:%M:%S)] sending waypoint to ($wp_x, $wp_y, $wp_z)..."
python3 - "$wp_x" "$wp_y" "$wp_z" <<'PY'
import sys, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped

wx, wy, wz = map(float, sys.argv[1:4])
rclpy.init()
n = Node('ablation_wp_pub')
pub = n.create_publisher(PointStamped, '/uav/current_waypoint', 10)

# Wait until at least one subscriber (mp_node) is connected.
for _ in range(50):
    if pub.get_subscription_count() > 0:
        break
    rclpy.spin_once(n, timeout_sec=0.1)

m = PointStamped()
m.header.frame_id = 'map'
m.point.x, m.point.y, m.point.z = wx, wy, wz

# Publish a few times so even latched/late subscribers catch it.
for _ in range(10):
    m.header.stamp = n.get_clock().now().to_msg()
    pub.publish(m)
    rclpy.spin_once(n, timeout_sec=0.05)
    time.sleep(0.05)

subs = pub.get_subscription_count()
print(f"waypoint published to {subs} subscriber(s).")
n.destroy_node()
rclpy.shutdown()
PY

# ── Wait for goal-reached or timeout ────────────────────────────────────────
t0=$(date +%s)
result="TIMEOUT"
while :; do
    elapsed=$(( $(date +%s) - t0 ))
    if (( elapsed > timeout_s )); then
        break
    fi
    # Read one position sample.
    pos=$(timeout 2 ros2 topic echo /drone/odom --once --field pose.pose.position 2>/dev/null || true)
    cx=$(echo "$pos" | awk '/^x:/ {print $2; exit}')
    cy=$(echo "$pos" | awk '/^y:/ {print $2; exit}')
    if [[ -n "${cx:-}" && -n "${cy:-}" ]]; then
        dist=$(python3 -c "import math; print(f'{math.hypot(${cx}-${wp_x}, ${cy}-${wp_y}):.3f}')")
        printf "\r  t=%3ds  pos=(%6.2f, %6.2f)  dist_to_goal=%sm  " \
               "$elapsed" "$cx" "$cy" "$dist"
        if python3 -c "import sys; sys.exit(0 if ${dist} < 0.5 else 1)"; then
            result="SUCCESS"
            break
        fi
    fi
    sleep 1
done
echo

# ── Stop bag ────────────────────────────────────────────────────────────────
echo "[$(date +%H:%M:%S)] result=$result (${elapsed}s) — stopping bag..."
kill -INT "$bag_pid" 2>/dev/null || true
wait "$bag_pid" 2>/dev/null || true

# ── Write run metadata ──────────────────────────────────────────────────────
cat > "$run_dir/meta.json" <<JSON
{
  "scenario": "$scenario",
  "sensor": "$sensor",
  "seed": $seed,
  "waypoint": {"x": $wp_x, "y": $wp_y, "z": $wp_z},
  "timeout_s": $timeout_s,
  "elapsed_s": $elapsed,
  "shell_result": "$result"
}
JSON

echo
echo "  bag dir : $run_dir"
echo "  next    : ablation/analyze_run.py $run_dir"
