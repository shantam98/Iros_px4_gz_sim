#!/usr/bin/env python3
"""
analyze_run.py — read one ablation run's rosbag, dump metrics to JSON.

Metrics:
  success         : did drone reach within 0.5 m of waypoint? (from meta.json)
  time_to_goal_s  : seconds from first cmd_vel to goal-reach (or NaN if timeout)
  path_length_m   : integrated XY distance from /drone/odom samples
  min_clearance_m : min /uav/mp_diag[8] (closest_obstacle_dist); ignores -1.0 sentinel
  mean_speed_mps  : path_length / time_to_goal
  jerk_rms        : RMS of d^3(pos)/dt^3 (smoothness, m/s^3)
  n_estop         : count of /uav/vfh_status == "ESTOP" frames
  n_avoiding      : count of /uav/vfh_status starting with "AVOIDING"

Usage:
  analyze_run.py <run_dir>
"""

import sys
import json
import math
from pathlib import Path

try:
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
except ImportError:
    sys.exit("ERROR: source /opt/ros/humble/setup.bash before running this script.")


def read_bag(bag_path: Path):
    """Yield (topic, msg, t_ns) tuples in time order."""
    # storage_id="" lets rosbag2 sniff the format (sqlite3 default, or mcap
    # if the host has the plugin).
    storage = StorageOptions(uri=str(bag_path), storage_id="")
    conv = ConverterOptions(input_serialization_format="cdr",
                            output_serialization_format="cdr")
    reader = SequentialReader()
    reader.open(storage, conv)
    type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}
    msg_classes = {n: get_message(t) for n, t in type_map.items()}
    while reader.has_next():
        topic, raw, t_ns = reader.read_next()
        yield topic, deserialize_message(raw, msg_classes[topic]), t_ns


def analyze(run_dir: Path):
    meta = json.loads((run_dir / "meta.json").read_text())
    wp = meta["waypoint"]
    bag_path = run_dir / "bag"

    odom_samples = []          # (t_s, x, y, z)
    first_cmd_t = None
    goal_reach_t = None
    min_clear = float("inf")
    n_estop = 0
    n_avoiding = 0

    for topic, msg, t_ns in read_bag(bag_path):
        t_s = t_ns * 1e-9
        if topic == "/drone/odom":
            p = msg.pose.pose.position
            odom_samples.append((t_s, p.x, p.y, p.z))
            d = math.hypot(p.x - wp["x"], p.y - wp["y"])
            if d < 0.5 and goal_reach_t is None and first_cmd_t is not None:
                goal_reach_t = t_s
        elif topic == "/uav/cmd_vel":
            if first_cmd_t is None:
                first_cmd_t = t_s
        elif topic == "/uav/mp_diag":
            if len(msg.data) > 8:
                obs = msg.data[8]   # closest_obstacle_dist
                if obs >= 0.0 and obs < min_clear:
                    min_clear = obs
        elif topic == "/uav/vfh_status":
            if msg.data == "ESTOP":
                n_estop += 1
            elif msg.data.startswith("AVOIDING"):
                n_avoiding += 1

    # Path length (integrated XY).
    path_len = 0.0
    for (t0, x0, y0, _), (t1, x1, y1, _) in zip(odom_samples, odom_samples[1:]):
        path_len += math.hypot(x1 - x0, y1 - y0)

    # Time to goal.
    if first_cmd_t is not None and goal_reach_t is not None:
        t_goal = goal_reach_t - first_cmd_t
    else:
        t_goal = float("nan")

    # Mean speed during traverse.
    if t_goal == t_goal and t_goal > 0:   # not NaN
        mean_speed = path_len / t_goal
    else:
        mean_speed = float("nan")

    # Jerk RMS — third derivative of position. Downsample to a uniform 20 Hz
    # grid first so dt-jitter from sim-time pauses can't blow up the
    # finite-difference. Clip outliers above 50 m/s^3 (anything faster than
    # the drone can physically do is sensor/clock noise, not real motion).
    jerk_rms = float("nan")
    if len(odom_samples) >= 20:
        ts = [s[0] for s in odom_samples]
        xs = [s[1] for s in odom_samples]
        ys = [s[2] for s in odom_samples]

        t0, tN = ts[0], ts[-1]
        dt_uniform = 0.05  # 20 Hz
        n_steps = max(int((tN - t0) / dt_uniform), 5)
        ux, uy = [], []
        j = 0
        for k in range(n_steps):
            tk = t0 + k * dt_uniform
            while j + 1 < len(ts) and ts[j + 1] < tk:
                j += 1
            ux.append(xs[j])
            uy.append(ys[j])

        def deriv(values, dt):
            return [(values[i] - values[i - 1]) / dt for i in range(1, len(values))]

        vx = deriv(ux, dt_uniform); vy = deriv(uy, dt_uniform)
        ax = deriv(vx, dt_uniform); ay = deriv(vy, dt_uniform)
        jx = deriv(ax, dt_uniform); jy = deriv(ay, dt_uniform)
        if jx:
            sq = [min(jx[i] ** 2 + jy[i] ** 2, 2500.0) for i in range(len(jx))]
            jerk_rms = math.sqrt(sum(sq) / len(sq))

    # Altitude trace: catch "drone descended unexpectedly" failures.
    zs = [s[3] for s in odom_samples]
    z_min  = round(min(zs), 2) if zs else None
    z_max  = round(max(zs), 2) if zs else None
    z_end  = round(zs[-1], 2) if zs else None
    # If z dropped below 0.3 m at any point, drone effectively touched the floor.
    z_floor_hit = z_min is not None and z_min < 0.3

    metrics = {
        "scenario":        meta["scenario"],
        "sensor":          meta["sensor"],
        "seed":            meta["seed"],
        "success":         goal_reach_t is not None,
        "time_to_goal_s":  None if t_goal != t_goal else round(t_goal, 2),
        "path_length_m":   round(path_len, 2),
        "min_clearance_m": None if min_clear == float("inf") else round(min_clear, 2),
        "mean_speed_mps":  None if mean_speed != mean_speed else round(mean_speed, 2),
        "jerk_rms":        None if jerk_rms != jerk_rms else round(jerk_rms, 3),
        "z_min_m":         z_min,
        "z_max_m":         z_max,
        "z_end_m":         z_end,
        "floor_hit":       z_floor_hit,
        "n_estop_frames":  n_estop,
        "n_avoiding_frames": n_avoiding,
        "shell_elapsed_s": meta["elapsed_s"],
        "shell_result":    meta["shell_result"],
    }

    out_path = run_dir / "metrics.json"
    out_path.write_text(json.dumps(metrics, indent=2))
    print(json.dumps(metrics, indent=2))
    print(f"\n  wrote: {out_path}")


if __name__ == "__main__":
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    analyze(Path(sys.argv[1]).resolve())
