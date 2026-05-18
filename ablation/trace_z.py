#!/usr/bin/env python3
"""trace_z.py <run_dir> — print z-pos, vz, cmd_vel.z, status over time.

Useful when a run failed in an unexpected way (sudden descent, oscillation,
etc.) and you want to see the exact frame when it happened.
"""
import sys
from pathlib import Path

from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def main(run_dir: Path):
    bag = run_dir / "bag"
    storage = StorageOptions(uri=str(bag), storage_id="")
    conv = ConverterOptions(input_serialization_format="cdr",
                            output_serialization_format="cdr")
    reader = SequentialReader()
    reader.open(storage, conv)
    types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types()}

    rows = []  # (t_s, kind, value...)
    t0 = None
    while reader.has_next():
        topic, raw, t_ns = reader.read_next()
        if t0 is None: t0 = t_ns
        t = (t_ns - t0) * 1e-9
        msg = deserialize_message(raw, types[topic])
        if topic == "/drone/odom":
            rows.append((t, "odom",
                         msg.pose.pose.position.x,
                         msg.pose.pose.position.y,
                         msg.pose.pose.position.z,
                         msg.twist.twist.linear.z))
        elif topic == "/uav/cmd_vel":
            rows.append((t, "cmd",
                         msg.twist.linear.x,
                         msg.twist.linear.y,
                         msg.twist.linear.z))
        elif topic == "/uav/vfh_status":
            rows.append((t, "stat", msg.data))
        elif topic == "/fmu/in/trajectory_setpoint":
            rows.append((t, "tsp", msg.velocity[0], msg.velocity[1], msg.velocity[2]))

    rows.sort(key=lambda r: r[0])

    # Print one combined line every ~0.5 s.
    print(f"{'t_s':>6}  {'pos_x':>6} {'pos_y':>6} {'pos_z':>6}  "
          f"{'cmd_z':>6}  {'tsp_z':>6}  status")
    last_t = -1.0
    cur = {"pos": (None,)*4, "cmd_z": None, "tsp_z": None, "status": "-"}
    for r in rows:
        t = r[0]
        kind = r[1]
        if kind == "odom":
            cur["pos"] = (r[2], r[3], r[4], r[5])
        elif kind == "cmd":
            cur["cmd_z"] = r[4]
        elif kind == "tsp":
            cur["tsp_z"] = r[4]
        elif kind == "stat":
            cur["status"] = r[2]
        if t - last_t >= 0.5:
            px, py, pz, _ = cur["pos"]
            print(f"{t:6.2f}  "
                  f"{px:6.2f} {py:6.2f} {pz:6.2f}  "
                  f"{cur['cmd_z']!s:>6}  {cur['tsp_z']!s:>6}  {cur['status']}")
            last_t = t


if __name__ == "__main__":
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    main(Path(sys.argv[1]).resolve())
