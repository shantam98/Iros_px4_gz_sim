#!/usr/bin/env python3
"""
summarize.py — gather metrics.json from every run and print a comparison table.

Usage:
  ./summarize.py                          # walks $HOME/irobot/planner_ws/bags
  ./summarize.py /path/to/bags_root
"""

import sys
import json
import os
from pathlib import Path

root = Path(sys.argv[1]) if len(sys.argv) > 1 \
       else Path(os.environ.get("ABLATION_BAGS",
                                Path.home() / "irobot/planner_ws/bags"))

rows = []
for metrics_file in sorted(root.rglob("metrics.json")):
    rows.append(json.loads(metrics_file.read_text()))

if not rows:
    sys.exit(f"No metrics.json files found under {root}.")

cols = ["scenario", "sensor", "seed", "success",
        "time_to_goal_s", "path_length_m",
        "min_clearance_m", "mean_speed_mps", "jerk_rms",
        "n_estop_frames", "n_avoiding_frames"]

widths = {c: max(len(c), max(len(str(r.get(c, ""))) for r in rows)) for c in cols}

def fmt(r):
    return "  ".join(f"{str(r.get(c, '')):<{widths[c]}}" for c in cols)

print(fmt({c: c for c in cols}))
print(fmt({c: "-" * widths[c] for c in cols}))
for r in rows:
    print(fmt(r))

# Per-scenario A/B comparison.
print("\n--- A/B comparison (by scenario) ---")
by_scen = {}
for r in rows:
    by_scen.setdefault(r["scenario"], {})[r["sensor"]] = r

for scen, runs in sorted(by_scen.items()):
    print(f"\n  {scen}:")
    for sensor, r in sorted(runs.items()):
        ok = "OK" if r["success"] else "FAIL"
        tt = r["time_to_goal_s"]
        mc = r["min_clearance_m"]
        print(f"    {sensor:<7}  {ok:<5}  "
              f"t_goal={tt}s  min_clear={mc}m  jerk={r['jerk_rms']}")
