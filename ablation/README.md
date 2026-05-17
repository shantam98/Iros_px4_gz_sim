# Ablation — MP planner: D415 vs depth fusion

8 runs total: 4 scenarios × 2 sensor configs × 1 seed.

## Pre-flight (once)

```bash
# Re-source planner_ws after recent rebuilds.
source ~/irobot/planner_ws/install/setup.bash

# Re-sync SDFs (scenario worlds + ToF-enabled drone model) into PX4 tree.
~/irobot/px4_sim/setup.sh
```

## Per-run loop

For each `(scenario, sensor)`:

1. **T1 — PX4 SITL + Gazebo** (manual terminal, not in `run_sim_slam.sh`):
   ```bash
   cd ~/irobot/px4_autopilot/PX4-Autopilot
   PX4_GZ_WORLD=scenario_1_pole make px4_sitl gz_f450
   #                ^^^^^^^^^^^^^^^^ swap per scenario
   ```

2. **T2 — DDS Agent**, **T3 — Sensor Bridge** — let `run_sim_slam.sh` start
   them. Skip T5 (no cuVSLAM needed for the MP-only ablation).

3. **T4 — Planner** (manual launch with the right `sensor_source`):
   ```bash
   source /opt/ros/humble/setup.bash
   source ~/irobot/px4_ros2_ws/install/setup.bash
   source ~/irobot/planner_ws/install/setup.bash
   ros2 launch uav_bringup full_stack.launch.py \
        with_global_planner:=false \
        with_vslam:=false \
        planner_backend:=mp \
        sensor_source:=d415          # or sensor_source:=fusion
   ```

4. Wait until `ros2 topic echo /uav/vfh_status --once` shows `NOMINAL`.

5. **Run** (any terminal):
   ```bash
   ~/irobot/px4_sim/ablation/start_run.sh scenario_1_pole d415 0
   ```

6. **Analyse** the run (any terminal):
   ```bash
   ~/irobot/px4_sim/ablation/analyze_run.py \
       ~/irobot/planner_ws/bags/scenario_1_pole__d415__seed0__<timestamp>
   ```

7. Ctrl+C T1 (PX4) and T4 (planner). Repeat with the next `(scenario, sensor)`.

## Once all 8 runs done

```bash
~/irobot/px4_sim/ablation/summarize.py
```

Prints a per-scenario A/B table.

## Scenario / waypoint summary

| Scenario | Goal `(x, y, z)` | Hypothesis |
|---|---|---|
| `scenario_1_pole` | `(6, 0, 1.5)` | D415 wins (longer range → earlier brake) |
| `scenario_2_slalom` | `(7, 0, 1.5)` | Fusion wins (side awareness during yaw) |
| `scenario_3_corner` | `(4, 4, 1.5)` | Fusion wins (D415 loses wall mid-turn) |
| `scenario_4_gap` | `(7, 0, 1.5)` | Toss-up — both should see walls head-on |

## Files

| File | Purpose |
|---|---|
| `waypoints.yaml` | per-scenario goal pose + run timeout |
| `start_run.sh` | one run: bag + waypoint + poll + stop |
| `analyze_run.py` | one bag → `metrics.json` |
| `summarize.py` | walks all `metrics.json` → table + A/B compare |
