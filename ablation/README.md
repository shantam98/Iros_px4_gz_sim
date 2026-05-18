# Ablation — MP planner: D415 vs depth fusion

8 runs total: 4 scenarios × 2 sensor configs × 1 seed.

## Setup (once per shell)

```bash
source ~/irobot/px4_sim/init_env.sh        # sources ROS + workspaces, exports paths
~/irobot/px4_sim/setup.sh                  # re-sync SDFs (scenario worlds + drone model) into PX4 tree
```

## Per-run loop

Two terminals, both with `init_env.sh` sourced.

**Terminal A — PX4 SITL (manual, kept running)**
```bash
cd $PX4_DIR && PX4_GZ_WORLD=scenario_1_pole make px4_sitl gz_f450
#                            ^^^^^^^^^^^^^^^^ swap per scenario
```

**Terminal B — one command runs everything else**
```bash
~/irobot/px4_sim/ablation/run_one.sh scenario_1_pole fusion 0
```

`run_one.sh` brings up T2 (DDS) + T3 (sensor bridge) + T4 (planner with the
chosen `sensor_source`) as background processes, waits until the planner is
ready, calls `start_run.sh` to record the bag + send the waypoint + poll for
goal, then calls `analyze_run.py` and tears T2/T3/T4 down. PX4 in terminal A
is untouched.

After it returns: change `PX4_GZ_WORLD` in terminal A (Ctrl+C, re-run with
next scenario), then re-run `run_one.sh` in terminal B with the new scenario.

8 invocations total:

```bash
~/irobot/px4_sim/ablation/run_one.sh scenario_1_pole   fusion 0
~/irobot/px4_sim/ablation/run_one.sh scenario_1_pole   d415   0
~/irobot/px4_sim/ablation/run_one.sh scenario_2_slalom fusion 0
~/irobot/px4_sim/ablation/run_one.sh scenario_2_slalom d415   0
~/irobot/px4_sim/ablation/run_one.sh scenario_3_corner fusion 0
~/irobot/px4_sim/ablation/run_one.sh scenario_3_corner d415   0
~/irobot/px4_sim/ablation/run_one.sh scenario_4_gap    fusion 0
~/irobot/px4_sim/ablation/run_one.sh scenario_4_gap    d415   0
```

Tip: pair the two configs per scenario before changing the PX4 world — saves
4 of the 8 PX4 restarts. So for scenario 1: leave T1 running, do `fusion`
then immediately `d415`, then Ctrl+C T1 and restart with scenario 2.

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
