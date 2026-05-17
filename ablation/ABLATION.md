# Ablation — `mp_node` planner with D415 vs depth fusion

Goal: hold the planner algorithm constant, vary only the **obstacle cloud source**, measure what changes. Same code, same params, same controller — the only thing that differs between Config A and Config B is one launch arg (`sensor_source:={d415,fusion}`).

This is **not** an MP-vs-MP+ESDF comparison. `mp_esdf_node` is out of scope here (ESDF/nvblox integration is on hold pending the TF-lookup root-cause).

---

## 1. What `mp_node` does

A 20 Hz reactive planner. Every cycle:

1. **Ingest the cloud** subscribed on `/drone/tof_merged/points` (after the launch-arg-controlled remap). Voxel-subsample by `history_subsample`, push into a body-frame FIFO of capacity `history_capacity`.
2. **Generate primitives** — sample a fixed family of short trajectories ahead of the drone:
   - `num_az_horizontal × num_curvatures = 18 × 5 = 90` horizontal arcs (azimuth swept across ±π, curvature swept across `±max_curvature`)
   - `len(elevation_angles_deg) × num_az_pitched = 4 × 18 = 72` pitched straight rays at ±15°, ±30° elevation
   - Total primitives evaluated per cycle: **162**.
3. **Score each primitive** against the cloud:
   - For every point in the history buffer, compute its distance to each primitive arc (`pointToArcDist2D`).
   - A primitive collides if any point comes within `collision_radius` of it; track `closest_obstacle_dist` per primitive.
   - **Cost** = `w_goal · Δangle_to_goal + w_prev · Δangle_to_prev + w_pos · pos_norm`, where `pos_norm = end_dist_to_goal / best_end_dist`. Collisions disqualify; closest non-colliding primitive wins.
4. **Speed** scales between `min_speed` and `max_speed` based on closest-obstacle proximity, with a `max_accel` slew limit on `cmd_vel` cycle-to-cycle.
5. **E-stop** fires when `closest_obstacle_dist < min_clearance`. Hysteresis (`obstacle_hysteresis_factor = 1.3`) prevents AVOIDING↔NOMINAL flicker. Recovery state caps speed to `recovery_max_speed` for `recovery_duration_sec` after each ESTOP.
6. **Stall detector** — if the drone's best-ever distance to goal hasn't improved by `stall_progress_min` within `stall_no_improve_window`, publishes `STALLED` status. Catches "drone bouncing off a wall and never converging" as well as full stops.
7. **Orbit detector** — accumulates yaw; if `> orbit_yaw_threshold` (270°) without `orbit_progress_min` of goal progress, publishes safety hover.
8. **Bypass FSM** — when one direction collides but the side is clear, latch a left/right bypass for `bypass_min_hold_cycles` to keep momentum past an obstacle instead of oscillating.

All defaults live in [`config/mp_params.yaml`](../../planner_ws/uav_local_planner/config/mp_params.yaml). Same YAML feeds both configs in this ablation — no per-config tuning.

---

## 2. The two configs

Only the cloud source differs.

| | **Config A — D415** | **Config B — fusion** |
|---|---|---|
| Cloud topic into `mp_node` | `/drone/rgbd/points` | `/drone/tof_merged/points` |
| Source sensor(s) | 1× Intel RealSense D415 RGBD | 5× MaixSense MS-A010 ring ToFs merged in `base_link` by `cloud_merge_node` |
| FOV (horizontal) | ~65° forward | ~360° (5 × ~72° ring overlap) |
| Effective range | ~5 m | ~3 m |
| Update rate | 30 Hz | 20 Hz (per ToF) |
| Approx points per cycle (post-subsample) | ~38 k from D415's ~307 k @ subsample=8 | ~3–5 k from 5 ToFs @ ~10 k each, subsample=8 |
| Latency | low (single sensor, single bridge) | low (parallel ToF reads + one merge) |
| Spatial gaps | rear and sides totally blind | small per-ToF cones at the ring seams; minor vertical gaps above/below the ring plane |
| Failure modes | reflective surfaces, depth shadows beyond 5 m | flat reflective walls (specular dropout); thin objects between ToFs |

Both configs go through the **same** mp_node binary, **same** mp_params, **same** setpoint_publisher.

---

## 3. What we're measuring

Per run, extracted from the rosbag by [`analyze_run.py`](analyze_run.py):

| Metric | Source topic / computation | Why |
|---|---|---|
| `success` | drone XY-pos within 0.5 m of waypoint before timeout (polled in `start_run.sh`) | Did the mission complete? |
| `time_to_goal_s` | first `/uav/cmd_vel` → first `/drone/odom` inside 0.5 m | Speed of completion |
| `path_length_m` | integrated XY distance from `/drone/odom` | Detour efficiency |
| `min_clearance_m` | min of `/uav/mp_diag[8]` (`closest_obstacle_dist`) | Safety margin |
| `mean_speed_mps` | `path_length / time_to_goal` | Cruise speed achieved |
| `jerk_rms` | RMS of d³(pos)/dt³ from odom | Smoothness — high jerk = jittery cmd_vel |
| `n_estop_frames` | count of `/uav/vfh_status == ESTOP` | How often the e-stop trigger fired |
| `n_avoiding_frames` | count of `/uav/vfh_status` starting with AVOIDING | How long the planner was actively dodging |

---

## 4. Scenarios

Top-down view in all diagrams. Drone spawns at `(0, 0)` facing **→ +X**. `●` = drone, `▓` = obstacle. Hover altitude `z = 1.5 m`.

### Scenario 1 — Single pole forward `scenario_1_pole.sdf`

```
Y↑
 │         ●  ← pole Ø 0.24 m
 │
 ●─────────────────────── GOAL (6, 0)
            pole at (4, 0)            X→
```

Single forward obstacle. Tests **range** — how early does the planner brake?

### Scenario 2 — Two-pillar slalom `scenario_2_slalom.sdf`

```
Y↑
 │      ●            ← pillar_A Ø 0.70 m at (3, +1)
 │       \
 ●────────\─────────── (curve right then left)
 │         \
 │          ●         ← pillar_B Ø 0.70 m at (5, -1)
                       GOAL (7, 0)            X→
```

Two offset pillars force an S-curve. Tests **lateral awareness during yaw** — when curving right around A, can the planner still see B on the left?

### Scenario 3 — Off-axis goal with barrier `scenario_3_corner.sdf`

```
Y↑
 │       │ ← barrier_wall (3×3×0.2 m)
 │       │    centered (2.5, +1.5), spans y∈[0, +3]
 │       │    GOAL (4, 4)
 │       │   *
 ●──────────────────────── X→
```

Diagonal goal blocked by a vertical wall. Tests **sustained off-axis obstacle tracking** — does the drone keep the wall in view as it detours around it?

### Scenario 4 — Narrow gap `scenario_4_gap.sdf`

```
Y↑
 │       ▓▓▓                ← wall_left  x=3, y∈[+0.5, +3]
 │       ▓▓▓
 ●─────  GAP 1 m  ──────── GOAL (7, 0)
 │       ▓▓▓
 │       ▓▓▓                ← wall_right x=3, y∈[-3, -0.5]            X→
```

1 m gap to thread through. Tests **side-clearance during transit** — once the drone enters the gap, can the planner still see the walls beside it?

---

## 5. Hypothesis matrix + expected outcome table

H = hypothesis (which sensor "should" win); E = expected reason; PLACEHOLDER values are filled in below to be overwritten with real measurements once the runs are done. Use → to read "expected to be roughly".

| Scenario | Sensor angle | H wins | Why |
|---|---|---|---|
| 1 | longer-range forward | **D415** | sees pole earlier → brakes once instead of hard |
| 2 | side-of-yaw blindspot | **Fusion** | D415 loses pillar_B mid-turn; ring ToFs keep it |
| 3 | sustained off-axis obstacle | **Fusion** | D415 loses wall once the drone moves past it laterally |
| 4 | side-clearance during transit | **Toss-up / mild fusion** | both see walls head-on; only matters if drone drifts mid-gap |

### Expected results table — values are PLACEHOLDERS

Fill in after running. Format: `value (success | fail)`; `—` = run not yet executed.

| Scenario | Sensor | success | time_to_goal_s | path_length_m | min_clearance_m | mean_speed_mps | jerk_rms | n_estop | n_avoiding |
|---|---|---|---|---|---|---|---|---|---|
| **1** pole | D415   | ✓ | ~4.0 | ~6.2 | ~0.80 | ~1.5 | ~3.0 | 0 | ~10 |
| **1** pole | Fusion | ✓ | ~5.0 | ~6.5 | ~0.65 | ~1.3 | ~4.0 | 0 | ~25 |
| **2** slalom | D415   | ✗ *(clip pillar_B)* | — | ~5.5 | ~0.20 | — | ~6 | ~5 | ~40 |
| **2** slalom | Fusion | ✓ | ~6.5 | ~8.0 | ~0.60 | ~1.2 | ~5 | 0 | ~60 |
| **3** corner | D415   | ✗ *(clip wall)*    | — | ~5.0 | ~0.15 | — | ~7 | ~8 | ~30 |
| **3** corner | Fusion | ✓ | ~7.5 | ~7.5 | ~0.55 | ~1.0 | ~5 | 0 | ~50 |
| **4** gap | D415   | ✓ | ~5.5 | ~7.2 | ~0.40 | ~1.3 | ~4 | 0 | ~30 |
| **4** gap | Fusion | ✓ | ~6.0 | ~7.2 | ~0.45 | ~1.2 | ~4 | 0 | ~35 |

Reading the placeholders:
- D415 expected to be *faster* on **scenario 1** (longer range, no detour, fewer ESTOP cycles).
- D415 expected to *fail* on **2** and **3** (side blindspots → clipping or ESTOP-on-impact).
- Fusion expected to be *slower but safer* across the board (lower mean_speed but higher min_clearance, more `AVOIDING` frames).
- **Scenario 4** expected to be a wash.

If the real numbers diverge from this (e.g. D415 also succeeds at scenario 2), interpretation:
- D415 succeeds where fusion was predicted to win → primitive curvature is mild enough that the side never *fully* leaves the FOV.
- Fusion fails on scenario 1 → ToF range too short for 2.5 m/s cruise; either lower `max_speed` or accept range limit.
- Both fail on scenario 4 → primitive geometry can't thread a 1 m gap; widen the gap or revisit primitive density.

### What constitutes a "story"

- **3 of 4 wins for fusion**: clean narrative — sensor coverage matters more than range for our missions.
- **2 / 2 split**: keep both, route per-mission (forward-traverse → D415; tight indoor → fusion).
- **Fusion wins 4/4**: D415-only architecture is wrong; switch the production stack.
- **D415 wins 4/4**: surprising; likely means ring ToFs are noisy enough that adding them hurts more than helps, even with coverage.

---

## 6. Filling the table

After the 8 runs:

```bash
# After each run
~/irobot/px4_sim/ablation/analyze_run.py <run_dir>

# After all 8
~/irobot/px4_sim/ablation/summarize.py
```

Copy `summarize.py` output's per-row values into the table above (overwrite the `~` placeholders). Track failure-mode notes in a footnote per row when `success=✗`.
