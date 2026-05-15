# PX4 v1.16.1 Migration Note

> Quick reference for aligning sim/cluster setup with the flight hardware.

## Why

The flight Pixhawk runs **PX4 v1.16.1** (release commit `94cb2012792b2ae89f0b147cfee53ee31ae550be`). The SITL setup was previously on `v1.17.0-alpha1-942-g9535559025` — a development branch with different message schemas and parameter defaults.

Symptoms on the old commit:
- `/fmu/out/vehicle_status_v4` (dev branch schema) vs the planner expecting `_v2`
- Behavioral differences in EKF2, commander state machine
- Risk of testing against a build that doesn't match what flies

Fix: downgrade SITL stack to **v1.16.1** so sim and hardware are bit-identical.

---

## What to change

Three independent git checkouts plus rebuilds. Order matters.

### 1. PX4-Autopilot → `v1.16.1`

```bash
cd $BASE/px4_autopilot/PX4-Autopilot

# Drop the setup.sh-applied CMakeLists.txt edit so checkout is clean
git checkout -- ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt

git checkout v1.16.1
git submodule update --init --recursive

# Tools/simulation/gz submodule may refuse update because setup.sh
# wrote our custom model/world files inside it. Force-clean it:
cd Tools/simulation/gz
git reset --hard HEAD
git clean -fdx
cd $BASE/px4_autopilot/PX4-Autopilot
git submodule update --init --recursive Tools/simulation/gz

# Confirm: should print commit 94cb2012... — matches Pixhawk firmware
git describe --tags        # → v1.16.1
git rev-parse HEAD         # → 94cb2012792b2ae89f0b147cfee53ee31ae550be
```

### 2. Re-apply custom sim files

```bash
cd $BASE/px4_sim
./setup.sh $BASE/px4_autopilot/PX4-Autopilot
```

Re-copies the `4022_gz_f450` airframe, the `f450` + `f450_base` (with stereo IR cameras) models, and the `indoor_obstacle.sdf` world into the freshly-checked-out PX4 tree.

### 3. `px4_msgs` → `release/1.16`

```bash
cd $BASE/px4_ros2_ws/src/px4_msgs        # or wherever px4_msgs lives in your tree
git fetch origin
git checkout release/1.16
git pull --ff-only
# Confirm: should be at v1.16.1 tag
git describe --tags
```

### 4. `px4_ros_com` → `release/1.16`

```bash
cd $BASE/px4_ros2_ws/src/px4_ros_com
git fetch origin
git checkout release/1.16
git pull --ff-only
```

---

## Rebuilds (do all three)

```bash
# px4_ros2_ws — recompile messages (heaviest of the three, ~3 min)
cd $BASE/px4_ros2_ws
rm -rf build/ install/ log/
colcon build --symlink-install
source install/setup.bash

# PX4 SITL firmware (~15-20 min)
cd $BASE/px4_autopilot/PX4-Autopilot
rm -rf build/
PX4_GZ_WORLD=indoor_obstacle make px4_sitl gz_f450

# planner_ws — recompile against new messages (~1 min)
source /opt/ros/humble/setup.bash
source $BASE/px4_ros2_ws/install/setup.bash
cd $BASE/planner_ws
rm -rf build/ install/ log/
colcon build --symlink-install
```

---

## One important config update — `vehicle_status_v1`

PX4 versions the topic name by **message schema** (not release). At v1.16.1, the topic is:

```
/fmu/out/vehicle_status_v1
```

(NOT `_v2` like the dev branch was using.) The planner reads this name from a launch parameter:

**File**: `planner_ws/uav_control/launch/control.launch.py`

```python
'vehicle_status_topic':  '/fmu/out/vehicle_status_v1',
```

If you cloned a tree built against the dev branch, change this string. The `setpoint_publisher_node.cpp` itself doesn't need recompiling — it reads the topic name at launch.

---

## Verification

```bash
# 1. Topic name matches firmware schema
ros2 topic list | grep vehicle_status
# Expected: /fmu/out/vehicle_status_v1

# 2. SITL binary reports v1.16.1
strings $BASE/px4_autopilot/PX4-Autopilot/build/px4_sitl_default/bin/px4 | grep v1.16.1
# Expected: v1.16.1

# 3. End-to-end: drone takes off
ros2 launch uav_bringup full_stack.launch.py
# Expected: STARTUP → TAKEOFF → HOVER, drone at 5 m altitude
```

---

## Common pitfalls

| Problem | Fix |
|---|---|
| `git submodule update` fails on `Tools/simulation/gz` | Force-clean the submodule (commands above in step 1) |
| `setup.sh` errors "anchor not found" | Confirm `4021_gz_x500_flow` exists in `ROMFS/.../airframes/CMakeLists.txt` — it does in v1.16.1 |
| planner_ws compile errors after px4_msgs swap | None expected in this stack — but if you see "field renamed", patch the offending C++ to use the new field name |
| `vehicle_status` timeout in T4 logs | Topic-name mismatch (still on `_v2` or `_v4`) → update `control.launch.py` to `_v1` |

---

## Future migrations

To re-align after any PX4 firmware upgrade:

1. Read the version off the Pixhawk: NSH `ver all` → note `PX4 version` and `PX4 git-hash`.
2. Check out the same tag in `PX4-Autopilot`, the matching `release/X.Y` branch in `px4_msgs` and `px4_ros_com`.
3. Inspect `px4_msgs/msg/VehicleStatus.msg` — the `MESSAGE_VERSION = N` line tells you the `_vN` suffix to use in `control.launch.py`.
4. Run the three rebuilds above.
