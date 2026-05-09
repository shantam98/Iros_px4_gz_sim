# UAV Stack — NUS Cloud Setup Guide

> **Platform:** NUS Vanda cluster · RHEL 9 · NVIDIA A40 · Singularity/Apptainer

---

## Directory Layout

Everything lives under `/scratch/<your_id>/irobot/`:

```
/scratch/<id>/irobot/
├── px4_sim/                        ← sim scripts, Singularity.def, sensor config
├── planner_ws/                     ← UAV ROS2 planner stack
├── px4_autopilot/                  ← PX4-Autopilot source (no subfolder)
├── Micro-XRCE-DDS-Agent/           ← DDS bridge (built here)
├── px4_ros2_ws/
│   └── src/
│       ├── px4_msgs/
│       └── px4_ros_com/
├── uav_stack.sif                   ← Singularity image (built once)
└── logs/                           ← runtime logs
```

---

## Git Repositories & Commits

Clone **all repos** at exactly these commits to guarantee compatibility.

| Repo | URL | Commit / Tag |
|---|---|---|
| px4_sim | `https://github.com/shantam98/Iros_px4_gz_sim.git` | `92214e9` |
| planner_ws | `https://github.com/shantam98/px4_sitl_planner.git` | `a6bab5d` |
| PX4-Autopilot | `https://github.com/PX4/PX4-Autopilot.git` | `9535559025` |
| Micro-XRCE-DDS-Agent | `https://github.com/eProsima/Micro-XRCE-DDS-Agent.git` | `v3.0.1` |
| px4_msgs | `https://github.com/PX4/px4_msgs.git` | `51e6678` |
| px4_ros_com | `https://github.com/PX4/px4_ros_com.git` | `86e9aeb` |

---

## Step-by-Step Setup

### Step 0 — Create base directories

```bash
mkdir -p /scratch/$USER/irobot
mkdir -p /scratch/$USER/singularity
cd /scratch/$USER/irobot
```

---

### Step 1 — Clone all repositories

```bash
BASE=/scratch/$USER/irobot

# ── Sim scripts (setup.sh, run_server.sh, Singularity.def, sensor_bridge.yaml)
git clone https://github.com/shantam98/Iros_px4_gz_sim.git px4_sim
cd px4_sim && git checkout 92214e9 && cd ..

# ── UAV planner stack
git clone https://github.com/shantam98/px4_sitl_planner.git planner_ws
cd planner_ws && git checkout a6bab5d && cd ..

# ── PX4-Autopilot  (clone directly as px4_autopilot — no subfolder)
git clone https://github.com/PX4/PX4-Autopilot.git px4_autopilot
cd px4_autopilot
git checkout 9535559025
git submodule update --init --recursive
cd ..

# ── Micro-XRCE-DDS-Agent
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent && git checkout v3.0.1 && cd ..

# ── px4_ros2_ws  (ROS2 msgs for PX4)
mkdir -p px4_ros2_ws/src && cd px4_ros2_ws/src

git clone https://github.com/PX4/px4_msgs.git
cd px4_msgs && git checkout 51e6678 && cd ..

git clone https://github.com/PX4/px4_ros_com.git
cd px4_ros_com && git checkout 86e9aeb && cd ..

cd ../..   # back to /scratch/$USER/irobot
```

---

### Step 2 — Build the Singularity image

First pull the ROS2 Humble base image:

```bash
singularity pull /scratch/$USER/singularity/ros2_humble.sif \
    docker://osrf/ros:humble-desktop-full
```

Edit `px4_sim/Singularity.def` — replace `YOURID` on the `From:` line with your actual scratch ID:

```
From: /scratch/<your_id>/singularity/ros2_humble.sif
```

Then build the UAV image (takes ~10–15 min):

```bash
cd /scratch/$USER/irobot/px4_sim
singularity build --fakeroot uav_stack.sif Singularity.def
mv uav_stack.sif /scratch/$USER/irobot/uav_stack.sif
```

---

### Step 3 — Install PX4 custom files (airframe + world)

Run `setup.sh` from inside the container, pointing it at the cluster PX4 path:

```bash
singularity exec --nv /scratch/$USER/irobot/uav_stack.sif bash -c \
  "cd /scratch/$USER/irobot/px4_sim && \
   ./setup.sh /scratch/$USER/irobot/px4_autopilot"
```

---

### Step 4 — Build Micro-XRCE-DDS-Agent

```bash
singularity exec --nv /scratch/$USER/irobot/uav_stack.sif bash -c "
  cd /scratch/$USER/irobot/Micro-XRCE-DDS-Agent
  mkdir -p build && cd build
  cmake .. -DCMAKE_BUILD_TYPE=Release
  make -j\$(nproc)
"
```

Binary will be at `Micro-XRCE-DDS-Agent/build/MicroXRCEAgent`.

---

### Step 5 — Build PX4-Autopilot (first-time only)

This compiles PX4 SITL with the Gazebo Harmonic gz_f450 target. Takes ~20 min.

```bash
singularity exec --nv /scratch/$USER/irobot/uav_stack.sif bash -c "
  cd /scratch/$USER/irobot/px4_autopilot
  PX4_GZ_WORLD=indoor_obstacle make px4_sitl gz_f450 -j\$(nproc)
"
```

---

### Step 6 — Build px4_ros2_ws

```bash
singularity exec --nv /scratch/$USER/irobot/uav_stack.sif bash -c "
  source /opt/ros/humble/setup.bash
  cd /scratch/$USER/irobot/px4_ros2_ws
  colcon build --symlink-install
"
```

---

### Step 7 — Build planner_ws

```bash
singularity exec --nv /scratch/$USER/irobot/uav_stack.sif bash -c "
  source /opt/ros/humble/setup.bash
  source /scratch/$USER/irobot/px4_ros2_ws/install/setup.bash
  cd /scratch/$USER/irobot/planner_ws
  colcon build --symlink-install
"
```

---

### Step 8 — Set vehicle_status topic (if needed)

Check which version your PX4 build publishes:

```bash
singularity exec --nv /scratch/$USER/irobot/uav_stack.sif bash -c "
  source /opt/ros/humble/setup.bash
  source /scratch/$USER/irobot/px4_ros2_ws/install/setup.bash
  source /scratch/$USER/irobot/planner_ws/install/setup.bash
  ros2 topic list | grep vehicle_status
"
```

- If it shows `/fmu/out/vehicle_status_v2` → no change needed.
- If it shows `/fmu/out/vehicle_status_v4` → edit this file:

```
planner_ws/uav_control/launch/control.launch.py
```

Change:
```python
'vehicle_status_topic':  '/fmu/out/vehicle_status_v2',
```
to:
```python
'vehicle_status_topic':  '/fmu/out/vehicle_status_v4',
```

Then rebuild uav_control only:
```bash
singularity exec --nv /scratch/$USER/irobot/uav_stack.sif bash -c "
  source /opt/ros/humble/setup.bash
  source /scratch/$USER/irobot/px4_ros2_ws/install/setup.bash
  cd /scratch/$USER/irobot/planner_ws
  colcon build --packages-select uav_control --symlink-install
"
```

---

## Running the Stack

```bash
cd /scratch/$USER/irobot/px4_sim
./run_server.sh $USER
```

This opens 5 xterm windows in order:

| Window | What runs | Gap before next |
|---|---|---|
| T1 | PX4 SITL + Gazebo | 30 s |
| T2 | MicroXRCE-DDS Agent | 20 s |
| T3 | ROS-GZ sensor bridge | 5 s |
| T4 | UAV planner stack | — |
| T5 | Interactive shell (sourced) | — |

Use **T5** to send waypoints and record bags.

### Send a waypoint (T5)

```bash
ros2 action send_goal /uav/navigate_to_goal \
  uav_planner_interface/action/NavigateToGoal \
  '{target_pose: {header: {frame_id: map}, pose: {position: {x: 5.0, y: 0.0, z: 1.5}}}}'
```

### Record a bag (T5)

```bash
ros2 bag record -o /scratch/$USER/irobot/logs/bag_$(date +%Y%m%d_%H%M%S) \
  /drone/odom /drone/tof_merged/points /uav/cmd_vel \
  /uav/current_waypoint /uav/global_path /uav/mission_complete \
  /uav/mp_diag /uav/vfh_status
```

---

## Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| T4 stuck: "Offboard mode command sent" | `vehicle_status` topic mismatch | Check topic version → Step 8 |
| `colcon build` CMakeCache error | Old build from different path | `rm -rf build/ install/ log/` and rebuild |
| `MicroXRCEAgent: command not found` | Not sourced or wrong path | Check `DDS_AGENT` path in run_server.sh |
| T3 bridge fails to start | bridge YAML not found | Confirm `px4_sim/sensor_bridge.yaml` exists |
| Gazebo doesn't open | Display not forwarded | Ensure X11 forwarding: `ssh -X user@server` |
