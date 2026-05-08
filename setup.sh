#!/usr/bin/env bash
# setup.sh — Copy custom px4_sim files into a PX4-Autopilot source tree.
# Run ONCE after cloning PX4-Autopilot on a new machine (before building).
#
# Usage:
#   ./setup.sh                                  # uses default PX4_DIR
#   ./setup.sh /path/to/PX4-Autopilot           # explicit path
#
# What it installs:
#   Airframe config  → ROMFS/px4fmu_common/init.d-posix/airframes/
#   Drone models     → Tools/simulation/gz/models/  (f450 + f450_base + meshes)
#   World file       → Tools/simulation/gz/worlds/

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PX4_FILES="$SCRIPT_DIR/px4_files"
PX4_DIR="${1:-$HOME/irobot/px4_autopilot/PX4-Autopilot}"

# ── Validate ──────────────────────────────────────────────────────────────────
if [ ! -d "$PX4_DIR" ]; then
  echo "ERROR: PX4-Autopilot directory not found: $PX4_DIR"
  echo "  Clone it first:  git clone https://github.com/PX4/PX4-Autopilot.git"
  echo "  Then:            cd PX4-Autopilot && git checkout 9535559025"
  echo "  Then:            git submodule update --init --recursive"
  exit 1
fi

AIRFRAMES_DIR="$PX4_DIR/ROMFS/px4fmu_common/init.d-posix/airframes"
MODELS_DIR="$PX4_DIR/Tools/simulation/gz/models"
WORLDS_DIR="$PX4_DIR/Tools/simulation/gz/worlds"

echo "========================================"
echo " PX4 custom sim setup"
echo " PX4_DIR : $PX4_DIR"
echo "========================================"

# ── 1. Airframe config ────────────────────────────────────────────────────────
echo ""
echo "[1/4] Installing airframe 4022_gz_f450..."
cp "$PX4_FILES/4022_gz_f450" "$AIRFRAMES_DIR/4022_gz_f450"
echo "      Copied → $AIRFRAMES_DIR/4022_gz_f450"

# ── 2. Register airframe in CMakeLists.txt ────────────────────────────────────
CMAKE="$AIRFRAMES_DIR/CMakeLists.txt"
echo ""
echo "[2/4] Checking CMakeLists.txt registration..."
if grep -q "4022_gz_f450" "$CMAKE"; then
  echo "      Already registered — skipping."
else
  # Insert 4022_gz_f450 directly after the 4021_gz_x500_flow entry
  sed -i '/4021_gz_x500_flow/a\\t4022_gz_f450' "$CMAKE"
  echo "      Inserted 4022_gz_f450 after 4021_gz_x500_flow"
fi

# ── 2b. Sync airframe to build rootfs if already built ───────────────────────
BUILD_AIRFRAMES="$PX4_DIR/build/px4_sitl_default/rootfs/etc/init.d-posix/airframes"
if [ -d "$BUILD_AIRFRAMES" ]; then
  cp "$PX4_FILES/4022_gz_f450" "$BUILD_AIRFRAMES/4022_gz_f450"
  echo "      Also synced → $BUILD_AIRFRAMES/4022_gz_f450"
else
  echo "      Build dir not found — will be created on next build."
fi

# ── 3. Drone models ───────────────────────────────────────────────────────────
echo ""
echo "[3/4] Installing drone models (f450, f450_base + meshes)..."
cp -r "$PX4_FILES/f450"      "$MODELS_DIR/f450"
cp -r "$PX4_FILES/f450_base" "$MODELS_DIR/f450_base"
echo "      Copied → $MODELS_DIR/f450"
echo "      Copied → $MODELS_DIR/f450_base  (includes meshes/)"

# ── 4. World file ─────────────────────────────────────────────────────────────
echo ""
echo "[4/4] Installing world file..."
cp "$PX4_FILES/indoor_obstacle.sdf" "$WORLDS_DIR/indoor_obstacle.sdf"
echo "      Copied → $WORLDS_DIR/indoor_obstacle.sdf"

# ── Done ──────────────────────────────────────────────────────────────────────
echo ""
echo "========================================"
echo " All files installed."
echo ""
echo " Next: build PX4 SITL"
echo "   cd $PX4_DIR"
echo "   PX4_GZ_WORLD=indoor_obstacle make px4_sitl gz_f450"
echo "========================================"
