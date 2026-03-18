#!/usr/bin/env python3
"""
Spawns one convert_metric_node per ToF sensor as subprocesses.
Run with:  python3 run_depth_converters.py

Kill with: Ctrl+C  (all child processes are cleaned up automatically)
"""

import subprocess
import signal
import sys

TOF_SENSORS = ["tof_0", "tof_1", "tof_2", "tof_3", "tof_4"]

PROCESSES = []


def launch_converter(sensor: str) -> subprocess.Popen:
    cmd = [
        "ros2", "run", "depth_image_proc", "convert_metric_node",
        "--ros-args",
        "--remap", f"image_raw:=/drone/{sensor}/depth",
        "--remap", f"image:=/drone/{sensor}/depth_mono",
        "--remap", f"__node:=depth_converter_{sensor}",   # unique node name
    ]
    print(f"[+] Starting depth converter for {sensor}")
    return subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def shutdown(sig=None, frame=None):
    print("\n[!] Shutting down all depth converter nodes...")
    for p in PROCESSES:
        p.terminate()
    for p in PROCESSES:
        try:
            p.wait(timeout=3)
        except subprocess.TimeoutExpired:
            p.kill()
    print("[✓] All nodes stopped.")
    sys.exit(0)


if __name__ == "__main__":
    signal.signal(signal.SIGINT,  shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    for sensor in TOF_SENSORS:
        PROCESSES.append(launch_converter(sensor))

    print(f"\n[✓] {len(PROCESSES)} converter nodes running.")
    print("     Topics: /drone/tof_N/depth  →  /drone/tof_N/depth_mono")
    print("     Press Ctrl+C to stop all.\n")

    # Wait for all processes — if any dies unexpectedly, report it
    while True:
        for i, p in enumerate(PROCESSES):
            ret = p.poll()
            if ret is not None:
                sensor = TOF_SENSORS[i]
                print(f"[!] Converter for {sensor} exited with code {ret}. Restarting...")
                PROCESSES[i] = launch_converter(sensor)
        signal.pause()