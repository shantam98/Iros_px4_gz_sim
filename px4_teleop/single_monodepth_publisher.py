#!/usr/bin/env python3
"""
Subscribes to 5 ToF depth images (32FC1, 100x100 each),
normalises each to mono8, stitches them side-by-side into a
500x100 image, and publishes on /drone/tof/depth_mono_combined.

Run with:  python3 tof_combined_depth.py
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2

TOF_SENSORS  = ["tof_0", "tof_1", "tof_2", "tof_3", "tof_4"]  # subscription order
TOF_DISPLAY  = ["tof_1", "tof_0", "tof_4", "tof_3", "tof_2"]  # clockwise panorama, front-center
INPUT_TOPIC  = "/drone/{sensor}/depth"          # 32FC1
OUTPUT_TOPIC = "/drone/tof/depth_mono_combined" # mono8, 500x100
IMG_W, IMG_H = 100, 100


class TofCombinedDepth(Node):
    def __init__(self):
        super().__init__("tof_combined_depth")

        self.bridge  = CvBridge()
        self.frames  = {s: None for s in TOF_SENSORS}   # latest image per sensor
        self.latest_stamp = None

        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        for sensor in TOF_DISPLAY:
            topic = INPUT_TOPIC.format(sensor=sensor)
            self.create_subscription(
                Image, topic,
                lambda msg, s=sensor: self._cb(msg, s),
                best_effort_qos,
            )
            self.get_logger().info(f"Subscribed to {topic}")

        self.pub = self.create_publisher(Image, OUTPUT_TOPIC, 10)

        # Publish at 20 Hz — matches ToF update rate
        self.create_timer(0.05, self._publish_cb)

        self.get_logger().info(f"Publishing combined image on {OUTPUT_TOPIC}")

    # ── Per-sensor callback ──────────────────────────────────
    def _cb(self, msg: Image, sensor: str):
        try:
            # 32FC1 float depth image (metres)
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
        except Exception as e:
            self.get_logger().warn(f"[{sensor}] cv_bridge error: {e}")
            return

        # Resize to expected size in case of any mismatch
        if img.shape != (IMG_H, IMG_W):
            img = cv2.resize(img, (IMG_W, IMG_H))

        self.frames[sensor]  = img
        self.latest_stamp    = msg.header.stamp

    # ── 20 Hz stitch + publish ───────────────────────────────
    def _publish_cb(self):
        # Need all 5 frames before publishing
        if any(f is None for f in self.frames.values()):
            missing = [s for s, f in self.frames.items() if f is None]
            self.get_logger().info(
                f"Waiting for: {missing}", throttle_duration_sec=2.0)
            return

        strips = []
        for sensor in TOF_SENSORS:
            img = self.frames[sensor]   # float32 metres

            # Normalise to 0-255 using the sensor clip range (0.2 – 3.0 m).
            # Closer = brighter (255), farther = darker (0).
            near, far = 0.2, 3.0
            norm = np.clip((far - img) / (far - near), 0.0, 1.0)
            mono = (norm * 255).astype(np.uint8)
            strips.append(mono)

        # Horizontal stitch → 500 x 100 mono8
        combined = np.hstack(strips)    # shape: (100, 500)

        # Optional: draw thin separator lines between sensors
        for i in range(1, len(TOF_SENSORS)):
            combined[:, i * IMG_W - 1] = 128   # mid-grey line

        out_msg = self.bridge.cv2_to_imgmsg(combined, encoding="mono8")
        out_msg.header.stamp    = self.latest_stamp or self.get_clock().now().to_msg()
        out_msg.header.frame_id = "tof_array_link"
        self.pub.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TofCombinedDepth()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()