#!/usr/bin/env python3

import sys
import tty
import termios
import math
import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import TrajectorySetpoint, VehicleOdometry

HELP_MSG = """
============================================
   PX4 Drone Teleop — Keyboard Control
============================================
Movement (NED Frame):
  W / S   →  Move Forward / Backward  (North)
  A / D   →  Move Left / Right        (East)
  ↑ / ↓   →  Move Up / Down           (Z, NED)
  Q / E   →  Rotate Left / Right      (Yaw)

Controls:
  SPACE   →  Hover (stop all movement)
  X       →  Emergency STOP & Exit

Speed adjustment:
  +       →  Increase speed
  -       →  Decrease speed
============================================
"""

KEY_BINDINGS = {
    'w':      ( 1.0,  0.0,  0.0,  0.0),
    's':      (-1.0,  0.0,  0.0,  0.0),
    'a':      ( 0.0, -1.0,  0.0,  0.0),
    'd':      ( 0.0,  1.0,  0.0,  0.0),
    '\x1b[A': ( 0.0,  0.0, -1.0,  0.0),  # up arrow   → up (negative Z NED)
    '\x1b[B': ( 0.0,  0.0,  1.0,  0.0),  # down arrow → down
    'q':      ( 0.0,  0.0,  0.0, -1.0),  # yaw CCW
    'e':      ( 0.0,  0.0,  0.0,  1.0),  # yaw CW
    ' ':      ( 0.0,  0.0,  0.0,  0.0),  # hover
}


def wrap_pi(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    if key == '\x1b':
        extra = sys.stdin.read(2)
        key = key + extra
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


class TeleopKeyboard(Node):
    def __init__(self):
        super().__init__('teleop_keyboard')

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.publisher = self.create_publisher(
            TrajectorySetpoint, '/teleop/setpoint', 10)

        self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self._odom_cb,
            px4_qos
        )

        self.speed        = 2.0
        self.yaw_speed    = 0.8
        self.current_yaw  = 0.0
        self.yaw_initialized = False

        # Current velocity command — written by key thread, read by timer
        self._lock = threading.Lock()
        self._vx = self._vy = self._vz = self._yr = 0.0
        self._running = True

        # 50 Hz publish timer — runs on the ROS executor thread
        self.create_timer(0.02, self._publish_cb)

        print(HELP_MSG)

    # ── Odometry callback (executor thread) ─────────────────
    def _odom_cb(self, msg):
        w, x, y, z = msg.q[0], msg.q[1], msg.q[2], msg.q[3]
        siny = 2.0 * (w * z + x * y)
        cosy = 1.0 - 2.0 * (y * y + z * z)
        self.current_yaw = math.atan2(siny, cosy)
        self.yaw_initialized = True

    # ── 50 Hz publish callback (executor thread) ─────────────
    def _publish_cb(self):
        if not self.yaw_initialized:
            return

        with self._lock:
            vx, vy, vz, yr = self._vx, self._vy, self._vz, self._yr

        # Integrate yaw target
        if yr != 0.0:
            self.current_yaw = wrap_pi(self.current_yaw + yr * self.yaw_speed * 0.02)

        # Rotate body-frame commands into NED world frame using current yaw.
        # Without this, W always means North regardless of drone heading.
        #   body x = forward (drone nose)
        #   body y = right
        #   NED  x = North,  NED y = East
        cy, sy = math.cos(self.current_yaw), math.sin(self.current_yaw)
        bvx = vx * self.speed
        bvy = vy * self.speed
        ned_vx = cy * bvx - sy * bvy   # North
        ned_vy = sy * bvx + cy * bvy   # East

        msg = TrajectorySetpoint()
        msg.position  = [float('nan'), float('nan'), float('nan')]
        msg.velocity  = [float(ned_vx),
                         float(ned_vy),
                         float(vz * self.speed)]  # Z stays as-is (NED down)
        msg.yaw       = float(self.current_yaw)
        msg.yawspeed  = float(yr * self.yaw_speed)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.publisher.publish(msg)

    # ── Key thread — blocks on stdin, never touches ROS ──────
    def _key_loop(self):
        settings = termios.tcgetattr(sys.stdin)
        print("Waiting for odometry...")
        while rclpy.ok() and not self.yaw_initialized:
            pass
        print("Ready! SPACE=hover  X=exit\n")

        try:
            while rclpy.ok() and self._running:
                key = get_key(settings)  # blocking — isolated in this thread

                if key in ('x', 'X'):
                    print('\nEmergency stop!')
                    with self._lock:
                        self._vx = self._vy = self._vz = self._yr = 0.0
                    self._running = False
                    break
                elif key in ('+', '='):
                    self.speed = min(self.speed + 0.5, 10.0)
                    print(f'Speed: {self.speed:.1f} m/s')
                elif key in ('-', '_'):
                    self.speed = max(self.speed - 0.5, 0.5)
                    print(f'Speed: {self.speed:.1f} m/s')
                elif key in KEY_BINDINGS:
                    vx, vy, vz, yr = KEY_BINDINGS[key]
                    with self._lock:
                        self._vx, self._vy, self._vz, self._yr = vx, vy, vz, yr
                    print(f'CMD → vx:{vx*self.speed:.1f} vy:{vy*self.speed:.1f} '
                          f'vz:{vz*self.speed:.1f} yr:{yr*self.yaw_speed:.2f}')
                else:
                    with self._lock:
                        self._vx = self._vy = self._vz = self._yr = 0.0
        except Exception as e:
            print(f'Key thread error: {e}')
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

    def start_key_thread(self):
        t = threading.Thread(target=self._key_loop, daemon=True)
        t.start()
        return t


def main(args=None):
    rclpy.init(args=args)
    node = TeleopKeyboard()

    # Key reading runs in its own thread — never blocks ROS spin
    key_thread = node.start_key_thread()

    try:
        # ROS executor runs freely: timer + odometry callbacks fire at full rate
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._running = False
        key_thread.join(timeout=1.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()