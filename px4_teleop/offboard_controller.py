#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)

TAKEOFF_HEIGHT = -5.0       # NED: negative = up (metres)
STARTUP_DELAY_SEC = 5.0     # Wait for EKF2 to converge before arming


class OffboardController(Node):
    def __init__(self):
        super().__init__('offboard_controller')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publishers — use versioned topic names to match PX4
        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos)
        self.trajectory_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos)
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos)

        # Subscribers — try versioned names first, fall back handled via topic discovery
        self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status_v2',       # versioned name from your PX4 log
            self.vehicle_status_callback, qos)
        self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position_v1',  # versioned name from your PX4 log
            self.local_position_callback, qos)
        self.create_subscription(
            TrajectorySetpoint, '/teleop/setpoint',
            self.teleop_callback, 10)

        # State
        self.vehicle_status = VehicleStatus()
        self.local_position = VehicleLocalPosition()
        self.teleop_setpoint = None
        self.offboard_counter = 0
        self.armed = False
        self.in_offboard_mode = False
        self.takeoff_complete = False
        self.status_received = False   # flag to confirm topic is being received

        self.cmd_throttle_counter = 0
        self.CMD_THROTTLE_TICKS = 10

        self.startup_ticks = 0
        self.STARTUP_TICKS = int(STARTUP_DELAY_SEC / 0.1)

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info(
            f'Offboard Controller started. Waiting {STARTUP_DELAY_SEC}s for EKF2...')

    # ── Callbacks ──────────────────────────────────────────────────────────

    def vehicle_status_callback(self, msg):
        if not self.status_received:
            self.get_logger().info('vehicle_status topic received! ✓')
            self.status_received = True
        self.vehicle_status = msg
        self.armed = (msg.arming_state == VehicleStatus.ARMING_STATE_ARMED)
        self.in_offboard_mode = (msg.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD)

    def local_position_callback(self, msg):
        self.local_position = msg
        if self.armed and msg.z < (TAKEOFF_HEIGHT * 0.8):
            if not self.takeoff_complete:
                self.takeoff_complete = True
                self.get_logger().info('Takeoff complete! Teleop is now active.')

    def teleop_callback(self, msg):
        self.teleop_setpoint = msg

    # ── Main loop ──────────────────────────────────────────────────────────

    def timer_callback(self):

        # Always send heartbeat and setpoint regardless of state
        if self.takeoff_complete:
            self.publish_offboard_control_mode(velocity=True, position=False)
            if self.teleop_setpoint is not None:
                self.trajectory_pub.publish(self.teleop_setpoint)
            else:
                self.publish_hover_setpoint()
        else:
            self.publish_offboard_control_mode(velocity=False, position=True)
            self.publish_takeoff_setpoint()

        # Startup delay
        if self.startup_ticks < self.STARTUP_TICKS:
            self.startup_ticks += 1
            remaining = (self.STARTUP_TICKS - self.startup_ticks) * 0.1
            if self.startup_ticks % 10 == 0:
                self.get_logger().info(f'Startup delay... {remaining:.0f}s remaining')
            return

        # Warn if vehicle_status topic not yet received after startup
        if not self.status_received:
            self.get_logger().warn(
                'vehicle_status not received yet! Check topic name with: '
                'ros2 topic list | grep vehicle_status',
                throttle_duration_sec=2.0)
            return

        # Offboard heartbeat warmup
        if self.offboard_counter < 10:
            self.offboard_counter += 1
            return

        # Throttle commands to avoid uORB queue overflow
        self.cmd_throttle_counter += 1
        if self.cmd_throttle_counter < self.CMD_THROTTLE_TICKS:
            return
        self.cmd_throttle_counter = 0

        # State machine
        if not self.in_offboard_mode:
            self.engage_offboard_mode()
            self.get_logger().info(
                f'Waiting for offboard mode... nav_state={self.vehicle_status.nav_state}')
        elif not self.armed:
            self.arm()
        # Once armed + offboard → setpoints handle the rest

    # ── Setpoints ──────────────────────────────────────────────────────────

    def publish_offboard_control_mode(self, velocity=False, position=False):
        msg = OffboardControlMode()
        msg.position = position
        msg.velocity = velocity
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = self.ts()
        self.offboard_mode_pub.publish(msg)

    def publish_takeoff_setpoint(self):
        msg = TrajectorySetpoint()
        msg.position = [0.0, 0.0, TAKEOFF_HEIGHT]
        msg.yaw = 0.0
        msg.timestamp = self.ts()
        self.trajectory_pub.publish(msg)

    def publish_hover_setpoint(self):
        msg = TrajectorySetpoint()
        msg.velocity = [0.0, 0.0, 0.0]
        msg.yawspeed = 0.0
        msg.position = [float('nan'), float('nan'), float('nan')]
        msg.timestamp = self.ts()
        self.trajectory_pub.publish(msg)

    # ── Commands ───────────────────────────────────────────────────────────

    def arm(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Sending arm command...')

    def disarm(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('Disarming...')

    def engage_offboard_mode(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self.ts()
        self.vehicle_command_pub.publish(msg)

    def ts(self):
        return int(self.get_clock().now().nanoseconds / 1000)


def main(args=None):
    rclpy.init(args=args)
    node = OffboardController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.disarm()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()