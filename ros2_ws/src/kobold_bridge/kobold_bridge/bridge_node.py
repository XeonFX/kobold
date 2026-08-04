"""ROS 2 bridge — the boundary between the MCUs and the rest of the stack.

Responsibilities, in order of importance:

  1. Refuse to command a board whose firmware protocol version disagrees.
  2. Translate /cmd_vel into drive commands, and stop commanding on any fault.
  3. Publish wheel odometry, raw IMU, ranges and battery state.

What it deliberately does NOT do:

  * No sensor fusion. Raw wheel odometry and raw IMU go out separately and
    robot_localization fuses them. Skid-steer heading from wheels is unreliable
    by construction, so the EKF needs to see both signals independently rather
    than a pre-blended guess.
  * No odom->base_link TF. robot_localization owns that transform; publishing it
    here too would produce two writers for one edge of the TF tree.
  * No safety decisions. Cliff and collision stops happen on the MCUs in under a
    millisecond. This node reports them, it does not implement them.
"""

from __future__ import annotations

import math
import threading
from typing import Optional

import rclpy
from geometry_msgs.msg import Quaternion, Twist, TwistWithCovariance
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import BatteryState, Imu, Range
from std_msgs.msg import Bool, String

from .link import Frame
from .protocol_generated import (
    BOARD_DRIVE,
    BOARD_SENSE,
    FAULT_CLIFF,
    FAULT_CRITICAL_BATTERY,
    FAULT_ESTOP,
    FAULT_LOW_BATTERY,
    FAULT_SAFETY_LINE,
    FAULT_WATCHDOG,
    MSG_LOG,
    MSG_RANGES,
    MSG_TELEMETRY,
    ClearFault,
    CmdVel,
    Estop,
    Log,
    Ranges,
    SetMode,
    SetThresholds,
    Telemetry,
)
from .serial_board import BoardVersionMismatch, SerialBoard

# Matches imu.h — if you change the MPU-6050 range there, change it here.
GYRO_LSB_PER_DEG_S = 65.5
ACCEL_LSB_PER_G = 8192.0
GRAVITY = 9.80665

RANGE_NO_ECHO = 0xFFFF

FAULT_NAMES = [
    (FAULT_CLIFF, "cliff"),
    (FAULT_SAFETY_LINE, "safety_line"),
    (FAULT_WATCHDOG, "watchdog"),
    (FAULT_LOW_BATTERY, "low_battery"),
    (FAULT_CRITICAL_BATTERY, "critical_battery"),
    (FAULT_ESTOP, "estop"),
]


def describe_faults(flags: int) -> str:
    names = [name for bit, name in FAULT_NAMES if flags & bit]
    return ",".join(names) if names else "none"


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class KoboldBridge(Node):
    def __init__(self):
        super().__init__("kobold_bridge")

        self.declare_parameter("drive_port", "/dev/robot-drive")
        self.declare_parameter("sense_port", "/dev/robot-sense")
        self.declare_parameter("baud", 921600)
        self.declare_parameter("mm_per_tick", 10.21)
        self.declare_parameter("track_width_mm", 150.0)
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("cmd_vel_repeat_hz", 20.0)
        self.declare_parameter("table_mode", False)
        self.declare_parameter("battery_cells", 3)

        self.mm_per_tick = self.get_parameter("mm_per_tick").value
        self.track_width_mm = self.get_parameter("track_width_mm").value
        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value

        latched = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
        )

        # ---- publishers ----
        self.pub_odom = self.create_publisher(Odometry, "wheel_odom", 20)
        self.pub_imu = self.create_publisher(Imu, "imu/data_raw", 50)
        self.pub_batt = self.create_publisher(BatteryState, "battery_state", 5)
        self.pub_faults = self.create_publisher(String, "diagnostics/faults", latched)
        self.pub_estopped = self.create_publisher(Bool, "estopped", latched)
        self.pub_range = {
            side: self.create_publisher(Range, f"range/{side}", 10)
            for side in ("front", "back", "left", "right")
        }

        # ---- subscribers ----
        self.create_subscription(Twist, "cmd_vel", self.on_cmd_vel, 10)
        self.create_subscription(Bool, "estop", self.on_estop, 10)
        self.create_subscription(Bool, "clear_faults", self.on_clear_faults, 10)

        # ---- odometry integration state ----
        self._lock = threading.Lock()
        self._prev_ticks: Optional[tuple] = None
        self._prev_t_ms: Optional[int] = None
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self._last_flags = 0
        self._cmd = (0, 0)

        # ---- boards ----
        self.drive = SerialBoard(
            self.get_parameter("drive_port").value,
            self.get_parameter("baud").value,
            name="drive",
            on_frame=self.on_drive_frame,
            on_disconnect=self.on_disconnect,
            # Both boards are CP2102 with the same factory serial, so udev can
            # only tell them apart by socket. This makes a swapped cable an
            # error instead of motor commands to the ultrasonic board.
            expected_board=BOARD_DRIVE,
        )
        self.sense = SerialBoard(
            self.get_parameter("sense_port").value,
            self.get_parameter("baud").value,
            name="sense",
            on_frame=self.on_sense_frame,
            on_disconnect=self.on_disconnect,
            expected_board=BOARD_SENSE,
        )

        self._connect()

        # Re-send the latest command continuously. The firmware watchdog stops
        # the motors after 300 ms of silence, so a bridge that only forwards on
        # change would produce a robot that stutters whenever a publisher is
        # slow. Repeating is cheap; stuttering is not.
        self.create_timer(
            1.0 / self.get_parameter("cmd_vel_repeat_hz").value, self._repeat_cmd
        )
        self.create_timer(2.0, self._report_link_health)

    # ------------------------------------------------------------- startup

    def _connect(self) -> None:
        for board in (self.drive, self.sense):
            try:
                board.open()
                version = board.wait_for_version()
                self.get_logger().info(
                    f"{board.name}: firmware {version.fw_major}.{version.fw_minor}."
                    f"{version.fw_patch}, protocol v{version.protocol_version}"
                )
            except BoardVersionMismatch as exc:
                # Fail loudly and refuse to run. A version-skewed board is
                # exactly the situation remote firmware updates create, and
                # driving one is how you find out the hard way.
                self.get_logger().fatal(str(exc))
                raise
            except (TimeoutError, OSError) as exc:
                self.get_logger().fatal(f"{board.name}: {exc}")
                raise

        mode = 1 if self.get_parameter("table_mode").value else 0
        self.drive.send_msg(SetMode(mode=mode, max_speed_mm_s=0))
        self.sense.send_msg(SetThresholds(danger_mm=150, warn_mm=400))
        if mode:
            self.get_logger().warn(
                "TABLE MODE: speed capped, cliff sensors are the only thing "
                "between this robot and the floor"
            )

    def destroy_node(self):
        try:
            self.drive.send_msg(CmdVel(0, 0))
        finally:
            self.drive.close()
            self.sense.close()
        super().destroy_node()

    # ------------------------------------------------------------ commands

    def on_cmd_vel(self, msg: Twist) -> None:
        linear_mm_s = int(max(-32768, min(32767, msg.linear.x * 1000.0)))
        angular_mrad_s = int(max(-32768, min(32767, msg.angular.z * 1000.0)))
        with self._lock:
            self._cmd = (linear_mm_s, angular_mrad_s)

    def _repeat_cmd(self) -> None:
        with self._lock:
            linear, angular = self._cmd
        self.drive.send_msg(CmdVel(linear_mm_s=linear, angular_mrad_s=angular))

    def on_estop(self, msg: Bool) -> None:
        self.drive.send_msg(Estop(engage=1 if msg.data else 0))
        if msg.data:
            with self._lock:
                self._cmd = (0, 0)
            self.get_logger().warn("E-STOP engaged")

    def on_clear_faults(self, msg: Bool) -> None:
        if not msg.data:
            return
        # The firmware refuses to clear a fault whose cause persists, so this
        # is a request, not a command.
        self.drive.send_msg(ClearFault(mask=0xFF))
        self.get_logger().info("fault clear requested")

    def on_disconnect(self, port: str) -> None:
        self.get_logger().error(f"{port}: disconnected")

    # -------------------------------------------------------------- frames

    def on_drive_frame(self, frame: Frame) -> None:
        if frame.type == MSG_TELEMETRY:
            self._handle_telemetry(frame.decode())
        elif frame.type == MSG_LOG:
            self._handle_log("drive", frame.decode())

    def on_sense_frame(self, frame: Frame) -> None:
        if frame.type == MSG_RANGES:
            self._handle_ranges(frame.decode())
        elif frame.type == MSG_LOG:
            self._handle_log("sense", frame.decode())

    def _handle_log(self, board: str, msg: Optional[Log]) -> None:
        if msg is None:
            return
        logger = self.get_logger()
        text = f"[{board}] {msg.text}"
        (logger.debug, logger.info, logger.warn, logger.error)[min(msg.level, 3)](text)

    def _handle_telemetry(self, t: Optional[Telemetry]) -> None:
        if t is None:
            return
        now = self.get_clock().now().to_msg()

        with self._lock:
            ticks = (t.ticks_fl, t.ticks_fr, t.ticks_rl, t.ticks_rr)
            prev = self._prev_ticks
            prev_t = self._prev_t_ms
            self._prev_ticks = ticks
            self._prev_t_ms = t.t_ms

            dt = 0.0
            if prev is not None and prev_t is not None:
                # int32 tick counters and a uint32 millisecond clock both wrap;
                # masking the difference makes the wrap a non-event instead of
                # a spike that teleports the robot across the map.
                dt = ((t.t_ms - prev_t) & 0xFFFFFFFF) / 1000.0

            if prev is not None and 0.0 < dt < 1.0:
                def diff(a, b):
                    d = (a - b) & 0xFFFFFFFF
                    return d - 0x100000000 if d > 0x7FFFFFFF else d

                d_left = (diff(ticks[0], prev[0]) + diff(ticks[2], prev[2])) / 2.0
                d_right = (diff(ticks[1], prev[1]) + diff(ticks[3], prev[3])) / 2.0

                dist_l = d_left * self.mm_per_tick / 1000.0
                dist_r = d_right * self.mm_per_tick / 1000.0
                dist = (dist_l + dist_r) / 2.0

                # Heading here comes from the wheels and is only used to keep a
                # rough dead-reckoned pose for debugging. The EKF gets gyro yaw
                # rate separately and should be trusted over this.
                dyaw = (dist_r - dist_l) / (self.track_width_mm / 1000.0)

                self.yaw += dyaw
                self.x += dist * math.cos(self.yaw)
                self.y += dist * math.sin(self.yaw)

                vx = dist / dt
                vyaw = dyaw / dt
            else:
                vx = vyaw = 0.0

            x, y, yaw = self.x, self.y, self.yaw

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation = yaw_to_quaternion(yaw)
        odom.twist.twist.linear.x = vx
        odom.twist.twist.angular.z = vyaw

        # Deliberately pessimistic on yaw: skid-steer wheel odometry scrubs
        # through every turn, and telling the EKF otherwise makes it trust a
        # signal it should mostly ignore in favour of the gyro.
        odom.pose.covariance[0] = 0.02      # x
        odom.pose.covariance[7] = 0.02      # y
        odom.pose.covariance[35] = 0.5      # yaw — high on purpose
        odom.twist.covariance[0] = 0.02
        odom.twist.covariance[35] = 0.5
        self.pub_odom.publish(odom)

        imu = Imu()
        imu.header.stamp = now
        imu.header.frame_id = "imu_link"
        imu.angular_velocity.x = math.radians(t.gyro_x / GYRO_LSB_PER_DEG_S)
        imu.angular_velocity.y = math.radians(t.gyro_y / GYRO_LSB_PER_DEG_S)
        imu.angular_velocity.z = math.radians(t.gyro_z / GYRO_LSB_PER_DEG_S)
        imu.linear_acceleration.x = (t.accel_x / ACCEL_LSB_PER_G) * GRAVITY
        imu.linear_acceleration.y = (t.accel_y / ACCEL_LSB_PER_G) * GRAVITY
        imu.linear_acceleration.z = (t.accel_z / ACCEL_LSB_PER_G) * GRAVITY
        # No orientation estimate from this driver — negative covariance[0] is
        # the REP-145 signal for "this field is not filled in".
        imu.orientation_covariance[0] = -1.0
        imu.angular_velocity_covariance[0] = 0.001
        imu.angular_velocity_covariance[4] = 0.001
        imu.angular_velocity_covariance[8] = 0.001
        imu.linear_acceleration_covariance[0] = 0.05
        imu.linear_acceleration_covariance[4] = 0.05
        imu.linear_acceleration_covariance[8] = 0.05
        self.pub_imu.publish(imu)

        cells = self.get_parameter("battery_cells").value
        batt = BatteryState()
        batt.header.stamp = now
        batt.voltage = t.batt_mv / 1000.0
        batt.present = t.batt_mv > 1000
        # Li-ion 3.0-4.2 V per cell. Voltage-based SoC is crude under load;
        # this is a fuel gauge, not a science instrument.
        v_cell = batt.voltage / cells if cells else 0.0
        batt.percentage = max(0.0, min(1.0, (v_cell - 3.0) / 1.2))
        batt.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        self.pub_batt.publish(batt)

        self._publish_faults(t.flags, t.cliff_mask)

    def _publish_faults(self, flags: int, cliff_mask: int) -> None:
        if flags == self._last_flags:
            return
        gained = flags & ~self._last_flags
        self._last_flags = flags

        if gained & FAULT_CLIFF:
            self.get_logger().error(
                f"CLIFF — motors stopped by firmware (mask 0b{cliff_mask:04b})"
            )
        if gained & FAULT_SAFETY_LINE:
            self.get_logger().warn("safety line asserted by sense board")
        if gained & FAULT_CRITICAL_BATTERY:
            self.get_logger().error("battery critical — motors latched off")
        elif gained & FAULT_LOW_BATTERY:
            self.get_logger().warn("battery low")

        self.pub_faults.publish(String(data=describe_faults(flags)))
        self.pub_estopped.publish(
            Bool(data=bool(flags & (FAULT_CLIFF | FAULT_ESTOP | FAULT_CRITICAL_BATTERY)))
        )

    def _handle_ranges(self, r: Optional[Ranges]) -> None:
        if r is None:
            return
        now = self.get_clock().now().to_msg()
        readings = {
            "front": r.front_mm, "back": r.back_mm,
            "left": r.left_mm, "right": r.right_mm,
        }

        for side, mm in readings.items():
            msg = Range()
            msg.header.stamp = now
            msg.header.frame_id = f"ultrasonic_{side}"
            msg.radiation_type = Range.ULTRASOUND
            msg.field_of_view = math.radians(15.0)
            msg.min_range = 0.02
            msg.max_range = 4.0
            # No echo publishes as +inf, never 0. A zero reads downstream as
            # "obstacle touching the sensor" and would stop the robot dead.
            msg.range = float("inf") if mm == RANGE_NO_ECHO else mm / 1000.0
            self.pub_range[side].publish(msg)

    # ---------------------------------------------------------- diagnostics

    def _report_link_health(self) -> None:
        for board in (self.drive, self.sense):
            s = board.stats
            errors = s.rx_crc_errors + s.rx_malformed
            if errors and s.rx_frames and errors / max(s.rx_frames, 1) > 0.01:
                self.get_logger().warn(
                    f"{board.name}: {errors} bad frames of {s.rx_frames} — "
                    f"check cabling and motor-wire routing"
                )


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = KoboldBridge()
        rclpy.spin(node)
    except (BoardVersionMismatch, TimeoutError, OSError) as exc:
        print(f"kobold_bridge: startup failed: {exc}")
        return 1
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
