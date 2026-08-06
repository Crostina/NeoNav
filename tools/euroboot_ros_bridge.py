#!/usr/bin/env python3
"""ROS 2 to TCP bridge for the Euroboot dashboard.

Run this on the Raspberry Pi after the micro-ROS agent, odom TF bridge, and
Nav2 are active. The Windows dashboard connects to this bridge with plain
newline-delimited JSON, so the dashboard does not need ROS installed locally.
"""

from __future__ import annotations

import argparse
import copy
import json
import math
import socket
import struct
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Quaternion, TransformStamped, Twist
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Odometry, Path as NavPath
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from tf2_ros import TransformBroadcaster

try:
    import serial
except ImportError:  # pragma: no cover - bridge runs on Pi where pyserial is installed
    serial = None

try:
    from pixhawk_mavlink_yaw_probe import parse_frames
except ImportError:  # pragma: no cover - keeps old deployments importable
    parse_frames = None


RUNTIME_GEOMETRY_PATH = Path("/home/maker/euroboot/config/euroboot_runtime_geometry.json")
RUNTIME_NAV2_PATH = Path("/home/maker/euroboot/config/euroboot_runtime_nav2.json")
RUNTIME_IMU_PATH = Path("/home/maker/euroboot/config/euroboot_runtime_imu.json")
RUNTIME_TURN_PATH = Path("/home/maker/euroboot/config/euroboot_runtime_turn.json")

FIRMWARE_WHEEL_DIAMETER_M = 0.04586
FIRMWARE_WHEEL_BASE_M = 0.15216
FIRMWARE_COUNTS_PER_REV = 1400

NAV2_PARAM_DEFAULTS = {
    "desired_linear_vel": 0.24,
    "lookahead_dist": 0.20,
    "min_lookahead_dist": 0.10,
    "max_lookahead_dist": 0.40,
    "lookahead_time": 0.50,
    "min_approach_linear_velocity": 0.05,
    "approach_velocity_scaling_dist": 0.30,
    "regulated_linear_scaling_min_radius": 0.36,
    "regulated_linear_scaling_min_speed": 0.055,
    "max_angular_accel": 3.30,
    "xy_goal_tolerance": 0.02,
    "yaw_goal_tolerance": 5.28,
}

IMU_DEFAULTS = {
    "use_pixhawk_yaw": True,
    "pixhawk_weight": 0.80,
    "encoder_weight": 0.20,
    "pixhawk_yaw_mode": "gyro",
    "pixhawk_yaw_sign": -1.0,
    "pixhawk_port": "/dev/serial0",
    "pixhawk_baud": 115200,
    "pixhawk_timeout_s": 0.75,
}

TURN_PARAM_DEFAULTS = {
    "preturn_heading_error_rad": 1.20,
    "preturn_min_distance_m": 0.08,
    "final_position_correction_enabled": 0.0,
    "turn_timeout_s": 6.0,
    "turn_yaw_tolerance_rad": 0.045,
    "turn_stable_samples": 2,
    "turn_min_angular_speed": 0.25,
    "turn_max_angular_speed": 1.55,
    "turn_kp": 2.20,
    "turn_linear_balance_kp": 1.00,
    "turn_linear_balance_limit_mps": 0.060,
    "turn_settle_s": 0.04,
}

NAV2_PARAM_TARGETS = {
    "desired_linear_vel": "FollowPath.desired_linear_vel",
    "lookahead_dist": "FollowPath.lookahead_dist",
    "min_lookahead_dist": "FollowPath.min_lookahead_dist",
    "max_lookahead_dist": "FollowPath.max_lookahead_dist",
    "lookahead_time": "FollowPath.lookahead_time",
    "min_approach_linear_velocity": "FollowPath.min_approach_linear_velocity",
    "approach_velocity_scaling_dist": "FollowPath.approach_velocity_scaling_dist",
    "regulated_linear_scaling_min_radius": "FollowPath.regulated_linear_scaling_min_radius",
    "regulated_linear_scaling_min_speed": "FollowPath.regulated_linear_scaling_min_speed",
    "max_angular_accel": "FollowPath.max_angular_accel",
    "xy_goal_tolerance": "goal_checker.xy_goal_tolerance",
    "yaw_goal_tolerance": "goal_checker.yaw_goal_tolerance",
}

WAYPOINT_GUARD_TOLERANCE_M = 0.08
FINAL_POSITION_GUARD_TOLERANCE_M = 0.055


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def yaw_from_quat(q: Quaternion) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def quat_from_yaw(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


@dataclass
class Pose2D:
    x: float
    y: float
    yaw: float


@dataclass
class MissionWaypoint:
    x: float
    y: float
    path_yaw: float
    final_yaw: float | None = None


class PixhawkYawReader:
    def __init__(self, node: Node, port: str, baud: int) -> None:
        self.node = node
        self.port = port
        self.baud = baud
        self.lock = threading.Lock()
        self.latest_yaw: float | None = None
        self.integrated_yaw: float = 0.0
        self.last_boot_ms: int | None = None
        self.latest_t: float = 0.0
        self.running = False
        self.thread: threading.Thread | None = None

    def start(self) -> None:
        if self.thread and self.thread.is_alive():
            return
        self.running = True
        self.thread = threading.Thread(target=self.loop, daemon=True)
        self.thread.start()

    def stop(self) -> None:
        self.running = False

    def update_port(self, port: str, baud: int) -> None:
        if port == self.port and baud == self.baud:
            return
        self.stop()
        self.port = port
        self.baud = baud
        self.thread = None
        self.start()

    def get(self) -> tuple[float | None, float, float | None]:
        with self.lock:
            yaw = self.latest_yaw
            integrated_yaw = self.integrated_yaw
            t = self.latest_t
        age = float("inf") if t <= 0.0 else time.monotonic() - t
        return yaw, age, integrated_yaw if t > 0.0 else None

    def loop(self) -> None:
        if serial is None or parse_frames is None:
            self.node.get_logger().warn("Pixhawk yaw reader unavailable: pyserial or MAVLink parser missing")
            return

        buffer = bytearray()
        while self.running:
            try:
                with serial.Serial(self.port, self.baud, timeout=0.2) as ser:
                    self.node.get_logger().info(f"Pixhawk yaw reader opened {self.port} @ {self.baud}")
                    while self.running:
                        buffer.extend(ser.read(512))
                        for frame in parse_frames(buffer):
                            if frame.msgid != 30 or len(frame.payload) < 28:
                                continue
                            boot_ms = struct.unpack_from("<I", frame.payload, 0)[0]
                            yaw = struct.unpack_from("<f", frame.payload, 12)[0]
                            yawspeed = struct.unpack_from("<f", frame.payload, 24)[0]
                            with self.lock:
                                if self.last_boot_ms is not None:
                                    dt_ms = (boot_ms - self.last_boot_ms) & 0xFFFFFFFF
                                    dt = dt_ms / 1000.0
                                    if 0.0 < dt < 0.25:
                                        self.integrated_yaw = normalize_angle(self.integrated_yaw + yawspeed * dt)
                                self.last_boot_ms = boot_ms
                                self.latest_yaw = yaw
                                self.latest_t = time.monotonic()
            except Exception as exc:
                self.node.get_logger().warn(f"Pixhawk yaw reader error: {exc}")
                time.sleep(1.0)


class DashboardBridge(Node):
    def __init__(self, host: str, port: int) -> None:
        super().__init__("euroboot_dashboard_bridge")
        self.host = host
        self.port = port
        self.odom_sub = self.create_subscription(Odometry, "/odom/unfiltered", self.odom_cb, 30)
        self.odom_pub = self.create_publisher(Odometry, "/odom", 30)
        self.cmd_sub = self.create_subscription(Twist, "/cmd_vel", self.cmd_cb, 30)
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.nav_client = ActionClient(self, FollowPath, "follow_path")
        self.tf_broadcaster = TransformBroadcaster(self)

        self.lock = threading.RLock()
        self.dashboard_clients: set[socket.socket] = set()
        self.raw_pose: Pose2D | None = None
        self.encoder_pose: Pose2D | None = None
        self.origin: Pose2D | None = None
        self.encoder_origin_yaw: float | None = None
        self.pixhawk_origin_yaw: float | None = None
        self.fused_origin_yaw: float | None = None
        self.geometry = self.load_geometry()
        self.nav2_params = self.load_nav2_params()
        self.imu_settings = self.load_imu_settings()
        self.turn_params = self.load_turn_params()
        self.pixhawk_reader = PixhawkYawReader(
            self,
            str(self.imu_settings["pixhawk_port"]),
            int(self.imu_settings["pixhawk_baud"]),
        )
        self.pixhawk_reader.start()
        self.mission_thread: threading.Thread | None = None
        self.cancel_requested = False
        self.shutdown_requested = False
        self.last_broadcast_t = 0.0
        self.last_tf_t = 0.0
        self.last_cmd_linear_x = 0.0
        self.last_cmd_angular_z = 0.0
        self.last_cmd_t = time.monotonic()
        self.last_actual_linear_x = 0.0
        self.last_actual_angular_z = 0.0

        self.server_thread = threading.Thread(target=self.server_loop, daemon=True)
        self.server_thread.start()
        self.get_logger().info(f"Euroboot dashboard bridge listening on {host}:{port}")

    def cmd_cb(self, msg: Twist) -> None:
        with self.lock:
            self.last_cmd_linear_x = float(msg.linear.x)
            self.last_cmd_angular_z = float(msg.angular.z)
            self.last_cmd_t = time.monotonic()

    def odom_cb(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        encoder_yaw = yaw_from_quat(msg.pose.pose.orientation)
        corrected_x, corrected_y = self.correct_xy(p.x, p.y)

        with self.lock:
            self.encoder_pose = Pose2D(corrected_x, corrected_y, encoder_yaw)
            fused_yaw, yaw_info = self.fused_yaw_locked(encoder_yaw)
            pose = Pose2D(corrected_x, corrected_y, fused_yaw)
            self.raw_pose = pose
            if self.origin is None:
                self.origin = Pose2D(pose.x, pose.y, pose.yaw)
                self.encoder_origin_yaw = encoder_yaw
                pixhawk_yaw, _age, pixhawk_gyro_yaw = self.pixhawk_reader.get()
                self.pixhawk_origin_yaw = self.selected_pixhawk_yaw(pixhawk_yaw, pixhawk_gyro_yaw)
            local = self.raw_to_local_locked(pose)
            cmd_linear_x = self.last_cmd_linear_x
            cmd_angular_z = self.last_cmd_angular_z
            cmd_age_s = time.monotonic() - self.last_cmd_t
            self.last_actual_linear_x = float(msg.twist.twist.linear.x)
            self.last_actual_angular_z = float(msg.twist.twist.angular.z)

        self.publish_fused_odom(msg, pose)

        now = time.monotonic()
        if now - self.last_tf_t >= 0.05:
            self.last_tf_t = now
            self.broadcast_tf(msg, pose)

        if now - self.last_broadcast_t < 0.10:
            return
        self.last_broadcast_t = now
        self.broadcast(
            {
                "type": "odom",
                "t": now,
                "x": local.x,
                "y": local.y,
                "yaw": local.yaw,
                "raw_x": pose.x,
                "raw_y": pose.y,
                "raw_yaw": pose.yaw,
                "encoder_yaw": encoder_yaw,
                "pixhawk_yaw": yaw_info["pixhawk_yaw"],
                "pixhawk_yaw_ros": yaw_info["pixhawk_yaw_ros"],
                "pixhawk_gyro_yaw": yaw_info["pixhawk_gyro_yaw"],
                "pixhawk_age_s": yaw_info["pixhawk_age_s"],
                "yaw_source": yaw_info["yaw_source"],
                "use_pixhawk_yaw": bool(self.imu_settings["use_pixhawk_yaw"]),
                "pixhawk_weight": float(self.imu_settings["pixhawk_weight"]),
                "encoder_weight": float(self.imu_settings["encoder_weight"]),
                "pixhawk_yaw_mode": str(self.imu_settings["pixhawk_yaw_mode"]),
                "pixhawk_yaw_sign": float(self.imu_settings["pixhawk_yaw_sign"]),
                "linear_x": msg.twist.twist.linear.x,
                "angular_z": msg.twist.twist.angular.z,
                "cmd_linear_x": cmd_linear_x,
                "cmd_angular_z": cmd_angular_z,
                "cmd_age_s": cmd_age_s,
            }
        )

    def geometry_distance_scale(self) -> float:
        wheel_diameter = float(self.geometry.get("wheel_diameter_m", FIRMWARE_WHEEL_DIAMETER_M))
        counts_per_rev = float(self.geometry.get("counts_per_rev", FIRMWARE_COUNTS_PER_REV))
        return (wheel_diameter / FIRMWARE_WHEEL_DIAMETER_M) * (FIRMWARE_COUNTS_PER_REV / counts_per_rev)

    def geometry_yaw_scale(self) -> float:
        wheel_base = float(self.geometry.get("wheel_base_m", FIRMWARE_WHEEL_BASE_M))
        return FIRMWARE_WHEEL_BASE_M / wheel_base

    def correct_xy(self, x: float, y: float) -> tuple[float, float]:
        scale = self.geometry_distance_scale()
        return x * scale, y * scale

    def fused_yaw_locked(self, encoder_yaw: float) -> tuple[float, dict[str, Any]]:
        encoder_origin = self.encoder_origin_yaw
        if encoder_origin is None:
            encoder_origin = encoder_yaw
            self.encoder_origin_yaw = encoder_origin
        fused_origin = self.fused_origin_yaw
        if fused_origin is None:
            fused_origin = encoder_origin
            self.fused_origin_yaw = fused_origin

        encoder_delta = normalize_angle(encoder_yaw - encoder_origin) * self.geometry_yaw_scale()
        encoder_delta = normalize_angle(encoder_delta)
        pixhawk_raw_yaw, pixhawk_age, pixhawk_gyro_yaw = self.pixhawk_reader.get()
        pixhawk_yaw = self.selected_pixhawk_yaw(pixhawk_raw_yaw, pixhawk_gyro_yaw)
        pixhawk_timeout = float(self.imu_settings["pixhawk_timeout_s"])
        pixhawk_weight = float(self.imu_settings["pixhawk_weight"])
        encoder_weight = float(self.imu_settings["encoder_weight"])

        if not bool(self.imu_settings["use_pixhawk_yaw"]) or pixhawk_weight <= 0.001:
            return normalize_angle(fused_origin + encoder_delta), {
                "yaw_source": "encoder",
                "pixhawk_yaw": pixhawk_raw_yaw,
                "pixhawk_yaw_ros": pixhawk_yaw,
                "pixhawk_gyro_yaw": pixhawk_gyro_yaw,
                "pixhawk_age_s": pixhawk_age,
            }

        if pixhawk_yaw is None or pixhawk_age > pixhawk_timeout:
            return normalize_angle(fused_origin + encoder_delta), {
                "yaw_source": "encoder_pixhawk_stale",
                "pixhawk_yaw": pixhawk_raw_yaw,
                "pixhawk_yaw_ros": pixhawk_yaw,
                "pixhawk_gyro_yaw": pixhawk_gyro_yaw,
                "pixhawk_age_s": pixhawk_age,
            }

        if self.pixhawk_origin_yaw is None:
            self.pixhawk_origin_yaw = pixhawk_yaw

        pixhawk_delta = normalize_angle(pixhawk_yaw - self.pixhawk_origin_yaw)
        total = max(0.001, pixhawk_weight + encoder_weight)
        pixhawk_weight /= total
        encoder_weight /= total

        sin_sum = pixhawk_weight * math.sin(pixhawk_delta) + encoder_weight * math.sin(encoder_delta)
        cos_sum = pixhawk_weight * math.cos(pixhawk_delta) + encoder_weight * math.cos(encoder_delta)
        fused_delta = math.atan2(sin_sum, cos_sum)
        return normalize_angle(fused_origin + fused_delta), {
            "yaw_source": "pixhawk_fused",
            "pixhawk_yaw": pixhawk_raw_yaw,
            "pixhawk_yaw_ros": pixhawk_yaw,
            "pixhawk_gyro_yaw": pixhawk_gyro_yaw,
            "pixhawk_age_s": pixhawk_age,
        }

    def selected_pixhawk_yaw(self, attitude_yaw: float | None, gyro_yaw: float | None) -> float | None:
        mode = str(self.imu_settings.get("pixhawk_yaw_mode", "gyro")).strip().lower()
        sign = float(self.imu_settings.get("pixhawk_yaw_sign", -1.0))
        source_yaw = gyro_yaw if mode == "gyro" else attitude_yaw
        if source_yaw is None:
            return None
        return normalize_angle(sign * source_yaw)

    def publish_fused_odom(self, msg: Odometry, pose: Pose2D) -> None:
        fused = copy.deepcopy(msg)
        fused.header.frame_id = msg.header.frame_id or "odom"
        fused.child_frame_id = msg.child_frame_id or "base_footprint"
        fused.pose.pose.position.x = pose.x
        fused.pose.pose.position.y = pose.y
        fused.pose.pose.orientation = quat_from_yaw(pose.yaw)
        self.odom_pub.publish(fused)

    def broadcast_tf(self, msg: Odometry, pose: Pose2D) -> None:
        tf = TransformStamped()
        tf.header = msg.header
        tf.header.frame_id = msg.header.frame_id or "odom"
        tf.child_frame_id = msg.child_frame_id or "base_footprint"
        tf.transform.translation.x = pose.x
        tf.transform.translation.y = pose.y
        tf.transform.translation.z = msg.pose.pose.position.z
        tf.transform.rotation = quat_from_yaw(pose.yaw)
        self.tf_broadcaster.sendTransform(tf)

    def raw_to_local_locked(self, pose: Pose2D) -> Pose2D:
        origin = self.origin or Pose2D(0.0, 0.0, 0.0)
        dx = pose.x - origin.x
        dy = pose.y - origin.y
        c = math.cos(origin.yaw)
        s = math.sin(origin.yaw)
        return Pose2D(
            x=c * dx + s * dy,
            y=-s * dx + c * dy,
            yaw=normalize_angle(pose.yaw - origin.yaw),
        )

    def local_to_raw_locked(self, pose: Pose2D) -> Pose2D:
        origin = self.origin or Pose2D(0.0, 0.0, 0.0)
        c = math.cos(origin.yaw)
        s = math.sin(origin.yaw)
        return Pose2D(
            x=origin.x + c * pose.x - s * pose.y,
            y=origin.y + s * pose.x + c * pose.y,
            yaw=normalize_angle(origin.yaw + pose.yaw),
        )

    def server_loop(self) -> None:
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind((self.host, self.port))
        srv.listen(4)
        srv.settimeout(0.5)

        while not self.shutdown_requested:
            try:
                client, addr = srv.accept()
            except socket.timeout:
                continue
            except OSError:
                break

            client.settimeout(0.5)
            with self.lock:
                self.dashboard_clients.add(client)
            self.send(client, {"type": "status", "level": "info", "message": f"connected from {addr[0]}:{addr[1]}"})
            threading.Thread(target=self.client_loop, args=(client, addr), daemon=True).start()

        srv.close()

    def client_loop(self, client: socket.socket, addr: tuple[str, int]) -> None:
        buffer = b""
        try:
            while not self.shutdown_requested:
                try:
                    chunk = client.recv(4096)
                except socket.timeout:
                    continue
                except OSError as exc:
                    self.get_logger().warn(f"dashboard client recv failed: {exc}")
                    break
                if not chunk:
                    break
                buffer += chunk
                while b"\n" in buffer:
                    line, buffer = buffer.split(b"\n", 1)
                    if not line.strip():
                        continue
                    try:
                        command = json.loads(line.decode("utf-8"))
                    except json.JSONDecodeError as exc:
                        self.send(client, {"type": "status", "level": "error", "message": f"bad json: {exc}"})
                        continue
                    self.handle_command(command, client)
        finally:
            with self.lock:
                self.dashboard_clients.discard(client)
            try:
                client.close()
            except OSError:
                pass
            self.get_logger().info(f"dashboard client disconnected: {addr[0]}:{addr[1]}")

    def handle_command(self, command: dict[str, Any], client: socket.socket) -> None:
        kind = command.get("type")
        if kind == "hello":
            self.send(client, {"type": "status", "level": "info", "message": "bridge ready"})
            self.send(client, {"type": "geometry", "geometry": self.geometry})
            self.send(client, {"type": "nav2_params", "params": self.nav2_params})
            self.send(client, {"type": "imu", "imu": self.imu_settings})
            self.send(client, {"type": "turn_params", "params": self.turn_params})
            return
        if kind == "reset_odom":
            self.reset_local_origin()
            return
        if kind == "set_geometry":
            self.set_geometry(command.get("geometry", {}))
            return
        if kind == "set_nav2_params":
            self.set_nav2_params(command.get("params", {}))
            return
        if kind == "set_imu":
            self.set_imu_settings(command.get("imu", {}))
            return
        if kind == "set_turn_params":
            self.set_turn_params(command.get("params", {}))
            return
        if kind == "stop":
            self.cancel_requested = True
            self.publish_stop()
            self.broadcast({"type": "mission_status", "state": "stopped", "message": "stop requested"})
            return
        if kind == "mission":
            waypoints = command.get("waypoints", [])
            self.start_mission(waypoints)
            return
        self.send(client, {"type": "status", "level": "error", "message": f"unknown command: {kind}"})

    def load_geometry(self) -> dict[str, float]:
        default = {
            "wheel_diameter_m": 0.04586,
            "wheel_base_m": 0.15216,
            "counts_per_rev": 1400,
            "max_rpm": 315,
        }
        try:
            data = json.loads(RUNTIME_GEOMETRY_PATH.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            return default

        for key, value in default.items():
            try:
                default[key] = float(data.get(key, value))
            except (TypeError, ValueError):
                pass
        return default

    def set_geometry(self, geometry: dict[str, Any]) -> None:
        try:
            updated = {
                "wheel_diameter_m": max(0.001, float(geometry["wheel_diameter_m"])),
                "wheel_base_m": max(0.001, float(geometry["wheel_base_m"])),
                "counts_per_rev": max(1, int(float(geometry["counts_per_rev"]))),
                "max_rpm": max(1, int(float(geometry["max_rpm"]))),
                "updated_at": time.strftime("%Y-%m-%d %H:%M:%S"),
                "note": "Runtime bridge geometry. Bridge scales /odom and TF for Nav2; ESP32 control constants still require rebuild/upload.",
            }
        except (KeyError, TypeError, ValueError):
            self.broadcast({"type": "status", "level": "error", "message": "invalid geometry values"})
            return

        self.geometry = updated
        RUNTIME_GEOMETRY_PATH.parent.mkdir(parents=True, exist_ok=True)
        RUNTIME_GEOMETRY_PATH.write_text(json.dumps(updated, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        self.broadcast({"type": "geometry", "geometry": updated})
        self.broadcast({"type": "status", "level": "info", "message": "runtime geometry saved; bridge /odom scaling updated"})

    def load_imu_settings(self) -> dict[str, Any]:
        settings = dict(IMU_DEFAULTS)
        try:
            data = json.loads(RUNTIME_IMU_PATH.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            return settings

        for key, value in IMU_DEFAULTS.items():
            if key not in data:
                continue
            try:
                if isinstance(value, bool):
                    settings[key] = bool(data[key])
                elif isinstance(value, int):
                    settings[key] = int(data[key])
                elif isinstance(value, float):
                    settings[key] = float(data[key])
                else:
                    settings[key] = str(data[key])
            except (TypeError, ValueError):
                pass
        return self.validate_imu_settings(settings)

    def set_imu_settings(self, imu: dict[str, Any]) -> None:
        try:
            updated = dict(self.imu_settings)
            for key in IMU_DEFAULTS:
                if key not in imu:
                    continue
                default = IMU_DEFAULTS[key]
                if isinstance(default, bool):
                    updated[key] = bool(imu[key])
                elif isinstance(default, int):
                    updated[key] = int(float(imu[key]))
                elif isinstance(default, float):
                    updated[key] = float(imu[key])
                else:
                    updated[key] = str(imu[key]).strip() or str(default)
            updated = self.validate_imu_settings(updated)
            updated["updated_at"] = time.strftime("%Y-%m-%d %H:%M:%S")
            updated["note"] = "Runtime yaw fusion settings from Euroboot dashboard."
        except (TypeError, ValueError):
            self.broadcast({"type": "status", "level": "error", "message": "invalid Pixhawk yaw settings"})
            return

        old_port = str(self.imu_settings.get("pixhawk_port", IMU_DEFAULTS["pixhawk_port"]))
        old_baud = int(self.imu_settings.get("pixhawk_baud", IMU_DEFAULTS["pixhawk_baud"]))
        self.imu_settings = {key: updated[key] for key in IMU_DEFAULTS}
        RUNTIME_IMU_PATH.parent.mkdir(parents=True, exist_ok=True)
        RUNTIME_IMU_PATH.write_text(json.dumps(updated, indent=2, sort_keys=True) + "\n", encoding="utf-8")

        new_port = str(self.imu_settings["pixhawk_port"])
        new_baud = int(self.imu_settings["pixhawk_baud"])
        if new_port != old_port or new_baud != old_baud:
            self.pixhawk_reader.update_port(new_port, new_baud)

        with self.lock:
            self.rebase_origin_locked()

        self.broadcast({"type": "imu", "imu": self.imu_settings})
        mode = "Pixhawk yaw fusion enabled" if self.imu_settings["use_pixhawk_yaw"] else "encoder yaw only"
        self.broadcast({"type": "status", "level": "info", "message": mode})

    def validate_imu_settings(self, settings: dict[str, Any]) -> dict[str, Any]:
        pixhawk_weight = min(1.0, max(0.0, float(settings["pixhawk_weight"])))
        encoder_weight = min(1.0, max(0.0, float(settings["encoder_weight"])))
        if pixhawk_weight + encoder_weight <= 0.001:
            pixhawk_weight = 0.80
            encoder_weight = 0.20
        mode = str(settings.get("pixhawk_yaw_mode", "gyro")).strip().lower()
        if mode not in {"gyro", "attitude"}:
            mode = "gyro"
        sign = -1.0 if float(settings.get("pixhawk_yaw_sign", -1.0)) < 0.0 else 1.0
        return {
            "use_pixhawk_yaw": bool(settings["use_pixhawk_yaw"]),
            "pixhawk_weight": pixhawk_weight,
            "encoder_weight": encoder_weight,
            "pixhawk_yaw_mode": mode,
            "pixhawk_yaw_sign": sign,
            "pixhawk_port": str(settings["pixhawk_port"]).strip() or str(IMU_DEFAULTS["pixhawk_port"]),
            "pixhawk_baud": max(1200, int(settings["pixhawk_baud"])),
            "pixhawk_timeout_s": min(5.0, max(0.10, float(settings["pixhawk_timeout_s"]))),
        }

    def load_nav2_params(self) -> dict[str, float]:
        params = dict(NAV2_PARAM_DEFAULTS)
        try:
            data = json.loads(RUNTIME_NAV2_PATH.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            return params

        for key, value in NAV2_PARAM_DEFAULTS.items():
            try:
                params[key] = float(data.get(key, value))
            except (TypeError, ValueError):
                pass
        return params

    def set_nav2_params(self, params: dict[str, Any]) -> None:
        try:
            updated = dict(self.nav2_params)
            for key in NAV2_PARAM_DEFAULTS:
                if key in params:
                    updated[key] = float(params[key])
            updated = self.validate_nav2_params(updated)
            updated["updated_at"] = time.strftime("%Y-%m-%d %H:%M:%S")
            updated["note"] = "Runtime Nav2 controller parameters from Euroboot dashboard."
        except (TypeError, ValueError):
            self.broadcast({"type": "status", "level": "error", "message": "invalid Nav2 parameter values"})
            return

        self.nav2_params = {key: float(updated[key]) for key in NAV2_PARAM_DEFAULTS}
        RUNTIME_NAV2_PATH.parent.mkdir(parents=True, exist_ok=True)
        RUNTIME_NAV2_PATH.write_text(json.dumps(updated, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        self.broadcast({"type": "nav2_params", "params": self.nav2_params})
        threading.Thread(target=self.apply_nav2_params, args=(dict(self.nav2_params),), daemon=True).start()

    def validate_nav2_params(self, params: dict[str, float]) -> dict[str, float]:
        params["desired_linear_vel"] = min(0.35, max(0.04, params["desired_linear_vel"]))
        params["lookahead_dist"] = min(0.60, max(0.05, params["lookahead_dist"]))
        params["min_lookahead_dist"] = min(params["lookahead_dist"], max(0.03, params["min_lookahead_dist"]))
        params["max_lookahead_dist"] = max(params["lookahead_dist"], min(0.80, params["max_lookahead_dist"]))
        params["lookahead_time"] = min(1.50, max(0.10, params["lookahead_time"]))
        params["min_approach_linear_velocity"] = min(0.15, max(0.01, params["min_approach_linear_velocity"]))
        params["approach_velocity_scaling_dist"] = min(0.50, max(0.05, params["approach_velocity_scaling_dist"]))
        params["regulated_linear_scaling_min_radius"] = min(1.00, max(0.10, params["regulated_linear_scaling_min_radius"]))
        params["regulated_linear_scaling_min_speed"] = min(0.20, max(0.01, params["regulated_linear_scaling_min_speed"]))
        params["max_angular_accel"] = min(6.00, max(0.50, params["max_angular_accel"]))
        params["xy_goal_tolerance"] = min(0.20, max(0.02, params["xy_goal_tolerance"]))
        params["yaw_goal_tolerance"] = min(6.28, max(0.05, params["yaw_goal_tolerance"]))
        return params

    def apply_nav2_params(self, params: dict[str, float]) -> None:
        failures: list[str] = []
        param_node = rclpy.create_node("euroboot_nav2_param_setter")
        try:
            client = AsyncParameterClient(param_node, "/controller_server")
            if not client.wait_for_services(timeout_sec=8.0):
                failures.append("/controller_server parameter service unavailable")
            else:
                ros_params = [
                    Parameter(NAV2_PARAM_TARGETS[key], Parameter.Type.DOUBLE, float(params[key]))
                    for key in NAV2_PARAM_TARGETS
                ]
                future = client.set_parameters(ros_params)
                executor = SingleThreadedExecutor()
                executor.add_node(param_node)
                executor.spin_until_future_complete(future, timeout_sec=10.0)
                executor.remove_node(param_node)
                if not future.done():
                    failures.append("parameter request timed out")
                else:
                    response = future.result()
                    results = getattr(response, "results", []) if response is not None else []
                    for key, result in zip(NAV2_PARAM_TARGETS, results):
                        if not result.successful:
                            failures.append(f"{key}: {result.reason}")
        except Exception as exc:
            failures.append(str(exc))
        finally:
            param_node.destroy_node()

        if failures:
            self.get_logger().warn("Nav2 parameter apply completed with failures: " + "; ".join(failures))
            self.broadcast({"type": "status", "level": "error", "message": "some Nav2 parameters could not be applied"})
        else:
            self.get_logger().info("Nav2 runtime parameters applied")
            self.broadcast({"type": "status", "level": "info", "message": "Nav2 parameters applied"})

    def load_turn_params(self) -> dict[str, float]:
        params = dict(TURN_PARAM_DEFAULTS)
        try:
            data = json.loads(RUNTIME_TURN_PATH.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            return params

        for key, value in TURN_PARAM_DEFAULTS.items():
            try:
                params[key] = float(data.get(key, value))
            except (TypeError, ValueError):
                pass
        return self.validate_turn_params(params)

    def set_turn_params(self, params: dict[str, Any]) -> None:
        try:
            updated = dict(self.turn_params)
            for key in TURN_PARAM_DEFAULTS:
                if key in params:
                    updated[key] = float(params[key])
            updated = self.validate_turn_params(updated)
            stored = dict(updated)
            stored["updated_at"] = time.strftime("%Y-%m-%d %H:%M:%S")
            stored["note"] = "Runtime in-place turn controller parameters from Euroboot tuning tools."
        except (TypeError, ValueError):
            self.broadcast({"type": "status", "level": "error", "message": "invalid turn parameter values"})
            return

        self.turn_params = updated
        RUNTIME_TURN_PATH.parent.mkdir(parents=True, exist_ok=True)
        RUNTIME_TURN_PATH.write_text(json.dumps(stored, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        self.broadcast({"type": "turn_params", "params": self.turn_params})
        self.broadcast({"type": "status", "level": "info", "message": "turn parameters applied"})

    def validate_turn_params(self, params: dict[str, float]) -> dict[str, float]:
        params["preturn_heading_error_rad"] = min(math.pi, max(0.05, params["preturn_heading_error_rad"]))
        params["preturn_min_distance_m"] = min(0.50, max(0.00, params["preturn_min_distance_m"]))
        params["final_position_correction_enabled"] = 1.0 if params["final_position_correction_enabled"] >= 0.5 else 0.0
        params["turn_timeout_s"] = min(15.0, max(1.0, params["turn_timeout_s"]))
        params["turn_yaw_tolerance_rad"] = min(0.25, max(0.005, params["turn_yaw_tolerance_rad"]))
        params["turn_stable_samples"] = int(min(10, max(1, round(params["turn_stable_samples"]))))
        params["turn_min_angular_speed"] = min(0.80, max(0.04, params["turn_min_angular_speed"]))
        params["turn_max_angular_speed"] = min(2.50, max(params["turn_min_angular_speed"], params["turn_max_angular_speed"]))
        params["turn_kp"] = min(5.0, max(0.20, params["turn_kp"]))
        params["turn_linear_balance_kp"] = min(3.0, max(0.0, params["turn_linear_balance_kp"]))
        params["turn_linear_balance_limit_mps"] = min(0.12, max(0.0, params["turn_linear_balance_limit_mps"]))
        params["turn_settle_s"] = min(0.30, max(0.00, params["turn_settle_s"]))
        return params

    def reset_local_origin(self) -> None:
        with self.lock:
            self.rebase_origin_locked()
        self.get_logger().info("dashboard odom origin reset")
        self.broadcast({"type": "status", "level": "info", "message": "dashboard odom origin reset"})

    def rebase_origin_locked(self) -> None:
        """Reset local odom while keeping encoder x/y and yaw in one frame.

        The ESP32 publishes x/y integrated in its encoder odom frame. Pixhawk yaw
        is useful as a relative heading correction, but its absolute angle must
        not become the global yaw for those encoder x/y coordinates after a
        dashboard reset. Otherwise local waypoints are rotated into the wrong
        raw frame when the Pixhawk and encoder absolute yaw differ.
        """
        if self.encoder_pose is not None:
            x = self.encoder_pose.x
            y = self.encoder_pose.y
            yaw = self.encoder_pose.yaw
            self.encoder_origin_yaw = yaw
        elif self.raw_pose is not None:
            x = self.raw_pose.x
            y = self.raw_pose.y
            yaw = self.raw_pose.yaw
            self.encoder_origin_yaw = yaw
        else:
            return

        if self.raw_pose is not None:
            mismatch = abs(normalize_angle(self.raw_pose.yaw - yaw))
            if mismatch > 0.35:
                self.get_logger().warn(
                    f"rebasing odom with fused/encoder yaw mismatch={math.degrees(mismatch):.1f} deg"
                )

        self.origin = Pose2D(x, y, yaw)
        self.fused_origin_yaw = yaw
        pixhawk_yaw, _age, pixhawk_gyro_yaw = self.pixhawk_reader.get()
        self.pixhawk_origin_yaw = self.selected_pixhawk_yaw(pixhawk_yaw, pixhawk_gyro_yaw)

    def start_mission(self, waypoints: list[Any]) -> None:
        if not waypoints:
            self.broadcast({"type": "mission_status", "state": "idle", "message": "no waypoints to execute"})
            return
        if self.mission_thread and self.mission_thread.is_alive():
            self.broadcast({"type": "mission_status", "state": "busy", "message": "mission already running"})
            return

        parsed: list[MissionWaypoint] = []
        for item in waypoints:
            try:
                final_yaw_raw = item.get("final_yaw")
                final_yaw = None if final_yaw_raw in (None, "") else float(final_yaw_raw)
                parsed.append(
                    MissionWaypoint(
                        x=float(item["x"]),
                        y=float(item["y"]),
                        path_yaw=float(item.get("yaw", 0.0)),
                        final_yaw=final_yaw,
                    )
                )
            except (KeyError, TypeError, ValueError):
                self.broadcast({"type": "mission_status", "state": "error", "message": f"invalid waypoint: {item}"})
                return

        self.cancel_requested = False
        self.mission_thread = threading.Thread(target=self.run_mission, args=(parsed,), daemon=True)
        self.mission_thread.start()

    def run_mission(self, waypoints: list[MissionWaypoint]) -> None:
        self.get_logger().info(f"mission start: {len(waypoints)} waypoint(s)")
        if not self.nav_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("mission failed: follow_path server unavailable")
            self.broadcast({"type": "mission_status", "state": "error", "message": "Nav2 follow_path server unavailable"})
            return

        for index, local_wp in enumerate(waypoints, start=1):
            if self.cancel_requested:
                break

            with self.lock:
                raw_start = self.raw_pose or Pose2D(0.0, 0.0, 0.0)
                raw_wp = self.local_to_raw_locked(Pose2D(local_wp.x, local_wp.y, local_wp.path_yaw))

            path_yaw = math.atan2(raw_wp.y - raw_start.y, raw_wp.x - raw_start.x)
            heading_error = normalize_angle(path_yaw - raw_start.yaw)
            segment_distance = math.hypot(raw_wp.x - raw_start.x, raw_wp.y - raw_start.y)
            preturn_min_distance = float(self.turn_params["preturn_min_distance_m"])
            preturn_heading_error = float(self.turn_params["preturn_heading_error_rad"])
            if segment_distance > preturn_min_distance and abs(heading_error) > preturn_heading_error:
                self.broadcast(
                    {
                        "type": "mission_status",
                        "state": "turning",
                        "index": index,
                        "total": len(waypoints),
                        "message": f"aligning for waypoint {index}/{len(waypoints)}",
                    }
                )
                self.get_logger().info(f"turn before waypoint {index}: target_yaw={path_yaw:.3f}")
                if not self.rotate_to_heading(path_yaw):
                    self.publish_stop()
                    self.get_logger().warn(f"turn before waypoint {index} did not settle; continuing with Nav2")
                    self.broadcast(
                        {
                            "type": "mission_status",
                            "state": "navigating",
                            "index": index,
                            "total": len(waypoints),
                            "message": f"turn for waypoint {index} imperfect; continuing",
                        }
                    )

                with self.lock:
                    raw_start = self.raw_pose or raw_start
                    raw_wp = self.local_to_raw_locked(Pose2D(local_wp.x, local_wp.y, local_wp.path_yaw))

            goal = FollowPath.Goal()
            goal.path = self.make_straight_path(raw_start, raw_wp)
            goal.controller_id = "FollowPath"
            goal.goal_checker_id = "goal_checker"
            goal.progress_checker_id = "progress_checker"

            self.broadcast(
                {
                    "type": "mission_status",
                    "state": "navigating",
                    "index": index,
                    "total": len(waypoints),
                    "message": f"going to waypoint {index}/{len(waypoints)}",
                }
            )
            self.get_logger().info(f"follow path for waypoint {index}: poses={len(goal.path.poses)}")

            if not self.follow_waypoint(goal, raw_wp, index, len(waypoints)):
                return

            if local_wp.final_yaw is not None:
                with self.lock:
                    raw_final = self.local_to_raw_locked(Pose2D(local_wp.x, local_wp.y, local_wp.final_yaw))
                self.broadcast(
                    {
                        "type": "mission_status",
                        "state": "turning",
                        "index": index,
                        "total": len(waypoints),
                        "message": f"final alignment at waypoint {index}/{len(waypoints)}",
                    }
                )
                self.get_logger().info(f"final alignment at waypoint {index}: target_yaw={raw_final.yaw:.3f}")
                if not self.rotate_to_heading(raw_final.yaw):
                    self.publish_stop()
                    self.get_logger().error(f"mission failed: final alignment at waypoint {index} timed out")
                    self.broadcast({"type": "mission_status", "state": "error", "message": f"final alignment at waypoint {index} timed out"})
                    return
                final_position_error = self.distance_to_raw_target(raw_final)
                if (
                    float(self.turn_params["final_position_correction_enabled"]) >= 0.5
                    and final_position_error > FINAL_POSITION_GUARD_TOLERANCE_M
                ):
                    with self.lock:
                        raw_start = self.raw_pose or raw_final
                        raw_final = self.local_to_raw_locked(Pose2D(local_wp.x, local_wp.y, local_wp.final_yaw))
                    correction_goal = FollowPath.Goal()
                    correction_goal.path = self.make_straight_path(raw_start, raw_final)
                    correction_goal.controller_id = "FollowPath"
                    correction_goal.goal_checker_id = "goal_checker"
                    correction_goal.progress_checker_id = "progress_checker"
                    self.broadcast(
                        {
                            "type": "mission_status",
                            "state": "navigating",
                            "index": index,
                            "total": len(waypoints),
                            "message": f"correcting final position at waypoint {index}/{len(waypoints)}",
                        }
                    )
                    if not self.follow_waypoint(
                        correction_goal,
                        raw_final,
                        index,
                        len(waypoints),
                        guard_tolerance=FINAL_POSITION_GUARD_TOLERANCE_M,
                    ):
                        return
                    with self.lock:
                        raw_final = self.local_to_raw_locked(Pose2D(local_wp.x, local_wp.y, local_wp.final_yaw))
                    self.rotate_to_heading(raw_final.yaw)
                elif final_position_error > FINAL_POSITION_GUARD_TOLERANCE_M:
                    self.get_logger().warn(
                        f"final alignment at waypoint {index} drifted {final_position_error:.3f} m; "
                        "skipping same-waypoint correction"
                    )
                    self.broadcast(
                        {
                            "type": "mission_status",
                            "state": "reached",
                            "index": index,
                            "total": len(waypoints),
                            "message": f"waypoint {index} turn drift {final_position_error:.2f} m; next leg starts from actual pose",
                        }
                    )

            self.broadcast(
                {
                    "type": "mission_status",
                    "state": "reached",
                    "index": index,
                    "total": len(waypoints),
                    "message": f"reached waypoint {index}/{len(waypoints)}",
                }
            )

        self.publish_stop()
        self.get_logger().info("mission complete")
        self.broadcast({"type": "mission_status", "state": "done", "message": "mission complete"})

    def follow_waypoint(
        self,
        goal: FollowPath.Goal,
        target: Pose2D,
        index: int,
        total: int,
        guard_tolerance: float = WAYPOINT_GUARD_TOLERANCE_M,
    ) -> bool:
        for attempt in range(1, 3):
            send_future = self.nav_client.send_goal_async(goal)
            handle = self.wait_for_future(send_future, timeout_s=10.0)
            if handle is None or not handle.accepted:
                self.get_logger().error(f"mission failed: waypoint {index} rejected")
                self.broadcast({"type": "mission_status", "state": "error", "message": f"waypoint {index} rejected"})
                self.publish_stop()
                return False

            result_future = handle.get_result_async()
            while rclpy.ok() and not result_future.done():
                if self.cancel_requested:
                    handle.cancel_goal_async()
                    self.publish_stop()
                    self.broadcast({"type": "mission_status", "state": "stopped", "message": "mission cancelled"})
                    return False
                time.sleep(0.05)

            result = result_future.result()
            status = result.status if result else GoalStatus.STATUS_UNKNOWN
            if status != GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().error(f"mission failed: waypoint {index} ended with status {status}")
                self.broadcast({"type": "mission_status", "state": "error", "message": f"waypoint {index} ended with status {status}"})
                self.publish_stop()
                return False

            distance = self.distance_to_raw_target(target)
            if distance <= guard_tolerance:
                return True

            self.get_logger().warn(
                f"Nav2 reported waypoint {index} reached early: distance={distance:.3f} m, attempt={attempt}"
            )
            self.broadcast(
                {
                    "type": "mission_status",
                    "state": "navigating",
                    "index": index,
                    "total": total,
                    "message": f"retrying waypoint {index}, distance {distance:.2f} m",
                }
            )
            self.publish_stop()
            time.sleep(0.15)

            with self.lock:
                raw_start = self.raw_pose or target
            goal.path = self.make_straight_path(raw_start, target)

        distance = self.distance_to_raw_target(target)
        self.get_logger().error(f"mission failed: waypoint {index} still {distance:.3f} m away after retry")
        self.broadcast({"type": "mission_status", "state": "error", "message": f"waypoint {index} still {distance:.2f} m away"})
        self.publish_stop()
        return False

    def distance_to_raw_target(self, target: Pose2D) -> float:
        with self.lock:
            pose = self.raw_pose
        if pose is None:
            return float("inf")
        return math.hypot(target.x - pose.x, target.y - pose.y)

    def rotate_to_heading(self, target_yaw: float) -> bool:
        params = dict(self.turn_params)
        deadline = time.monotonic() + float(params["turn_timeout_s"])
        tolerance = float(params["turn_yaw_tolerance_rad"])
        stable_samples = int(params["turn_stable_samples"])
        min_angular = float(params["turn_min_angular_speed"])
        max_angular = float(params["turn_max_angular_speed"])
        kp = float(params["turn_kp"])
        balance_kp = float(params["turn_linear_balance_kp"])
        balance_limit = float(params["turn_linear_balance_limit_mps"])
        settle_s = float(params["turn_settle_s"])
        stable_count = 0
        last_error = math.pi
        self.publish_stop()
        time.sleep(0.08)
        while rclpy.ok() and time.monotonic() < deadline and not self.cancel_requested:
            with self.lock:
                pose = self.raw_pose
                actual_linear_x = self.last_actual_linear_x
            if pose is None:
                time.sleep(0.03)
                continue

            error = normalize_angle(target_yaw - pose.yaw)
            last_error = error
            if abs(error) < tolerance:
                self.cmd_pub.publish(Twist())
                stable_count += 1
                if stable_count >= stable_samples:
                    self.publish_stop()
                    time.sleep(settle_s)
                    return True
                time.sleep(0.03)
                continue
            else:
                stable_count = 0

            cmd = Twist()
            angular = max(min_angular, min(max_angular, abs(error) * kp))
            cmd.angular.z = math.copysign(angular, error)
            if balance_kp > 0.0 and balance_limit > 0.0:
                cmd.linear.x = max(-balance_limit, min(balance_limit, -balance_kp * actual_linear_x))
            self.cmd_pub.publish(cmd)
            time.sleep(0.03)

        self.publish_stop()
        if abs(last_error) < 0.18:
            self.get_logger().warn(f"turn timed out but continuing with small error={last_error:.3f} rad")
            return True
        self.get_logger().error(f"turn timed out with error={last_error:.3f} rad")
        return False

    def make_straight_path(self, start: Pose2D, goal: Pose2D) -> NavPath:
        path = NavPath()
        path.header.frame_id = "odom"
        path.header.stamp = self.get_clock().now().to_msg()

        dx = goal.x - start.x
        dy = goal.y - start.y
        distance = math.hypot(dx, dy)
        path_yaw = math.atan2(dy, dx) if distance > 0.02 else goal.yaw
        steps = max(2, int(math.ceil(distance / 0.05)) + 1)

        for i in range(steps):
            ratio = i / (steps - 1)
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = start.x + dx * ratio
            pose.pose.position.y = start.y + dy * ratio
            pose.pose.position.z = 0.0
            pose.pose.orientation = quat_from_yaw(path_yaw)
            path.poses.append(pose)

        return path

    def wait_for_future(self, future: Any, timeout_s: float) -> Any:
        deadline = time.monotonic() + timeout_s
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        if not future.done():
            return None
        return future.result()

    def publish_stop(self) -> None:
        for _ in range(8):
            try:
                self.cmd_pub.publish(Twist())
            except Exception:
                return
            time.sleep(0.02)

    def send(self, client: socket.socket, message: dict[str, Any]) -> None:
        data = (json.dumps(message, separators=(",", ":")) + "\n").encode("utf-8")
        try:
            client.sendall(data)
        except OSError:
            with self.lock:
                self.dashboard_clients.discard(client)

    def broadcast(self, message: dict[str, Any]) -> None:
        with self.lock:
            clients = list(self.dashboard_clients)
        for client in clients:
            self.send(client, message)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8765)
    args = parser.parse_args()

    rclpy.init()
    node = DashboardBridge(args.host, args.port)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.shutdown_requested = True
        node.publish_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
