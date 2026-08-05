#!/usr/bin/env python3
"""ROS 2 to TCP bridge for the Euroboot dashboard.

Run this on the Raspberry Pi after the micro-ROS agent, odom TF bridge, and
Nav2 are active. The Windows dashboard connects to this bridge with plain
newline-delimited JSON, so the dashboard does not need ROS installed locally.
"""

from __future__ import annotations

import argparse
import json
import math
import socket
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
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from tf2_ros import TransformBroadcaster


RUNTIME_GEOMETRY_PATH = Path("/home/maker/euroboot/config/euroboot_runtime_geometry.json")
RUNTIME_NAV2_PATH = Path("/home/maker/euroboot/config/euroboot_runtime_nav2.json")

NAV2_PARAM_DEFAULTS = {
    "desired_linear_vel": 0.22,
    "lookahead_dist": 0.14,
    "min_lookahead_dist": 0.07,
    "max_lookahead_dist": 0.28,
    "lookahead_time": 0.45,
    "min_approach_linear_velocity": 0.05,
    "approach_velocity_scaling_dist": 0.20,
    "regulated_linear_scaling_min_radius": 0.32,
    "regulated_linear_scaling_min_speed": 0.05,
    "max_angular_accel": 3.40,
    "xy_goal_tolerance": 0.06,
    "yaw_goal_tolerance": 6.28,
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

PRETURN_HEADING_ERROR_RAD = 1.20
PRETURN_MIN_DISTANCE_M = 0.08
WAYPOINT_GUARD_TOLERANCE_M = 0.08


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


class DashboardBridge(Node):
    def __init__(self, host: str, port: int) -> None:
        super().__init__("euroboot_dashboard_bridge")
        self.host = host
        self.port = port
        self.odom_sub = self.create_subscription(Odometry, "/odom/unfiltered", self.odom_cb, 30)
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.nav_client = ActionClient(self, FollowPath, "follow_path")
        self.tf_broadcaster = TransformBroadcaster(self)

        self.lock = threading.RLock()
        self.dashboard_clients: set[socket.socket] = set()
        self.raw_pose: Pose2D | None = None
        self.origin: Pose2D | None = None
        self.geometry = self.load_geometry()
        self.nav2_params = self.load_nav2_params()
        self.mission_thread: threading.Thread | None = None
        self.cancel_requested = False
        self.shutdown_requested = False
        self.last_broadcast_t = 0.0
        self.last_tf_t = 0.0

        self.server_thread = threading.Thread(target=self.server_loop, daemon=True)
        self.server_thread.start()
        self.get_logger().info(f"Euroboot dashboard bridge listening on {host}:{port}")

    def odom_cb(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        pose = Pose2D(p.x, p.y, yaw_from_quat(msg.pose.pose.orientation))

        with self.lock:
            self.raw_pose = pose
            if self.origin is None:
                self.origin = Pose2D(pose.x, pose.y, pose.yaw)
            local = self.raw_to_local_locked(pose)

        now = time.monotonic()
        if now - self.last_tf_t >= 0.05:
            self.last_tf_t = now
            self.broadcast_tf(msg)

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
                "linear_x": msg.twist.twist.linear.x,
                "angular_z": msg.twist.twist.angular.z,
            }
        )

    def broadcast_tf(self, msg: Odometry) -> None:
        tf = TransformStamped()
        tf.header = msg.header
        tf.header.frame_id = msg.header.frame_id or "odom"
        tf.child_frame_id = msg.child_frame_id or "base_footprint"
        tf.transform.translation.x = msg.pose.pose.position.x
        tf.transform.translation.y = msg.pose.pose.position.y
        tf.transform.translation.z = msg.pose.pose.position.z
        tf.transform.rotation = msg.pose.pose.orientation
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
                "note": "Runtime dashboard/bridge geometry. ESP32 firmware constants still require rebuild/upload.",
            }
        except (KeyError, TypeError, ValueError):
            self.broadcast({"type": "status", "level": "error", "message": "invalid geometry values"})
            return

        self.geometry = updated
        RUNTIME_GEOMETRY_PATH.parent.mkdir(parents=True, exist_ok=True)
        RUNTIME_GEOMETRY_PATH.write_text(json.dumps(updated, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        self.broadcast({"type": "geometry", "geometry": updated})
        self.broadcast({"type": "status", "level": "info", "message": "runtime geometry saved on robot"})

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

    def reset_local_origin(self) -> None:
        with self.lock:
            if self.raw_pose is not None:
                self.origin = Pose2D(self.raw_pose.x, self.raw_pose.y, self.raw_pose.yaw)
        self.get_logger().info("dashboard odom origin reset")
        self.broadcast({"type": "status", "level": "info", "message": "dashboard odom origin reset"})

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
            if segment_distance > PRETURN_MIN_DISTANCE_M and abs(heading_error) > PRETURN_HEADING_ERROR_RAD:
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

    def follow_waypoint(self, goal: FollowPath.Goal, target: Pose2D, index: int, total: int) -> bool:
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
            if distance <= WAYPOINT_GUARD_TOLERANCE_M:
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
        deadline = time.monotonic() + 7.0
        stable_count = 0
        last_error = math.pi
        while rclpy.ok() and time.monotonic() < deadline and not self.cancel_requested:
            with self.lock:
                pose = self.raw_pose
            if pose is None:
                time.sleep(0.03)
                continue

            error = normalize_angle(target_yaw - pose.yaw)
            last_error = error
            if abs(error) < 0.030:
                self.cmd_pub.publish(Twist())
                stable_count += 1
                if stable_count >= 3:
                    self.publish_stop()
                    time.sleep(0.04)
                    return True
                time.sleep(0.03)
                continue
            else:
                stable_count = 0

            cmd = Twist()
            angular = max(0.45, min(2.20, abs(error) * 3.2))
            cmd.angular.z = math.copysign(angular, error)
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
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown_requested = True
        node.publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
