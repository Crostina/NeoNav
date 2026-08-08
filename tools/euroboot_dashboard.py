#!/usr/bin/env python3
"""Euroboot dashboard.

The dashboard can run offline for UI work, or connect to
tools/euroboot_ros_bridge.py on the Raspberry Pi for live odometry and Nav2
waypoint missions.
"""

from __future__ import annotations

import json
import math
import queue
import re
import socket
import threading
import tkinter as tk
import csv
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from tkinter import ttk
from typing import Any


PROJECT_ROOT = Path(__file__).resolve().parents[1]
CONFIG_PATH = PROJECT_ROOT / "config" / "custom" / "euroboot_esp32_config.h"
DASHBOARD_SETTINGS_PATH = PROJECT_ROOT / "config" / "euroboot_dashboard_settings.json"
DEBUG_DIR = PROJECT_ROOT / "debug_runs"

DEFAULT_NAV2_PARAMS = {
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

DEFAULT_IMU_SETTINGS = {
    "use_pixhawk_yaw": True,
    "pixhawk_weight": 0.80,
    "encoder_weight": 0.20,
    "pixhawk_yaw_mode": "gyro",
    "pixhawk_yaw_sign": -1.0,
    "pixhawk_port": "/dev/serial0",
    "pixhawk_baud": 115200,
    "pixhawk_timeout_s": 0.75,
}


def read_define(name: str, default: float) -> float:
    if not CONFIG_PATH.exists():
        return default

    pattern = re.compile(rf"^\s*#define\s+{re.escape(name)}\s+(.+?)\s*$")
    for line in CONFIG_PATH.read_text(encoding="utf-8").splitlines():
        match = pattern.match(line)
        if not match:
            continue
        raw_value = match.group(1).split("//", 1)[0].strip()
        try:
            return float(raw_value)
        except ValueError:
            return default
    return default


def load_dashboard_settings() -> dict:
    try:
        return json.loads(DASHBOARD_SETTINGS_PATH.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return {}


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def differential_wheel_speeds(linear_x: float, angular_z: float, wheel_base_m: float) -> tuple[float, float]:
    half_track = wheel_base_m * 0.5
    return linear_x - angular_z * half_track, linear_x + angular_z * half_track


@dataclass
class RobotGeometry:
    wheel_diameter_m: float
    wheel_base_m: float
    counts_per_rev: int
    max_rpm: int

    @property
    def wheel_radius_m(self) -> float:
        return self.wheel_diameter_m / 2.0

    @property
    def circumference_m(self) -> float:
        return math.pi * self.wheel_diameter_m

    @property
    def body_length_m(self) -> float:
        return max(0.18, self.wheel_base_m * 1.35)

    @property
    def body_width_m(self) -> float:
        return self.wheel_base_m + self.wheel_diameter_m * 1.2


@dataclass
class RobotPose:
    x_m: float = 0.0
    y_m: float = 0.0
    yaw_rad: float = 0.0


@dataclass
class Waypoint:
    x_m: float
    y_m: float
    final_yaw_deg: float | None = None
    drive_mode: str = "forward"


DRIVE_MODES = ("forward", "backward")


def normalize_drive_mode(value: Any) -> str:
    text = str(value or "forward").strip().lower()
    if text in {"back", "backward", "reverse", "rev"}:
        return "backward"
    return "forward"


def segment_robot_yaw(start: tuple[float, float], end: tuple[float, float], drive_mode: str) -> float:
    travel_yaw = math.atan2(end[1] - start[1], end[0] - start[0])
    if normalize_drive_mode(drive_mode) == "backward":
        return normalize_angle(travel_yaw + math.pi)
    return travel_yaw


class BridgeClient:
    def __init__(self, inbox: queue.Queue[dict]) -> None:
        self.inbox = inbox
        self.sock: socket.socket | None = None
        self.reader_thread: threading.Thread | None = None
        self.lock = threading.Lock()
        self.running = False

    @property
    def connected(self) -> bool:
        return self.sock is not None and self.running

    def connect(self, host: str, port: int) -> None:
        self.close()
        sock = socket.create_connection((host, port), timeout=4.0)
        sock.settimeout(0.5)
        self.sock = sock
        self.running = True
        self.reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self.reader_thread.start()
        self.send({"type": "hello"})

    def close(self) -> None:
        self.running = False
        with self.lock:
            sock = self.sock
            self.sock = None
        if sock is not None:
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            try:
                sock.close()
            except OSError:
                pass

    def send(self, message: dict) -> None:
        data = (json.dumps(message, separators=(",", ":")) + "\n").encode("utf-8")
        with self.lock:
            if self.sock is None:
                raise RuntimeError("dashboard bridge is not connected")
            self.sock.sendall(data)

    def _reader_loop(self) -> None:
        buffer = b""
        try:
            while self.running and self.sock is not None:
                try:
                    chunk = self.sock.recv(4096)
                except socket.timeout:
                    continue
                if not chunk:
                    break
                buffer += chunk
                while b"\n" in buffer:
                    line, buffer = buffer.split(b"\n", 1)
                    if not line.strip():
                        continue
                    try:
                        self.inbox.put(json.loads(line.decode("utf-8")))
                    except json.JSONDecodeError as exc:
                        self.inbox.put({"type": "status", "level": "error", "message": f"bad bridge json: {exc}"})
        except OSError as exc:
            if self.running:
                self.inbox.put({"type": "status", "level": "error", "message": f"bridge connection lost: {exc}"})
        finally:
            self.running = False
            self.inbox.put({"type": "connection", "connected": False})


class EurobootDashboard:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("Euroboot Dashboard")
        self.root.minsize(1180, 760)

        self.settings = load_dashboard_settings()
        saved_geometry = self.settings.get("geometry", {})
        self.geometry = RobotGeometry(
            wheel_diameter_m=float(saved_geometry.get("wheel_diameter_m", read_define("WHEEL_DIAMETER", 0.04586))),
            wheel_base_m=float(saved_geometry.get("wheel_base_m", read_define("LR_WHEELS_DISTANCE", 0.15216))),
            counts_per_rev=int(saved_geometry.get("counts_per_rev", int(read_define("COUNTS_PER_REV1", 1400)))),
            max_rpm=int(saved_geometry.get("max_rpm", int(read_define("MOTOR_MAX_RPM", 450) * read_define("MAX_RPM_RATIO", 0.70)))),
        )
        saved_nav2 = self.settings.get("nav2_params", {})
        self.nav2_params = {
            key: float(saved_nav2.get(key, value))
            for key, value in DEFAULT_NAV2_PARAMS.items()
        }
        saved_imu = self.settings.get("imu", {})
        self.imu_settings = {
            key: saved_imu.get(key, value)
            for key, value in DEFAULT_IMU_SETTINGS.items()
        }
        self.pose = RobotPose()
        self.trail: list[tuple[float, float]] = [(0.0, 0.0)]
        self.waypoints: list[Waypoint] = []
        self.drawn_track: list[tuple[float, float]] = []
        self.selected_waypoint_index: int | None = None
        self.drawing_track = False
        self.bridge_queue: queue.Queue[dict] = queue.Queue()
        self.bridge = BridgeClient(self.bridge_queue)

        self.add_waypoint_mode = tk.BooleanVar(value=True)
        self.draw_track_mode_var = tk.BooleanVar(value=False)
        self.save_debug_var = tk.BooleanVar(value=bool(self.settings.get("save_debug", False)))
        self.auto_corner_heading_var = tk.BooleanVar(value=bool(self.settings.get("auto_corner_heading", True)))
        self.use_pixhawk_yaw_var = tk.BooleanVar(value=bool(self.imu_settings.get("use_pixhawk_yaw", True)))
        self.scale_px_per_m = tk.DoubleVar(value=260.0)
        self.status_text = tk.StringVar(value="Offline. Start the Pi bridge, then connect.")
        self.connection_text = tk.StringVar(value="Disconnected")
        self.mission_text = tk.StringVar(value="Idle")
        self.speed_text = tk.StringVar(value="vx=0.000 m/s   wz=0.000 rad/s")
        self.debug_text = tk.StringVar(value="Debug idle")
        self.debug_csv_file = None
        self.debug_writer = None
        self.debug_csv_path: Path | None = None
        self.debug_meta_path: Path | None = None
        self.current_mission_state = "idle"

        self.vars = {
            "bridge_host": tk.StringVar(value=str(self.settings.get("bridge_host", "192.168.137.97"))),
            "bridge_port": tk.StringVar(value=str(self.settings.get("bridge_port", "8765"))),
            "wheel_diameter": tk.StringVar(value=f"{self.geometry.wheel_diameter_m:.5f}"),
            "wheel_base": tk.StringVar(value=f"{self.geometry.wheel_base_m:.5f}"),
            "counts_per_rev": tk.StringVar(value=str(self.geometry.counts_per_rev)),
            "max_rpm": tk.StringVar(value=str(self.geometry.max_rpm)),
            "nav2_desired_linear_vel": tk.StringVar(value=f"{self.nav2_params['desired_linear_vel']:.3f}"),
            "nav2_lookahead_dist": tk.StringVar(value=f"{self.nav2_params['lookahead_dist']:.3f}"),
            "nav2_min_lookahead_dist": tk.StringVar(value=f"{self.nav2_params['min_lookahead_dist']:.3f}"),
            "nav2_max_lookahead_dist": tk.StringVar(value=f"{self.nav2_params['max_lookahead_dist']:.3f}"),
            "nav2_lookahead_time": tk.StringVar(value=f"{self.nav2_params['lookahead_time']:.3f}"),
            "nav2_min_approach_linear_velocity": tk.StringVar(value=f"{self.nav2_params['min_approach_linear_velocity']:.3f}"),
            "nav2_approach_velocity_scaling_dist": tk.StringVar(value=f"{self.nav2_params['approach_velocity_scaling_dist']:.3f}"),
            "nav2_regulated_linear_scaling_min_radius": tk.StringVar(value=f"{self.nav2_params['regulated_linear_scaling_min_radius']:.3f}"),
            "nav2_regulated_linear_scaling_min_speed": tk.StringVar(value=f"{self.nav2_params['regulated_linear_scaling_min_speed']:.3f}"),
            "nav2_max_angular_accel": tk.StringVar(value=f"{self.nav2_params['max_angular_accel']:.3f}"),
            "nav2_xy_goal_tolerance": tk.StringVar(value=f"{self.nav2_params['xy_goal_tolerance']:.3f}"),
            "nav2_yaw_goal_tolerance": tk.StringVar(value=f"{self.nav2_params['yaw_goal_tolerance']:.3f}"),
            "imu_pixhawk_weight": tk.StringVar(value=f"{float(self.imu_settings.get('pixhawk_weight', 0.8)):.2f}"),
            "imu_encoder_weight": tk.StringVar(value=f"{float(self.imu_settings.get('encoder_weight', 0.2)):.2f}"),
            "imu_yaw_mode": tk.StringVar(value=str(self.imu_settings.get("pixhawk_yaw_mode", "gyro"))),
            "imu_yaw_sign": tk.StringVar(value=f"{float(self.imu_settings.get('pixhawk_yaw_sign', -1.0)):.0f}"),
            "imu_port": tk.StringVar(value=str(self.imu_settings.get("pixhawk_port", "/dev/serial0"))),
            "imu_baud": tk.StringVar(value=str(self.imu_settings.get("pixhawk_baud", 115200))),
            "imu_timeout": tk.StringVar(value=f"{float(self.imu_settings.get('pixhawk_timeout_s', 0.75)):.2f}"),
            "pose_x": tk.StringVar(value="0.000"),
            "pose_y": tk.StringVar(value="0.000"),
            "pose_yaw": tk.StringVar(value="0.0"),
            "waypoint_x": tk.StringVar(value=""),
            "waypoint_y": tk.StringVar(value=""),
            "waypoint_theta": tk.StringVar(value=""),
            "waypoint_drive_mode": tk.StringVar(value="forward"),
        }

        self._build_layout()
        self._bind_events()
        self.redraw()
        self.root.after(50, self.process_bridge_messages)
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    def _build_layout(self) -> None:
        style = ttk.Style()
        style.configure("Panel.TFrame", background="#f6f7f9")
        style.configure("Small.TLabel", foreground="#4a5568")

        self.root.columnconfigure(0, weight=1)
        self.root.columnconfigure(1, weight=0)
        self.root.rowconfigure(0, weight=1)

        canvas_frame = ttk.Frame(self.root)
        canvas_frame.grid(row=0, column=0, sticky="nsew")
        canvas_frame.columnconfigure(0, weight=1)
        canvas_frame.rowconfigure(0, weight=1)

        self.canvas = tk.Canvas(canvas_frame, background="#fbfcfd", highlightthickness=0)
        self.canvas.grid(row=0, column=0, sticky="nsew")

        bottom_bar = ttk.Frame(canvas_frame, padding=(12, 8))
        bottom_bar.grid(row=1, column=0, sticky="ew")
        bottom_bar.columnconfigure(0, weight=1)
        ttk.Label(bottom_bar, textvariable=self.status_text).grid(row=0, column=0, sticky="w")
        ttk.Label(bottom_bar, text="Zoom").grid(row=0, column=1, padx=(8, 4))
        ttk.Scale(
            bottom_bar,
            from_=120,
            to=520,
            orient="horizontal",
            variable=self.scale_px_per_m,
            command=lambda _value: self.redraw(),
            length=180,
        ).grid(row=0, column=2, sticky="e")

        side_shell = ttk.Frame(self.root, width=370, style="Panel.TFrame")
        side_shell.grid(row=0, column=1, sticky="ns")
        side_shell.grid_propagate(False)
        side_shell.rowconfigure(0, weight=1)
        side_shell.columnconfigure(0, weight=1)

        self.side_canvas = tk.Canvas(side_shell, width=350, background="#f6f7f9", highlightthickness=0)
        side_scrollbar = ttk.Scrollbar(side_shell, orient="vertical", command=self.side_canvas.yview)
        self.side_canvas.configure(yscrollcommand=side_scrollbar.set)
        self.side_canvas.grid(row=0, column=0, sticky="nsew")
        side_scrollbar.grid(row=0, column=1, sticky="ns")

        side = ttk.Frame(self.side_canvas, padding=14, style="Panel.TFrame")
        self.side_window = self.side_canvas.create_window((0, 0), window=side, anchor="nw")
        side.bind(
            "<Configure>",
            lambda event: self.side_canvas.configure(scrollregion=self.side_canvas.bbox("all")),
        )
        self.side_canvas.bind(
            "<Configure>",
            lambda event: self.side_canvas.itemconfigure(self.side_window, width=event.width),
        )
        self.side_canvas.bind_all("<MouseWheel>", self.on_mousewheel)
        side.columnconfigure(0, weight=1)

        ttk.Label(side, text="Euroboot", font=("Segoe UI", 18, "bold")).grid(row=0, column=0, sticky="w")
        ttk.Label(side, textvariable=self.connection_text, style="Small.TLabel").grid(row=1, column=0, sticky="w", pady=(0, 12))

        self._section(side, 2, "Bridge")
        self._field(side, 3, "Pi host", "bridge_host")
        self._field(side, 4, "TCP port", "bridge_port")
        bridge_buttons = ttk.Frame(side)
        bridge_buttons.grid(row=5, column=0, sticky="ew", pady=(8, 16))
        bridge_buttons.columnconfigure((0, 1), weight=1)
        ttk.Button(bridge_buttons, text="Connect", command=self.connect_bridge).grid(row=0, column=0, sticky="ew", padx=(0, 4))
        ttk.Button(bridge_buttons, text="Disconnect", command=self.disconnect_bridge).grid(row=0, column=1, sticky="ew", padx=(4, 0))

        self._section(side, 6, "Mission")
        ttk.Label(side, textvariable=self.mission_text, style="Small.TLabel").grid(row=7, column=0, sticky="w")
        mission_buttons = ttk.Frame(side)
        mission_buttons.grid(row=8, column=0, sticky="ew", pady=(8, 16))
        mission_buttons.columnconfigure((0, 1, 2), weight=1)
        ttk.Button(mission_buttons, text="Run", command=self.run_mission).grid(row=0, column=0, sticky="ew", padx=(0, 4))
        ttk.Button(mission_buttons, text="Stop", command=self.stop_robot).grid(row=0, column=1, sticky="ew", padx=4)
        ttk.Button(mission_buttons, text="Clear Odom", command=self.clear_robot_odom).grid(row=0, column=2, sticky="ew", padx=(4, 0))
        mission_options = ttk.Frame(side)
        mission_options.grid(row=9, column=0, sticky="ew")
        mission_options.columnconfigure((0, 1), weight=1)
        ttk.Checkbutton(mission_options, text="Save Debug Data", variable=self.save_debug_var, command=self.save_settings).grid(row=0, column=0, sticky="w")
        ttk.Checkbutton(mission_options, text="Auto Corner Heading", variable=self.auto_corner_heading_var, command=self.save_settings).grid(row=0, column=1, sticky="w")
        ttk.Label(side, textvariable=self.debug_text, style="Small.TLabel").grid(row=10, column=0, sticky="w", pady=(2, 14))

        self._section(side, 11, "Robot Geometry")
        self._field(side, 12, "Wheel diameter (m)", "wheel_diameter")
        self._field(side, 13, "Wheel distance (m)", "wheel_base")
        self._field(side, 14, "Encoder CPR", "counts_per_rev")
        self._field(side, 15, "Configured max RPM", "max_rpm")
        geometry_buttons = ttk.Frame(side)
        geometry_buttons.grid(row=16, column=0, sticky="ew", pady=(8, 12))
        geometry_buttons.columnconfigure((0, 1), weight=1)
        ttk.Button(geometry_buttons, text="Apply Local", command=self.apply_geometry).grid(row=0, column=0, sticky="ew", padx=(0, 4))
        ttk.Button(geometry_buttons, text="Apply To Robot", command=self.apply_geometry_to_robot).grid(row=0, column=1, sticky="ew", padx=(4, 0))

        self.geometry_summary = tk.StringVar()
        ttk.Label(side, textvariable=self.geometry_summary, style="Small.TLabel", justify="left").grid(row=17, column=0, sticky="ew", pady=(0, 14))

        self._section(side, 18, "Yaw Source")
        ttk.Checkbutton(
            side,
            text="Use Pixhawk yaw",
            variable=self.use_pixhawk_yaw_var,
            command=self.apply_imu_settings_local,
        ).grid(row=19, column=0, sticky="w")
        self._field(side, 20, "Pixhawk weight", "imu_pixhawk_weight", command=self.apply_imu_settings_local)
        self._field(side, 21, "Encoder weight", "imu_encoder_weight", command=self.apply_imu_settings_local)
        self._field(side, 22, "Yaw mode", "imu_yaw_mode", command=self.apply_imu_settings_local)
        self._field(side, 23, "Yaw sign", "imu_yaw_sign", command=self.apply_imu_settings_local)
        self._field(side, 24, "Pixhawk port", "imu_port", command=self.apply_imu_settings_local)
        self._field(side, 25, "Pixhawk baud", "imu_baud", command=self.apply_imu_settings_local)
        self._field(side, 26, "Timeout (s)", "imu_timeout", command=self.apply_imu_settings_local)
        imu_buttons = ttk.Frame(side)
        imu_buttons.grid(row=27, column=0, sticky="ew", pady=(8, 12))
        imu_buttons.columnconfigure((0, 1), weight=1)
        ttk.Button(imu_buttons, text="80/20 Defaults", command=self.load_default_imu_settings).grid(row=0, column=0, sticky="ew", padx=(0, 4))
        ttk.Button(imu_buttons, text="Apply To Robot", command=self.apply_imu_settings_to_robot).grid(row=0, column=1, sticky="ew", padx=(4, 0))

        self._section(side, 28, "Nav2 Tight-Box Tune")
        self._field(side, 29, "Speed (m/s)", "nav2_desired_linear_vel", command=self.apply_nav2_params_local)
        self._field(side, 30, "Lookahead (m)", "nav2_lookahead_dist", command=self.apply_nav2_params_local)
        self._field(side, 31, "Min lookahead (m)", "nav2_min_lookahead_dist", command=self.apply_nav2_params_local)
        self._field(side, 32, "Max lookahead (m)", "nav2_max_lookahead_dist", command=self.apply_nav2_params_local)
        self._field(side, 33, "Lookahead time (s)", "nav2_lookahead_time", command=self.apply_nav2_params_local)
        self._field(side, 34, "Approach speed", "nav2_min_approach_linear_velocity", command=self.apply_nav2_params_local)
        self._field(side, 35, "Approach dist", "nav2_approach_velocity_scaling_dist", command=self.apply_nav2_params_local)
        self._field(side, 36, "Turn radius scale", "nav2_regulated_linear_scaling_min_radius", command=self.apply_nav2_params_local)
        self._field(side, 37, "Min turn speed", "nav2_regulated_linear_scaling_min_speed", command=self.apply_nav2_params_local)
        self._field(side, 38, "Angular accel", "nav2_max_angular_accel", command=self.apply_nav2_params_local)
        self._field(side, 39, "XY tolerance", "nav2_xy_goal_tolerance", command=self.apply_nav2_params_local)
        self._field(side, 40, "Yaw tolerance", "nav2_yaw_goal_tolerance", command=self.apply_nav2_params_local)
        nav2_buttons = ttk.Frame(side)
        nav2_buttons.grid(row=41, column=0, sticky="ew", pady=(8, 12))
        nav2_buttons.columnconfigure((0, 1), weight=1)
        ttk.Button(nav2_buttons, text="Tight Defaults", command=self.load_tight_nav2_defaults).grid(row=0, column=0, sticky="ew", padx=(0, 4))
        ttk.Button(nav2_buttons, text="Apply To Robot", command=self.apply_nav2_params_to_robot).grid(row=0, column=1, sticky="ew", padx=(4, 0))

        self._section(side, 42, "Live Pose")
        self._field(side, 43, "x (m)", "pose_x")
        self._field(side, 44, "y (m)", "pose_y")
        self._field(side, 45, "yaw (deg)", "pose_yaw")
        ttk.Label(side, textvariable=self.speed_text, style="Small.TLabel").grid(row=46, column=0, sticky="w", pady=(2, 14))

        self._section(side, 47, "Waypoints")
        waypoint_modes = ttk.Frame(side)
        waypoint_modes.grid(row=48, column=0, sticky="ew", pady=(2, 8))
        waypoint_modes.columnconfigure((0, 1), weight=1)
        ttk.Checkbutton(waypoint_modes, text="Add Points", variable=self.add_waypoint_mode).grid(row=0, column=0, sticky="w")
        ttk.Checkbutton(waypoint_modes, text="Draw Track", variable=self.draw_track_mode_var).grid(row=0, column=1, sticky="w")
        waypoint_buttons = ttk.Frame(side)
        waypoint_buttons.grid(row=49, column=0, sticky="ew")
        waypoint_buttons.columnconfigure((0, 1, 2, 3), weight=1)
        ttk.Button(waypoint_buttons, text="Clear", command=self.clear_waypoints).grid(row=0, column=0, sticky="ew", padx=(0, 4))
        ttk.Button(waypoint_buttons, text="1m Demo", command=self.load_demo_path).grid(row=0, column=1, sticky="ew", padx=4)
        ttk.Button(waypoint_buttons, text="Run Track", command=self.run_drawn_track).grid(row=0, column=2, sticky="ew", padx=4)
        ttk.Button(waypoint_buttons, text="Clear Track", command=self.clear_drawn_track).grid(row=0, column=3, sticky="ew", padx=(4, 0))
        self.waypoint_list = tk.Listbox(side, height=6, activestyle="dotbox")
        self.waypoint_list.grid(row=50, column=0, sticky="ew", pady=(10, 14))
        self.waypoint_list.bind("<<ListboxSelect>>", self.on_waypoint_select)

        self._section(side, 51, "Selected Point")
        self._waypoint_field(side, 52, "x (m)", "waypoint_x")
        self._waypoint_field(side, 53, "y (m)", "waypoint_y")
        self._waypoint_field(side, 54, "Final theta deg", "waypoint_theta")
        self._waypoint_mode_field(side, 55, "Travel mode", "waypoint_drive_mode")
        selected_buttons = ttk.Frame(side)
        selected_buttons.grid(row=56, column=0, sticky="ew", pady=(8, 14))
        selected_buttons.columnconfigure((0, 1), weight=1)
        ttk.Button(selected_buttons, text="Update Selected", command=self.update_selected_waypoint).grid(row=0, column=0, sticky="ew", padx=(0, 4))
        ttk.Button(selected_buttons, text="Delete Selected", command=self.delete_selected_waypoint).grid(row=0, column=1, sticky="ew", padx=(4, 0))

        self._section(side, 57, "Heading")
        self.heading_canvas = tk.Canvas(side, width=188, height=188, background="#f6f7f9", highlightthickness=0)
        self.heading_canvas.grid(row=58, column=0, pady=(4, 0))

    def _section(self, parent: ttk.Frame, row: int, title: str) -> None:
        ttk.Label(parent, text=title, font=("Segoe UI", 11, "bold")).grid(row=row, column=0, sticky="w", pady=(0, 6))

    def _field(self, parent: ttk.Frame, row: int, label: str, key: str, command: Any | None = None) -> None:
        frame = ttk.Frame(parent)
        frame.grid(row=row, column=0, sticky="ew", pady=2)
        frame.columnconfigure(1, weight=1)
        ttk.Label(frame, text=label, style="Small.TLabel").grid(row=0, column=0, sticky="w")
        entry = ttk.Entry(frame, textvariable=self.vars[key], width=13)
        entry.grid(row=0, column=1, sticky="e")
        entry.bind("<Return>", lambda _event: (command or self.apply_geometry)())

    def _waypoint_field(self, parent: ttk.Frame, row: int, label: str, key: str) -> None:
        frame = ttk.Frame(parent)
        frame.grid(row=row, column=0, sticky="ew", pady=2)
        frame.columnconfigure(1, weight=1)
        ttk.Label(frame, text=label, style="Small.TLabel").grid(row=0, column=0, sticky="w")
        entry = ttk.Entry(frame, textvariable=self.vars[key], width=13)
        entry.grid(row=0, column=1, sticky="e")
        entry.bind("<Return>", lambda _event: self.update_selected_waypoint())
        entry.bind("<FocusOut>", lambda _event: self.update_selected_waypoint())

    def _waypoint_mode_field(self, parent: ttk.Frame, row: int, label: str, key: str) -> None:
        frame = ttk.Frame(parent)
        frame.grid(row=row, column=0, sticky="ew", pady=2)
        frame.columnconfigure(1, weight=1)
        ttk.Label(frame, text=label, style="Small.TLabel").grid(row=0, column=0, sticky="w")
        combo = ttk.Combobox(frame, textvariable=self.vars[key], values=DRIVE_MODES, width=11, state="readonly")
        combo.grid(row=0, column=1, sticky="e")
        combo.bind("<<ComboboxSelected>>", lambda _event: self.update_selected_waypoint())

    def on_mousewheel(self, event: tk.Event) -> None:
        if hasattr(self, "side_canvas"):
            self.side_canvas.yview_scroll(int(-1 * (event.delta / 120)), "units")

    def _bind_events(self) -> None:
        self.canvas.bind("<Configure>", lambda _event: self.redraw())
        self.canvas.bind("<Button-1>", self.on_canvas_click)
        self.canvas.bind("<B1-Motion>", self.on_canvas_drag)
        self.canvas.bind("<ButtonRelease-1>", self.on_canvas_release)

    def connect_bridge(self) -> None:
        try:
            self.bridge.connect(self.vars["bridge_host"].get().strip(), int(self.vars["bridge_port"].get()))
        except (OSError, ValueError) as exc:
            self.connection_text.set("Disconnected")
            self.status_text.set(f"Bridge connect failed: {exc}")
            return
        self.connection_text.set("Connected")
        self.status_text.set("Connected to Pi bridge. Waiting for live odometry.")
        self.apply_geometry_to_robot(silent=True)
        self.apply_imu_settings_to_robot(silent=True)
        self.apply_nav2_params_to_robot(silent=True)

    def disconnect_bridge(self) -> None:
        self.bridge.close()
        self.connection_text.set("Disconnected")
        self.status_text.set("Disconnected from Pi bridge.")

    def process_bridge_messages(self) -> None:
        redraw_needed = False
        while True:
            try:
                message = self.bridge_queue.get_nowait()
            except queue.Empty:
                break

            kind = message.get("type")
            if kind == "connection":
                self.connection_text.set("Connected" if message.get("connected") else "Disconnected")
            elif kind == "status":
                self.status_text.set(str(message.get("message", "")))
            elif kind == "mission_status":
                self.current_mission_state = str(message.get("state", ""))
                self.mission_text.set(str(message.get("message", message.get("state", ""))))
                self.status_text.set(str(message.get("message", "")))
                self.write_debug_event(message)
                if self.current_mission_state in {"done", "error", "stopped"}:
                    self.finish_debug_record(self.current_mission_state)
            elif kind == "geometry":
                self.status_text.set("Robot runtime geometry confirmed.")
            elif kind == "nav2_params":
                params = message.get("params", {})
                if isinstance(params, dict):
                    self.set_nav2_vars(params)
                    self.status_text.set("Robot Nav2 parameters confirmed.")
            elif kind == "imu":
                imu = message.get("imu", {})
                if isinstance(imu, dict):
                    self.set_imu_vars(imu)
                    self.status_text.set("Robot yaw source confirmed.")
            elif kind == "odom":
                self.pose = RobotPose(float(message["x"]), float(message["y"]), float(message["yaw"]))
                self.vars["pose_x"].set(f"{self.pose.x_m:.3f}")
                self.vars["pose_y"].set(f"{self.pose.y_m:.3f}")
                self.vars["pose_yaw"].set(f"{math.degrees(self.pose.yaw_rad):.1f}")
                self.speed_text.set(
                    f"vx={float(message.get('linear_x', 0.0)):.3f} m/s   "
                    f"wz={float(message.get('angular_z', 0.0)):.3f} rad/s   "
                    f"yaw={message.get('yaw_source', 'encoder')}"
                )
                if not self.trail or math.hypot(self.pose.x_m - self.trail[-1][0], self.pose.y_m - self.trail[-1][1]) > 0.003:
                    self.trail.append((self.pose.x_m, self.pose.y_m))
                    self.trail = self.trail[-5000:]
                self.write_debug_odom(message)
                redraw_needed = True

        if redraw_needed:
            self.redraw()
        self.root.after(50, self.process_bridge_messages)

    def run_mission(self) -> None:
        if not self.waypoints:
            self.status_text.set("Add at least one waypoint first.")
            return
        if not self.bridge.connected:
            self.status_text.set("Connect to the Pi bridge first.")
            return
        payload = []
        previous = (self.pose.x_m, self.pose.y_m)
        for index, point in enumerate(self.waypoints):
            next_point = self.waypoints[index + 1] if index + 1 < len(self.waypoints) else point
            drive_mode = normalize_drive_mode(point.drive_mode)
            if math.hypot(point.x_m - previous[0], point.y_m - previous[1]) > 0.02:
                yaw = segment_robot_yaw(previous, (point.x_m, point.y_m), drive_mode)
            elif math.hypot(next_point.x_m - point.x_m, next_point.y_m - point.y_m) > 0.02:
                yaw = segment_robot_yaw(
                    (point.x_m, point.y_m),
                    (next_point.x_m, next_point.y_m),
                    next_point.drive_mode,
                )
            else:
                yaw = self.pose.yaw_rad
            if point.final_yaw_deg is not None:
                final_yaw = math.radians(point.final_yaw_deg)
            elif self.auto_corner_heading_var.get() and index + 1 < len(self.waypoints):
                final_yaw = segment_robot_yaw(
                    (point.x_m, point.y_m),
                    (next_point.x_m, next_point.y_m),
                    next_point.drive_mode,
                )
            else:
                final_yaw = None
            payload.append(
                {
                    "x": point.x_m,
                    "y": point.y_m,
                    "yaw": normalize_angle(yaw),
                    "final_yaw": None if final_yaw is None else normalize_angle(final_yaw),
                    "drive_mode": drive_mode,
                }
            )
            previous = (point.x_m, point.y_m)
        self.start_debug_record(payload, mission_mode="waypoints")
        self.bridge.send({"type": "mission", "mode": "waypoints", "waypoints": payload})
        self.mission_text.set(f"Sent {len(payload)} waypoint mission.")

    def run_drawn_track(self) -> None:
        if len(self.drawn_track) < 2:
            self.status_text.set("Draw a track with at least two points first.")
            return
        if not self.bridge.connected:
            self.status_text.set("Connect to the Pi bridge first.")
            return

        compact: list[tuple[float, float]] = []
        for point in self.drawn_track:
            if not compact or math.hypot(point[0] - compact[-1][0], point[1] - compact[-1][1]) >= 0.03:
                compact.append(point)
        if len(compact) < 2:
            self.status_text.set("Drawn track is too short.")
            return

        payload = []
        previous = (self.pose.x_m, self.pose.y_m)
        for index, (x_m, y_m) in enumerate(compact):
            next_point = compact[index + 1] if index + 1 < len(compact) else (x_m, y_m)
            if math.hypot(x_m - previous[0], y_m - previous[1]) > 0.02:
                yaw = math.atan2(y_m - previous[1], x_m - previous[0])
            elif math.hypot(next_point[0] - x_m, next_point[1] - y_m) > 0.02:
                yaw = math.atan2(next_point[1] - y_m, next_point[0] - x_m)
            else:
                yaw = self.pose.yaw_rad
            payload.append(
                {
                    "x": x_m,
                    "y": y_m,
                    "yaw": normalize_angle(yaw),
                    "final_yaw": None,
                    "drive_mode": "forward",
                }
            )
            previous = (x_m, y_m)

        self.start_debug_record(payload, mission_mode="path")
        self.bridge.send({"type": "mission", "mode": "path", "waypoints": payload})
        self.mission_text.set(f"Sent drawn track with {len(payload)} path points.")

    def stop_robot(self) -> None:
        if self.bridge.connected:
            self.bridge.send({"type": "stop"})
        self.mission_text.set("Stop requested")

    def clear_robot_odom(self) -> None:
        self.trail = [(0.0, 0.0)]
        if self.bridge.connected:
            self.bridge.send({"type": "reset_odom"})
            self.status_text.set("Requested dashboard odom reset.")
        else:
            self.pose = RobotPose()
            self.vars["pose_x"].set("0.000")
            self.vars["pose_y"].set("0.000")
            self.vars["pose_yaw"].set("0.0")
            self.status_text.set("Offline odom reset.")
        self.redraw()

    def apply_geometry(self) -> None:
        try:
            self.geometry = RobotGeometry(
                wheel_diameter_m=max(0.001, float(self.vars["wheel_diameter"].get())),
                wheel_base_m=max(0.001, float(self.vars["wheel_base"].get())),
                counts_per_rev=max(1, int(float(self.vars["counts_per_rev"].get()))),
                max_rpm=max(1, int(float(self.vars["max_rpm"].get()))),
            )
        except ValueError:
            self.status_text.set("Geometry contains an invalid number.")
            return
        self.save_settings()
        self.status_text.set("Geometry updated locally.")
        self.redraw()

    def apply_geometry_to_robot(self, silent: bool = False) -> None:
        self.apply_geometry()
        if not self.bridge.connected:
            if not silent:
                self.status_text.set("Geometry saved locally. Connect bridge to apply on robot.")
            return
        try:
            self.bridge.send({"type": "set_geometry", "geometry": self.geometry_payload()})
        except RuntimeError as exc:
            if not silent:
                self.status_text.set(str(exc))
            return
        if not silent:
            self.status_text.set("Geometry sent to robot bridge.")

    def apply_nav2_params_local(self) -> None:
        try:
            params = self.nav2_payload()
        except ValueError:
            self.status_text.set("Nav2 tune contains an invalid number.")
            return
        self.nav2_params = params
        self.save_settings()
        self.status_text.set("Nav2 tune saved locally.")

    def apply_nav2_params_to_robot(self, silent: bool = False) -> None:
        try:
            params = self.nav2_payload()
        except ValueError:
            if not silent:
                self.status_text.set("Nav2 tune contains an invalid number.")
            return
        self.nav2_params = params
        self.save_settings()
        if not self.bridge.connected:
            if not silent:
                self.status_text.set("Nav2 tune saved locally. Connect bridge to apply on robot.")
            return
        try:
            self.bridge.send({"type": "set_nav2_params", "params": params})
        except RuntimeError as exc:
            if not silent:
                self.status_text.set(str(exc))
            return
        if not silent:
            self.status_text.set("Nav2 tune sent to robot.")

    def apply_imu_settings_local(self) -> None:
        try:
            imu = self.imu_payload()
        except ValueError:
            self.status_text.set("Yaw source settings contain an invalid number.")
            return
        self.imu_settings = imu
        self.save_settings()
        self.status_text.set("Yaw source settings saved locally.")

    def apply_imu_settings_to_robot(self, silent: bool = False) -> None:
        try:
            imu = self.imu_payload()
        except ValueError:
            if not silent:
                self.status_text.set("Yaw source settings contain an invalid number.")
            return
        self.imu_settings = imu
        self.save_settings()
        if not self.bridge.connected:
            if not silent:
                self.status_text.set("Yaw source saved locally. Connect bridge to apply on robot.")
            return
        try:
            self.bridge.send({"type": "set_imu", "imu": imu})
        except RuntimeError as exc:
            if not silent:
                self.status_text.set(str(exc))
            return
        if not silent:
            self.status_text.set("Yaw source settings sent to robot.")

    def imu_payload(self) -> dict:
        pixhawk_weight = min(1.0, max(0.0, float(self.vars["imu_pixhawk_weight"].get())))
        encoder_weight = min(1.0, max(0.0, float(self.vars["imu_encoder_weight"].get())))
        if pixhawk_weight + encoder_weight <= 0.001:
            pixhawk_weight = 0.8
            encoder_weight = 0.2
        yaw_mode = self.vars["imu_yaw_mode"].get().strip().lower()
        if yaw_mode not in {"gyro", "attitude"}:
            yaw_mode = "gyro"
        yaw_sign = -1.0 if float(self.vars["imu_yaw_sign"].get()) < 0.0 else 1.0
        return {
            "use_pixhawk_yaw": bool(self.use_pixhawk_yaw_var.get()),
            "pixhawk_weight": pixhawk_weight,
            "encoder_weight": encoder_weight,
            "pixhawk_yaw_mode": yaw_mode,
            "pixhawk_yaw_sign": yaw_sign,
            "pixhawk_port": self.vars["imu_port"].get().strip() or "/dev/serial0",
            "pixhawk_baud": max(1200, int(float(self.vars["imu_baud"].get()))),
            "pixhawk_timeout_s": min(5.0, max(0.10, float(self.vars["imu_timeout"].get()))),
        }

    def set_imu_vars(self, imu: dict[str, Any]) -> None:
        merged = dict(DEFAULT_IMU_SETTINGS)
        merged.update(imu)
        self.use_pixhawk_yaw_var.set(bool(merged["use_pixhawk_yaw"]))
        self.vars["imu_pixhawk_weight"].set(f"{float(merged['pixhawk_weight']):.2f}")
        self.vars["imu_encoder_weight"].set(f"{float(merged['encoder_weight']):.2f}")
        self.vars["imu_yaw_mode"].set(str(merged["pixhawk_yaw_mode"]))
        self.vars["imu_yaw_sign"].set(f"{float(merged['pixhawk_yaw_sign']):.0f}")
        self.vars["imu_port"].set(str(merged["pixhawk_port"]))
        self.vars["imu_baud"].set(str(int(float(merged["pixhawk_baud"]))))
        self.vars["imu_timeout"].set(f"{float(merged['pixhawk_timeout_s']):.2f}")
        self.imu_settings = self.imu_payload()
        self.save_settings()

    def load_default_imu_settings(self) -> None:
        self.set_imu_vars(DEFAULT_IMU_SETTINGS)
        self.status_text.set("Loaded Pixhawk 80/20 yaw defaults.")

    def nav2_payload(self) -> dict:
        return {
            "desired_linear_vel": float(self.vars["nav2_desired_linear_vel"].get()),
            "lookahead_dist": float(self.vars["nav2_lookahead_dist"].get()),
            "min_lookahead_dist": float(self.vars["nav2_min_lookahead_dist"].get()),
            "max_lookahead_dist": float(self.vars["nav2_max_lookahead_dist"].get()),
            "lookahead_time": float(self.vars["nav2_lookahead_time"].get()),
            "min_approach_linear_velocity": float(self.vars["nav2_min_approach_linear_velocity"].get()),
            "approach_velocity_scaling_dist": float(self.vars["nav2_approach_velocity_scaling_dist"].get()),
            "regulated_linear_scaling_min_radius": float(self.vars["nav2_regulated_linear_scaling_min_radius"].get()),
            "regulated_linear_scaling_min_speed": float(self.vars["nav2_regulated_linear_scaling_min_speed"].get()),
            "max_angular_accel": float(self.vars["nav2_max_angular_accel"].get()),
            "xy_goal_tolerance": float(self.vars["nav2_xy_goal_tolerance"].get()),
            "yaw_goal_tolerance": float(self.vars["nav2_yaw_goal_tolerance"].get()),
        }

    def set_nav2_vars(self, params: dict[str, Any]) -> None:
        for key, value in DEFAULT_NAV2_PARAMS.items():
            raw_value = params.get(key, value)
            try:
                numeric = float(raw_value)
            except (TypeError, ValueError):
                numeric = value
            self.vars[f"nav2_{key}"].set(f"{numeric:.3f}")
            self.nav2_params[key] = numeric
        self.save_settings()

    def load_tight_nav2_defaults(self) -> None:
        self.set_nav2_vars(DEFAULT_NAV2_PARAMS)
        self.status_text.set("Loaded tight-box Nav2 defaults.")

    def geometry_payload(self) -> dict:
        return {
            "wheel_diameter_m": self.geometry.wheel_diameter_m,
            "wheel_base_m": self.geometry.wheel_base_m,
            "counts_per_rev": self.geometry.counts_per_rev,
            "max_rpm": self.geometry.max_rpm,
        }

    def save_settings(self) -> None:
        DASHBOARD_SETTINGS_PATH.parent.mkdir(parents=True, exist_ok=True)
        self.settings.update(
            {
                "bridge_host": self.vars["bridge_host"].get().strip(),
                "bridge_port": self.vars["bridge_port"].get().strip(),
                "save_debug": bool(self.save_debug_var.get()),
                "auto_corner_heading": bool(self.auto_corner_heading_var.get()),
                "geometry": self.geometry_payload(),
                "imu": self.imu_settings,
                "nav2_params": self.nav2_params,
                "next_debug_id": int(self.settings.get("next_debug_id", 1)),
            }
        )
        DASHBOARD_SETTINGS_PATH.write_text(json.dumps(self.settings, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    def start_debug_record(self, waypoints: list[dict], mission_mode: str = "waypoints") -> None:
        self.finish_debug_record("replaced")
        if not self.save_debug_var.get():
            return

        DEBUG_DIR.mkdir(parents=True, exist_ok=True)
        debug_id = int(self.settings.get("next_debug_id", 1))
        self.settings["next_debug_id"] = debug_id + 1
        self.save_settings()

        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        base = f"euroboot_debug_{stamp}_id{debug_id:04d}"
        self.debug_csv_path = DEBUG_DIR / f"{base}.csv"
        self.debug_meta_path = DEBUG_DIR / f"{base}.json"
        metadata = {
            "id": debug_id,
            "created_at": datetime.now().isoformat(timespec="seconds"),
            "geometry": self.geometry_payload(),
            "imu": self.imu_settings,
            "nav2_params": self.nav2_params,
            "mission_mode": mission_mode,
            "waypoints": waypoints,
            "drawn_track": [{"x": x, "y": y} for x, y in self.drawn_track] if mission_mode == "path" else [],
            "auto_corner_heading": bool(self.auto_corner_heading_var.get()),
            "bridge_host": self.vars["bridge_host"].get().strip(),
            "bridge_port": self.vars["bridge_port"].get().strip(),
            "note": "Dashboard debug record. Runtime geometry does not imply ESP32 firmware constants were flashed.",
        }
        self.debug_meta_path.write_text(json.dumps(metadata, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        self.debug_csv_file = self.debug_csv_path.open("w", newline="", encoding="utf-8")
        self.debug_writer = csv.DictWriter(
            self.debug_csv_file,
            fieldnames=[
                "t_bridge",
                "mission_state",
                "x_m",
                "y_m",
                "yaw_rad",
                "raw_x_m",
                "raw_y_m",
                "raw_yaw_rad",
                "encoder_yaw_rad",
                "pixhawk_yaw_rad",
                "pixhawk_yaw_ros_rad",
                "pixhawk_gyro_yaw_rad",
                "pixhawk_age_s",
                "yaw_source",
                "pixhawk_weight",
                "encoder_weight",
                "pixhawk_yaw_mode",
                "pixhawk_yaw_sign",
                "linear_x_mps",
                "angular_z_radps",
                "cmd_linear_x_mps",
                "cmd_angular_z_radps",
                "cmd_age_s",
                "actual_left_mps",
                "actual_right_mps",
                "cmd_left_mps",
                "cmd_right_mps",
                "turn_abs_balance_error_mps",
                "turn_linear_leak_ratio",
                "event_type",
                "event_message",
            ],
        )
        self.debug_writer.writeheader()
        self.debug_text.set(f"Debug id {debug_id:04d} recording")

    def write_debug_odom(self, message: dict) -> None:
        if self.debug_writer is None:
            return
        linear_x = float(message.get("linear_x", 0.0) or 0.0)
        angular_z = float(message.get("angular_z", 0.0) or 0.0)
        cmd_linear_x = float(message.get("cmd_linear_x", 0.0) or 0.0)
        cmd_angular_z = float(message.get("cmd_angular_z", 0.0) or 0.0)
        actual_left, actual_right = differential_wheel_speeds(linear_x, angular_z, self.geometry.wheel_base_m)
        cmd_left, cmd_right = differential_wheel_speeds(cmd_linear_x, cmd_angular_z, self.geometry.wheel_base_m)
        turn_tangent = abs(angular_z) * self.geometry.wheel_base_m * 0.5
        turn_balance = ""
        turn_leak = ""
        if abs(angular_z) > 0.12:
            turn_balance = abs(abs(actual_left) - abs(actual_right))
            turn_leak = abs(linear_x) / max(0.001, turn_tangent)
        self.debug_writer.writerow(
            {
                "t_bridge": message.get("t", ""),
                "mission_state": self.current_mission_state,
                "x_m": message.get("x", ""),
                "y_m": message.get("y", ""),
                "yaw_rad": message.get("yaw", ""),
                "raw_x_m": message.get("raw_x", ""),
                "raw_y_m": message.get("raw_y", ""),
                "raw_yaw_rad": message.get("raw_yaw", ""),
                "encoder_yaw_rad": message.get("encoder_yaw", ""),
                "pixhawk_yaw_rad": message.get("pixhawk_yaw", ""),
                "pixhawk_yaw_ros_rad": message.get("pixhawk_yaw_ros", ""),
                "pixhawk_gyro_yaw_rad": message.get("pixhawk_gyro_yaw", ""),
                "pixhawk_age_s": message.get("pixhawk_age_s", ""),
                "yaw_source": message.get("yaw_source", ""),
                "pixhawk_weight": message.get("pixhawk_weight", ""),
                "encoder_weight": message.get("encoder_weight", ""),
                "pixhawk_yaw_mode": message.get("pixhawk_yaw_mode", ""),
                "pixhawk_yaw_sign": message.get("pixhawk_yaw_sign", ""),
                "linear_x_mps": message.get("linear_x", ""),
                "angular_z_radps": message.get("angular_z", ""),
                "cmd_linear_x_mps": message.get("cmd_linear_x", ""),
                "cmd_angular_z_radps": message.get("cmd_angular_z", ""),
                "cmd_age_s": message.get("cmd_age_s", ""),
                "actual_left_mps": actual_left,
                "actual_right_mps": actual_right,
                "cmd_left_mps": cmd_left,
                "cmd_right_mps": cmd_right,
                "turn_abs_balance_error_mps": turn_balance,
                "turn_linear_leak_ratio": turn_leak,
                "event_type": "odom",
                "event_message": "",
            }
        )

    def write_debug_event(self, message: dict) -> None:
        if self.debug_writer is None:
            return
        self.debug_writer.writerow(
            {
                "t_bridge": "",
                "mission_state": message.get("state", self.current_mission_state),
                "x_m": self.pose.x_m,
                "y_m": self.pose.y_m,
                "yaw_rad": self.pose.yaw_rad,
                "raw_x_m": "",
                "raw_y_m": "",
                "raw_yaw_rad": "",
                "encoder_yaw_rad": "",
                "pixhawk_yaw_rad": "",
                "pixhawk_yaw_ros_rad": "",
                "pixhawk_gyro_yaw_rad": "",
                "pixhawk_age_s": "",
                "yaw_source": self.imu_settings.get("use_pixhawk_yaw", ""),
                "pixhawk_weight": self.imu_settings.get("pixhawk_weight", ""),
                "encoder_weight": self.imu_settings.get("encoder_weight", ""),
                "pixhawk_yaw_mode": self.imu_settings.get("pixhawk_yaw_mode", ""),
                "pixhawk_yaw_sign": self.imu_settings.get("pixhawk_yaw_sign", ""),
                "linear_x_mps": "",
                "angular_z_radps": "",
                "cmd_linear_x_mps": "",
                "cmd_angular_z_radps": "",
                "cmd_age_s": "",
                "actual_left_mps": "",
                "actual_right_mps": "",
                "cmd_left_mps": "",
                "cmd_right_mps": "",
                "turn_abs_balance_error_mps": "",
                "turn_linear_leak_ratio": "",
                "event_type": "mission_status",
                "event_message": message.get("message", ""),
            }
        )

    def finish_debug_record(self, status: str) -> None:
        if self.debug_csv_file is None:
            return
        self.debug_csv_file.flush()
        self.debug_csv_file.close()
        self.debug_csv_file = None
        self.debug_writer = None
        if self.debug_meta_path and self.debug_meta_path.exists():
            try:
                metadata = json.loads(self.debug_meta_path.read_text(encoding="utf-8"))
            except (OSError, json.JSONDecodeError):
                metadata = {}
            metadata["finished_at"] = datetime.now().isoformat(timespec="seconds")
            metadata["status"] = status
            self.debug_meta_path.write_text(json.dumps(metadata, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        if self.debug_csv_path:
            self.debug_text.set(f"Saved {self.debug_csv_path.name}")

    def clear_waypoints(self) -> None:
        self.waypoints.clear()
        self.selected_waypoint_index = None
        self.vars["waypoint_x"].set("")
        self.vars["waypoint_y"].set("")
        self.vars["waypoint_theta"].set("")
        self.vars["waypoint_drive_mode"].set("forward")
        self.refresh_waypoints()
        self.status_text.set("Waypoints cleared.")
        self.redraw()

    def load_demo_path(self) -> None:
        self.waypoints = [Waypoint(1.0, 0.0, drive_mode="forward")]
        self.select_waypoint(0)
        self.refresh_waypoints()
        self.status_text.set("Loaded 1 meter forward demo waypoint.")
        self.redraw()

    def clear_drawn_track(self) -> None:
        self.drawn_track.clear()
        self.drawing_track = False
        self.status_text.set("Drawn track cleared.")
        self.redraw()

    def on_canvas_click(self, event: tk.Event) -> None:
        world = self.screen_to_world(event.x, event.y)
        if self.draw_track_mode_var.get():
            self.drawn_track = [world]
            self.drawing_track = True
            self.status_text.set(f"Drawing track from x={world[0]:.3f}, y={world[1]:.3f}.")
        elif self.add_waypoint_mode.get():
            self.waypoints.append(Waypoint(world[0], world[1], drive_mode=self.vars["waypoint_drive_mode"].get()))
            self.selected_waypoint_index = len(self.waypoints) - 1
            self.refresh_waypoints()
            self.select_waypoint(self.selected_waypoint_index)
            self.status_text.set(f"Added waypoint {len(self.waypoints)} at x={world[0]:.3f}, y={world[1]:.3f}.")
        else:
            self.pose.x_m, self.pose.y_m = world
            self.vars["pose_x"].set(f"{self.pose.x_m:.3f}")
            self.vars["pose_y"].set(f"{self.pose.y_m:.3f}")
            self.trail.append((self.pose.x_m, self.pose.y_m))
            self.status_text.set(f"Moved offline robot pose to x={world[0]:.3f}, y={world[1]:.3f}.")
        self.redraw()

    def on_canvas_drag(self, event: tk.Event) -> None:
        if self.draw_track_mode_var.get() and self.drawing_track:
            point = self.screen_to_world(event.x, event.y)
            if not self.drawn_track or math.hypot(point[0] - self.drawn_track[-1][0], point[1] - self.drawn_track[-1][1]) >= 0.02:
                self.drawn_track.append(point)
                self.redraw()
            return
        if self.add_waypoint_mode.get() or self.bridge.connected:
            return
        x_m, y_m = self.screen_to_world(event.x, event.y)
        self.pose.x_m = x_m
        self.pose.y_m = y_m
        self.vars["pose_x"].set(f"{x_m:.3f}")
        self.vars["pose_y"].set(f"{y_m:.3f}")
        self.redraw()

    def on_canvas_release(self, event: tk.Event) -> None:
        if not self.draw_track_mode_var.get() or not self.drawing_track:
            return
        point = self.screen_to_world(event.x, event.y)
        if not self.drawn_track or math.hypot(point[0] - self.drawn_track[-1][0], point[1] - self.drawn_track[-1][1]) >= 0.01:
            self.drawn_track.append(point)
        self.drawing_track = False
        self.status_text.set(f"Drawn track ready: {len(self.drawn_track)} points.")
        self.redraw()

    def refresh_waypoints(self) -> None:
        self.waypoint_list.delete(0, tk.END)
        for index, waypoint in enumerate(self.waypoints, start=1):
            theta = "" if waypoint.final_yaw_deg is None else f"  theta={waypoint.final_yaw_deg:.1f} deg"
            mode = normalize_drive_mode(waypoint.drive_mode)
            self.waypoint_list.insert(tk.END, f"{index:02d}  {mode[:3]}  x={waypoint.x_m:.3f} m   y={waypoint.y_m:.3f} m{theta}")
        if self.selected_waypoint_index is not None and self.selected_waypoint_index < len(self.waypoints):
            self.waypoint_list.selection_clear(0, tk.END)
            self.waypoint_list.selection_set(self.selected_waypoint_index)
            self.waypoint_list.activate(self.selected_waypoint_index)

    def on_waypoint_select(self, _event: tk.Event) -> None:
        selection = self.waypoint_list.curselection()
        if not selection:
            return
        self.select_waypoint(int(selection[0]))

    def select_waypoint(self, index: int) -> None:
        if index < 0 or index >= len(self.waypoints):
            return
        self.selected_waypoint_index = index
        waypoint = self.waypoints[index]
        self.vars["waypoint_x"].set(f"{waypoint.x_m:.3f}")
        self.vars["waypoint_y"].set(f"{waypoint.y_m:.3f}")
        self.vars["waypoint_theta"].set("" if waypoint.final_yaw_deg is None else f"{waypoint.final_yaw_deg:.1f}")
        self.vars["waypoint_drive_mode"].set(normalize_drive_mode(waypoint.drive_mode))
        self.refresh_waypoints()
        self.redraw()

    def update_selected_waypoint(self) -> None:
        if self.selected_waypoint_index is None or self.selected_waypoint_index >= len(self.waypoints):
            return
        try:
            x_m = float(self.vars["waypoint_x"].get())
            y_m = float(self.vars["waypoint_y"].get())
            theta_raw = self.vars["waypoint_theta"].get().strip()
            final_yaw_deg = None if theta_raw == "" else float(theta_raw)
        except ValueError:
            self.status_text.set("Selected waypoint contains an invalid number.")
            return
        drive_mode = normalize_drive_mode(self.vars["waypoint_drive_mode"].get())
        self.vars["waypoint_drive_mode"].set(drive_mode)
        self.waypoints[self.selected_waypoint_index] = Waypoint(x_m, y_m, final_yaw_deg, drive_mode)
        self.refresh_waypoints()
        self.status_text.set(f"Updated waypoint {self.selected_waypoint_index + 1}.")
        self.redraw()

    def delete_selected_waypoint(self) -> None:
        if self.selected_waypoint_index is None or self.selected_waypoint_index >= len(self.waypoints):
            return
        deleted = self.selected_waypoint_index
        del self.waypoints[deleted]
        if not self.waypoints:
            self.selected_waypoint_index = None
            self.vars["waypoint_x"].set("")
            self.vars["waypoint_y"].set("")
            self.vars["waypoint_theta"].set("")
            self.vars["waypoint_drive_mode"].set("forward")
        else:
            self.selected_waypoint_index = min(deleted, len(self.waypoints) - 1)
            waypoint = self.waypoints[self.selected_waypoint_index]
            self.vars["waypoint_x"].set(f"{waypoint.x_m:.3f}")
            self.vars["waypoint_y"].set(f"{waypoint.y_m:.3f}")
            self.vars["waypoint_theta"].set("" if waypoint.final_yaw_deg is None else f"{waypoint.final_yaw_deg:.1f}")
            self.vars["waypoint_drive_mode"].set(normalize_drive_mode(waypoint.drive_mode))
        self.refresh_waypoints()
        self.status_text.set("Deleted selected waypoint.")
        self.redraw()

    def canvas_center(self) -> tuple[float, float]:
        return self.canvas.winfo_width() / 2.0, self.canvas.winfo_height() / 2.0

    def world_to_screen(self, x_m: float, y_m: float) -> tuple[float, float]:
        cx, cy = self.canvas_center()
        scale = self.scale_px_per_m.get()
        return cx + x_m * scale, cy - y_m * scale

    def screen_to_world(self, x_px: float, y_px: float) -> tuple[float, float]:
        cx, cy = self.canvas_center()
        scale = self.scale_px_per_m.get()
        return (x_px - cx) / scale, (cy - y_px) / scale

    def redraw(self) -> None:
        self.canvas.delete("all")
        self._draw_grid()
        self._draw_drawn_track()
        self._draw_waypoints()
        self._draw_trail()
        self._draw_robot()
        self._draw_heading()
        self._update_summary()

    def _draw_grid(self) -> None:
        width = self.canvas.winfo_width()
        height = self.canvas.winfo_height()
        if width <= 1 or height <= 1:
            return
        scale = self.scale_px_per_m.get()
        step_m = 0.25
        step_px = step_m * scale
        cx, cy = self.canvas_center()

        x = cx % step_px
        while x < width:
            self.canvas.create_line(x, 0, x, height, fill="#e8edf2")
            x += step_px

        y = cy % step_px
        while y < height:
            self.canvas.create_line(0, y, width, y, fill="#e8edf2")
            y += step_px

        self.canvas.create_line(0, cy, width, cy, fill="#b7c2ce", width=2)
        self.canvas.create_line(cx, 0, cx, height, fill="#b7c2ce", width=2)
        self.canvas.create_text(cx + 10, cy + 16, text="dashboard odom origin", fill="#65758b", anchor="w")

    def _draw_trail(self) -> None:
        if len(self.trail) < 2:
            return
        points: list[float] = []
        for x_m, y_m in self.trail:
            sx, sy = self.world_to_screen(x_m, y_m)
            points.extend([sx, sy])
        self.canvas.create_line(*points, fill="#1677ff", width=2, smooth=True)

    def _draw_drawn_track(self) -> None:
        if len(self.drawn_track) < 2:
            return
        flat: list[float] = []
        for x_m, y_m in self.drawn_track:
            sx, sy = self.world_to_screen(x_m, y_m)
            flat.extend([sx, sy])
        self.canvas.create_line(*flat, fill="#7c3aed", width=3, smooth=True)
        for sx, sy in (self.world_to_screen(*self.drawn_track[0]), self.world_to_screen(*self.drawn_track[-1])):
            self.canvas.create_oval(sx - 5, sy - 5, sx + 5, sy + 5, fill="#7c3aed", outline="#4c1d95", width=1)

    def _draw_waypoints(self) -> None:
        if not self.waypoints:
            return
        screen_points = [self.world_to_screen(waypoint.x_m, waypoint.y_m) for waypoint in self.waypoints]
        segment_start = self.world_to_screen(self.pose.x_m, self.pose.y_m)
        for waypoint, segment_end in zip(self.waypoints, screen_points):
            reverse = normalize_drive_mode(waypoint.drive_mode) == "backward"
            color = "#9333ea" if reverse else "#16a34a"
            dash = (3, 3) if reverse else (6, 4)
            self.canvas.create_line(
                segment_start[0],
                segment_start[1],
                segment_end[0],
                segment_end[1],
                fill=color,
                width=2,
                dash=dash,
                arrow=tk.LAST,
            )
            segment_start = segment_end
        for index, (sx, sy) in enumerate(screen_points, start=1):
            selected = self.selected_waypoint_index == index - 1
            radius = 9 if selected else 7
            fill = "#f59e0b" if selected else "#16a34a"
            outline = "#b45309" if selected else "#0f7a34"
            waypoint = self.waypoints[index - 1]
            if normalize_drive_mode(waypoint.drive_mode) == "backward" and not selected:
                fill = "#9333ea"
                outline = "#5b21b6"
            self.canvas.create_oval(sx - radius, sy - radius, sx + radius, sy + radius, fill=fill, outline=outline, width=2)
            label_color = "#5b21b6" if normalize_drive_mode(waypoint.drive_mode) == "backward" else "#0f7a34"
            self.canvas.create_text(sx + 12, sy - 12, text=str(index), fill=label_color, font=("Segoe UI", 10, "bold"))
            if waypoint.final_yaw_deg is not None:
                yaw = math.radians(waypoint.final_yaw_deg)
                tip = (sx + math.cos(yaw) * 28, sy - math.sin(yaw) * 28)
                self.canvas.create_line(sx, sy, tip[0], tip[1], fill="#b45309", width=2, arrow=tk.LAST)

    def _draw_robot(self) -> None:
        scale = self.scale_px_per_m.get()
        yaw = self.pose.yaw_rad
        center = self.world_to_screen(self.pose.x_m, self.pose.y_m)
        length = self.geometry.body_length_m * scale
        width = self.geometry.body_width_m * scale
        wheel_radius = max(5.0, self.geometry.wheel_radius_m * scale)
        wheel_length = max(18.0, self.geometry.wheel_diameter_m * 1.7 * scale)
        wheel_width = max(7.0, self.geometry.wheel_diameter_m * 0.38 * scale)

        body = self._rotated_rect(center, length, width, yaw)
        self.canvas.create_polygon(body, fill="#f8fafc", outline="#334155", width=2)
        nose = self._transform_points(center, yaw, [(length * 0.45, 0), (length * 0.22, -width * 0.22), (length * 0.22, width * 0.22)])
        self.canvas.create_polygon(nose, fill="#ef4444", outline="#b91c1c", width=1)

        left_wheel_center = self._transform_point(center, yaw, 0, width * 0.50)
        right_wheel_center = self._transform_point(center, yaw, 0, -width * 0.50)
        for wheel_center in (left_wheel_center, right_wheel_center):
            self.canvas.create_polygon(self._rotated_rect(wheel_center, wheel_length, wheel_width, yaw), fill="#111827", outline="#020617", width=1)

        sx, sy = center
        arrow_tip = self._transform_point(center, yaw, length * 0.72, 0)
        self.canvas.create_line(sx, sy, arrow_tip[0], arrow_tip[1], fill="#dc2626", width=3, arrow=tk.LAST)
        self.canvas.create_oval(sx - wheel_radius * 0.35, sy - wheel_radius * 0.35, sx + wheel_radius * 0.35, sy + wheel_radius * 0.35, fill="#2563eb", outline="")
        label = f"x={self.pose.x_m:.2f}  y={self.pose.y_m:.2f}  yaw={math.degrees(self.pose.yaw_rad):.1f} deg"
        self.canvas.create_text(sx + 14, sy + 18, text=label, fill="#1f2937", anchor="w", font=("Segoe UI", 9, "bold"))

    def _draw_heading(self) -> None:
        c = self.heading_canvas
        c.delete("all")
        width = int(c["width"])
        height = int(c["height"])
        cx, cy = width / 2, height / 2
        radius = 76
        c.create_oval(cx - radius, cy - radius, cx + radius, cy + radius, outline="#cbd5e1", width=2)
        c.create_text(cx, cy - radius - 10, text="+X", fill="#475569", font=("Segoe UI", 9, "bold"))
        c.create_text(cx + radius + 12, cy, text="-Y", fill="#475569", font=("Segoe UI", 9))
        c.create_text(cx, cy + radius + 10, text="-X", fill="#475569", font=("Segoe UI", 9))
        c.create_text(cx - radius - 12, cy, text="+Y", fill="#475569", font=("Segoe UI", 9))
        tip_x = cx + math.cos(self.pose.yaw_rad) * (radius - 10)
        tip_y = cy - math.sin(self.pose.yaw_rad) * (radius - 10)
        c.create_line(cx, cy, tip_x, tip_y, fill="#dc2626", width=4, arrow=tk.LAST)
        c.create_text(cx, cy + 2, text=f"{math.degrees(self.pose.yaw_rad):.1f} deg", fill="#0f172a", font=("Segoe UI", 11, "bold"))

    def _update_summary(self) -> None:
        max_linear_mps = self.geometry.circumference_m * self.geometry.max_rpm / 60.0
        self.geometry_summary.set(
            f"Radius: {self.geometry.wheel_radius_m:.5f} m\n"
            f"Circumference: {self.geometry.circumference_m:.5f} m\n"
            f"Max linear speed est.: {max_linear_mps:.3f} m/s\n"
            f"Track width: {self.geometry.wheel_base_m:.5f} m"
        )

    def _rotated_rect(self, center: tuple[float, float], length: float, width: float, yaw: float) -> list[float]:
        half_l = length / 2.0
        half_w = width / 2.0
        points = [(-half_l, -half_w), (half_l, -half_w), (half_l, half_w), (-half_l, half_w)]
        transformed = self._transform_points(center, yaw, points)
        return [coord for point in transformed for coord in point]

    def _transform_points(self, center: tuple[float, float], yaw: float, points: list[tuple[float, float]]) -> list[tuple[float, float]]:
        return [self._transform_point(center, yaw, x, y) for x, y in points]

    def _transform_point(self, center: tuple[float, float], yaw: float, x_forward: float, y_left: float) -> tuple[float, float]:
        sx, sy = center
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        x_screen = sx + x_forward * cos_yaw - y_left * sin_yaw
        y_screen = sy - (x_forward * sin_yaw + y_left * cos_yaw)
        return x_screen, y_screen

    def on_close(self) -> None:
        self.finish_debug_record("dashboard_closed")
        self.save_settings()
        self.bridge.close()
        self.root.destroy()


def main() -> None:
    root = tk.Tk()
    EurobootDashboard(root)
    root.mainloop()


if __name__ == "__main__":
    main()
