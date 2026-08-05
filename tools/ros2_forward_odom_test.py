#!/usr/bin/env python3
import argparse
import csv
import math
import time
from datetime import datetime
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


class ForwardOdomTest(Node):
    def __init__(self, distance_m: float, speed_mps: float, timeout_s: float, output_dir: Path):
        super().__init__("euroboot_forward_odom_test")
        self.distance_m = distance_m
        self.speed_mps = speed_mps
        self.timeout_s = timeout_s
        self.output_dir = output_dir

        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.sub = self.create_subscription(Odometry, "/odom/unfiltered", self.odom_cb, 10)
        self.timer = self.create_timer(0.05, self.tick)

        self.start_time = time.monotonic()
        self.first_odom = None
        self.last_odom = None
        self.done = False
        self.stop_sent_at = None
        self.samples = []

    def odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        t = time.monotonic() - self.start_time
        sample = {
            "t_s": t,
            "x_m": p.x,
            "y_m": p.y,
            "yaw_rad": yaw,
            "linear_x_mps": msg.twist.twist.linear.x,
            "angular_z_radps": msg.twist.twist.angular.z,
        }
        if self.first_odom is None:
            self.first_odom = sample
        self.last_odom = sample
        self.samples.append(sample)

    def traveled(self):
        if self.first_odom is None or self.last_odom is None:
            return 0.0
        dx = self.last_odom["x_m"] - self.first_odom["x_m"]
        dy = self.last_odom["y_m"] - self.first_odom["y_m"]
        return math.hypot(dx, dy)

    def publish_stop(self):
        self.pub.publish(Twist())

    def tick(self):
        elapsed = time.monotonic() - self.start_time
        distance = self.traveled()

        if self.first_odom is None:
            self.publish_stop()
            return

        if not self.done and distance < self.distance_m and elapsed < self.timeout_s:
            cmd = Twist()
            cmd.linear.x = self.speed_mps
            self.pub.publish(cmd)
            return

        self.publish_stop()
        if not self.done:
            self.done = True
            self.stop_sent_at = time.monotonic()
            reason = "distance" if distance >= self.distance_m else "timeout"
            self.get_logger().info(
                f"stop reason={reason} distance={distance:.4f}m elapsed={elapsed:.2f}s"
            )

        if self.stop_sent_at and (time.monotonic() - self.stop_sent_at) > 1.0:
            self.timer.cancel()

    def save(self):
        self.output_dir.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        path = self.output_dir / f"forward_odom_test_{stamp}.csv"
        with path.open("w", newline="") as f:
            fieldnames = [
                "t_s",
                "x_m",
                "y_m",
                "yaw_rad",
                "linear_x_mps",
                "angular_z_radps",
            ]
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(self.samples)
        return path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--distance", type=float, default=1.0)
    parser.add_argument("--speed", type=float, default=0.10)
    parser.add_argument("--timeout", type=float, default=35.0)
    parser.add_argument("--output-dir", type=Path, default=Path("/home/maker/euroboot/test_results"))
    args = parser.parse_args()

    rclpy.init()
    node = ForwardOdomTest(args.distance, args.speed, args.timeout, args.output_dir)
    try:
        while rclpy.ok() and not (node.done and node.timer.is_canceled()):
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        for _ in range(10):
            node.publish_stop()
            rclpy.spin_once(node, timeout_sec=0.02)
        path = node.save()
        distance = node.traveled()
        if node.first_odom and node.last_odom:
            dx = node.last_odom["x_m"] - node.first_odom["x_m"]
            dy = node.last_odom["y_m"] - node.first_odom["y_m"]
            yaw_delta = node.last_odom["yaw_rad"] - node.first_odom["yaw_rad"]
        else:
            dx = dy = yaw_delta = 0.0
        print(f"RESULT file={path}")
        print(f"RESULT samples={len(node.samples)} distance={distance:.4f} dx={dx:.4f} dy={dy:.4f} yaw_delta={yaw_delta:.4f}")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
