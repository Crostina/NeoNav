#!/usr/bin/env python3
import argparse
import csv
import math
import time
from datetime import datetime
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Odometry, Path as NavPath
from rclpy.action import ActionClient
from rclpy.node import Node


def yaw_from_quat(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def quat_from_yaw(yaw):
    from geometry_msgs.msg import Quaternion
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class FollowPathTest(Node):
    def __init__(self, distance_m, timeout_s, output_dir, step_m):
        super().__init__("euroboot_nav2_follow_path_test")
        self.distance_m = distance_m
        self.timeout_s = timeout_s
        self.output_dir = output_dir
        self.step_m = step_m
        self.odom = None
        self.start = None
        self.samples = []
        self.start_time = time.monotonic()
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.odom_sub = self.create_subscription(Odometry, "/odom/unfiltered", self.odom_cb, 20)
        self.client = ActionClient(self, FollowPath, "follow_path")

    def odom_cb(self, msg):
        self.odom = msg
        p = msg.pose.pose.position
        yaw = yaw_from_quat(msg.pose.pose.orientation)
        self.samples.append({
            "t_s": time.monotonic() - self.start_time,
            "x_m": p.x,
            "y_m": p.y,
            "yaw_rad": yaw,
            "linear_x_mps": msg.twist.twist.linear.x,
            "angular_z_radps": msg.twist.twist.angular.z,
        })

    def wait_for_odom(self):
        deadline = time.monotonic() + 10.0
        while rclpy.ok() and self.odom is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.odom is None:
            raise RuntimeError("no /odom/unfiltered received")

    def make_path(self):
        self.wait_for_odom()
        p = self.odom.pose.pose.position
        yaw = yaw_from_quat(self.odom.pose.pose.orientation)
        self.start = (p.x, p.y, yaw)

        path = NavPath()
        path.header.frame_id = "odom"
        path.header.stamp = self.get_clock().now().to_msg()

        steps = max(2, int(math.ceil(abs(self.distance_m) / self.step_m)) + 1)
        for i in range(steps):
            d = self.distance_m * i / (steps - 1)
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = p.x + d * math.cos(yaw)
            pose.pose.position.y = p.y + d * math.sin(yaw)
            pose.pose.position.z = 0.0
            pose.pose.orientation = quat_from_yaw(yaw)
            path.poses.append(pose)
        return path

    def send_goal(self):
        if not self.client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("follow_path action server not available")

        goal = FollowPath.Goal()
        goal.path = self.make_path()
        goal.controller_id = "FollowPath"
        goal.goal_checker_id = "goal_checker"
        goal.progress_checker_id = "progress_checker"

        future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        handle = future.result()
        if not handle or not handle.accepted:
            raise RuntimeError("FollowPath goal rejected")
        return handle

    def stop(self):
        self.cmd_pub.publish(Twist())

    def save_and_print(self, status_name):
        self.output_dir.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        path = self.output_dir / f"nav2_follow_path_{stamp}.csv"
        with path.open("w", newline="") as f:
            fieldnames = ["t_s", "x_m", "y_m", "yaw_rad", "linear_x_mps", "angular_z_radps"]
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(self.samples)

        if self.start and self.samples:
            x0, y0, yaw0 = self.start
            x1 = self.samples[-1]["x_m"]
            y1 = self.samples[-1]["y_m"]
            yaw1 = self.samples[-1]["yaw_rad"]
            dx = x1 - x0
            dy = y1 - y0
            forward = math.cos(yaw0) * dx + math.sin(yaw0) * dy
            lateral = -math.sin(yaw0) * dx + math.cos(yaw0) * dy
            dist = math.hypot(dx, dy)
            yaw_delta = yaw1 - yaw0
        else:
            dx = dy = forward = lateral = dist = yaw_delta = 0.0

        print(f"RESULT status={status_name} file={path}")
        print(
            f"RESULT samples={len(self.samples)} distance={dist:.4f} "
            f"body_forward={forward:.4f} body_y_error={lateral:.4f} "
            f"odom_dx={dx:.4f} odom_dy={dy:.4f} yaw_delta={yaw_delta:.4f}"
        )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--distance", type=float, default=1.0)
    parser.add_argument("--timeout", type=float, default=25.0)
    parser.add_argument("--step", type=float, default=0.05)
    parser.add_argument("--output-dir", type=Path, default=Path("/home/maker/euroboot/test_results"))
    args = parser.parse_args()

    rclpy.init()
    node = FollowPathTest(args.distance, args.timeout, args.output_dir, args.step)
    status_name = "UNKNOWN"
    try:
        handle = node.send_goal()
        result_future = handle.get_result_async()
        deadline = time.monotonic() + args.timeout
        while rclpy.ok() and not result_future.done() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if result_future.done():
            status = result_future.result().status
            status_name = "SUCCEEDED" if status == GoalStatus.STATUS_SUCCEEDED else f"STATUS_{status}"
        else:
            handle.cancel_goal_async()
            status_name = "TIMEOUT_CANCELLED"
    finally:
        for _ in range(15):
            node.stop()
            rclpy.spin_once(node, timeout_sec=0.02)
        node.save_and_print(status_name)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
