#!/usr/bin/env python3
"""
Obstacle Detector Node
======================
Subscribes to a 2-D LiDAR scan, applies a configurable box filter, clusters
the filtered points, and publishes an obstacle-detected flag.

All tunable values live in  config/params.yaml — edit that file then relaunch.

Published topics
----------------
  /obstacle_detected   std_msgs/Bool   True  = obstacle present
                                       False = obstacle cleared

  /coverage_stop       std_msgs/Bool   mirrors /obstacle_detected when auto_pause=true
                                       (pauses / resumes coverage_path_follower)

Subscribed topics
-----------------
  /scan  (configurable)   sensor_msgs/LaserScan

Parameters — see config/params.yaml for descriptions
------------------------------------------------------
  scan_topic        str    default: /scan
  x_min / x_max     float  box filter X bounds  (default ±0.5 m)
  y_min / y_max     float  box filter Y bounds  (default ±0.5 m)
  cluster_distance  float  max gap inside a cluster  (default 0.15 m)
  min_cluster_size  int    min points per cluster    (default 5)
  min_clusters      int    clusters needed to trigger (default 1)
  auto_pause        bool   publish /coverage_stop     (default true)
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


class ObstacleDetectorNode(Node):

    def __init__(self):
        super().__init__('obstacle_detector_node')

        # ── Declare parameters ─────────────────────────────────────────────────
        self.declare_parameter('scan_topic',       '/scan')
        self.declare_parameter('x_min',            -0.5)
        self.declare_parameter('x_max',             0.5)
        self.declare_parameter('y_min',            -0.5)
        self.declare_parameter('y_max',             0.5)
        self.declare_parameter('cluster_distance',  0.15)
        self.declare_parameter('min_cluster_size',  5)
        self.declare_parameter('min_clusters',      1)
        self.declare_parameter('auto_pause',        True)

        # ── Load parameters ────────────────────────────────────────────────────
        self._x_min          = self.get_parameter('x_min').value
        self._x_max          = self.get_parameter('x_max').value
        self._y_min          = self.get_parameter('y_min').value
        self._y_max          = self.get_parameter('y_max').value
        self._cluster_dist   = self.get_parameter('cluster_distance').value
        self._min_cluster_sz = self.get_parameter('min_cluster_size').value
        self._min_clusters   = self.get_parameter('min_clusters').value
        self._auto_pause     = self.get_parameter('auto_pause').value
        scan_topic           = self.get_parameter('scan_topic').value

        # ── State ──────────────────────────────────────────────────────────────
        self._obstacle_active = False

        # ── Publishers ─────────────────────────────────────────────────────────
        self._pub_obstacle = self.create_publisher(Bool, '/obstacle_detected', 10)
        self._pub_stop     = self.create_publisher(Bool, '/coverage_stop',     10)

        # ── Subscriber ─────────────────────────────────────────────────────────
        self.create_subscription(LaserScan, scan_topic, self._cb_scan, 10)

        self.get_logger().info(
            f'Obstacle detector ready\n'
            f'  scan:    {scan_topic}\n'
            f'  box:     x[{self._x_min}, {self._x_max}]  '
            f'y[{self._y_min}, {self._y_max}]  (metres)\n'
            f'  cluster: size>={self._min_cluster_sz}  '
            f'gap<={self._cluster_dist} m  '
            f'min_clusters={self._min_clusters}\n'
            f'  auto_pause: {self._auto_pause}'
        )

    # ── Scan callback ──────────────────────────────────────────────────────────

    def _cb_scan(self, msg: LaserScan):
        # 1. Convert polar → Cartesian, keep only points inside the box
        points = []
        angle = msg.angle_min
        for r in msg.ranges:
            if math.isfinite(r) and msg.range_min <= r <= msg.range_max:
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                if self._x_min <= x <= self._x_max and self._y_min <= y <= self._y_max:
                    points.append((x, y))
            angle += msg.angle_increment

        # 2. Cluster consecutive nearby points
        clusters = self._find_clusters(points)

        # 3. Evaluate trigger
        obstacle = len(clusters) >= self._min_clusters

        # 4. Publish only on state change
        if obstacle != self._obstacle_active:
            self._obstacle_active = obstacle
            self._pub_obstacle.publish(Bool(data=obstacle))
            if self._auto_pause:
                self._pub_stop.publish(Bool(data=obstacle))
            if obstacle:
                self.get_logger().warn(
                    f'OBSTACLE DETECTED: {len(clusters)} cluster(s) with '
                    f'{sum(len(c) for c in clusters)} points in box'
                )
            else:
                self.get_logger().info('Obstacle CLEARED — coverage can resume')

    # ── Clustering ────────────────────────────────────────────────────────────

    def _find_clusters(self, points: list) -> list:
        """
        Group scan-ordered points into clusters.
        A new cluster starts whenever the Euclidean gap between consecutive
        points exceeds cluster_distance.  Returns only clusters that meet
        min_cluster_size.
        """
        if not points:
            return []

        clusters = []
        current  = [points[0]]

        for i in range(1, len(points)):
            dx = points[i][0] - points[i - 1][0]
            dy = points[i][1] - points[i - 1][1]
            if math.sqrt(dx * dx + dy * dy) <= self._cluster_dist:
                current.append(points[i])
            else:
                if len(current) >= self._min_cluster_sz:
                    clusters.append(current)
                current = [points[i]]

        if len(current) >= self._min_cluster_sz:
            clusters.append(current)

        return clusters


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
