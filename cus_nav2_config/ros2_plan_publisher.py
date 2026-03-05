#!/usr/bin/env python3
"""
ROS2 Coverage Plan Publisher
============================
Reads the JSON plan saved by coverage_planner.py and publishes the waypoints
as standard ROS2 messages so any navigation stack can consume them.

Published topics
----------------
  /coverage_path       nav_msgs/Path          — full concatenated path (all zones in sequence)
  /coverage_poses      geometry_msgs/PoseArray — same points, as a pose array
  /coverage_zone_path  nav_msgs/Path          — path for the *currently active* zone only
  /coverage_status     std_msgs/String        — JSON status (current zone, progress, etc.)

Parameters
----------
  plan_file   (str)   path to the JSON plan  [default: coverage_plan.json]
  sequence    (str)   comma-separated override for zone order, e.g. "b,a,c"
  frame_id    (str)   TF frame for all messages          [default: map]
  publish_hz  (float) timer frequency in Hz              [default: 1.0]
  zone_index  (int)   publish only this zone (0-based)   [default: -1 = all]

Usage examples
--------------
  # Publish full plan at 1 Hz
  ros2 run <your_pkg> ros2_plan_publisher.py --ros-args \
      -p plan_file:=/path/to/coverage_plan.json

  # Override sequence at runtime
  ros2 run <your_pkg> ros2_plan_publisher.py --ros-args \
      -p plan_file:=plan.json -p sequence:=b,a,d

  # Publish only zone 'c' (index depends on sequence order)
  ros2 run <your_pkg> ros2_plan_publisher.py --ros-args \
      -p plan_file:=plan.json -p sequence:=c
"""

import json
import math
import sys

try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Pose, PoseArray, PoseStamped, Quaternion, Point
    from nav_msgs.msg import Path
    from std_msgs.msg import Bool, String
except ImportError:
    print(
        "rclpy not found — this script requires a sourced ROS2 environment.\n"
        "Source your ROS2 setup.bash and try again."
    )
    sys.exit(1)


# ──────────────────────────────────────────────────────────────────────────────
# Helpers
# ──────────────────────────────────────────────────────────────────────────────

def yaw_to_quaternion(yaw: float) -> Quaternion:
    """Convert a 2-D heading (yaw, radians) to a ROS Quaternion."""
    return Quaternion(
        x=0.0,
        y=0.0,
        z=math.sin(yaw / 2.0),
        w=math.cos(yaw / 2.0),
    )


def waypoints_to_path(waypoints, header) -> Path:
    path = Path()
    path.header = header
    for wp in waypoints:
        ps = PoseStamped()
        ps.header = header
        ps.pose.position = Point(x=wp["x"], y=wp["y"], z=0.0)
        ps.pose.orientation = yaw_to_quaternion(wp.get("yaw", 0.0))
        path.poses.append(ps)
    return path


def waypoints_to_pose_array(waypoints, header) -> PoseArray:
    pa = PoseArray()
    pa.header = header
    for wp in waypoints:
        p = Pose()
        p.position = Point(x=wp["x"], y=wp["y"], z=0.0)
        p.orientation = yaw_to_quaternion(wp.get("yaw", 0.0))
        pa.poses.append(p)
    return pa


# ──────────────────────────────────────────────────────────────────────────────
# Node
# ──────────────────────────────────────────────────────────────────────────────

class CoveragePlanPublisher(Node):

    def __init__(self):
        super().__init__("coverage_plan_publisher")

        # Parameters
        self.declare_parameter("plan_file",  "coverage_plan.json")
        self.declare_parameter("sequence",   "")          # "" → use plan's sequence
        self.declare_parameter("frame_id",   "map")
        self.declare_parameter("publish_hz", 1.0)
        self.declare_parameter("zone_index", -1)          # -1 → publish all zones

        # Publishers
        self._pub_path   = self.create_publisher(Path,      "/coverage_path",       10)
        self._pub_poses  = self.create_publisher(PoseArray, "/coverage_poses",      10)
        self._pub_zone   = self.create_publisher(Path,      "/coverage_zone_path",  10)
        self._pub_status = self.create_publisher(String,    "/coverage_status",     10)

        # ── Subscribers (HMI control topics) ─────────────────────────────────
        # /coverage_stop  Bool   — True = pause publishing, False = resume
        # /coverage_start Bool   — True = (re)start from beginning of sequence
        # /coverage_sequence String — JSON list, overrides the execution order
        self.create_subscription(Bool,   "/coverage_stop",     self._cb_stop,  10)
        self.create_subscription(Bool,   "/coverage_start",    self._cb_start, 10)
        self.create_subscription(String, "/coverage_sequence", self._cb_seq,   10)

        # Load plan
        plan_file = self.get_parameter("plan_file").value
        self._plan = self._load_plan(plan_file)
        self.get_logger().info(
            f"Coverage plan loaded from '{plan_file}'  "
            f"({len(self._plan.get('zones', {}))} zones)"
        )

        hz = self.get_parameter("publish_hz").value
        self._timer = self.create_timer(1.0 / max(0.01, hz), self._publish)

        # Runtime state
        self._active_zone_idx = 0
        self._stopped = False          # True → publish empty paths
        self._seq_override: list = []  # non-empty → use instead of plan sequence

    # ── Plan loading ──────────────────────────────────────────────────────────

    def _load_plan(self, path: str) -> dict:
        try:
            with open(path) as f:
                return json.load(f)
        except FileNotFoundError:
            self.get_logger().error(f"Plan file not found: '{path}'")
            return {}
        except json.JSONDecodeError as exc:
            self.get_logger().error(f"JSON parse error in '{path}': {exc}")
            return {}

    # ── HMI control callbacks ────────────────────────────────────────────────

    def _cb_stop(self, msg: Bool):
        self._stopped = msg.data
        if self._stopped:
            # Immediately publish empty paths so the nav stack stops
            header = self._make_header()
            self._pub_path .publish(Path(header=header))
            self._pub_poses.publish(PoseArray(header=header))
            self._pub_zone .publish(Path(header=header))
            self.get_logger().info("Coverage STOPPED by HMI")
        else:
            self.get_logger().info("Coverage RESUMED by HMI")

    def _cb_start(self, msg: Bool):
        if msg.data:
            self._stopped         = False
            self._active_zone_idx = 0
            self.get_logger().info("Coverage START received from HMI")

    def _cb_seq(self, msg: String):
        try:
            seq = json.loads(msg.data)
            self._seq_override    = seq
            self._active_zone_idx = 0
            self.get_logger().info(f"Sequence updated by HMI: {seq}")
        except Exception as exc:
            self.get_logger().warn(f"Bad sequence message: {exc}")

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _active_sequence(self):
        """Resolve the zone execution sequence.
        Priority: HMI live override > ROS parameter override > plan sequence."""
        zones = self._plan.get("zones", {})

        # 1. Live sequence pushed by HMI
        if self._seq_override:
            return [k for k in self._seq_override if k in zones]

        # 2. ROS parameter
        override = self.get_parameter("sequence").value.strip()
        if override:
            keys = [k.strip() for k in override.split(",") if k.strip() in zones]
            if keys:
                return keys

        # 3. Plan default
        seq = self._plan.get("sequence", list(zones.keys()))
        return [k for k in seq if k in zones]

    def _make_header(self):
        from std_msgs.msg import Header
        h = Header()
        h.stamp    = self.get_clock().now().to_msg()
        h.frame_id = self.get_parameter("frame_id").value
        return h

    # ── Publish ───────────────────────────────────────────────────────────────

    def _publish(self):
        if not self._plan:
            return

        seq    = self._active_sequence()
        zones  = self._plan.get("zones", {})
        header = self._make_header()

        # ── Full path (all zones in sequence) ─────────────────────────────────
        all_wps = []
        for key in seq:
            all_wps.extend(zones[key].get("waypoints", []))

        self._pub_path.publish(waypoints_to_path(all_wps, header))
        self._pub_poses.publish(waypoints_to_pose_array(all_wps, header))

        # ── Active zone path ──────────────────────────────────────────────────
        zone_idx_param = self.get_parameter("zone_index").value
        if zone_idx_param >= 0:
            # Fixed zone index
            active_key = seq[zone_idx_param % len(seq)] if seq else None
        else:
            # Cycle through zones each publish cycle (useful for visualisation)
            active_key = seq[self._active_zone_idx % len(seq)] if seq else None
            self._active_zone_idx = (self._active_zone_idx + 1) % max(1, len(seq))

        if active_key and active_key in zones:
            zone_wps = zones[active_key].get("waypoints", [])
            self._pub_zone.publish(waypoints_to_path(zone_wps, header))
        else:
            self._pub_zone.publish(Path(header=header))

        # ── Status ────────────────────────────────────────────────────────────
        status = {
            "sequence":     seq,
            "active_zone":  active_key,
            "total_zones":  len(seq),
            "total_wps":    len(all_wps),
            "frame_id":     self.get_parameter("frame_id").value,
            "stamp":        self.get_clock().now().nanoseconds,
        }
        self._pub_status.publish(String(data=json.dumps(status)))

        self.get_logger().info(
            f"Published {len(all_wps)} waypoints across {len(seq)} zones  "
            f"(active: '{active_key}')",
            throttle_duration_sec=5.0,
        )


# ──────────────────────────────────────────────────────────────────────────────
# Entry point
# ──────────────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = CoveragePlanPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
