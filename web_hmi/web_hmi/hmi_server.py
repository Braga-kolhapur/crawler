#!/usr/bin/env python3
"""
Coverage HMI Web Server
=======================
Mobile-first web interface for monitoring and controlling the coverage robot.
Serves a real-time interactive HMI over HTTP/WebSocket (Flask-SocketIO).

Usage:
    pip install flask flask-socketio pillow pyyaml
    python3 hmi_server.py coverage_plan.json [--host 0.0.0.0] [--port 5002]

    Then open on any device on the same Wi-Fi network:
        http://<your-pc-ip>:5002

ROS2 topics published  (when rclpy is available):
    /coverage_stop      std_msgs/Bool    True=stop,  False=resume
    /coverage_start     std_msgs/Bool    True=begin executing the plan
    /coverage_sequence  std_msgs/String  JSON list — new execution order
    /cmd_vel            geometry_msgs/Twist  manual teleop velocity

ROS2 topics subscribed:
    /coverage_status    std_msgs/String  JSON blob from ros2_plan_publisher
    /amcl_pose          geometry_msgs/PoseWithCovarianceStamped  robot pose
    /map                nav_msgs/OccupancyGrid  live SLAM map
"""

import os, sys, json, threading, base64, io, argparse, socket as _socket
import subprocess as _subprocess
from pathlib import Path
from typing import Any, Dict, List

try:
    from flask import Flask, render_template, jsonify
    from flask_socketio import SocketIO, emit
except ImportError:
    sys.exit("pip install flask flask-socketio")

try:
    from PIL import Image
except ImportError:
    sys.exit("pip install Pillow")

# ── Optional ROS2 ─────────────────────────────────────────────────────────────
try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Bool, String
    from geometry_msgs.msg import PoseWithCovarianceStamped, Twist, TwistStamped
    from nav_msgs.msg import OccupancyGrid
    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False

# ── Template path resolution for ROS2 package ────────────────────────────────
def _get_template_folder():
    """Find templates folder in ROS2 install space or development directory."""
    # Try ROS2 share directory first (installed package)
    try:
        from ament_index_python.packages import get_package_share_directory
        pkg_share = get_package_share_directory('web_hmi')
        template_dir = os.path.join(pkg_share, 'templates')
        if os.path.exists(template_dir):
            return template_dir
    except Exception:
        pass

    # Fall back to relative path (development / standalone mode)
    script_dir = Path(__file__).resolve().parent.parent
    template_dir = script_dir / 'templates'
    if template_dir.exists():
        return str(template_dir)

    # Last resort: check current directory
    if Path('templates').exists():
        return 'templates'

    raise RuntimeError("Could not find templates directory")

# ── Flask / SocketIO ──────────────────────────────────────────────────────────
app = Flask(__name__, template_folder=_get_template_folder())
app.config["SECRET_KEY"] = "cov_hmi_key"
sio = SocketIO(app, cors_allowed_origins="*", async_mode="threading")

# ── Shared state (guarded by _lock) ──────────────────────────────────────────
_lock = threading.Lock()
_st: Dict[str, Any] = {
    "plan":          {},
    "sequence":      [],
    "is_running":    False,
    "is_paused":     False,
    "current_zone":  None,
    "robot_x":       None,
    "robot_y":       None,
    "visited":       [],      # [{zone: "a", idx: 3}, …]
    "coverage_pct":  0.0,
    "ros_connected": False,
    "live_map_b64":  "",      # latest OccupancyGrid as PNG base64
}
_plan_dir  = "."
_maps_dir  = "."   # resolved at startup to cus_nav2_config/maps/ if available
_ros_node  = None
_use_stamped = False  # set by main() before starting ROS thread
_sim_mode    = False  # toggled by UI sim checkbox
_procs: Dict[str, Any] = {}  # tracked child subprocesses


def _resolve_maps_dir() -> str:
    """Return the maps/ directory of cus_nav2_config if the package is installed."""
    try:
        from ament_index_python.packages import get_package_share_directory
        pkg = get_package_share_directory('cus_nav2_config')
        d = os.path.join(pkg, 'maps')
        os.makedirs(d, exist_ok=True)
        return d
    except Exception:
        pass
    return "."


# ── State helpers ─────────────────────────────────────────────────────────────

def _snapshot() -> dict:
    with _lock:
        return {
            "is_running":    _st["is_running"],
            "is_paused":     _st["is_paused"],
            "current_zone":  _st["current_zone"],
            "coverage_pct":  round(_st["coverage_pct"], 1),
            "robot_x":       _st["robot_x"],
            "robot_y":       _st["robot_y"],
            "sequence":      list(_st["sequence"]),
            "visited":       list(_st["visited"]),
            "ros_connected": _st["ros_connected"],
        }


def _node_status() -> dict:
    def _running(key):
        return key in _procs and _procs[key].poll() is None
    return {
        "robot":    _running("robot"),
        "slam":     _running("slam"),
        "autonomy": _running("autonomy"),
        "obstacle": _running("obstacle"),
        # legacy keys kept for UI back-compat
        "amcl":     _running("autonomy"),
        "nav2":     _running("autonomy"),
    }


def _mark_visited(rx: float, ry: float, threshold: float = 0.25):
    """Called while holding _lock. Mark waypoints near (rx, ry) as visited."""
    zones = _st["plan"].get("zones", {})
    visited_set = {(v["zone"], v["idx"]) for v in _st["visited"]}
    total = done = 0

    for key in _st["sequence"]:
        for idx, wp in enumerate(zones.get(key, {}).get("waypoints", [])):
            if wp.get("type") == "transition":
                continue
            total += 1
            tag = (key, idx)
            if tag in visited_set:
                done += 1
            elif abs(wp["x"] - rx) < threshold and abs(wp["y"] - ry) < threshold:
                _st["visited"].append({"zone": key, "idx": idx})
                visited_set.add(tag)
                done += 1

    _st["coverage_pct"] = (done / total * 100.0) if total else 0.0


# ── Plan loading ──────────────────────────────────────────────────────────────

def load_plan(path: str) -> bool:
    global _plan_dir
    try:
        with open(path) as f:
            plan = json.load(f)
        _plan_dir = str(Path(path).resolve().parent)
        # Cache resolved plan path for autonomy launch
        on_start_autonomy._plan_file_cache = str(Path(path).resolve())
        with _lock:
            _st["plan"]         = plan
            _st["sequence"]     = list(plan.get("sequence", []))
            _st["visited"]      = []
            _st["coverage_pct"] = 0.0
            _st["current_zone"] = None
        return True
    except Exception as e:
        print(f"[hmi] Could not load plan: {e}")
        return False


def _map_image_b64() -> str:
    """Read the map PGM referenced by the plan and return a PNG as base64."""
    with _lock:
        pgm = _st["plan"].get("map", {}).get("pgm_file", "map.pgm")

    # Only look in cus_nav2_config/maps/
    candidates = []
    try:
        from ament_index_python.packages import get_package_share_directory
        pkg_maps = os.path.join(get_package_share_directory('cus_nav2_config'), 'maps')
        candidates.append(os.path.join(pkg_maps, os.path.basename(pgm)))
        import glob as _glob
        candidates.extend(_glob.glob(os.path.join(pkg_maps, '*.pgm')))
    except Exception:
        pass

    for candidate in candidates:
        if os.path.exists(candidate):
            try:
                img = Image.open(candidate).convert("RGB")
                buf = io.BytesIO()
                img.save(buf, "PNG")
                return base64.b64encode(buf.getvalue()).decode()
            except Exception as e:
                print(f"[hmi] Map image error: {e}")
    return ""


# ── Node process management ───────────────────────────────────────────────────

def _start_ros_node(name: str, cmd: List[str]) -> bool:
    """Launch a ROS2 subprocess. Returns True if newly started."""
    if name in _procs and _procs[name].poll() is None:
        return False  # already running
    try:
        p = _subprocess.Popen(cmd,
                              stdout=_subprocess.DEVNULL,
                              stderr=_subprocess.DEVNULL)
        _procs[name] = p
        print(f"[hmi] Started '{name}': {' '.join(cmd)}")
        return True
    except Exception as e:
        print(f"[hmi] Could not start '{name}': {e}")
        return False


def _stop_ros_node(name: str) -> bool:
    """Terminate a tracked subprocess."""
    p = _procs.pop(name, None)
    if not p:
        return False
    try:
        p.terminate()
        p.wait(timeout=3)
    except _subprocess.TimeoutExpired:
        p.kill()
    print(f"[hmi] Stopped '{name}'")
    return True


def _stop_all_nodes():
    for name in list(_procs.keys()):
        _stop_ros_node(name)


# ── ROS2 bridge ───────────────────────────────────────────────────────────────

if HAS_ROS2:
    class _Bridge(Node):
        def __init__(self, use_stamped=False):
            super().__init__("coverage_hmi")
            self._use_stamped = use_stamped
            self._pub_stop   = self.create_publisher(Bool,   "/coverage_stop",     10)
            self._pub_seq    = self.create_publisher(String, "/coverage_sequence", 10)

            # TRANSIENT_LOCAL so coverage_path_follower receives the start signal
            # even if it subscribes slightly after the message is published
            from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
            _start_qos = QoSProfile(depth=1)
            _start_qos.durability  = DurabilityPolicy.TRANSIENT_LOCAL
            _start_qos.reliability = ReliabilityPolicy.RELIABLE
            self._pub_start  = self.create_publisher(Bool, "/coverage_start", _start_qos)
            if use_stamped:
                self._pub_cmdvel = self.create_publisher(TwistStamped, "/cmd_vel", 10)
                self.get_logger().info("HMI cmd_vel publisher using TwistStamped")
            else:
                self._pub_cmdvel = self.create_publisher(Twist, "/cmd_vel", 10)

            self.create_subscription(String, "/coverage_status",
                                     self._cb_status, 10)
            self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose",
                                     self._cb_pose, 10)

            # Live SLAM map — use TRANSIENT_LOCAL so we get the last map on subscribe
            try:
                from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
                map_qos = QoSProfile(depth=1)
                map_qos.durability  = DurabilityPolicy.TRANSIENT_LOCAL
                map_qos.reliability = ReliabilityPolicy.RELIABLE
            except Exception:
                map_qos = 1  # fallback to depth-only QoS
            self.create_subscription(OccupancyGrid, "/map", self._cb_map, map_qos)
            self.create_subscription(Bool, "/obstacle_detected",
                                     self._cb_obstacle, 10)

            with _lock:
                _st["ros_connected"] = True
            self.get_logger().info("HMI bridge ready")

        # ── Subscribers ───────────────────────────────────────────────────────

        def _cb_status(self, msg: String):
            try:
                d = json.loads(msg.data)
                with _lock:
                    _st["current_zone"] = d.get("active_zone")
            except Exception:
                pass
            sio.emit("state", _snapshot())

        def _cb_pose(self, msg: PoseWithCovarianceStamped):
            rx = msg.pose.pose.position.x
            ry = msg.pose.pose.position.y
            with _lock:
                _st["robot_x"] = rx
                _st["robot_y"] = ry
                _mark_visited(rx, ry)
            sio.emit("state", _snapshot())

        def _cb_obstacle(self, msg: Bool):
            sio.emit("obstacle_detected", {"detected": msg.data})

        def _cb_map(self, msg: OccupancyGrid):
            """Convert OccupancyGrid to PNG and store as base64."""
            try:
                w, h = msg.info.width, msg.info.height
                if w == 0 or h == 0:
                    return
                # Build RGB image: 0=free→white, 100=occupied→black, -1=unknown→gray
                pix = bytearray(w * h * 3)
                for i, v in enumerate(msg.data):
                    if v == 0:
                        c = 255
                    elif v == 100:
                        c = 0
                    elif v < 0:
                        c = 205   # unknown → light gray
                    else:
                        c = max(0, 255 - int(v * 2.55))
                    pix[i * 3] = pix[i * 3 + 1] = pix[i * 3 + 2] = c
                img = Image.frombytes("RGB", (w, h), bytes(pix))
                img = img.transpose(Image.FLIP_TOP_BOTTOM)  # ROS origin = bottom-left
                if w > 800 or h > 800:
                    img.thumbnail((800, 800), Image.NEAREST)
                buf = io.BytesIO()
                img.save(buf, "PNG")
                b64 = base64.b64encode(buf.getvalue()).decode()
                with _lock:
                    _st["live_map_b64"] = b64
            except Exception as e:
                self.get_logger().warn(f"Map cb error: {e}")

        # ── Publishers ────────────────────────────────────────────────────────

        def pub_stop(self, flag: bool):
            self._pub_stop.publish(Bool(data=flag))

        def pub_start(self):
            self._pub_stop.publish(Bool(data=False))
            self._pub_start.publish(Bool(data=True))

        def pub_seq(self, seq: List[str]):
            self._pub_seq.publish(String(data=json.dumps(seq)))

        def pub_cmdvel(self, linear_x: float, angular_z: float):
            if self._use_stamped:
                msg = TwistStamped()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'base_link'
                msg.twist.linear.x  = float(linear_x)
                msg.twist.angular.z = float(angular_z)
            else:
                msg = Twist()
                msg.linear.x  = float(linear_x)
                msg.angular.z = float(angular_z)
            self._pub_cmdvel.publish(msg)


def _ros_thread():
    global _ros_node
    rclpy.init()
    _ros_node = _Bridge(use_stamped=_use_stamped)
    try:
        rclpy.spin(_ros_node)
    finally:
        _ros_node.destroy_node()
        rclpy.try_shutdown()


# ── HTTP routes ───────────────────────────────────────────────────────────────

@app.route("/")
def index():
    return render_template("index.html")


@app.route("/api/plan")
def api_plan():
    with _lock:
        return jsonify({"plan": _st["plan"], "sequence": _st["sequence"]})


@app.route("/api/map_image")
def api_map_image():
    return jsonify({"b64": _map_image_b64()})


@app.route("/api/live_map_image")
def api_live_map_image():
    with _lock:
        b64 = _st["live_map_b64"]
    return jsonify({"b64": b64})


@app.route("/api/state")
def api_state():
    return jsonify(_snapshot())


@app.route("/api/node_status")
def api_node_status():
    return jsonify(_node_status())


# ── SocketIO events — coverage plan control ───────────────────────────────────

@sio.on("connect")
def on_connect():
    emit("state", _snapshot())
    emit("node_status", _node_status())


@sio.on("cmd_start")
def on_start(data=None):
    seq = (data or {}).get("sequence", [])
    with _lock:
        _st["is_running"] = True
        _st["is_paused"]  = False
        if seq:
            _st["sequence"] = seq
    if _ros_node:
        _ros_node.pub_start()
        if seq:
            _ros_node.pub_seq(seq)
    sio.emit("state", _snapshot())


@sio.on("cmd_stop")
def on_stop():
    with _lock:
        _st["is_running"] = False
        _st["is_paused"]  = False
    if _ros_node:
        _ros_node.pub_stop(True)
    sio.emit("state", _snapshot())


@sio.on("cmd_pause")
def on_pause():
    with _lock:
        _st["is_paused"] = not _st["is_paused"]
        paused = _st["is_paused"]
    if _ros_node:
        _ros_node.pub_stop(paused)
    sio.emit("state", _snapshot())


@sio.on("update_sequence")
def on_update_seq(data):
    seq = data.get("sequence", [])
    with _lock:
        _st["sequence"] = seq
        running = _st["is_running"]
    if _ros_node and running:
        _ros_node.pub_seq(seq)
    sio.emit("state", _snapshot())


@sio.on("reset_progress")
def on_reset():
    with _lock:
        _st["visited"]      = []
        _st["coverage_pct"] = 0.0
    sio.emit("state", _snapshot())


@sio.on("inject_pose")   # demo / testing: manually set robot position
def on_inject_pose(data):
    rx, ry = float(data.get("x", 0)), float(data.get("y", 0))
    with _lock:
        _st["robot_x"] = rx
        _st["robot_y"] = ry
        _mark_visited(rx, ry)
    sio.emit("state", _snapshot())


# ── SocketIO events — manual mode (teleop + mapping) ─────────────────────────

@sio.on("set_sim")
def on_set_sim(data):
    """Toggle simulation mode from the UI sim checkbox."""
    global _sim_mode, _use_stamped
    _sim_mode    = bool((data or {}).get("sim", False))
    _use_stamped = _sim_mode   # sim → TwistStamped, real → Twist


@sio.on("cmd_vel")
def on_cmd_vel(data):
    """Publish Twist/TwistStamped from virtual joystick."""
    lx = float(data.get("linear_x",  0))
    az = float(data.get("angular_z", 0))
    if _ros_node:
        _ros_node.pub_cmdvel(lx, az)


@sio.on("start_mapping")
def on_start_mapping():
    """Launch slam_toolbox for SLAM mapping (respects sim mode)."""
    use_sim = "true" if _sim_mode else "false"

    slam_cmd = [
        "ros2", "launch", "slam_toolbox", "online_async_launch.py",
        f"use_sim_time:={use_sim}",
    ]
    try:
        from ament_index_python.packages import get_package_share_directory
        params_file = os.path.join(
            get_package_share_directory('cus_nav2_config'),
            'params', 'mapper_params_online_async.yaml'
        )
        if os.path.exists(params_file):
            slam_cmd.append(f"slam_params_file:={params_file}")
    except Exception:
        pass

    _start_ros_node("slam", slam_cmd)
    '''if not _sim_mode:
        _start_ros_node("static_tf", [
            "ros2", "run", "tf2_ros", "static_transform_publisher",
            "0.11", "0", "0.15", "0", "0", "0", "base_link", "laser",
        ])'''
    sio.emit("node_status", _node_status())


@sio.on("stop_mapping")
def on_stop_mapping():
    _stop_ros_node("slam")
    _stop_ros_node("static_tf")
    sio.emit("node_status", _node_status())


@sio.on("start_robot")
def on_start_robot():
    """Launch robot driver — turtlebot3_gazebo in sim mode, create_bringup + rplidar otherwise."""
    if _sim_mode:
        _start_ros_node("robot", [
            "ros2", "launch", "turtlebot3_gazebo", "turtlebot3_world.launch.py",
        ])
        sio.emit("toast", "Starting TurtleBot3 Gazebo...")
    else:
        _start_ros_node("robot", [
            "ros2", "launch", "create_bringup", "create_2.launch",
        ])
        _start_ros_node("rplidar", [
            "ros2", "launch", "rplidar_ros", "rplidar_c1_launch.py",
        ])
        _start_ros_node("static_tf", [
            "ros2", "run", "tf2_ros", "static_transform_publisher",
            "0.11", "0", "0.15", "0", "0", "0", "base_link", "laser",
        ])
        sio.emit("toast", "Starting Create 2 + RPLidar...")
    sio.emit("node_status", _node_status())


@sio.on("stop_robot")
def on_stop_robot():
    """Stop the robot driver and any associated hardware nodes."""
    if _ros_node:
        _ros_node.pub_cmdvel(0.0, 0.0)
    _stop_ros_node("robot")
    _stop_ros_node("rplidar")
    _stop_ros_node("static_tf")
    sio.emit("node_status", _node_status())
    sio.emit("toast", "Stopping robot driver...")


@sio.on("save_map")
def on_save_map(data):
    """Run map_saver_cli in a background thread, saving into cus_nav2_config/maps/."""
    name     = (data or {}).get("name", "map_001").strip().replace("/", "_") or "map_001"
    maps_dir = _maps_dir
    try:
        os.makedirs(maps_dir, exist_ok=True)
    except Exception as e:
        sio.emit("toast", f"Cannot create maps dir: {e}")
        return
    out_path = os.path.join(maps_dir, name)

    def _do_save():
        try:
            r = _subprocess.run(
                ["ros2", "run", "nav2_map_server", "map_saver_cli",
                 "-f", out_path],
                capture_output=True, text=True, timeout=15,
            )
            if r.returncode == 0:
                sio.emit("toast", f"✓ Map saved: maps/{name}")
            else:
                msg = (r.stderr or r.stdout or "unknown error")[:120]
                sio.emit("toast", f"Save error: {msg}")
        except _subprocess.TimeoutExpired:
            sio.emit("toast", "Map save timed out (15 s)")
        except Exception as e:
            sio.emit("toast", f"Save failed: {e}")

    threading.Thread(target=_do_save, daemon=True).start()
    sio.emit("toast", f"Saving → maps/{name}…")


# ── SocketIO events — auto / autonomy mode ────────────────────────────────────

@sio.on("start_autonomy")
def on_start_autonomy(data=None):
    """
    Launch the full autonomous coverage stack via cus_nav2_config auto_mode.launch.py
    (map_server + amcl + controller_server + plan_publisher + path_follower).
    """
    _stop_ros_node("teleop")

    # Resolve map yaml from cus_nav2_config/maps/
    map_yaml = ""
    try:
        from ament_index_python.packages import get_package_share_directory
        pkg = get_package_share_directory('cus_nav2_config')
        map_yaml = os.path.join(pkg, 'maps', 'my_map.yaml')
    except Exception:
        pass

    # Resolve plan file from cus_nav2_config/paths/
    import glob as _glob
    plan_file = ""
    try:
        from ament_index_python.packages import get_package_share_directory
        pkg = get_package_share_directory('cus_nav2_config')
        candidates = sorted(_glob.glob(os.path.join(pkg, 'paths', '*.json')))
        if candidates:
            plan_file = candidates[0]
    except Exception:
        pass

    use_sim = "true" if _sim_mode else "false"
    cmd = [
        "ros2", "launch", "cus_nav2_config", "auto_mode.launch.py",
        f"map:={map_yaml}",
        f"use_sim_time:={use_sim}",
        f"stamped_cmd_vel:={use_sim}",
    ]
    if plan_file:
        cmd.append(f"plan_file:={plan_file}")

    _start_ros_node("autonomy", cmd)

    sio.emit("state", _snapshot())
    sio.emit("node_status", _node_status())
    sio.emit("toast", "Autonomy stack started — set initial pose, then press Start Coverage")


@sio.on("stop_autonomy")
def on_stop_autonomy():
    """Kill the autonomy launch group and bring the robot to a halt."""
    with _lock:
        _st["is_running"] = False
        _st["is_paused"]  = False
    if _ros_node:
        _ros_node.pub_stop(True)
        _ros_node.pub_cmdvel(0.0, 0.0)
    _stop_ros_node("autonomy")
    sio.emit("state", _snapshot())
    sio.emit("node_status", _node_status())


@sio.on("start_coverage")
def on_start_coverage():
    """Send /coverage_start=True after AMCL has converged and initial pose is set."""
    if _ros_node:
        _ros_node.pub_start()
    with _lock:
        _st["is_running"] = True
        _st["is_paused"]  = False
    sio.emit("state", _snapshot())
    sio.emit("toast", "Coverage started")


# ── SocketIO events — obstacle detector ──────────────────────────────────────

@sio.on("start_obstacle_detector")
def on_start_obstacle_detector():
    """Launch the obstacle detector node."""
    use_sim = "true" if _sim_mode else "false"
    _start_ros_node("obstacle", [
        "ros2", "launch", "obstacle_detector", "obstacle_detector.launch.py",
        f"use_sim_time:={use_sim}",
    ])
    sio.emit("node_status", _node_status())
    sio.emit("toast", "Obstacle detector started")


@sio.on("stop_obstacle_detector")
def on_stop_obstacle_detector():
    """Stop the obstacle detector node and clear any active obstacle state."""
    _stop_ros_node("obstacle")
    sio.emit("obstacle_detected", {"detected": False})
    sio.emit("node_status", _node_status())
    sio.emit("toast", "Obstacle detector stopped")


# ── Entry point ───────────────────────────────────────────────────────────────

def _local_ip() -> str:
    try:
        with _socket.socket(_socket.AF_INET, _socket.SOCK_DGRAM) as s:
            s.connect(("8.8.8.8", 80))
            return s.getsockname()[0]
    except Exception:
        return "localhost"


def main():
    # Filter out ROS2 arguments if present (when launched via ros2 launch)
    import sys
    argv = sys.argv[1:]

    if HAS_ROS2:
        try:
            # Remove ROS-specific arguments (--ros-args, -r, etc.)
            from rclpy.utilities import remove_ros_args
            argv = remove_ros_args(argv)
        except (ImportError, Exception):
            # Fallback: manually filter out --ros-args and everything after
            if '--ros-args' in argv:
                argv = argv[:argv.index('--ros-args')]

    ap = argparse.ArgumentParser(description="Coverage HMI web server")
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=5006)
    ap.add_argument("--stamped", action="store_true",
                    help="Publish cmd_vel as TwistStamped instead of Twist")
    args = ap.parse_args(argv)

    global _use_stamped, _maps_dir
    _use_stamped = args.stamped
    _maps_dir    = _resolve_maps_dir()

    # Auto-discover plan from cus_nav2_config/paths/
    plan_file = None
    try:
        from ament_index_python.packages import get_package_share_directory
        import glob as _glob
        pkg_paths = os.path.join(
            get_package_share_directory('cus_nav2_config'), 'paths')
        candidates = sorted(_glob.glob(os.path.join(pkg_paths, '*.json')))
        if candidates:
            plan_file = candidates[0]
    except Exception:
        pass

    if plan_file and os.path.exists(plan_file):
        load_plan(plan_file)
        print(f"[hmi] Plan loaded: {plan_file}")
    else:
        print("[hmi] No plan found in cus_nav2_config/paths/ — server ready without a plan.")

    if HAS_ROS2:
        t = threading.Thread(target=_ros_thread, daemon=True)
        t.start()
        msg_type = "TwistStamped" if _use_stamped else "Twist"
        print(f"[hmi] ROS2 bridge started (cmd_vel: {msg_type})")
    else:
        print("[hmi] ROS2 not available — running in standalone / demo mode")

    ip = _local_ip()
    print(f"\n[hmi] HMI ready:")
    print(f"      http://localhost:{args.port}     (this machine)")
    print(f"      http://{ip}:{args.port}  (phone / tablet on same Wi-Fi)\n")

    sio.run(app, host=args.host, port=args.port,
            debug=False, use_reloader=False, log_output=False,
            allow_unsafe_werkzeug=True)


if __name__ == "__main__":
    main()
