# Web HMI - Coverage Robot Web Interface

Mobile-first web interface for monitoring and controlling the coverage robot. Provides real-time visualization, teleoperation, and autonomous coverage execution control.

## Features

### Auto Mode (Autonomous Coverage)
- Interactive map visualization with pan/zoom
- Real-time robot position tracking
- Drag-to-reorder zone execution sequence
- Coverage progress tracking
- Zone-by-zone status monitoring
- Start/Stop/Pause controls
- Automatic AMCL + Nav2 launch

### Manual Mode (Teleoperation & Mapping)
- Virtual joystick for robot control
- Live SLAM map visualization (picture-in-picture)
- Start/Stop SLAM mapping
- Save maps with custom names
- Real-time velocity feedback
- Node status monitoring

## Installation

### Prerequisites

Install Python dependencies:
```bash
pip install flask flask-socketio Pillow pyyaml
```

### Build the Package

```bash
cd ~/ros2_ws
colcon build --packages-select web_hmi
source install/setup.bash
```

## Usage

### Running the HMI Server

**Option 1: Using ROS2 launch**
```bash
# Basic launch
ros2 launch web_hmi hmi_server.launch.py

# With custom plan file
ros2 launch web_hmi hmi_server.launch.py plan_file:=/path/to/coverage_plan.json

# With custom port
ros2 launch web_hmi hmi_server.launch.py port:=8080
```

**Option 2: Using ROS2 run**
```bash
# Default settings
ros2 run web_hmi hmi_server

# With plan file
ros2 run web_hmi hmi_server coverage_plan.json

# With custom host/port
ros2 run web_hmi hmi_server --host 0.0.0.0 --port 5002
```

**Option 3: Launch with SLAM mapping**
```bash
ros2 launch web_hmi hmi_with_mapping.launch.py start_slam:=true
```

### Accessing the Web Interface

After starting the server, open a web browser and navigate to:
- On the same machine: `http://localhost:5002`
- From a mobile device on the same WiFi: `http://<your-pc-ip>:5002`

The server will print the access URLs when it starts.

## ROS2 Topics

### Published Topics
- `/coverage_stop` (std_msgs/Bool) - Stop/resume coverage execution
- `/coverage_start` (std_msgs/Bool) - Start coverage plan execution
- `/coverage_sequence` (std_msgs/String) - JSON array of zone execution order
- `/cmd_vel` (geometry_msgs/Twist) - Manual teleop velocity commands

### Subscribed Topics
- `/coverage_status` (std_msgs/String) - Coverage execution status from planner
- `/amcl_pose` (geometry_msgs/PoseWithCovarianceStamped) - Robot localization
- `/map` (nav_msgs/OccupancyGrid) - Live SLAM map for visualization

## Integration with Other Packages

This package integrates with:

1. **slam_toolbox** - For SLAM mapping in manual mode
2. **nav2_bringup** - For AMCL localization and Nav2 navigation
3. **nav2_map_server** - For saving generated maps
4. **Coverage Planner** (your package) - Receives commands, publishes status

## Configuration

### Coverage Plan JSON Format

The HMI expects a coverage plan JSON file with this structure:
```json
{
  "map": {
    "pgm_file": "map.pgm",
    "resolution": 0.05,
    "origin": [0, 0, 0],
    "height_px": 400
  },
  "zones": {
    "a": {
      "color": "#E74C3C",
      "pixel_bounds": {"x1": 10, "y1": 10, "x2": 100, "y2": 100},
      "waypoints": [
        {"x": 1.0, "y": 1.0, "type": "coverage", "stripe": 0},
        ...
      ]
    }
  },
  "sequence": ["a", "b", "c"]
}
```

## Development

### Project Structure
```
web_hmi/
├── web_hmi/
│   ├── __init__.py
│   └── hmi_server.py          # Main Flask/SocketIO server
├── templates/
│   └── index.html             # Web UI (HTML/CSS/JS)
├── launch/
│   ├── hmi_server.launch.py
│   └── hmi_with_mapping.launch.py
├── package.xml
├── setup.py
└── README.md
```

## Troubleshooting

### Templates not found
If you get a template error, ensure the package is properly installed:
```bash
colcon build --packages-select web_hmi --symlink-install
source install/setup.bash
```

### ROS2 not available
The HMI can run in standalone mode without ROS2 (for UI development/testing):
```bash
python3 web_hmi/web_hmi/hmi_server.py test_plan.json
```

### Port already in use
Change the port:
```bash
ros2 run web_hmi hmi_server --port 8080
```

### Cannot access from mobile device
1. Ensure firewall allows connections on the HMI port
2. Make sure both devices are on the same WiFi network
3. Use the IP address printed by the server on startup

## License

MIT
