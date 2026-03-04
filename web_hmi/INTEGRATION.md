# Web HMI Integration Guide

This guide explains how to integrate the Web HMI with other packages in your ROS2 workspace.

## Overview

The Web HMI acts as a **central control and monitoring interface** for your coverage robot system. It:
- Provides a web-based UI accessible from mobile devices
- Publishes commands to start/stop/pause coverage execution
- Subscribes to robot status, pose, and map data
- Manages ROS2 nodes (SLAM, AMCL, Nav2) through subprocess control

## Architecture

```
┌─────────────────┐
│   Web Browser   │  (Mobile/Desktop)
│  (Port 5002)    │
└────────┬────────┘
         │ HTTP/WebSocket
         ↓
┌─────────────────────────────────────────────┐
│           Web HMI Server                    │
│  (Flask + SocketIO + ROS2 Node)            │
│                                             │
│  Publishers:                                │
│  • /coverage_start                          │
│  • /coverage_stop                           │
│  • /coverage_sequence                       │
│  • /cmd_vel                                 │
│                                             │
│  Subscribers:                               │
│  • /coverage_status                         │
│  • /amcl_pose                               │
│  • /map                                     │
└──────────┬──────────────────────────────────┘
           │
           │ ROS2 Topics
           ↓
┌──────────────────────────────────────────────┐
│      Your Coverage System                    │
│  (Coverage Planner, Nav2, SLAM, etc.)       │
└──────────────────────────────────────────────┘
```

## Integration Points

### 1. Coverage Planner Integration

Your coverage planner package should:

#### Subscribe to HMI Commands:
```python
# /coverage_start (std_msgs/Bool)
# When True, begin executing the coverage plan
def coverage_start_callback(msg):
    if msg.data:
        self.start_coverage_execution()

# /coverage_stop (std_msgs/Bool)
# When True, stop; when False, resume
def coverage_stop_callback(msg):
    if msg.data:
        self.pause_or_stop_coverage()
    else:
        self.resume_coverage()

# /coverage_sequence (std_msgs/String)
# JSON array of zone IDs to execute in order
def coverage_sequence_callback(msg):
    new_sequence = json.loads(msg.data)
    self.update_execution_order(new_sequence)
```

#### Publish Status Updates:
```python
# /coverage_status (std_msgs/String)
# Publish JSON status updates for the HMI to display
status_msg = String()
status_msg.data = json.dumps({
    "active_zone": "zone_a",      # Currently executing zone
    "state": "running",            # running, paused, stopped, completed
    "progress": 0.65,              # 0.0 to 1.0
    "message": "Covering zone A"
})
self.status_publisher.publish(status_msg)
```

### 2. Navigation Integration

The HMI automatically launches and manages:
- **AMCL** (`nav2_bringup/localization_launch.py`)
- **Nav2** (`nav2_bringup/navigation_launch.py`)
- **SLAM Toolbox** (`slam_toolbox/online_async_launch.py`)

Your system should:
- Subscribe to `/cmd_vel` for manual control mode
- Publish to `/amcl_pose` for robot localization
- Publish to `/map` for live map updates

### 3. Robot Driver Integration

Your robot driver (e.g., `create_driver`) should:
- Subscribe to `/cmd_vel` for velocity commands
- Publish odometry and sensor data
- Work with AMCL for localization

## Example: Simple Coverage Planner Node

Here's a minimal example showing how to create a coverage planner that integrates with the HMI:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
import json

class SimpleCoveragePlanner(Node):
    def __init__(self):
        super().__init__('simple_coverage_planner')

        # Subscribers (listen to HMI commands)
        self.create_subscription(Bool, '/coverage_start',
                               self.start_callback, 10)
        self.create_subscription(Bool, '/coverage_stop',
                               self.stop_callback, 10)
        self.create_subscription(String, '/coverage_sequence',
                               self.sequence_callback, 10)

        # Publishers (send status to HMI)
        self.status_pub = self.create_publisher(String, '/coverage_status', 10)

        # State
        self.is_running = False
        self.is_paused = False
        self.current_zone = None
        self.sequence = []

        # Timer to publish status updates
        self.create_timer(1.0, self.publish_status)

        self.get_logger().info('Coverage Planner ready')

    def start_callback(self, msg):
        if msg.data:
            self.is_running = True
            self.is_paused = False
            self.get_logger().info('Starting coverage execution')
            # Start executing the plan...

    def stop_callback(self, msg):
        if msg.data:
            self.is_paused = True
            self.get_logger().info('Pausing coverage')
        else:
            self.is_paused = False
            self.get_logger().info('Resuming coverage')

    def sequence_callback(self, msg):
        self.sequence = json.loads(msg.data)
        self.get_logger().info(f'Updated sequence: {self.sequence}')

    def publish_status(self):
        """Publish status updates to HMI"""
        status = {
            'active_zone': self.current_zone,
            'state': 'running' if self.is_running and not self.is_paused else 'stopped',
            'progress': self.calculate_progress(),
            'message': f'Executing zone {self.current_zone}'
        }
        msg = String()
        msg.data = json.dumps(status)
        self.status_pub.publish(msg)

    def calculate_progress(self):
        # Calculate actual progress based on waypoints covered
        return 0.5  # Placeholder

def main():
    rclpy.init()
    node = SimpleCoveragePlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Existing Packages in Workspace

Your workspace already contains these packages that can integrate with Web HMI:

1. **create_behavior_tree** - Behavior tree for robot control
   - Can subscribe to `/coverage_start` to trigger coverage behaviors
   - Should publish waypoint navigation goals

2. **create_driver** - iRobot Create driver
   - Already publishes odometry
   - Subscribes to `/cmd_vel` (compatible with HMI manual mode)

3. **cus_nav2_config** - Custom Nav2 configuration
   - Used when HMI launches AMCL + Nav2 in auto mode

4. **simple_teleop** - Teleoperation
   - Alternative to HMI's joystick control
   - Can run alongside HMI (but avoid conflicts on `/cmd_vel`)

5. **rplidar_ros** - LIDAR driver
   - Provides laser scans for SLAM and navigation

## Launch File Integration

You can create a comprehensive launch file that starts everything:

```python
# your_package/launch/full_system.launch.py
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    return LaunchDescription([
        # Start robot driver
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('create_bringup'),
                    'launch', 'create_2.launch'
                ])
            ])
        ),

        # Start LIDAR
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('rplidar_ros'),
                    'launch', 'rplidar.launch.py'
                ])
            ])
        ),

        # Start Web HMI
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('web_hmi'),
                    'launch', 'hmi_server.launch.py'
                ])
            ])
        ),

        # Start your coverage planner
        Node(
            package='your_coverage_package',
            executable='coverage_planner_node',
            name='coverage_planner',
            output='screen'
        ),
    ])
```

## Testing Integration

1. **Start the HMI alone:**
   ```bash
   ros2 launch web_hmi hmi_server.launch.py
   ```

2. **Test manual mode (no coverage planner needed):**
   - Start your robot driver
   - Start LIDAR
   - Open HMI in browser
   - Switch to "Manual" tab
   - Test joystick control

3. **Test with coverage planner:**
   - Start your coverage planner node
   - Start HMI with a coverage plan JSON
   - Open HMI in browser
   - Switch to "Auto" tab
   - Click "Start Robot"
   - Monitor status updates

## Troubleshooting

### HMI doesn't receive robot pose
- Check that AMCL is running: `ros2 node list | grep amcl`
- Verify `/amcl_pose` is being published: `ros2 topic echo /amcl_pose`

### Coverage commands not working
- Verify your planner subscribes to:
  - `/coverage_start`
  - `/coverage_stop`
  - `/coverage_sequence`
- Check: `ros2 topic info /coverage_start`

### Manual mode joystick not moving robot
- Verify robot driver subscribes to `/cmd_vel`
- Check: `ros2 topic echo /cmd_vel` while moving joystick
- Ensure no other node is publishing to `/cmd_vel`

## Next Steps

1. **Create or adapt your coverage planner** to subscribe to HMI commands
2. **Publish status updates** so the HMI can display progress
3. **Test the full system** with a coverage plan JSON
4. **Customize the HMI** by modifying templates/index.html if needed

## Reference Coverage Plan JSON

The HMI expects a coverage plan JSON file. Here's the format:

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
        {"x": 1.5, "y": 1.0, "type": "coverage", "stripe": 0},
        {"x": 2.0, "y": 1.5, "type": "transition"}
      ]
    }
  },
  "sequence": ["a", "b", "c"]
}
```
