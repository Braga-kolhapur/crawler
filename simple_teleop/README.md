Simple teleop node

Usage:

1. Build your workspace and source the install setup:

```bash
colcon build --packages-select simple_teleop
source install/setup.bash
```

2. Run the teleop node:

```bash
ros2 run simple_teleop teleop
```

Controls:
- Arrow keys: drive (up/down linear, left/right angular)
- Space: stop (publish zero velocity)
- q: quit

The node publishes `geometry_msgs/msg/TwistStamped` on `/cmd_vel`.
