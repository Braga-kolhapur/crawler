# Quick Start Guide - Create2 Behavior Tree

## Prerequisites

Ensure you have:
- ROS2 installed (tested on Humble)
- Create2 robot with driver running
- Python 3.8+
- py_trees library

## Installation Steps

### 1. Install py_trees

```bash
pip install py_trees
# or for ROS2
sudo apt-get install python3-py-trees
```

### 2. Build the Package

```bash
cd ~/Documents/ros2_ws
colcon build --packages-select create_behavior_tree
source install/setup.bash
```

### 3. Verify Installation

```bash
# Check if package is built
ros2 pkg list | grep create_behavior_tree

# Check if executable is available
which behavior_tree_node
```

## Running the Behavior Tree

### Basic Launch

```bash
# Terminal 1: Build and source
cd ~/Documents/ros2_ws
source install/setup.bash

# Terminal 2: Launch the behavior tree
ros2 launch create_behavior_tree behavior_tree_launch.py
```

### Advanced Launch Options

```bash
# With custom update frequency
ros2 launch create_behavior_tree behavior_tree_launch.py tree_update_hz:=20.0

# Direct node execution
ros2 run create_behavior_tree behavior_tree_node
```

## Monitoring the Robot

### In separate terminals, monitor:

```bash
# Monitor velocity commands
ros2 topic echo /cmd_vel

# Monitor bumper sensors
ros2 topic echo /create/bumper

# Monitor cliff sensors
ros2 topic echo /create/cliff

# Check robot state
ros2 topic list | grep create
```

## Configuration

### Edit Configuration File

```bash
# Open the YAML config file
nano ~/Documents/ros2_ws/src/crawler/create_behavior_tree/config/behavior_tree_config.yaml
```

Key parameters to tune:
- `forward_speed`: How fast the robot moves forward (0.1-0.4 m/s)
- `turn_angular_speed`: How fast the robot turns (0.2-0.8 rad/s)
- `tree_update_hz`: How often the behavior tree updates (10-20 Hz)

### Load Custom Config in Code

```python
from create_behavior_tree.config import load_config_from_yaml
config = load_config_from_yaml('/path/to/config.yaml')
```

## Testing

### Run Test Suite

```bash
# In terminal running behavior tree
cd ~/Documents/ros2_ws
source install/setup.bash

# Terminal 2:
python3 -m create_behavior_tree.test_behavior_tree

# Or directly
ros2 run create_behavior_tree behavior_tree_node &
python3 src/crawler/create_behavior_tree/create_behavior_tree/test_behavior_tree.py
```

### Manual Testing

Publish test events:

```bash
# Test left bumper
ros2 topic pub /create/bumper create_msgs/Bumper '{header: {frame_id: "base_link"}, is_left_pressed: true, is_right_pressed: false, is_light_left: false, is_light_front_left: false, is_light_center_left: false, is_light_center_right: false, is_light_front_right: false, is_light_right: false, light_signal_left: 0, light_signal_front_left: 0, light_signal_center_left: 0, light_signal_center_right: 0, light_signal_front_right: 0, light_signal_right: 0}'

# Test cliff detection
ros2 topic pub /create/cliff create_msgs/Cliff '{header: {frame_id: "base_link"}, is_cliff_left: true, is_cliff_front_left: false, is_cliff_right: false, is_cliff_front_right: false}' --once
```

## Troubleshooting

### Issue: "Module not found: create_behavior_tree"
**Solution**: Make sure to source the setup.bash
```bash
source ~/Documents/ros2_ws/install/setup.bash
```

### Issue: "Failed to connect to Create driver"
**Solution**: Ensure Create driver is running
```bash
ros2 launch create_bringup create_2.launch
```

### Issue: Robot not responding to commands
**Solution**: Check if velocity commands are being published
```bash
ros2 topic echo /cmd_vel
```

### Issue: Sensors not working
**Solution**: Verify sensor topics exist
```bash
ros2 topic list | grep -E "bumper|cliff"
```

## Development Tips

### Adding New Behaviors

1. Create a new behavior class in `custom_nodes.py`
2. Inherit from `py_trees.behaviour.Behaviour`
3. Implement `update()` method returning `common.Status`
4. Add to tree in `tree_builder.py`

Example:
```python
class MyBehavior(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name)
    
    def update(self):
        # Your logic here
        return common.Status.SUCCESS
```

### Debugging the Tree

Use debug utilities in `tree_debug.py`:

```python
from create_behavior_tree.tree_debug import print_tree_structure
print_tree_structure(root)
```

### Logging

Enable debug logging:
```bash
ros2 run create_behavior_tree behavior_tree_node --ros-args --log-level debug
```

## Next Steps

1. Read the [README.md](README.md) for detailed documentation
2. Check [advanced_behaviors.py](create_behavior_tree/advanced_behaviors.py) for custom behavior examples
3. Explore [tree_builder.py](create_behavior_tree/tree_builder.py) to understand tree structure
4. Modify [behavior_tree_config.yaml](config/behavior_tree_config.yaml) to tune robot behavior

## Getting Help

- Check ROS2 docs: https://docs.ros.org/en/humble/
- py_trees docs: https://py-trees.readthedocs.io/
- Create2 docs: https://www.irobotics.com/create-platform
