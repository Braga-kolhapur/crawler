# Create Behavior Tree - Production Edition

**Status: ✅ Production-Ready | Refactored & Optimized**

A comprehensive Python-based behavior tree for autonomous Create2 robot control using py_trees and ROS2. Features full sensor integration, advanced safety rules, and production-grade abstractions.

## Overview

This package implements a complete 6-level safety priority behavior tree that manages the Create2 robot's autonomous behavior with comprehensive sensor integration (8+ sensors) and motor control (4 motor types). The system provides production-grade reliability through abstraction layers and thread-safe operations.

## Key Features

- ✅ **Complete Sensor Coverage**: 8+ sensors (bumper, cliff, wheel drop, battery, charging state, etc.)
- ✅ **Advanced Motor Control**: All 4 motor types (wheels, main brush, vacuum, side brush) with safety constraints
- ✅ **6-Level Safety Priority**: Wheel drop → Critical battery → Low battery → Bumper → Cliff → Standstill → Normal
- ✅ **Automatic Safety Rules**: Motor disable on wheel drop OR standstill (zero velocity)
- ✅ **Battery Management**: Automatic thresholds (10% warning, 5% emergency)
- ✅ **Thread-Safe Operations**: Concurrent sensor/motor access with locks
- ✅ **Production Architecture**: Clean abstraction layers (SensorManager, MotorController)
- ✅ **Comprehensive Documentation**: 3000+ lines with examples and architecture diagrams
- ✅ **Easy Extensibility**: Pattern-based design for adding sensors and behaviors

## Sensor Setup

The behavior tree monitors the following sensors from the Create2:

### Bumper Sensors (2)
- Left bumper: `create_msgs/Bumper.is_left_pressed`
- Right bumper: `create_msgs/Bumper.is_right_pressed`
- Light sensors (6): Front obstacle detection

### Cliff Sensors (4)
- Left cliff
- Front-left cliff
- Right cliff
- Front-right cliff

### Wheel Drop Sensor
- Detects if wheels have dropped off a surface

## 6-Level Safety Priority Hierarchy

```
Root (Parallel Execution)
├── 🚨 CRITICAL SAFETY (Priority 1-2)
│   ├── Wheel Drop Detected → Emergency Stop (all motors off)
│   └── Battery Critical (<5%) → Stop Completely
├── ⚠️  PROTECTIVE ACTIONS (Priority 3-4)
│   ├── Cliff Detected → Stop + Backup + Turn
│   ├── Battery Low (<10%) → Reduce Speed
│   ├── Bumper Contact → Turn Away (Left/Right/Both)
│   └── Robot Standstill → Disable Motors (power saving)
└── 🟢 NORMAL OPERATION
    └── Default Forward Movement
```

### Safety Rules (Implemented)

| Priority | Condition | Action | Reason |
|----------|-----------|--------|--------|
| 🚨 1 | Wheel Drop | EMERGENCY STOP | Prevents damage on fall |
| 🚨 2 | Battery < 5% | STOP COMPLETELY | Critical power emergency |
| ⚠️ 3 | Cliff Detected | STOP + BACKUP + TURN | Prevent cliff fall |
| ⚠️ 3 | Battery < 10% | REDUCE SPEED | Preserve remaining power |
| ⚡ 4 | Bumper Contact | TURN AWAY | Collision recovery |
| ⚡ 5 | Standstill (v=0) | DISABLE MOTORS | Power conservation |
| 🟢 6 | Normal | MOVE FORWARD | Default exploration |

## 💻 All Monitored Sensors

The refactored system monitors **8+ sensor topics** with full integration:

| Sensor | Topic | Type | Purpose |
|--------|-------|------|---------|
| **Bumper** | `/bumper` | create_msgs/Bumper | Contact detection + light sensors |
| **Cliff** | `/cliff` | create_msgs/Cliff | 4-point cliff edge detection |
| **Wheel Drop** | `/wheeldrop` | std_msgs/Empty | Emergency fall detection |
| **Battery Charge** | `/battery/charge_ratio` | std_msgs/Float32 | Battery level (0-1) |
| **Battery State** | `/battery/charging_state` | create_msgs/ChargingState | Charging mode enum |
| **Battery Voltage** | `/battery/voltage` | std_msgs/Float32 | Voltage monitoring |
| **Battery Current** | `/battery/current` | std_msgs/Float32 | Current draw monitoring |
| **Velocity Feedback** | `/cmd_vel` | geometry_msgs/Twist | Motor velocity (standstill detection) |

## 🎮 Motor Control (4 Motor Types)

| Motor | Topic | Type | Range | Control |
|-------|-------|------|-------|---------|
| **Wheels** | `/cmd_vel` | geometry_msgs/Twist | velocity-based | Forward/turn/stop |
| **Main Brush** | `/main_brush_motor` | create_msgs/MotorSetpoint | [-1.0, 1.0] | Full duty cycle |
| **Vacuum** | `/vacuum_motor` | create_msgs/MotorSetpoint | [0.0, 1.0] | Power only |
| **Side Brush** | `/side_brush_motor` | create_msgs/MotorSetpoint | [-1.0, 1.0] | Full duty cycle |

## Installation & Deployment

### Quick Setup

```bash
# Install dependencies
pip install py_trees pyyaml rclpy create-msgs

# Build package
cd ~/Documents/ros2_ws
colcon build --packages-select create_behavior_tree
source install/setup.bash
```

### Run the Production System

**Terminal 1 - Hardware Driver:**
```bash
ros2 launch create_bringup create_2.launch.py
```

**Terminal 2 - Behavior Tree:**
```bash
ros2 run create_behavior_tree refactored_behavior_tree_node
```

**Terminal 3 - Monitoring (Optional):**
```bash
ros2 topic echo /cmd_vel  # Watch velocity commands
```

### Configuration

Update tree behavior in `config/behavior_tree_config.yaml`:
- `tree_update_hz`: Tree execution frequency (default 10 Hz)
- `battery_low_threshold`: Low battery warning (default 0.10 = 10%)
- `battery_critical_threshold`: Critical battery (default 0.05 = 5%)
- `standstill_timeout`: Time to disable motors (default 2.0 seconds)

### Custom Parameters at Runtime

```bash
ros2 run create_behavior_tree refactored_behavior_tree_node \
  --ros-args -p tree_update_hz:=20.0
```

## 📚 Documentation

For detailed information, see:

- **[INDEX.md](INDEX.md)** - Navigation guide for all documentation
- **[DEPLOYMENT_CHECKLIST.md](DEPLOYMENT_CHECKLIST.md)** - Step-by-step deployment (17 phases)
- **[REFACTORING_GUIDE.md](REFACTORING_GUIDE.md)** - Complete 10-part tutorial
- **[REFACTORING_SUMMARY.md](REFACTORING_SUMMARY.md)** - Executive summary
- **[SENSOR_MOTOR_ANALYSIS.py](create_behavior_tree/SENSOR_MOTOR_ANALYSIS.py)** - Technical reference
- **[ARCHITECTURE_DIAGRAMS.py](create_behavior_tree/ARCHITECTURE_DIAGRAMS.py)** - Visual architecture

## 🔧 Architecture

### Core Components

```
ROS2 Topics (Sensors) 
    ↓
SensorManager (sensor_motor_manager.py)
    ↓
Behavior Nodes (refactored_nodes.py)
    + CheckWheelDrop, CheckCliffDetection, CheckBumperContact
    + CheckLowBattery, CheckCriticalBattery, CheckRobotStandstill
    + MoveForwardAction, TurnLeftAction, TurnRightAction, etc.
    ↓
Behavior Tree (refactored_tree_builder.py)
    + 6-level safety priority
    + Parallel safety + Navigation selector
    ↓
MotorController (sensor_motor_manager.py)
    ↓
ROS2 Topics (Motors → Create2 Hardware)
```

### File Structure

**Core Implementation (Must-have):**
- `sensor_motor_manager.py` - Abstraction layer (SensorManager + MotorController)
- `refactored_nodes.py` - 20 behavior tree nodes
- `refactored_tree_builder.py` - Tree assembly with safety hierarchy
- `refactored_behavior_tree_node.py` - Main ROS2 entry point

**Reference/Documentation:**
- `ARCHITECTURE_DIAGRAMS.py` - Visual diagrams
- `SENSOR_MOTOR_ANALYSIS.py` - Sensor/motor technical specs
- `ARCHITECTURE.py` - Design patterns (legacy)
- `advanced_behaviors.py` - Example implementations
- `getting_started.py` - Tutorial examples

## 🚀 Quick Examples

### Check Sensor State (in code)

```python
from create_behavior_tree.sensor_motor_manager import SensorManager

sensor_mgr = SensorManager(node)
battery = sensor_mgr.get_battery_info()
print(f"Battery: {battery.charge_ratio * 100:.1f}%")

if sensor_mgr.is_wheel_drop_active():
    print("⚠️  WHEEL DROP DETECTED!")
```

### Control Motors (in code)

```python
from create_behavior_tree.sensor_motor_manager import MotorController

motor_ctrl = MotorController(node)

# Move forward
motor_ctrl.set_velocity(linear=0.2, angular=0.0)

# Turn right
motor_ctrl.set_velocity(linear=0.15, angular=-0.4)

# Emergency stop
motor_ctrl.emergency_stop()
```

### Extend with New Behavior

See **[REFACTORING_GUIDE.md - Part 9](REFACTORING_GUIDE.md#part-9-extending-the-system)** for complete examples.

## ✅ Features Implemented

- ✅ All Create2 sensors integrated (8+)
- ✅ All motor types controlled (4)
- ✅ 6-level safety priority hierarchy
- ✅ Wheel drop emergency stop
- ✅ Battery monitoring & thresholds
- ✅ Standstill detection → motor disable
- ✅ Thread-safe concurrent operations
- ✅ Modular node-based architecture
- ✅ Easy pattern-based extensibility
- ✅ Production-grade documentation (3000+ lines)
- ✅ Visual architecture diagrams
- ✅ Comprehensive deployment guide

## 📋 Debugging & Monitoring

```bash
# View all active topics
ros2 topic list

# Monitor specific sensor
ros2 topic echo /bumper

# Record sensor data for analysis
ros2 bag record -o sensor_data /bumper /cliff /cmd_vel /battery/charge_ratio

# See node info
ros2 node info /refactored_behavior_tree_node
```

## 🤝 Contributing

To add new sensors or behaviors:

1. **Add Sensor:** Create subscription in `SensorManager`, add query method
2. **Add Behavior Node:** Create class in `refactored_nodes.py`, implement `update()` method
3. **Update Tree:** Add to tree composition in `refactored_tree_builder.py`
4. **Document:** Update this README and reference docs

See [REFACTORING_GUIDE.md - Part 9](REFACTORING_GUIDE.md#part-9-extending-the-system) for detailed examples.

### Monitoring Tree Status

The behavior tree status can be logged by uncommenting the debug line in `behavior_tree_node.py`:

```python
self.get_logger().debug(f"Tree status: {status}")
```

For more detailed visualization, use:
```python
print(behavior_tree_node.get_tree_status_string())
```

## Behavior Tree Concepts

### Status Types
- `SUCCESS`: Behavior completed successfully
- `FAILURE`: Behavior failed
- `RUNNING`: Behavior is still executing

### Composites
- **Selector**: Returns SUCCESS if any child succeeds (OR gate)
- **Sequence**: Returns SUCCESS only if all children succeed (AND gate)
- **Parallel**: Requires success from multiple children simultaneously

### Behavior Nodes
- **Condition**: Checks sensor states and returns SUCCESS/FAILURE
- **Action**: Performs motor commands and returns SUCCESS/RUNNING/FAILURE

## Troubleshooting

### Topics Not Connecting
- Ensure the Create driver is running: `ros2 launch create_bringup create_2.launch`
- Check topic names: `ros2 topic list`
- Monitor sensor data: `ros2 topic echo /create/bumper`

### Robot Not Responding
- Verify velocity command topic: `ros2 topic echo /cmd_vel`
- Check Create driver is operational
- Ensure emergency motors are not disabled

### Tree Not Updating
- Check update frequency: `ros2 topic hz /cmd_vel`
- Review error logs: `ros2 topic echo /rosout`
- Verify tree_update_hz parameter

## References

- [py_trees Documentation](https://py-trees.readthedocs.io/)
- [ROS2 Documentation](https://docs.ros.org/en/humble/)
- [Create2 Platform Documentation](https://www.irobotics.com/create-platform)

## License

MIT License

## Author

Your Name (user@example.com)
