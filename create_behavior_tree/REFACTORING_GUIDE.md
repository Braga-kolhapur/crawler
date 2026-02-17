# Create2 Behavior Tree - Sensor & Motor Refactoring Guide

## Overview

The behavior tree has been completely refactored to use clean abstraction layers for sensor management and motor control. This document explains the improvements, sensor mappings, and safety rules.

---

## Part 1: Sensor Analysis & Topic Mapping

### All Available Sensors on Create2

#### 1. **Bumper Sensors** (`/bumper`)
- **Message Type:** `create_msgs/Bumper`
- **Contact Sensors:**
  - `is_left_pressed` (bool) - Physical bumper contact on left
  - `is_right_pressed` (bool) - Physical bumper contact on right
- **Light Sensors (Obstacle Detection):**
  - `is_light_left`, `is_light_front_left`, `is_light_center_left` (bool)
  - `is_light_center_right`, `is_light_front_right`, `is_light_right` (bool)
- **Light Signal Strength (0-4095):**
  - `light_signal_left`, `light_signal_front_left`, `light_signal_center_left`
  - `light_signal_center_right`, `light_signal_front_right`, `light_signal_right`

#### 2. **Cliff Sensors** (`/cliff`)
- **Message Type:** `create_msgs/Cliff`
- **Detection Points:**
  - `is_cliff_left` - Left wheel edge cliff
  - `is_cliff_front_left` - Front-left area cliff
  - `is_cliff_right` - Right wheel edge cliff
  - `is_cliff_front_right` - Front-right area cliff

#### 3. **Wheel Drop Sensor** (`/wheeldrop`)
- **Message Type:** `std_msgs/Empty` (Event-based)
- **Trigger:** Published when robot detects wheels lifted off ground
- **Priority:** **CRITICAL** - Causes immediate emergency stop
- **Recovery:** Manual intervention required

#### 4. **Battery Monitoring**
| Topic | Type | Range | Purpose |
|-------|------|-------|---------|
| `/battery/charge_ratio` | Float32 | 0.0-1.0 | Battery percentage |
| `/battery/voltage` | Float32 | 14-16V | System voltage |
| `/battery/current` | Float32 | Variable | Power draw |
| `/battery/charge` | Float32 | Amp-hours | Capacity used |
| `/battery/capacity` | Float32 | ~2-4 Ah | Total capacity |
| `/battery/temperature` | Int16 | Celsius | Battery temp |
| `/battery/charging_state` | ChargingState | Enum | Charging status |

**Battery Thresholds:**
- Low Battery: `charge_ratio < 0.1` (10%)
- Critical Battery: `charge_ratio < 0.05` (5%)
- Safe Operation: `charge_ratio > 0.2` (20%)

#### 5. **Velocity Indication** (Derived)
- **Source:** `/cmd_vel` subscription monitoring
- **Use Case:** Detect robot standstill for power saving
- **Standstill Detection:** `velocity = 0 for > 2 seconds`

#### 6. **Odometry & Diagnostics**
- `/odom` - Robot position and velocity
- `/joint_states` - Wheel encoder data
- `/mode` - Robot operation mode
- Various button press events

---

## Part 2: Motor Control & Safety Rules

### Motor Control Topics

All motors use `create_msgs/MotorSetpoint` message with `duty_cycle` field.

#### Main Brush Motor (`/main_brush_motor`)
- **Duty Cycle Range:** [-1.0, 1.0]
- **0.0:** OFF | **Positive:** Forward | **Negative:** Reverse
- **Power Draw:** **SIGNIFICANT** - Disable when not needed
- **Safety:** DISABLE when wheel drop OR velocity = 0

#### Vacuum Motor (`/vacuum_motor`)
- **Duty Cycle Range:** [0.0, 1.0] ← **Positive only!**
- **0.0:** OFF | **1.0:** Maximum suction
- **Power Draw:** **SIGNIFICANT** - Disable when not needed  
- **Safety:** DISABLE when wheel drop OR velocity = 0

#### Side Brush Motor (`/side_brush_motor`)
- **Duty Cycle Range:** [-1.0, 1.0]
- **Power Draw:** Low-moderate
- **Can control:** Even during standstill (optional)

#### Wheel Motors (via `/cmd_vel`)
- **Command:** `geometry_msgs/Twist`
- **Auto-disable:** After 0.2 seconds without command

---

## Part 3: Safety Rules & Priority Hierarchy

### Rule 1: WHEEL DROP (EMERGENCY)
**Trigger:** `/wheeldrop` event received

**Actions (Simultaneous):**
- ❌ Stop wheels: `/cmd_vel` linear.x = 0.0
- ❌ Disable vacuum: `/vacuum_motor` duty_cycle = 0.0
- ❌ Disable main brush: `/main_brush_motor` duty_cycle = 0.0
- 🚨 Log EMERGENCY

**Priority:** HIGHEST - Interrupts everything

---

### Rule 2: ROBOT STANDSTILL
**Trigger:** `velocity = 0 for > 2 seconds`

**Actions:**
- ⚡ Disable vacuum (power saving)
- ⚡ Disable main brush (power saving)

**Priority:** HIGH - But doesn't stop navigation

---

### Rule 3: LOW BATTERY (< 10%)
**Trigger:** `charge_ratio < 0.1`

**Actions:**
- ⚠️ Reduce speed: max_speed = 0.15 m/s (from 0.2)
- ⚡ Disable vacuum and main brush
- 📢 Log WARNING

**Priority:** HIGH - Extends operation duration

---

### Rule 4: CRITICAL BATTERY (< 5%)
**Trigger:** `charge_ratio < 0.05`

**Actions:**
- 🛑 Stop all operations
- 🏠 Return to dock (behavior TBD)

**Priority:** CRITICAL - Highest safety concern

---

### Rule 5: BUMPER CONTACT (Collision)
**Trigger:** Contact sensor press

**Actions:**
- Left bumper → Turn right + continue
- Right bumper → Turn left + continue
- Both bumpers → 180° turn (stuck situation)

**Priority:** MEDIUM - Can be overridden by safety rules

---

### Rule 6: CLIFF DETECTION (Fall Prevention)
**Trigger:** Any `is_cliff_*` = true

**Actions:**
- Stop immediately
- Backup slightly
- Turn away from cliff

**Priority:** MEDIUM - Falls are serious!

---

## Part 4: Refactored Architecture

### SensorManager Class

**Responsibilities:**
- Subscribe to all sensor topics
- Thread-safe data access
- Provide clean query methods

**Key Methods:**
```python
is_wheel_drop_active() → bool
is_cliff_detected() → bool
get_cliff_direction() → str  # "left", "front_left", "right", "front_right"
is_bumper_contact() → bool
get_bumper_direction() → str  # "left", "right", "both"
is_low_battery(threshold=0.1) → bool
is_critical_battery(threshold=0.05) → bool
is_robot_standstill(duration_sec=2.0) → bool
get_battery_info() → BatteryInfo
```

**Benefits:**
- ✅ Sensors decoupled from behaviors
- ✅ Easy to add new sensors
- ✅ Easy to modify thresholds
- ✅ Testable with mock sensors

### MotorController Class

**Responsibilities:**
- Publish to motor topics
- State tracking
- Safety enforcement

**Key Methods:**
```python
set_main_brush(duty_cycle) → bool
set_vacuum(duty_cycle) → bool
set_side_brush(duty_cycle) → bool
disable_all_motors() → bool
emergency_stop() → bool
set_velocity(linear_x, angular_z) → bool
stop_wheels() → bool
get_state() → dict
```

**Benefits:**
- ✅ Motors abstracted from behaviors
- ✅ Prevents conflicting commands
- ✅ Tracks what's currently active
- ✅ Centralized safety logic

---

## Part 5: New Behavior Nodes

All nodes now use `SensorManager` and `MotorController` abstractions:

### Sensor Condition Nodes
- `CheckWheelDrop` - Is wheel dropped?
- `CheckCliffDetection` - Is cliff detected?
- `CheckBumperContact` - Any bumper pressed?
- `CheckLeftBumper` - Only left?
- `CheckRightBumper` - Only right?
- `CheckBothBumpers` - Both?
- `CheckLowBattery` - Battery < 10%?
- `CheckCriticalBattery` - Battery < 5%?
- `CheckRobotStandstill` - Velocity = 0 for duration?

### Motor Action Nodes
- `MoveForwardAction` - Move forward
- `TurnLeftAction` - Turn left
- `TurnRightAction` - Turn right
- `Turn180Action` - 180° turn
- `StopWheels` - Stop movement
- `BackupAction` - Backup
- `DisableAllMotors` - Disable vacuum/brush
- `EmergencyStop` - Full stop
- `ReduceSpeedAction` - Low battery speed
- `WaitDuration` - Pause between actions

---

## Part 6: Running the Refactored System

### Original System
```bash
ros2 launch create_behavior_tree behavior_tree_launch.py
```

### Refactored System
```bash
ros2 run create_behavior_tree refactored_behavior_tree_node
```

### Comparison of Node Names
| Purpose | Topic Published | Topic Subscribed |
|---------|-----------------|------------------|
| Original | Bumper/Cliff from custom | All sensors |
| Refactored | Motor commands | All sensors + /cmd_vel |
| Both | `/cmd_vel` | Same topics |

---

## Part 7: Monitoring & Debugging

### View Sensor Status
```python
# In another terminal, after running the node
from create_behavior_tree.sensor_motor_manager import SensorManager
# Or access via the node's print_sensor_status() method
```

### View Motor State
```python
# Check what motors are currently active
motor_state = motor_controller.get_state()
print(motor_state)
# Output:
# {
#     'main_brush': 0.0,
#     'vacuum': 0.0,
#     'side_brush': 0.0,
#     'linear_velocity': 0.2,
#     'angular_velocity': 0.0
# }
```

### View Tree Structure
```bash
# Print tree during operation (add to node)
node.print_tree_info()
node.print_sensor_status()
```

---

## Part 8: Sensor-to-Action Mapping

| Sensor | Condition | Action | Safety Priority |
|--------|-----------|--------|-----------------|
| Wheel Drop | Event | Emergency stop, disable motors | ⛔ CRITICAL |
| Battery | ratio < 0.05 | Stop, return to dock | ⛔ CRITICAL |
| Battery | ratio < 0.1 | Reduce speed, disable motors | ⚠️ HIGH |
| Cliff | Any sensor true | Stop, backup, turn away | ⚠️ HIGH |
| Bumper | Both pressed | 180° turn | ⚡ MEDIUM |
| Bumper | Left only | Turn right | ⚡ MEDIUM |
| Bumper | Right only | Turn left | ⚡ MEDIUM |
| Standstill | velocity=0 for 2s | Disable vacuum/brush | ⚡ MEDIUM |
| Light Sensor | signal > threshold | Optional smooth nav | 🟢 LOW |

---

## Part 9: Extending the System

### Adding a New Sensor

**Step 1:** Add subscription in `SensorManager`:
```python
def __init__(self, node):
    self.new_sensor_data = None
    self.new_sensor_sub = node.create_subscription(
        MyMsgType, '/my_topic', self._callback, 10
    )

def _callback(self, msg):
    self.new_sensor_data = msg
```

**Step 2:** Add query method:
```python
def is_my_condition_true(self) -> bool:
    with self.lock:
        return some_check(self.new_sensor_data)
```

**Step 3:** Create behavior node:
```python
class CheckMyCondition(py_trees.behaviour.Behaviour):
    def __init__(self, name, sensor_manager):
        super().__init__(name)
        self.sensor_mgr = sensor_manager
    
    def update(self):
        if self.sensor_mgr.is_my_condition_true():
            return common.Status.SUCCESS
        return common.Status.FAILURE
```

**Step 4:** Add to tree in `refactored_tree_builder.py`

---

## Part 10: Configuration Tuning

Edit `create_behavior_tree_config.yaml`:

```yaml
# Speed constraints
velocity:
  forward_speed: 0.2          # Normal forward
  turn_speed: 0.15            # Turn speed
  turn_angular_speed: 0.4     # Turn rotation

# Motor control
standstill_timeout: 2.0       # Seconds before disabling motors
low_battery_threshold: 0.1    # 10%
critical_battery_threshold: 0.05  # 5%

# Safety
cliff_detection_enabled: true
wheel_drop_detection_enabled: true
enable_emergency_stop: true
```

---

## Summary of Improvements

| Aspect | Before | After |
|--------|--------|-------|
| **Sensor Abstraction** | Direct topic access | SensorManager class |
| **Motor Control** | Direct publishers | MotorController class |
| **Code Reusability** | Limited | High - Mix & match nodes |
| **Testability** | Difficult | Easy - Mock managers |
| **Maintainability** | Hard to extend | Add sensor/behavior pair |
| **Safety** | Basic | Comprehensive with priorities |
| **Battery Management** | Manual | Automated thresholds |
| **Standstill Detection** | None | Automatic power saving |
| **Documentation** | Minimal | Complete with examples |

---

## Files Overview

```
create_behavior_tree/
├── sensor_motor_manager.py       # SensorManager, MotorController
├── refactored_nodes.py           # All behavior nodes (clean)
├── refactored_tree_builder.py    # Tree composition
├── refactored_behavior_tree_node.py # Main entry point
├── SENSOR_MOTOR_ANALYSIS.py      # This analysis
└── [Original files remain for backwards compatibility]
```

---

## Quick Reference: Safety Priority

```
🚨 EMERGENCY (Highest)
├─ Wheel Drop Detected → Emergency Stop

⛔ CRITICAL
├─ Battery < 5% → Stop

⚠️ HIGH  
├─ Cliff Detected → Stop + Backup
├─ Battery < 10% → Reduce Speed
└─ Bumper Collision → Avoid

⚡ MEDIUM
├─ Robot Standstill → Save Power
└─ Explore Safely

🟢 LOW (Lowest)
└─ Light Sensors → Smooth Navigation
```

---

## Next Steps

1. **Test** the refactored system with mock sensor events
2. **Tune** thresholds in `behavior_tree_config.yaml`
3. **Monitor** actual robot behavior
4. **Extend** with additional sensors as needed
5. **Deploy** to production when confident

---

## References

- [Behavior Tree Concepts](ARCHITECTURE.py)
- [Original Analysis](README.md)
- [Code Examples](getting_started.py)
- [Configuration](config/behavior_tree_config.yaml)
