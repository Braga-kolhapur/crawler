"""
Complete Sensor and Motor Analysis for Create2 Robot

This document provides a comprehensive mapping of all sensors, topics, 
and motor controls available on the Create2 platform based on the 
create_driver package analysis.
"""

# ============================================================================
# SENSOR TOPICS - COMPLETE REFERENCE
# ============================================================================

"""
BUMPER SENSORS (/bumper)
├── Message Type: create_msgs/Bumper
├── Header: std_msgs/Header (frame_id, timestamp)
├── Contact Sensors:
│   ├── is_left_pressed: bool
│   │   └─ Detects physical contact on left side
│   └── is_right_pressed: bool
│       └─ Detects physical contact on right side
├── Light Sensors (Create2+, Bumper-based Detection):
│   ├── is_light_left: bool (front left corner)
│   ├── is_light_front_left: bool (front-left area)
│   ├── is_light_center_left: bool (center-left area)
│   ├── is_light_center_right: bool (center-right area)
│   ├── is_light_front_right: bool (front-right area)
│   └── is_light_right: bool (front right corner)
└── Light Signal Strength (0-4095 scale):
    ├── light_signal_left: uint16
    ├── light_signal_front_left: uint16
    ├── light_signal_center_left: uint16
    ├── light_signal_center_right: uint16
    ├── light_signal_front_right: uint16
    └── light_signal_right: uint16

CLIFF SENSORS (/cliff)
├── Message Type: create_msgs/Cliff
├── Header: std_msgs/Header
├── Cliff Detection (4 directions):
│   ├── is_cliff_left: bool (left wheel cliff)
│   ├── is_cliff_front_left: bool (front-left cliff)
│   ├── is_cliff_right: bool (right wheel cliff)
│   └── is_cliff_front_right: bool (front-right cliff)

WHEEL DROP SENSOR (/wheeldrop)
├── Message Type: std_msgs/Empty
├── Published When: Robot detects wheels lifted off ground
├── Priority: CRITICAL - Robot loses contact with ground
├── Action on Trigger: EMERGENCY STOP - disable all motors

BATTERY INFORMATION
├── /battery/voltage: std_msgs/Float32
│   └─ Voltage in Volts (typically 14-16V)
├── /battery/current: std_msgs/Float32
│   └─ Current in Amps (positive = discharging)
├── /battery/charge: std_msgs/Float32
│   └─ Charge in Amp-hours
├── /battery/capacity: std_msgs/Float32
│   └─ Battery capacity in Amp-hours
├── /battery/charge_ratio: std_msgs/Float32
│   ├─ Range: 0.0 (empty) to 1.0 (full)
│   └─ Critical thresholds:
│       ├─ < 0.1 (10%): LOW BATTERY warning
│       └─ < 0.05 (5%): CRITICAL - Return to dock immediately
├── /battery/temperature: std_msgs/Int16
│   └─ Temperature in Celsius
└── /battery/charging_state: create_msgs/ChargingState
    ├── CHARGE_NONE (0): Not charging
    ├── CHARGE_RECONDITION (1): Reconditioning
    ├── CHARGE_FULL (2): Fully charged
    ├── CHARGE_TRICKLE (3): Trickle charging
    ├── CHARGE_WAITING (4): Waiting to charge
    └── CHARGE_FAULT (5): Charging fault

VELOCITY INDICATION
├── Source: /cmd_vel subscription timestamp
├── Derivable From: /odom or /joint_states
├── Use Case: Detect robot standstill
│   ├─ Zero velocity detected for duration X
│   └─ Stop vacuum and main brush to save power
"""

# ============================================================================
# MOTOR CONTROL TOPICS - COMPLETE REFERENCE
# ============================================================================

"""
MOTOR CONTROL OVERVIEW
All motors use create_msgs/MotorSetpoint message with duty_cycle field

MAIN BRUSH MOTOR (/main_brush_motor)
├── Message: create_msgs/MotorSetpoint
├── Field: duty_cycle (float32)
│   ├─ Range: [-1.0, 1.0]
│   ├─ 0.0: Motor OFF
│   ├─ Positive: Forward rotation
│   └─ Negative: Reverse rotation
├── Purpose: Main sweeping brush (center)
├── Power Draw: Significant (disable when not needed)
├── Safety:
│   └─ DISABLE when:
│       ├─ Wheel drop detected
│       ├─ Robot is stationary (v=0)
│       └─ Battery < 10%

VACUUM MOTOR (/vacuum_motor)
├── Message: create_msgs/MotorSetpoint
├── Field: duty_cycle (float32)
│   ├─ Range: [0.0, 1.0]  ← NOTE: Positive only!
│   ├─ 0.0: Vacuum OFF
│   └─ 1.0: Maximum suction
├── Purpose: Suction/pickup
├── Power Draw: Significant (disable when not needed)
├── Safety:
│   └─ DISABLE when:
│       ├─ Wheel drop detected
│       ├─ Robot is stationary (v=0)
│       └─ Battery < 10%

SIDE BRUSH MOTOR (/side_brush_motor)
├── Message: create_msgs/MotorSetpoint
├── Field: duty_cycle (float32)
│   ├─ Range: [-1.0, 1.0]
│   ├─ 0.0: Motor OFF
│   ├─ Positive: Forward
│   └─ Negative: Reverse
├── Purpose: Side brush for edge sweeping
├── Power Draw: Low-moderate
├── Can be: Used for navigation assists
└── Safety:
    └─ Safely controllable during standstill

WHEEL MOTORS (Controlled via /cmd_vel)
├── Command: geometry_msgs/Twist
├── Fields:
│   ├─ linear.x: Forward/backward velocity (m/s)
│   └─ angular.z: Rotation velocity (rad/s)
├── Auto-disable: When no /cmd_vel received for >latch_duration (default 0.2s)
└── Can be: Commanded to zero immediately
"""

# ============================================================================
# CRITICAL BEHAVIORS & SAFETY RULES
# ============================================================================

"""
RULE 1: WHEEL DROP DETECTION
├── Trigger: /wheeldrop topic receives any message
├── Priority: HIGHEST (interrupt all behaviors)
├── Actions Required (simultaneous):
│   ├─ STOP wheels: Set /cmd_vel linear.x = 0.0, angular.z = 0.0
│   ├─ DISABLE vacuum: /vacuum_motor duty_cycle = 0.0
│   ├─ DISABLE main brush: /main_brush_motor duty_cycle = 0.0
│   └─ LOG: "EMERGENCY - WHEEL DROP DETECTED"
├── Duration: Hold until wheel drop clears
└── Recovery: Manual intervention required

RULE 2: ROBOT STANDSTILL (Zero Velocity)
├── Trigger: Robot velocity = 0 for > standstill_duration (e.g., 2 seconds)
├── Priority: HIGH (but can be overridden by active navigation)
├── Actions:
│   ├─ DISABLE vacuum: /vacuum_motor duty_cycle = 0.0
│   ├─ DISABLE main brush: /main_brush_motor duty_cycle = 0.0
│   └─ LOG: "Robot standstill - disabling motors to save power"
├── Duration: While velocity remains zero
└── Recovery: Automatic when movement resumes

RULE 3: LOW BATTERY (charge_ratio < 0.1)
├── Trigger: /battery/charge_ratio < 0.1 (10%)
├── Priority: HIGH
├── Actions:
│   ├─ LOG: "LOW BATTERY WARNING"
│   ├─ REDUCE speed: Set max_speed = 0.15 m/s (from 0.2)
│   ├─ DISABLE vacuum: /vacuum_motor = 0.0
│   ├─ DISABLE main brush: /main_brush_motor = 0.0
│   └─ PREPARE to return to dock
├── Duration: Until charging begins
└── Recovery: Return to dock or manual charging

RULE 4: CRITICAL BATTERY (charge_ratio < 0.05)
├── Trigger: /battery/charge_ratio < 0.05 (5%)
├── Priority: CRITICAL
├── Actions:
│   ├─ LOG: "CRITICAL BATTERY - RETURNING TO DOCK"
│   ├─ STOP all operations
│   └─ Activate homing behavior (return to dock)
├── Duration: Until dock reached
└── Recovery: Automatic when fully charged

RULE 5: BUMPER CONTACT (Collision Detection)
├── Triggers:
│   ├─ is_left_pressed = true → Turn right
│   ├─ is_right_pressed = true → Turn left
│   └─ Both = true → 180° turn (stuck situation)
├── Priority: MEDIUM-HIGH
├── Actions:
│   ├─ STOP forward movement
│   ├─ Execute collision recovery maneuver
│   └─ Resume navigation after recovery
└── These can be overridden by wheel drop/critical battery

RULE 6: CLIFF DETECTION (Fall Prevention)
├── Trigger: Any is_cliff_* = true
├── Priority: MEDIUM-HIGH
├── Actions:
│   ├─ STOP movement immediately
│   ├─ BACKUP slightly
│   └─ Turn away from cliff direction
└── These can be overridden by wheel drop

RULE 7: LIGHT SENSOR OPTIONAL BEHAVIOR
├── Trigger: Light sensor signals > threshold
├── Priority: LOW (used for smooth navigation)
├── Purpose: Obstacle detection before contact
└── Note: Can be tuned for aggressive exploration

PRIORITY HIERARCHY (Highest to Lowest):
1. WHEEL DROP (Emergency stop)
2. CRITICAL BATTERY (Return to dock)
3. CLIFF DETECTION (Stop movement)
4. BUMPER CONTACT (Collision recovery)
5. LOW BATTERY (Reduced operations)
6. ROBOT STANDSTILL (Power conservation)
7. ACTIVE NAVIGATION (Default behaviors)
8. LIGHT SENSORS (Optional enhancement)
"""

# ============================================================================
# SENSOR-TO-ACTION MAPPING
# ============================================================================

"""
SENSOR_ID  │ TOPIC                    │ MESSAGE TYPE           │ ACTION
───────────┼─────────────────────────┼──────────────────────┼────────────────────
1          │ /bumper                 │ Bumper                │ Collision response
1a         │ └─ is_left_pressed      │ bool                  │ Turn right
1b         │ └─ is_right_pressed     │ bool                  │ Turn left
1c         │ └─ Both pressed         │ bool                  │ 180° turn (stuck)
1d         │ └─ light_signal_*       │ uint16 [0-4095]       │ Optional smooth nav
───────────┼─────────────────────────┼──────────────────────┼────────────────────
2          │ /cliff                  │ Cliff                 │ Fall prevention
2a         │ └─ is_cliff_left        │ bool                  │ Turn away + backup
2b         │ └─ is_cliff_front_left  │ bool                  │ Turn away + backup
2c         │ └─ is_cliff_right       │ bool                  │ Turn away + backup
2d         │ └─ is_cliff_front_right │ bool                  │ Turn away + backup
───────────┼─────────────────────────┼──────────────────────┼────────────────────
3          │ /wheeldrop              │ Empty (event)         │ EMERGENCY STOP
3a         │ └─ Message received     │ Empty                 │ Disable all motors
───────────┼─────────────────────────┼──────────────────────┼────────────────────
4          │ /battery/charge_ratio   │ Float32 [0.0-1.0]     │ Power management
4a         │ └─ < 0.1 (10%)          │ Float32               │ Low power mode
4b         │ └─ < 0.05 (5%)          │ Float32               │ Return to dock
4c         │ └─ > 0.1                │ Float32               │ Normal operations
───────────┼─────────────────────────┼──────────────────────┼────────────────────
5          │ /cmd_vel (feedback)     │ Twist                 │ Velocity monitoring
5a         │ └─ linear.x ≠ 0         │ float32               │ Robot moving
5b         │ └─ linear.x = 0 (>2s)   │ float32               │ Disable non-critical
───────────┼─────────────────────────┼──────────────────────┼────────────────────
6          │ /battery/voltage        │ Float32               │ Monitoring only
6a         │ └─ Extreme values       │ Float32               │ Diagnostic warning
───────────┼─────────────────────────┼──────────────────────┼────────────────────
7          │ /battery/charging_state │ ChargingState         │ Power management
7a         │ └─ CHARGE_FULL          │ uint8                 │ Ready operation
7b         │ └─ CHARGE_FAULT         │ uint8                 │ Diagnostic warning
───────────┼─────────────────────────┼──────────────────────┼────────────────────
M1         │ /main_brush_motor       │ MotorSetpoint         │ Motor control
M1a        │ └─ Disable on drop      │ duty_cycle = 0.0      │ Safety action
M1b        │ └─ Disable on standstill│ duty_cycle = 0.0      │ Power saving
───────────┼─────────────────────────┼──────────────────────┼────────────────────
M2         │ /vacuum_motor           │ MotorSetpoint         │ Motor control
M2a        │ └─ Disable on drop      │ duty_cycle = 0.0      │ Safety action
M2b        │ └─ Disable on standstill│ duty_cycle = 0.0      │ Power saving
───────────┼─────────────────────────┼──────────────────────┼────────────────────
M3         │ /side_brush_motor       │ MotorSetpoint         │ Motor control
M3a        │ └─ Can use any time     │ duty_cycle varies     │ Nav assist (optional)
"""

# ============================================================================
# IMPLEMENTATION GUIDELINES FOR BEHAVIOR TREE
# ============================================================================

"""
SENSOR ABSTRACTION LAYER DESIGN
───────────────────────────────

For a maintainable and extensible behavior tree, create sensor abstractions:

1. SENSOR_MANAGER CLASS
   Subscribes to all topics and maintains sensor state
   
   Members:
   - bumper_data: Bumper message (latest)
   - cliff_data: Cliff message (latest)
   - wheel_drop_detected: bool (true when event received)
   - battery_ratio: float [0.0-1.0]
   - battery_voltage: float
   - battery_charging_state: ChargingState (enum)
   - last_velocity_time: time (track standstill)
   - current_velocity: float (linear.x from /cmd_vel)
   
   Methods:
   - is_wheel_drop_active() → bool
   - is_cliff_detected() → bool
   - is_bumper_contact() → bool
   - is_low_battery() → bool
   - is_critical_battery() → bool
   - is_robot_standstill(duration_sec) → bool
   - get_bumper_direction() → "left", "right", "both", "none"
   - get_cliff_direction() → "left", "front_left", "right", "front_right", "none"

2. MOTOR_CONTROLLER CLASS
   Publishes to motor topics
   
   Methods:
   - set_main_brush(duty_cycle: float) → void
   - set_vacuum(duty_cycle: float) → void
   - set_side_brush(duty_cycle: float) → void
   - disable_all_motors() → void (set all to 0.0)
   - emergency_stop() → void
   
   State Tracking:
   - Track what's currently on
   - Prevent redundant commands
   - Log all changes

3. BEHAVIOR TREE NODES (Use abstraction)
   Each node queries sensor state, NOT direct topic access
   Each node uses motor controller, NOT direct publishers
   
   Benefits:
   - Sensors decoupled from behaviors
   - Easy to add new sensors
   - Easy to modify actions
   - Testable with mock objects

RECOMMENDED NODE STRUCTURE:
───────────────────────────

class CheckWheelDrop(Behavior):
    def __init__(self, sensor_manager):
        self.sensor_mgr = sensor_manager
    
    def update(self):
        if self.sensor_mgr.is_wheel_drop_active():
            return SUCCESS
        return FAILURE

class CheckLowBattery(Behavior):
    def __init__(self, sensor_manager):
        self.sensor_mgr = sensor_manager
    
    def update(self):
        if self.sensor_mgr.is_low_battery():
            return SUCCESS
        return FAILURE

class DisableAllMotors(Behavior):
    def __init__(self, motor_controller):
        self.motors = motor_controller
    
    def update(self):
        self.motors.disable_all_motors()
        return SUCCESS
"""

if __name__ == "__main__":
    print(__doc__)
