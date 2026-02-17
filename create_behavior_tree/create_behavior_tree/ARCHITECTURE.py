"""
Behavior Tree Architecture Documentation

This visualizes the behavior tree structure and explains each component.
"""

# BEHAVIOR TREE STRUCTURE
# ======================

"""
The complete behavior tree for Create2 robot:

                        ┌─────────────────────────────────┐
                        │      Root Parallel Node         │
                        │  (Allows emergency stop to run  │
                        │   independently of navigation)  │
                        └────┬──────────────────────┬──────┘
                             │                      │
                    ┌────────┴─┐            ┌──────┴─────────┐
                    │           │            │                │
            ┌───────▼──┐  ┌─────▼────┐  ┌───▼────────────────┐
            │Emergency │  │Navigation │  │ ...other systems  │
            │  Stop    │  │ Selector  │  │ (if any)          │
            └───┬──────┘  └─────┬─────┘  └──────────────────┘
                │               │
        ┌───────▼──────┐        │
        │Wheel Drop    │    ┌───┴────────────────────────────────┬──────────────┐
        │Sequence      │    │                                    │              │
        └───┬──────────┘    │                                    │              │
            │           ┌───▼──────────┐  ┌──────────────┐  ┌───▼──────────┐  │
            │           │Both Bumpers  │  │Left Bumper   │  │Right Bumper  │  │
            │           │180° Turn     │  │Turn Right    │  │Turn Left     │  │
            │           │Sequence      │  │Sequence      │  │Sequence      │  │
            │           └──────────────┘  └──────────────┘  └──────────────┘  │
            │                                                                   │
        ┌───▼──────────────────┐                                           ┌───▼──────┐
        │Wheel Drop Check?     │                                           │Cliff     │
        │(Condition)           │                                           │Detection │
        └───┬──────────────────┘                                           │Sequence  │
            │                                                              └───┬──────┘
            │                                                                  │
        ┌───▼──────────────────┐                                           ┌───▼──────┐
        │Disable Motors        │                                           │Any Cliff │
        │(Action)              │                                           │Check?    │
        └──────────────────────┘                                           │(Cond)    │
                                                                           └───┬──────┘
                                                                               │
                                                                           ┌───▼──────┐
                                                                           │Stop      │
                                                                           │(Action)  │
                                                                           └──────────┘

        Default Behavior:
        ┌──────────────────┐
        │Move Forward      │
        │(Action)          │
        └──────────────────┘
"""

# EXECUTION FLOW
# ==============

"""
1. PARALLEL ROOT
   - Runs emergency stop and navigation in parallel
   - Succeeds if both succeed, fails if either fails
   - Emergency stop has priority via higher importance

2. EMERGENCY STOP (Left Branch)
   - Sequence: Check wheel drop → If true, disable motors
   - If wheel drop detected → ALL MOTORS DISABLED (priority action)
   
3. NAVIGATION (Selector - Choose First Match)
   - Selector tries each behavior until one succeeds
   - Order of priority:
   
   a) Both Bumpers? (Highest Priority)
      → Collision from front and back → 180° turn
      → Stop → Turn 180° → Delay
      
   b) Left Bumper Only?
      → Obstacle on left → Turn right
      → Turn while moving → Delay
      
   c) Right Bumper Only?
      → Obstacle on right → Turn left
      → Turn while moving → Delay
      
   d) Cliff Detected?
      → Hazard ahead → Stop immediately
      → Check any cliff sensor → Stop
      
   e) Default: Move Forward (Fallback)
      → No obstacles → Continue forward
      → Always succeeds
"""

# SENSOR MAPPINGS
# ===============

"""
INPUT SENSORS:
- Bumper (create_msgs/Bumper):
  * is_left_pressed: bool
  * is_right_pressed: bool
  * is_light_left/front_left/center_left/front_right/center_right/right: bool
  * light_signal_*: uint16 (0-4095)

- Cliff (create_msgs/Cliff):
  * is_cliff_left: bool
  * is_cliff_front_left: bool
  * is_cliff_right: bool
  * is_cliff_front_right: bool

- Wheel Drop: Inferred from sensor data (driver specific)

OUTPUT COMMANDS:
- Twist (geometry_msgs/Twist) to /cmd_vel:
  * linear.x: Forward velocity (m/s)
  * angular.z: Rotation velocity (rad/s)

- Services (std_srvs/SetBool):
  * /create/cmd_wheel_motors (bool data)
  * /create/cmd_main_brush_motor (bool data)
  * /create/cmd_vacuum_motor (bool data)
"""

# BEHAVIOR LOGIC TABLE
# ====================

"""
CONDITION               │ ACTION                  │ DURATION  │ NEXT STATE
─────────────────────────────────────────────────────────────────────────
Wheel Drop Detected    │ Disable all motors      │ Immediate │ Emergency Stop
─────────────────────────────────────────────────────────────────────────
Both Bumpers           │ Stop + 180° turn        │ 3.5 sec   │ Resume navigation
─────────────────────────────────────────────────────────────────────────
Left Bumper            │ Turn right while moving │ 0.5 sec   │ Resume navigation
─────────────────────────────────────────────────────────────────────────
Right Bumper           │ Turn left while moving  │ 0.5 sec   │ Resume navigation
─────────────────────────────────────────────────────────────────────────
Cliff Detected         │ Stop immediately        │ 0.5 sec   │ Resume navigation
─────────────────────────────────────────────────────────────────────────
No Sensors Active      │ Move forward            │ Continuous│ Continuous
"""

# COMPOSITION PATTERNS USED
# ==========================

"""
1. SELECTOR (OR Gate)
   - Navigation Selector: Try behaviors in priority order
   - Returns SUCCESS if any child succeeds
   - Used for: Decision making between competing behaviors

2. SEQUENCE (AND Gate)
   - Both Bumpers Sequence: Check condition → Then action
   - Wheel Drop Sequence: Check condition → Then action
   - Returns SUCCESS only if ALL children succeed
   - Used for: Complex behaviors with multiple steps

3. PARALLEL
   - Root: Run emergency stop and navigation together
   - Returns SUCCESS if all children succeed
   - Used for: Independent parallel behaviors

4. Simple Behavior Nodes
   - Conditions: Check sensor state
   - Actions: Send velocity commands
   - Returns: SUCCESS, FAILURE, or RUNNING
"""

# EXTENDING THE TREE
# ==================

"""
To add new behaviors:

1. CREATE A NEW BEHAVIOR CLASS
   class MyBehavior(py_trees.behaviour.Behaviour):
       def __init__(self, name, ...):
           super().__init__(name)
       
       def update(self) -> common.Status:
           # Implement logic
           return common.Status.SUCCESS

2. ADD TO TREE IN tree_builder.py
   selector.add_children([
       my_behavior,
       existing_behaviors...
   ])

3. TEST THE BEHAVIOR
   # Use test_behavior_tree.py to simulate sensor events

IMPORTANT: Remember the selector tries children in ORDER
           Place higher-priority behaviors first!
"""

# TIMING ANALYSIS
# ===============

"""
Tree Update Frequency: 10 Hz (100ms per update)

Time For Each Behavior:
- Sensor check: ~1ms (negligible)
- Velocity command: ~10ms
- Tree tick: ~50ms total
- Brake: 50ms between updates

Response Time:
- Bump detection to turn: 100-200ms (1-2 tree updates)
- Cliff detection to stop: 100-200ms
- Emergency stop: 100ms (parallel, high priority)

Behavior Durations:
- Left/Right turn recovery: 0.5 seconds (5 cycles)
- 180° turn: 3 seconds (30 cycles)
- Default recovery delay: 0.5-1.0 seconds
"""

if __name__ == "__main__":
    print(__doc__)
