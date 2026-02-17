#!/usr/bin/env python3
"""
Complete Getting Started Example for Create2 Behavior Tree

This example demonstrates how to use and customize the behavior tree.
Run this to understand how everything works together.
"""

import sys
import os
from pathlib import Path

# Add the create_behavior_tree package to path
pkg_path = Path(__file__).parent
sys.path.insert(0, str(pkg_path.parent))

from create_behavior_tree.config import get_default_config, print_config, BehaviorTreeConfig
from create_behavior_tree.tree_builder import create_tree_and_update_function
from create_behavior_tree.tree_debug import print_tree_info, print_tree_structure
from create_behavior_tree.custom_nodes import SensorData
import py_trees


def example_1_understand_tree_structure():
    """Example 1: Learn about the tree structure."""
    print("\n" + "=" * 80)
    print("EXAMPLE 1: Understanding Tree Structure")
    print("=" * 80)
    print("""
This behavior tree uses a hierarchical decision-making approach:

1. ROOT (Parallel)
   - Emergency Stop sequence (watches for wheel drop)
   - Navigation Selector (chooses best movement)

2. NAVIGATION SELECTOR (priority-based)
   - First checks: Both bumpers? → 180° turn
   - Then checks: Left bumper? → Turn right
   - Then checks: Right bumper? → Turn left
   - Then checks: Cliff? → Stop
   - Finally: Move forward (default)

3. KEY POINT: Selector tries in order, uses FIRST match
   This means most dangerous conditions are checked first.

Benefits:
- Safe: Emergency stop runs in parallel
- Responsive: Highest priorities checked first
- Logical: Behaviors organized by risk level
    """)


def example_2_customize_speeds():
    """Example 2: Customize robot speeds."""
    print("\n" + "=" * 80)
    print("EXAMPLE 2: Customizing Robot Speeds")
    print("=" * 80)
    
    # Load default config
    config = get_default_config()
    print("\nDefault Configuration:")
    print_config(config)
    
    # Modify for conservative movement
    config.velocity.forward_speed = 0.1
    config.velocity.turn_angular_speed = 0.2
    config.timing.tree_update_hz = 20.0
    config.timing.delay_after_turn = 1.0
    
    print("\nModified Configuration (Conservative Mode):")
    print_config(config)
    
    print("\nConfiguration Changes Made:")
    print("  ✓ Reduced forward speed: 0.2 → 0.1 m/s")
    print("  ✓ Reduced turn speed: 0.4 → 0.2 rad/s")
    print("  ✓ Increased update frequency: 10 → 20 Hz")
    print("  ✓ Increased turn delay: 0.5 → 1.0 sec")
    print("\nResult: Robot moves slower, more carefully, with better responsiveness")


def example_3_understand_behaviors():
    """Example 3: Understand individual behaviors."""
    print("\n" + "=" * 80)
    print("EXAMPLE 3: Understanding Individual Behaviors")
    print("=" * 80)
    print("""
The behavior tree is made of SMALL, REUSABLE COMPONENTS:

CONDITION BEHAVIORS (Check current state):
  1. BumperCheckLeft - Is left bumper pressed?
  2. BumperCheckRight - Is right bumper pressed?
  3. BumperCheckBoth - Are both bumpers pressed?
  4. CliffCheckLeft - Is left cliff detected?
  5. CliffCheckRight - Is right cliff detected?
  6. CliffCheckAny - Is any cliff detected?
  7. WheelDropCheck - Is wheel drop detected?

ACTION BEHAVIORS (Send commands):
  1. MoveForward - Move robot forward
  2. TurnSlightRight - Turn right while moving
  3. TurnSlightLeft - Turn left while moving
  4. Turn180Degrees - Make a 180 degree turn
  5. Stop - Stop all movement
  6. DisableMotors - Emergency stop all motors
  7. Delay - Wait for specified duration

COMPOSITION:
  - Sequences: Conditions checked in order (AND logic)
  - Selectors: Options tried in order (OR logic)
  - Parallel: Multiple behaviors at once

Each component has:
  - Name: For debugging and logging
  - Status: SUCCESS, FAILURE, or RUNNING
  - Feedback: Current state message
    """)


def example_4_behavior_flow():
    """Example 4: Trace behavior flow."""
    print("\n" + "=" * 80)
    print("EXAMPLE 4: Behavior Flow Trace")
    print("=" * 80)
    print("""
SCENARIO: Robot bumps into wall on left side

FLOW TRACE:
1. Tree Update (10 Hz = every 100ms)

2. Parallel Root starts (Emergency + Navigation)

3. Emergency Stop checks:
   → WheelDropCheck() → Wheel not dropped → FAILURE
   → Continue to Navigation (since emergency didn't succeed)

4. Navigation Selector tries in order:

   a) Both Bumpers Sequence:
      → BumperCheckBoth() → Not both pressed → FAILURE
      → Try next selector option
      
   b) Left Bumper Sequence:
      → BumperCheckLeft() → LEFT BUMPER PRESSED! → SUCCESS
      → Continue sequence...
      → TurnSlightRight() → Send turn command → SUCCESS
      → Delay() → Wait 0.5 seconds → SUCCESS
      → Sequence complete → Selector returns SUCCESS
      
5. Selector found a match, no more options tried

6. Root Parallel: Both branches succeeded → RUNNING (continuous)

7. Next update (100ms later): Test again, may continue same behavior
   or action changes based on new sensor data

TIMELINE:
  Time    Action
  ─────────────────────────────────────────
  0ms     Left bumper detected
  100ms   First tree update → Turn command
  200ms   Check again → Still bumper? Continue turn
  600ms   Delay time expired → Sequence SUCCESS
  700ms   Check again → Bumper released? → Move forward
    """)


def example_5_extending_tree():
    """Example 5: How to extend the tree."""
    print("\n" + "=" * 80)
    print("EXAMPLE 5: Extending the Behavior Tree")
    print("=" * 80)
    print("""
STEP 1: Create a new behavior in custom_nodes.py

    class AvoidObstacleLeft(py_trees.behaviour.Behaviour):
        def __init__(self, name, node, sensor_data):
            super().__init__(name)
            self.node = node
            self.sensor_data = sensor_data
            self.publisher = node.create_publisher(Twist, '/cmd_vel', 10)
        
        def update(self):
            with self.sensor_data.lock:
                if self.sensor_data.bumper_data.is_light_left > 2000:
                    # Obstacle on left, turn right
                    twist = Twist()
                    twist.linear.x = 0.1
                    twist.angular.z = -0.3
                    self.publisher.publish(twist)
                    return common.Status.SUCCESS
            return common.Status.FAILURE

STEP 2: Add to tree in tree_builder.py

    # Before MoveForward, add:
    avoid_left = AvoidObstacleLeft("Avoid Left", node, sensor_data)
    
    # Update navigation selector children:
    navigation_selector.add_children([
        ...,
        avoid_left,  # NEW - Try this before moving forward
        move_forward
    ])

STEP 3: Test with mock sensor events

    from create_behavior_tree.test_behavior_tree import BehaviorTreeTester
    tester = BehaviorTreeTester()
    tester.publish_bumper_event(left_light_signal=3000)

BEST PRACTICES:
  ✓ Keep behaviors small and focused
  ✓ Use descriptive names
  ✓ Make behaviors reusable
  ✓ Always handle edge cases
  ✓ Return correct Status values
  ✓ Thread-safe for sensor access
  ✓ Add feedback messages for debugging
    """)


def example_6_practical_tuning():
    """Example 6: Practical tuning guide."""
    print("\n" + "=" * 80)
    print("EXAMPLE 6: Practical Tuning Guide")
    print("=" * 80)
    print("""
PROBLEM: Robot is too slow
SOLUTION: Increase forward_speed and turn_angular_speed
  velocity:
    forward_speed: 0.4        # Increase speed
    turn_angular_speed: 0.6   # Faster turns
  tree_update_hz: 20.0        # More frequent updates

PROBLEM: Robot crashes into walls
SOLUTION: Detect obstacles earlier or softer response
  approach 1: Increase light sensor threshold to detect earlier
    sensors:
      light_sensor_threshold: 1000  # More sensitive
  
  approach 2: Reduce turn speed for gentler avoidance
    velocity:
      turn_angular_speed: 0.2  # Less aggressive

PROBLEM: Robot overshoots and misses turns
SOLUTION: Increase delays and reduce turn duration
  timing:
    delay_after_turn: 1.0         # Longer pause
    turn_180_duration: 4.0        # Slower 180 turn
    delay_before_backup: 1.0      # More recovery time

PROBLEM: Emergency stop triggers too often (false positives)
SOLUTION: Check sensor tuning or add debouncing
  sensors:
    bumper_debounce_count: 3  # Require 3 readings to confirm
    wheel_drop_detection_enabled: false  # Disable if faulty

MONITORING:
  ros2 topic echo /cmd_vel        # See actual commands
  ros2 topic echo /create/bumper  # Check sensor data
  ros2 topic hz /cmd_vel          # Verify update rate
    """)


def example_7_debugging_tips():
    """Example 7: Debugging tips."""
    print("\n" + "=" * 80)
    print("EXAMPLE 7: Debugging Tips")
    print("=" * 80)
    print("""
ENABLE DEBUG LOGGING:
  ros2 run create_behavior_tree behavior_tree_node --ros-args --log-level debug

VIEW TREE STRUCTURE:
  from create_behavior_tree.tree_debug import print_tree_structure
  print_tree_structure(root)

CHECK NODE STATUS:
  from create_behavior_tree.tree_debug import print_tree_info
  print_tree_info(root)

ADD CUSTOM LOGGING:
  In custom_nodes.py behavior update():
    self.feedback_message = f"Status: {condition_value}"
  Then view with:
    node.get_logger().info(f"Node feedback: {behavior.feedback_message}")

TRACE EXECUTION:
  Add print statements in update() methods:
    print(f"Checking {self.name}...")

SIMULATE SENSOR EVENTS:
  python3 -m create_behavior_tree.test_behavior_tree
  Or publish manually:
    ros2 topic pub /create/bumper create_msgs/Bumper '...'

MONITOR ALL TOPICS:
  ros2 topic list
  ros2 topic hz <topic_name>
  ros2 topic echo <topic_name>

CHECK TREE EXECUTION TIME:
  from time import time
  start = time()
  root.tick_once()
  elapsed = time() - start
  print(f"Tree update took {elapsed*1000:.2f}ms")
    """)


def main():
    """Run all examples."""
    print("\n")
    print("╔" + "=" * 78 + "╗")
    print("║" + " CREATE2 BEHAVIOR TREE - COMPLETE GETTING STARTED GUIDE ".center(78) + "║")
    print("╚" + "=" * 78 + "╝")
    
    examples = [
        example_1_understand_tree_structure,
        example_2_customize_speeds,
        example_3_understand_behaviors,
        example_4_behavior_flow,
        example_5_extending_tree,
        example_6_practical_tuning,
        example_7_debugging_tips,
    ]
    
    print("\nThis guide contains 7 practical examples:")
    for i, example in enumerate(examples, 1):
        print(f"  {i}. {example.__doc__.strip()}")
    
    print("\n" + "-" * 80)
    
    # Run examples
    try:
        while True:
            print("\nSelect an example to run (1-7) or 'all' for all examples, 'q' to quit:")
            choice = input("Your choice: ").strip().lower()
            
            if choice == 'q':
                print("\nExiting. Happy robotics! 🤖")
                break
            elif choice == 'all':
                for example in examples:
                    example()
                    input("\nPress Enter to continue...")
            else:
                try:
                    idx = int(choice) - 1
                    if 0 <= idx < len(examples):
                        examples[idx]()
                    else:
                        print("Invalid choice. Please select 1-7.")
                except ValueError:
                    print("Invalid input. Please enter a number or 'all'/'q'.")
    
    except KeyboardInterrupt:
        print("\n\nExiting. Happy robotics! 🤖")


if __name__ == "__main__":
    main()
