"""
Refactored Main Behavior Tree Node using SensorManager and MotorController

This is the improved version with better sensor/motor management.
"""

import rclpy
from rclpy.node import Node
import py_trees
from py_trees import common
from create_behavior_tree.sensor_motor_manager import SensorManager, MotorController
from create_behavior_tree.refactored_tree_builder import (
    build_refactored_behavior_tree,
    create_and_update_function
)


class RefactoredBehaviorTreeNode(Node):
    """
    Refactored ROS2 node that runs the Create2 behavior tree.
    
    This version uses SensorManager and MotorController abstractions
    for cleaner, more maintainable code.
    """

    def __init__(self):
        super().__init__('refactored_behavior_tree_node')

        # Create sensor and motor managers
        self.sensor_mgr = SensorManager(self)
        self.motor_ctrl = MotorController(self)

        # Tree update frequency
        self.declare_parameter('tree_update_hz', 10.0)
        update_hz = self.get_parameter('tree_update_hz').value
        self.update_period = 1.0 / update_hz

        # Create behavior tree
        self.root = build_refactored_behavior_tree(self.sensor_mgr, self.motor_ctrl)
        self.update_tree = create_and_update_function(self.root)

        # Logging
        self.get_logger().info('Refactored Behavior Tree Node initialized')
        self.get_logger().info(f'Tree update frequency: {update_hz} Hz')
        self.get_logger().info('Using SensorManager and MotorController abstractions')

        # Timer for tree updates
        self.tree_timer = self.create_timer(self.update_period, self._tree_update_callback)

    def _tree_update_callback(self) -> None:
        """Update behavior tree periodically."""
        try:
            status = self.update_tree()
            # Uncomment for debug logging
            # self.get_logger().debug(f"Tree status: {status}")
        except Exception as e:
            self.get_logger().error(f"Error updating behavior tree: {str(e)}")
            import traceback
            traceback.print_exc()

    def get_tree_status_string(self) -> str:
        """Get a string representation of the tree status."""
        return py_trees.display.unicode_tree(
            self.root,
            show_status=True,
            show_disabled=True
        )

    def print_tree_info(self) -> None:
        """Print tree structure and status."""
        print("\n" + "=" * 80)
        print("BEHAVIOR TREE STATUS")
        print("=" * 80)
        print(self.get_tree_status_string())
        print("=" * 80 + "\n")

    def print_sensor_status(self) -> None:
        """Print current sensor status."""
        print("\n" + "=" * 80)
        print("SENSOR STATUS")
        print("=" * 80)
        
        print(f"\nWheel Drop: {self.sensor_mgr.is_wheel_drop_active()}")
        print(f"Cliff Detected: {self.sensor_mgr.is_cliff_detected()}")
        if self.sensor_mgr.is_cliff_detected():
            print(f"  Direction: {self.sensor_mgr.get_cliff_direction()}")
        
        print(f"Bumper Contact: {self.sensor_mgr.is_bumper_contact()}")
        if self.sensor_mgr.is_bumper_contact():
            print(f"  Direction: {self.sensor_mgr.get_bumper_direction()}")
        
        battery = self.sensor_mgr.get_battery_info()
        print(f"\nBattery:")
        print(f"  Ratio: {battery.charge_ratio * 100:.1f}%")
        print(f"  Voltage: {battery.voltage:.2f}V")
        print(f"  Current: {battery.current:.2f}A")
        print(f"  Low Battery: {self.sensor_mgr.is_low_battery()}")
        print(f"  Critical Battery: {self.sensor_mgr.is_critical_battery()}")
        
        print(f"\nRobot Standstill: {self.sensor_mgr.is_robot_standstill()}")
        print(f"Current Velocity: {self.sensor_mgr.current_velocity:.3f} m/s")
        
        motor_state = self.motor_ctrl.get_state()
        print(f"\nMotor State:")
        print(f"  Main Brush: {motor_state['main_brush']:.2f}")
        print(f"  Vacuum: {motor_state['vacuum']:.2f}")
        print(f"  Side Brush: {motor_state['side_brush']:.2f}")
        print(f"  Linear Velocity: {motor_state['linear_velocity']:.3f} m/s")
        print(f"  Angular Velocity: {motor_state['angular_velocity']:.3f} rad/s")
        
        print("=" * 80 + "\n")


def main():
    """Main entry point."""
    rclpy.init()

    behavior_tree_node = RefactoredBehaviorTreeNode()

    try:
        rclpy.spin(behavior_tree_node)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean shutdown
        behavior_tree_node.get_logger().info("Shutting down...")
        behavior_tree_node.motor_ctrl.emergency_stop()
        behavior_tree_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
