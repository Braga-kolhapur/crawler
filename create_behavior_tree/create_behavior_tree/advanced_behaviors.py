"""
Advanced example: Extending the behavior tree with custom behaviors.

This module demonstrates how to create custom behavior tree nodes
and integrate them into the existing tree structure.
"""

import py_trees
from py_trees import common, composites
from geometry_msgs.msg import Twist
import math


class ObstacleAvoidanceNode(py_trees.behaviour.Behaviour):
    """
    Advanced obstacle avoidance using light sensor data.
    
    This behavior uses the light sensor signals to implement
    more nuanced obstacle avoidance than simple binary detection.
    """

    def __init__(self, name, node, sensor_data, threshold=2000):
        super().__init__(name)
        self.node = node
        self.sensor_data = sensor_data
        self.threshold = threshold  # Light sensor signal threshold
        self.publisher = node.create_publisher(Twist, '/cmd_vel', 10)

    def update(self) -> common.Status:
        """Implement proportional obstacle avoidance."""
        with self.sensor_data.lock:
            bumper = self.sensor_data.bumper_data

        # Get light sensor signals (0-4095 range)
        signals = {
            'left': bumper.light_signal_left,
            'front_left': bumper.light_signal_front_left,
            'center_left': bumper.light_signal_center_left,
            'center_right': bumper.light_signal_center_right,
            'front_right': bumper.light_signal_front_right,
            'right': bumper.light_signal_right,
        }

        # Calculate weighted turning based on detected obstacles
        left_signal = (signals['left'] + signals['front_left'] + signals['center_left']) / 3
        right_signal = (signals['right'] + signals['front_right'] + signals['center_right']) / 3

        # Normalize to -1..1 range
        left_influence = min(1.0, max(-1.0, (left_signal - self.threshold) / 2000))
        right_influence = min(1.0, max(-1.0, (right_signal - self.threshold) / 2000))

        # Calculate turning command (positive = left, negative = right)
        angular_z = (right_influence - left_influence) * 0.5

        # Only turn if significant obstacle detected
        if abs(angular_z) > 0.1:
            twist = Twist()
            twist.linear.x = 0.1  # Reduced speed
            twist.angular.z = angular_z
            self.publisher.publish(twist)
            self.feedback_message = f"Obstacle avoidance: angular_z={angular_z:.2f}"
            return common.Status.SUCCESS
        else:
            self.feedback_message = "No significant obstacle"
            return common.Status.FAILURE


class RotationNode(py_trees.behaviour.Behaviour):
    """
    Generic rotation behavior with configurable angle and direction.
    """

    def __init__(self, name, node, angle_degrees, duration=None):
        super().__init__(name)
        self.node = node
        self.angle_degrees = angle_degrees
        # If duration not specified, estimate from angle (360 deg in ~3 seconds)
        self.duration = duration or abs(angle_degrees) / 120.0
        self.publisher = node.create_publisher(Twist, '/cmd_vel', 10)
        self.start_time = None

    def setup(self, **kwargs):
        """Reset timer on setup."""
        self.start_time = None

    def update(self) -> common.Status:
        """Execute rotation."""
        from time import time

        if self.start_time is None:
            self.start_time = time()

        elapsed = time() - self.start_time

        if elapsed < self.duration:
            twist = Twist()
            # Positive angle = counterclockwise (positive angular velocity)
            twist.angular.z = (1.0 if self.angle_degrees > 0 else -1.0)
            self.publisher.publish(twist)
            percent = (elapsed / self.duration) * 100
            self.feedback_message = f"Rotating {percent:.0f}%"
            return common.Status.RUNNING
        else:
            # Stop
            twist = Twist()
            self.publisher.publish(twist)
            self.start_time = None
            self.feedback_message = "Rotation complete"
            return common.Status.SUCCESS


class VelocityControlNode(py_trees.behaviour.Behaviour):
    """
    Flexible velocity control with linear and angular components.
    """

    def __init__(self, name, node, linear_x=0.0, angular_z=0.0, duration=None):
        super().__init__(name)
        self.node = node
        self.linear_x = linear_x
        self.angular_z = angular_z
        self.duration = duration  # Duration to apply command (None = indefinite)
        self.publisher = node.create_publisher(Twist, '/cmd_vel', 10)
        self.start_time = None

    def setup(self, **kwargs):
        """Reset timer on setup."""
        self.start_time = None

    def update(self) -> common.Status:
        """Apply velocity command."""
        from time import time

        if self.start_time is None:
            self.start_time = time()

        twist = Twist()
        twist.linear.x = self.linear_x
        twist.angular.z = self.angular_z
        self.publisher.publish(twist)

        if self.duration is None:
            self.feedback_message = f"Velocity: linear={self.linear_x:.2f}, angular={self.angular_z:.2f}"
            return common.Status.RUNNING
        else:
            elapsed = time() - self.start_time
            if elapsed >= self.duration:
                self.start_time = None
                self.feedback_message = "Velocity command complete"
                return common.Status.SUCCESS
            else:
                self.feedback_message = f"Velocity: {(elapsed/self.duration)*100:.0f}%"
                return common.Status.RUNNING


class ConditionalNode(py_trees.behaviour.Behaviour):
    """
    Conditional node that executes a lambda function to determine status.
    Useful for quick conditional checks.
    """

    def __init__(self, name, condition_func):
        super().__init__(name)
        self.condition_func = condition_func

    def update(self) -> common.Status:
        """Check condition."""
        try:
            if self.condition_func():
                self.feedback_message = "Condition TRUE"
                return common.Status.SUCCESS
            else:
                self.feedback_message = "Condition FALSE"
                return common.Status.FAILURE
        except Exception as e:
            self.feedback_message = f"Error: {str(e)}"
            return common.Status.FAILURE


class LoggingDecorator(py_trees.decorators.Decorator):
    """
    Decorator that logs when a behavior runs.
    """

    def __init__(self, name, child, logger=None):
        super().__init__(name, child)
        self.logger = logger

    def update(self) -> common.Status:
        """Update with logging."""
        status = self.decorated.tick_once()

        if self.logger:
            self.logger.debug(f"{self.name}: {self.decorated.name} -> {status}")

        return status


# Example: Building an extended behavior tree with custom nodes

def build_advanced_tree(node, sensor_data):
    """
    Example of building a more advanced behavior tree with custom nodes.
    
    This demonstrates how to use the custom nodes above to create
    more sophisticated robot behaviors.
    """
    from create_behavior_tree.custom_nodes import (
        MoveForward, BumperCheckBoth, Stop, Delay
    )

    # Advanced obstacle avoidance
    obstacle_avoidance = ObstacleAvoidanceNode(
        "Advanced Obstacle Avoidance",
        node,
        sensor_data,
        threshold=1500
    )

    # Rotate 45 degrees CCW
    turn_45_ccw = RotationNode("Turn 45° CCW", node, 45.0)

    # Move backward slowly for 2 seconds
    backup_sequence = composites.Sequence(
        name="Backup",
        memory=False,
        children=[
            Stop("Stop", node),
            Delay("Delay", 0.5),
            VelocityControlNode("Move Backward", node, linear_x=-0.1, duration=2.0),
        ]
    )

    # Example recovery behavior for persistent collision
    recovery_selector = composites.Selector(
        name="Collision Recovery",
        memory=False,
        children=[
            turn_45_ccw,
            backup_sequence,
            MoveForward("Resume Forward", node)
        ]
    )

    # Use conditional node for complex logic
    def check_battery_low():
        # This would check actual battery status
        return False

    battery_check = ConditionalNode("Battery OK", check_battery_low)

    # Main tree
    root = composites.Sequence(
        name="Advanced Tree Root",
        memory=False,
        children=[
            battery_check,
            recovery_selector,
        ]
    )

    return root


if __name__ == "__main__":
    print("Advanced Behavior Tree Example")
    print("=" * 80)
    print("\nThis module provides:")
    print("  - ObstacleAvoidanceNode: Proportional obstacle detection")
    print("  - RotationNode: Generic rotation behavior")
    print("  - VelocityControlNode: Flexible motion control")
    print("  - ConditionalNode: Lambda-based conditions")
    print("  - LoggingDecorator: Debug logging for behaviors")
    print("\nSee the module docstrings for usage examples.")
