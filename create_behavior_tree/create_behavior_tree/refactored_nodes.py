"""
Refactored Custom Behavior Tree Nodes using Sensor and Motor Manager

These nodes use the abstracted SensorManager and MotorController
for cleaner, more maintainable code.
"""

import py_trees
from py_trees import common
import time


# ============================================================================
# SENSOR CONDITION NODES
# ============================================================================

class CheckWheelDrop(py_trees.behaviour.Behaviour):
    """Check if wheel drop is detected."""

    def __init__(self, name: str, sensor_manager):
        super().__init__(name)
        self.sensor_mgr = sensor_manager

    def update(self) -> common.Status:
        """Check wheel drop status."""
        if self.sensor_mgr.is_wheel_drop_active():
            self.feedback_message = "Wheel drop ACTIVE"
            return common.Status.SUCCESS
        self.feedback_message = "No wheel drop"
        return common.Status.FAILURE


class CheckCliffDetection(py_trees.behaviour.Behaviour):
    """Check if any cliff is detected."""

    def __init__(self, name: str, sensor_manager):
        super().__init__(name)
        self.sensor_mgr = sensor_manager

    def update(self) -> common.Status:
        """Check cliff status."""
        if self.sensor_mgr.is_cliff_detected():
            direction = self.sensor_mgr.get_cliff_direction()
            self.feedback_message = f"Cliff detected: {direction}"
            return common.Status.SUCCESS
        self.feedback_message = "No cliff"
        return common.Status.FAILURE


class CheckBumperContact(py_trees.behaviour.Behaviour):
    """Check if bumper is in contact."""

    def __init__(self, name: str, sensor_manager):
        super().__init__(name)
        self.sensor_mgr = sensor_manager

    def update(self) -> common.Status:
        """Check bumper status."""
        if self.sensor_mgr.is_bumper_contact():
            direction = self.sensor_mgr.get_bumper_direction()
            self.feedback_message = f"Bumper contact: {direction}"
            return common.Status.SUCCESS
        self.feedback_message = "No bumper contact"
        return common.Status.FAILURE


class CheckLeftBumper(py_trees.behaviour.Behaviour):
    """Check if only left bumper is pressed."""

    def __init__(self, name: str, sensor_manager):
        super().__init__(name)
        self.sensor_mgr = sensor_manager

    def update(self) -> common.Status:
        """Check left bumper only."""
        direction = self.sensor_mgr.get_bumper_direction()
        if direction == "left":
            self.feedback_message = "Left bumper pressed"
            return common.Status.SUCCESS
        self.feedback_message = "Left bumper not pressed"
        return common.Status.FAILURE


class CheckRightBumper(py_trees.behaviour.Behaviour):
    """Check if only right bumper is pressed."""

    def __init__(self, name: str, sensor_manager):
        super().__init__(name)
        self.sensor_mgr = sensor_manager

    def update(self) -> common.Status:
        """Check right bumper only."""
        direction = self.sensor_mgr.get_bumper_direction()
        if direction == "right":
            self.feedback_message = "Right bumper pressed"
            return common.Status.SUCCESS
        self.feedback_message = "Right bumper not pressed"
        return common.Status.FAILURE


class CheckBothBumpers(py_trees.behaviour.Behaviour):
    """Check if both bumpers are pressed."""

    def __init__(self, name: str, sensor_manager):
        super().__init__(name)
        self.sensor_mgr = sensor_manager

    def update(self) -> common.Status:
        """Check both bumpers."""
        direction = self.sensor_mgr.get_bumper_direction()
        if direction == "both":
            self.feedback_message = "Both bumpers pressed"
            return common.Status.SUCCESS
        self.feedback_message = "Not both bumpers"
        return common.Status.FAILURE


class CheckLowBattery(py_trees.behaviour.Behaviour):
    """Check if battery is low (< 10%)."""

    def __init__(self, name: str, sensor_manager, threshold: float = 0.1):
        super().__init__(name)
        self.sensor_mgr = sensor_manager
        self.threshold = threshold

    def update(self) -> common.Status:
        """Check low battery."""
        battery = self.sensor_mgr.get_battery_info()
        if battery.charge_ratio < self.threshold:
            pct = battery.charge_ratio * 100
            self.feedback_message = f"Low battery: {pct:.1f}%"
            return common.Status.SUCCESS
        self.feedback_message = f"Battery OK: {battery.charge_ratio*100:.1f}%"
        return common.Status.FAILURE


class CheckCriticalBattery(py_trees.behaviour.Behaviour):
    """Check if battery is critical (< 5%)."""

    def __init__(self, name: str, sensor_manager, threshold: float = 0.05):
        super().__init__(name)
        self.sensor_mgr = sensor_manager
        self.threshold = threshold

    def update(self) -> common.Status:
        """Check critical battery."""
        battery = self.sensor_mgr.get_battery_info()
        if battery.charge_ratio < self.threshold:
            pct = battery.charge_ratio * 100
            self.feedback_message = f"CRITICAL battery: {pct:.1f}%"
            return common.Status.SUCCESS
        self.feedback_message = f"Battery above critical"
        return common.Status.FAILURE


class CheckRobotStandstill(py_trees.behaviour.Behaviour):
    """Check if robot is stationary."""

    def __init__(self, name: str, sensor_manager, duration_sec: float = 2.0):
        super().__init__(name)
        self.sensor_mgr = sensor_manager
        self.duration = duration_sec

    def update(self) -> common.Status:
        """Check standstill status."""
        if self.sensor_mgr.is_robot_standstill(self.duration):
            self.feedback_message = f"Robot standstill for {self.duration}s"
            return common.Status.SUCCESS
        self.feedback_message = "Robot moving or not enough standstill time"
        return common.Status.FAILURE


# ============================================================================
# MOTOR CONTROL ACTION NODES
# ============================================================================

class DisableAllMotors(py_trees.behaviour.Behaviour):
    """Disable all non-wheel motors (safety action)."""

    def __init__(self, name: str, motor_controller):
        super().__init__(name)
        self.motor_ctrl = motor_controller

    def update(self) -> common.Status:
        """Disable all motors."""
        if self.motor_ctrl.disable_all_motors():
            self.feedback_message = "All motors disabled"
            return common.Status.SUCCESS
        self.feedback_message = "Failed to disable motors"
        return common.Status.FAILURE


class EmergencyStop(py_trees.behaviour.Behaviour):
    """Emergency stop - wheels and all motors."""

    def __init__(self, name: str, motor_controller):
        super().__init__(name)
        self.motor_ctrl = motor_controller
        self.stopped = False

    def setup(self, **kwargs):
        """Reset on setup."""
        self.stopped = False

    def update(self) -> common.Status:
        """Execute emergency stop."""
        if not self.stopped:
            if self.motor_ctrl.emergency_stop():
                self.stopped = True
                self.feedback_message = "EMERGENCY STOP executed"
                return common.Status.SUCCESS
            else:
                self.feedback_message = "Failed to execute emergency stop"
                return common.Status.FAILURE
        self.feedback_message = "Emergency stop maintained"
        return common.Status.SUCCESS


class StopWheels(py_trees.behaviour.Behaviour):
    """Stop wheel movement."""

    def __init__(self, name: str, motor_controller):
        super().__init__(name)
        self.motor_ctrl = motor_controller

    def update(self) -> common.Status:
        """Stop wheels."""
        if self.motor_ctrl.stop_wheels():
            self.feedback_message = "Wheels stopped"
            return common.Status.SUCCESS
        self.feedback_message = "Failed to stop wheels"
        return common.Status.FAILURE


class MoveForwardAction(py_trees.behaviour.Behaviour):
    """Move robot forward."""

    def __init__(self, name: str, motor_controller, speed: float = 0.2):
        super().__init__(name)
        self.motor_ctrl = motor_controller
        self.speed = speed

    def update(self) -> common.Status:
        """Move forward."""
        if self.motor_ctrl.set_velocity(self.speed, 0.0):
            self.feedback_message = f"Moving forward at {self.speed} m/s"
            return common.Status.SUCCESS
        self.feedback_message = "Failed to command forward motion"
        return common.Status.FAILURE


class TurnRightAction(py_trees.behaviour.Behaviour):
    """Turn robot right."""

    def __init__(self, name: str, motor_controller, 
                 linear_speed: float = 0.15, angular_speed: float = 0.4):
        super().__init__(name)
        self.motor_ctrl = motor_controller
        self.linear_speed = linear_speed
        self.angular_speed = angular_speed

    def update(self) -> common.Status:
        """Turn right."""
        if self.motor_ctrl.set_velocity(self.linear_speed, -self.angular_speed):
            self.feedback_message = f"Turning right"
            return common.Status.SUCCESS
        self.feedback_message = "Failed to command right turn"
        return common.Status.FAILURE


class TurnLeftAction(py_trees.behaviour.Behaviour):
    """Turn robot left."""

    def __init__(self, name: str, motor_controller,
                 linear_speed: float = 0.15, angular_speed: float = 0.4):
        super().__init__(name)
        self.motor_ctrl = motor_controller
        self.linear_speed = linear_speed
        self.angular_speed = angular_speed

    def update(self) -> common.Status:
        """Turn left."""
        if self.motor_ctrl.set_velocity(self.linear_speed, self.angular_speed):
            self.feedback_message = f"Turning left"
            return common.Status.SUCCESS
        self.feedback_message = "Failed to command left turn"
        return common.Status.FAILURE


class Turn180Action(py_trees.behaviour.Behaviour):
    """Execute 180 degree turn."""

    def __init__(self, name: str, motor_controller, duration: float = 3.0):
        super().__init__(name)
        self.motor_ctrl = motor_controller
        self.duration = duration
        self.start_time = None

    def setup(self, **kwargs):
        """Reset on setup."""
        self.start_time = None

    def update(self) -> common.Status:
        """Execute 180 degree turn."""
        if self.start_time is None:
            self.start_time = time.time()

        elapsed = time.time() - self.start_time

        if elapsed < self.duration:
            # Keep turning
            if not self.motor_ctrl.set_velocity(0.0, 1.0):
                self.feedback_message = "Failed to turn"
                return common.Status.FAILURE
            pct = (elapsed / self.duration) * 100
            self.feedback_message = f"Turning 180°: {pct:.0f}%"
            return common.Status.RUNNING
        else:
            # Turn complete
            self.motor_ctrl.stop_wheels()
            self.start_time = None
            self.feedback_message = "180° turn complete"
            return common.Status.SUCCESS


class WaitDuration(py_trees.behaviour.Behaviour):
    """Wait for specified duration."""

    def __init__(self, name: str, duration: float = 1.0):
        super().__init__(name)
        self.duration = duration
        self.start_time = None

    def setup(self, **kwargs):
        """Reset on setup."""
        self.start_time = None

    def update(self) -> common.Status:
        """Wait for duration."""
        if self.start_time is None:
            self.start_time = time.time()

        elapsed = time.time() - self.start_time

        if elapsed < self.duration:
            pct = (elapsed / self.duration) * 100
            self.feedback_message = f"Waiting: {pct:.0f}%"
            return common.Status.RUNNING
        else:
            self.start_time = None
            self.feedback_message = "Wait complete"
            return common.Status.SUCCESS


class ReduceSpeedAction(py_trees.behaviour.Behaviour):
    """Reduce robot speed for low battery."""

    def __init__(self, name: str, motor_controller, reduced_speed: float = 0.15):
        super().__init__(name)
        self.motor_ctrl = motor_controller
        self.reduced_speed = reduced_speed

    def update(self) -> common.Status:
        """Move at reduced speed."""
        if self.motor_ctrl.set_velocity(self.reduced_speed, 0.0):
            self.feedback_message = f"Moving at reduced speed {self.reduced_speed} m/s"
            return common.Status.SUCCESS
        self.feedback_message = "Failed to set reduced speed"
        return common.Status.FAILURE


class BackupAction(py_trees.behaviour.Behaviour):
    """Back up robot."""

    def __init__(self, name: str, motor_controller, 
                 backup_speed: float = -0.1, duration: float = 1.0):
        super().__init__(name)
        self.motor_ctrl = motor_controller
        self.backup_speed = backup_speed
        self.duration = duration
        self.start_time = None

    def setup(self, **kwargs):
        """Reset on setup."""
        self.start_time = None

    def update(self) -> common.Status:
        """Back up."""
        if self.start_time is None:
            self.start_time = time.time()

        elapsed = time.time() - self.start_time

        if elapsed < self.duration:
            if not self.motor_ctrl.set_velocity(self.backup_speed, 0.0):
                self.feedback_message = "Failed to backup"
                return common.Status.FAILURE
            self.feedback_message = f"Backing up..."
            return common.Status.RUNNING
        else:
            self.motor_ctrl.stop_wheels()
            self.start_time = None
            self.feedback_message = "Backup complete"
            return common.Status.SUCCESS
