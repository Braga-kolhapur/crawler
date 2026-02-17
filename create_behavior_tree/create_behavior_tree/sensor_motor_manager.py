"""
Sensor Manager and Motor Controller

This module provides high-level abstractions for sensor monitoring
and motor control, making the behavior tree cleaner and more maintainable.
"""

import threading
from dataclasses import dataclass
from typing import Optional
import rclpy
from rclpy.node import Node
from create_msgs.msg import Bumper, Cliff, ChargingState, MotorSetpoint
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Empty
import logging


@dataclass
class BatteryInfo:
    """Battery information snapshot."""
    voltage: float = 0.0
    current: float = 0.0
    charge: float = 0.0
    capacity: float = 0.0
    charge_ratio: float = 0.0
    temperature: int = 0
    charging_state: int = ChargingState.CHARGE_NONE


class SensorManager:
    """
    Manages all sensor inputs from the Create2 robot.
    
    Subscribes to all sensor topics and provides clean interfaces
    for querying sensor state by the behavior tree.
    """

    def __init__(self, node: Node):
        """
        Initialize sensor manager.
        
        Args:
            node: ROS2 node for creating subscriptions
        """
        self.node = node
        self.lock = threading.Lock()
        self.logger = node.get_logger()

        # Sensor state
        self.bumper_data = Bumper()
        self.cliff_data = Cliff()
        self.battery_info = BatteryInfo()
        self.wheel_drop_detected = False
        self.wheel_drop_time = None
        self.last_cmd_vel = None
        self.last_cmd_vel_time = None
        self.current_velocity = 0.0

        # Subscriptions
        self.bumper_sub = node.create_subscription(
            Bumper, '/bumper', self._bumper_callback, 10
        )
        self.cliff_sub = node.create_subscription(
            Cliff, '/cliff', self._cliff_callback, 10
        )
        self.wheel_drop_sub = node.create_subscription(
            Empty, '/wheeldrop', self._wheel_drop_callback, 10
        )
        self.battery_ratio_sub = node.create_subscription(
            Float32, '/battery/charge_ratio', self._battery_ratio_callback, 10
        )
        self.charging_state_sub = node.create_subscription(
            ChargingState, '/battery/charging_state', self._charging_state_callback, 10
        )
        self.voltage_sub = node.create_subscription(
            Float32, '/battery/voltage', self._voltage_callback, 10
        )
        self.current_sub = node.create_subscription(
            Float32, '/battery/current', self._current_callback, 10
        )
        self.cmd_vel_sub = node.create_subscription(
            Twist, '/cmd_vel', self._cmd_vel_callback, 10
        )

        self.logger.info("SensorManager initialized")

    def _bumper_callback(self, msg: Bumper) -> None:
        """Handle bumper sensor data."""
        with self.lock:
            self.bumper_data = msg

    def _cliff_callback(self, msg: Cliff) -> None:
        """Handle cliff sensor data."""
        with self.lock:
            self.cliff_data = msg

    def _wheel_drop_callback(self, msg: Empty) -> None:
        """Handle wheel drop event."""
        with self.lock:
            self.wheel_drop_detected = True
            self.wheel_drop_time = self.node.get_clock().now()
            self.logger.warn("WHEEL DROP DETECTED!")

    def _battery_ratio_callback(self, msg: Float32) -> None:
        """Handle battery ratio."""
        with self.lock:
            self.battery_info.charge_ratio = msg.data

    def _charging_state_callback(self, msg: ChargingState) -> None:
        """Handle charging state."""
        with self.lock:
            self.battery_info.charging_state = msg.state

    def _voltage_callback(self, msg: Float32) -> None:
        """Handle voltage."""
        with self.lock:
            self.battery_info.voltage = msg.data

    def _current_callback(self, msg: Float32) -> None:
        """Handle current."""
        with self.lock:
            self.battery_info.current = msg.data

    def _cmd_vel_callback(self, msg: Twist) -> None:
        """Monitor velocity commands to track robot movement."""
        with self.lock:
            self.last_cmd_vel = msg
            self.last_cmd_vel_time = self.node.get_clock().now()
            self.current_velocity = msg.linear.x

    # =====================================================================
    # QUERY METHODS - Used by behavior tree
    # =====================================================================

    def is_wheel_drop_active(self) -> bool:
        """Check if wheel drop is currently detected."""
        with self.lock:
            return self.wheel_drop_detected

    def is_cliff_detected(self) -> bool:
        """Check if any cliff sensor is triggered."""
        with self.lock:
            return (self.cliff_data.is_cliff_left or
                   self.cliff_data.is_cliff_front_left or
                   self.cliff_data.is_cliff_right or
                   self.cliff_data.is_cliff_front_right)

    def get_cliff_direction(self) -> Optional[str]:
        """
        Get which cliff direction(s) are detected.
        
        Returns:
            Direction string or None if no cliff
        """
        with self.lock:
            if self.cliff_data.is_cliff_left:
                return "left"
            elif self.cliff_data.is_cliff_front_left:
                return "front_left"
            elif self.cliff_data.is_cliff_right:
                return "right"
            elif self.cliff_data.is_cliff_front_right:
                return "front_right"
        return None

    def is_bumper_contact(self) -> bool:
        """Check if any bumper is pressed."""
        with self.lock:
            return (self.bumper_data.is_left_pressed or
                   self.bumper_data.is_right_pressed)

    def get_bumper_direction(self) -> Optional[str]:
        """
        Get which bumper(s) are pressed.
        
        Returns:
            "left", "right", "both", or None
        """
        with self.lock:
            left = self.bumper_data.is_left_pressed
            right = self.bumper_data.is_right_pressed
            
            if left and right:
                return "both"
            elif left:
                return "left"
            elif right:
                return "right"
        return None

    def is_low_battery(self, threshold: float = 0.1) -> bool:
        """
        Check if battery is low (below 10% by default).
        
        Args:
            threshold: Battery ratio threshold [0.0, 1.0]
            
        Returns:
            True if battery below threshold
        """
        with self.lock:
            return self.battery_info.charge_ratio < threshold

    def is_critical_battery(self, threshold: float = 0.05) -> bool:
        """
        Check if battery is critical (below 5% by default).
        
        Args:
            threshold: Battery ratio threshold [0.0, 1.0]
            
        Returns:
            True if battery below critical threshold
        """
        with self.lock:
            return self.battery_info.charge_ratio < threshold

    def is_robot_standstill(self, duration_sec: float = 2.0) -> bool:
        """
        Check if robot has been stationary for specified duration.
        
        Args:
            duration_sec: Seconds of zero velocity to consider standstill
            
        Returns:
            True if robot velocity is zero and has been for duration
        """
        with self.lock:
            if self.current_velocity != 0.0:
                return False

            if self.last_cmd_vel_time is None:
                return False

            elapsed = (self.node.get_clock().now() - self.last_cmd_vel_time).nanoseconds / 1e9
            return elapsed > duration_sec

    def get_battery_info(self) -> BatteryInfo:
        """
        Get current battery information.
        
        Returns:
            BatteryInfo snapshot
        """
        with self.lock:
            return BatteryInfo(
                voltage=self.battery_info.voltage,
                current=self.battery_info.current,
                charge=self.battery_info.charge,
                capacity=self.battery_info.capacity,
                charge_ratio=self.battery_info.charge_ratio,
                temperature=self.battery_info.temperature,
                charging_state=self.battery_info.charging_state,
            )

    def clear_wheel_drop_flag(self) -> None:
        """Clear the wheel drop detected flag (after handling emergency)."""
        with self.lock:
            self.wheel_drop_detected = False
            self.wheel_drop_time = None
            self.logger.info("Wheel drop flag cleared")


class MotorController:
    """
    Controls all robot motors safely.
    
    Publishes to motor control topics with state tracking
    and safety checks.
    """

    def __init__(self, node: Node):
        """
        Initialize motor controller.
        
        Args:
            node: ROS2 node for creating publishers
        """
        self.node = node
        self.lock = threading.Lock()
        self.logger = node.get_logger()

        # Publishers
        self.main_brush_pub = node.create_publisher(
            MotorSetpoint, '/main_brush_motor', 10
        )
        self.vacuum_pub = node.create_publisher(
            MotorSetpoint, '/vacuum_motor', 10
        )
        self.side_brush_pub = node.create_publisher(
            MotorSetpoint, '/side_brush_motor', 10
        )
        self.velocity_pub = node.create_publisher(
            Twist, '/cmd_vel', 10
        )

        # State tracking
        self.main_brush_state = 0.0
        self.vacuum_state = 0.0
        self.side_brush_state = 0.0
        self.current_linear_vel = 0.0
        self.current_angular_vel = 0.0

        self.logger.info("MotorController initialized")

    def set_main_brush(self, duty_cycle: float) -> bool:
        """
        Set main brush motor duty cycle.
        
        Args:
            duty_cycle: Value in range [-1.0, 1.0]
            
        Returns:
            True if successful
        """
        duty_cycle = max(-1.0, min(1.0, duty_cycle))

        try:
            with self.lock:
                if abs(duty_cycle - self.main_brush_state) < 0.01:
                    return True  # No change needed

                msg = MotorSetpoint()
                msg.duty_cycle = duty_cycle
                self.main_brush_pub.publish(msg)
                self.main_brush_state = duty_cycle

                self.logger.debug(f"Main brush set to: {duty_cycle:.2f}")
                return True
        except Exception as e:
            self.logger.error(f"Error setting main brush: {e}")
            return False

    def set_vacuum(self, duty_cycle: float) -> bool:
        """
        Set vacuum motor duty cycle.
        
        Args:
            duty_cycle: Value in range [0.0, 1.0]
            
        Returns:
            True if successful
        """
        duty_cycle = max(0.0, min(1.0, duty_cycle))

        try:
            with self.lock:
                if abs(duty_cycle - self.vacuum_state) < 0.01:
                    return True  # No change needed

                msg = MotorSetpoint()
                msg.duty_cycle = duty_cycle
                self.vacuum_pub.publish(msg)
                self.vacuum_state = duty_cycle

                self.logger.debug(f"Vacuum set to: {duty_cycle:.2f}")
                return True
        except Exception as e:
            self.logger.error(f"Error setting vacuum: {e}")
            return False

    def set_side_brush(self, duty_cycle: float) -> bool:
        """
        Set side brush motor duty cycle.
        
        Args:
            duty_cycle: Value in range [-1.0, 1.0]
            
        Returns:
            True if successful
        """
        duty_cycle = max(-1.0, min(1.0, duty_cycle))

        try:
            with self.lock:
                if abs(duty_cycle - self.side_brush_state) < 0.01:
                    return True  # No change needed

                msg = MotorSetpoint()
                msg.duty_cycle = duty_cycle
                self.side_brush_pub.publish(msg)
                self.side_brush_state = duty_cycle

                self.logger.debug(f"Side brush set to: {duty_cycle:.2f}")
                return True
        except Exception as e:
            self.logger.error(f"Error setting side brush: {e}")
            return False

    def disable_all_motors(self) -> bool:
        """
        Disable all non-wheel motors (safety action).
        
        Returns:
            True if all operations successful
        """
        success = True
        self.logger.warn("DISABLING ALL MOTORS")

        success = success and self.set_main_brush(0.0)
        success = success and self.set_vacuum(0.0)
        success = success and self.set_side_brush(0.0)

        return success

    def emergency_stop(self) -> bool:
        """
        Emergency stop - disable everything.
        
        Returns:
            True if successful
        """
        self.logger.error("EMERGENCY STOP ACTIVATED")

        # Stop wheels
        try:
            msg = Twist()
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.velocity_pub.publish(msg)
        except Exception as e:
            self.logger.error(f"Error stopping wheels: {e}")
            return False

        # Disable all motors
        return self.disable_all_motors()

    def set_velocity(self, linear_x: float, angular_z: float) -> bool:
        """
        Set robot velocity.
        
        Args:
            linear_x: Linear velocity in m/s
            angular_z: Angular velocity in rad/s
            
        Returns:
            True if successful
        """
        try:
            with self.lock:
                msg = Twist()
                msg.linear.x = linear_x
                msg.angular.z = angular_z
                self.velocity_pub.publish(msg)
                self.current_linear_vel = linear_x
                self.current_angular_vel = angular_z
                return True
        except Exception as e:
            self.logger.error(f"Error setting velocity: {e}")
            return False

    def stop_wheels(self) -> bool:
        """Stop wheel movement."""
        return self.set_velocity(0.0, 0.0)

    def get_state(self) -> dict:
        """
        Get current motor state.
        
        Returns:
            Dictionary with all motor states
        """
        with self.lock:
            return {
                'main_brush': self.main_brush_state,
                'vacuum': self.vacuum_state,
                'side_brush': self.side_brush_state,
                'linear_velocity': self.current_linear_vel,
                'angular_velocity': self.current_angular_vel,
            }
