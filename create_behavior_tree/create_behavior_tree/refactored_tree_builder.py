"""
Refactored Behavior Tree Builder using SensorManager and MotorController

This is the improved version that uses abstracted sensor and motor management.
"""

import py_trees
from py_trees import common, composites
from create_behavior_tree.refactored_nodes import (
    CheckWheelDrop,
    CheckCliffDetection,
    CheckBumperContact,
    CheckLeftBumper,
    CheckRightBumper,
    CheckBothBumpers,
    CheckLowBattery,
    CheckCriticalBattery,
    CheckRobotStandstill,
    DisableAllMotors,
    EmergencyStop,
    StopWheels,
    MoveForwardAction,
    TurnRightAction,
    TurnLeftAction,
    Turn180Action,
    WaitDuration,
    ReduceSpeedAction,
    BackupAction,
)


def build_refactored_behavior_tree(sensor_mgr, motor_ctrl):
    """
    Build the refactored behavior tree using sensor/motor abstractions.

    Tree Structure (Priority Order):
    
    Root (Parallel)
    ├── Critical Safety Sequence
    │   ├── Wheel Drop Check → Emergency Stop
    │   └── Critical Battery Check → Return to Dock (placeholder)
    └── Main Navigation Selector
        ├── Low Battery Sequence → Reduced Speed Movement
        ├── Both Bumpers Sequence → 180° Turn
        ├── Left Bumper Sequence → Turn Right
        ├── Right Bumper Sequence → Turn Left
        ├── Cliff Detection Sequence → Stop + Backup
        ├── Standstill + Motors On → Disable Vacuum/Brush
        └── Default Forward Movement

    Args:
        sensor_mgr: SensorManager instance
        motor_ctrl: MotorController instance

    Returns:
        Root behavior tree node
    """

    # ========== CRITICAL SAFETY: WHEEL DROP ==========
    wheel_drop_check = CheckWheelDrop("Wheel Drop?", sensor_mgr)
    emergency_stop = EmergencyStop("EMERGENCY STOP", motor_ctrl)

    wheel_drop_sequence = composites.Sequence(
        name="Wheel Drop Emergency",
        memory=False,
        children=[wheel_drop_check, emergency_stop]
    )

    # ========== CRITICAL SAFETY: CRITICAL BATTERY ==========
    critical_battery_check = CheckCriticalBattery(
        "Battery <5%?", sensor_mgr, threshold=0.05
    )
    stop_for_critical_battery = StopWheels("Stop", motor_ctrl)

    critical_battery_sequence = composites.Sequence(
        name="Critical Battery Stop",
        memory=False,
        children=[critical_battery_check, stop_for_critical_battery]
    )

    # ========== Safety Parallel: Highest Priority ==========
    safety_parallel = composites.Parallel(
        name="Critical Safety",
        policy=composites.ParallelPolicy.SuccessOnAll(synchronise=False),
        children=[wheel_drop_sequence, critical_battery_sequence]
    )

    # ========== LOW BATTERY: REDUCED SPEED MODE ==========
    low_battery_check = CheckLowBattery("Battery <10%?", sensor_mgr, threshold=0.1)
    disable_motors_low_batt = DisableAllMotors("Disable Motors", motor_ctrl)
    move_reduced = ReduceSpeedAction("Move Reduced Speed", motor_ctrl, 0.15)

    low_battery_sequence = composites.Sequence(
        name="Low Battery Mode",
        memory=False,
        children=[low_battery_check, disable_motors_low_batt, move_reduced]
    )

    # ========== BOTH BUMPERS: 180 DEGREE TURN ==========
    both_bumpers_check = CheckBothBumpers("Both Bumpers?", sensor_mgr)
    stop_bumpers = StopWheels("Stop", motor_ctrl)
    turn_180 = Turn180Action("Turn 180°", motor_ctrl, duration=3.0)
    wait_after_turn = WaitDuration("Wait", duration=0.5)

    both_bumpers_sequence = composites.Sequence(
        name="Both Bumpers 180° Turn",
        memory=False,
        children=[both_bumpers_check, stop_bumpers, turn_180, wait_after_turn]
    )

    # ========== LEFT BUMPER: TURN RIGHT ==========
    left_bumper_check = CheckLeftBumper("Left Bumper?", sensor_mgr)
    turn_right = TurnRightAction("Turn Right", motor_ctrl, 0.15, 0.4)
    wait_after_left = WaitDuration("Wait", duration=0.5)

    left_bumper_sequence = composites.Sequence(
        name="Left Bumper Turn Right",
        memory=False,
        children=[left_bumper_check, turn_right, wait_after_left]
    )

    # ========== RIGHT BUMPER: TURN LEFT ==========
    right_bumper_check = CheckRightBumper("Right Bumper?", sensor_mgr)
    turn_left = TurnLeftAction("Turn Left", motor_ctrl, 0.15, 0.4)
    wait_after_right = WaitDuration("Wait", duration=0.5)

    right_bumper_sequence = composites.Sequence(
        name="Right Bumper Turn Left",
        memory=False,
        children=[right_bumper_check, turn_left, wait_after_right]
    )

    # ========== CLIFF DETECTION: STOP AND BACKUP ==========
    cliff_check = CheckCliffDetection("Cliff Detected?", sensor_mgr)
    stop_cliff = StopWheels("Stop", motor_ctrl)
    wait_before_backup = WaitDuration("Wait", duration=0.5)
    backup = BackupAction("Backup", motor_ctrl, backup_speed=-0.1, duration=1.0)

    cliff_sequence = composites.Sequence(
        name="Cliff Detection Stop+Backup",
        memory=False,
        children=[cliff_check, stop_cliff, wait_before_backup, backup]
    )

    # ========== STANDSTILL: DISABLE VACUUM/BRUSH MOTORS ==========
    standstill_check = CheckRobotStandstill("Standstill >2s?", sensor_mgr, duration=2.0)
    disable_motors_standstill = DisableAllMotors("Disable Motors", motor_ctrl)

    standstill_sequence = composites.Sequence(
        name="Standstill Power Save",
        memory=False,
        children=[standstill_check, disable_motors_standstill]
    )

    # ========== DEFAULT: FORWARD MOVEMENT ==========
    move_forward = MoveForwardAction("Move Forward", motor_ctrl, speed=0.2)

    # ========== MAIN NAVIGATION SELECTOR ==========
    # Try in priority order: low battery, both bumpers, left, right, cliff, standstill, forward
    navigation_selector = composites.Selector(
        name="Navigation",
        memory=False,
        children=[
            low_battery_sequence,
            both_bumpers_sequence,
            left_bumper_sequence,
            right_bumper_sequence,
            cliff_sequence,
            standstill_sequence,
            move_forward
        ]
    )

    # ========== ROOT: PARALLEL EXECUTION ==========
    root = composites.Parallel(
        name="Robot Behavior Tree Root",
        policy=composites.ParallelPolicy.SuccessOnAll(synchronise=False),
        children=[safety_parallel, navigation_selector]
    )

    return root


def create_and_update_function(root):
    """
    Create update function for periodic tree execution.

    Args:
        root: Root node of the behavior tree

    Returns:
        Function to call each update cycle
    """

    def update_tree():
        """Update function to be called periodically."""
        root.tick_once()
        return root.status

    return update_tree
