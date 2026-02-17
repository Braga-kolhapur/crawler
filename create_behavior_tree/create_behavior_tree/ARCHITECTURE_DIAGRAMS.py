#!/usr/bin/env python3
"""
Visual Architecture Diagrams for Refactored Behavior Tree

This file contains ASCII diagrams and Mermaid flowcharts showing the
complete architecture of the refactored system.
"""

SENSOR_FLOW_DIAGRAM = """
╔════════════════════════════════════════════════════════════════════════════╗
║                    SENSOR DATA FLOW ARCHITECTURE                           ║
╚════════════════════════════════════════════════════════════════════════════╝

                           Create2 Robot Hardware
                                    │
                    ┌───────────────┼───────────────┐
                    │               │               │
              ┌─────▼─────┐    ┌────▼────┐    ┌───▼────┐
              │  Bumpers  │    │  Cliffs │    │Battery │
              │  (2x+6x)  │    │  (4x)   │    │(6 msg) │
              └─────┬─────┘    └────┬────┘    └───┬────┘
                    │               │               │
        ┌───────────┴───────────┬───┴──────────────┴──────┐
        │                       │                         │
   ROS2 Topics            ROS2 Topics               ROS2 Topics
  /bumper msg         /cliff msg                 /battery/* msgs
  /wheeldrop          (periodic)                 /cmd_vel feedback
  (events)
        │                       │                         │
        └───────────────────────┼─────────────────────────┘
                                │
                    ┌───────────▼────────────┐
                    │   SensorManager        │
                    │                        │
                    │  - Thread-safe         │
                    │  - State caching       │
                    │  - Query methods       │
                    │  - Debouncing logic    │
                    │  - Snapshot support    │
                    └───────────┬────────────┘
                                │
            ┌───────────────────┼───────────────────┐
            │                   │                   │
      ┌─────▼──────┐      ┌────▼────┐       ┌────▼───────┐
      │is_*()      │      │get_*()  │       │check_*()   │
      │methods     │      │methods  │       │methods     │
      │(bool)      │      │(info)   │       │(bool)      │
      └─────┬──────┘      └────┬────┘       └────┬───────┘
            │                   │                 │
            └───────────────────┼─────────────────┘
                                │
                    ┌───────────▼────────────┐
                    │  Behavior Tree Nodes   │
                    │                        │
                    │  CheckWheelDrop()      │
                    │  CheckBattery()        │
                    │  CheckBumper()         │
                    │  CheckCliff()          │
                    │  ... (13 total)        │
                    └────────────────────────┘
"""

MOTOR_CONTROL_DIAGRAM = """
╔════════════════════════════════════════════════════════════════════════════╗
║                     MOTOR CONTROL FLOW ARCHITECTURE                        ║
╚════════════════════════════════════════════════════════════════════════════╝

            ┌──────────────────────────────────────┐
            │   Behavior Tree Action Nodes         │
            │                                      │
            │  MoveForwardAction()                 │
            │  TurnLeftAction()                    │
            │  TurnRightAction()                   │
            │  DisableAllMotors()                  │
            │  EmergencyStop()                     │
            │  ... (10 total)                      │
            └──────────────────┬───────────────────┘
                               │
                    ┌──────────▼──────────┐
                    │  MotorController    │
                    │                     │
                    │  - State tracking   │
                    │  - Safety checks    │
                    │  - Deduplication    │
                    │  - Logging          │
                    │  - Emergency logic  │
                    └──────────┬──────────┘
                               │
            ┌──────────────────┼──────────────────┐
            │                  │                  │
      ┌─────▼──────┐    ┌─────▼──────┐    ┌─────▼──────┐
      │set_*()     │    │disable_*() │    │emergency() │
      │methods     │    │methods     │    │methods     │
      │(publish)   │    │(publish)   │    │(publish)   │
      └─────┬──────┘    └─────┬──────┘    └─────┬──────┘
            │                  │                  │
            └──────────────────┼──────────────────┘
                               │
        ┌──────────────────────┼──────────────────────┐
        │                      │                      │
   ┌────▼────┐          ┌─────▼────┐         ┌──────▼────┐
   │/cmd_vel │          │Brush Mtrs │         │Vacuum Mtr │
   │(Twist)  │          │(SetPoint) │         │(SetPoint) │
   └────┬────┘          └─────┬────┘         └──────┬────┘
        │                      │                     │
   Wheels on Robot    Main+Side Brush         Vacuum Motor
   
   ┌────────────────────────────────────────────────────────┐
   │              SAFETY CONSTRAINTS ON MOTOR CONTROL:      │
   │                                                        │
   │ ✓ Motor 1 (vacuum) OFF if:                            │
   │   - Wheel drop detected                               │
   │   - Robot velocity = 0 for > 2 seconds                │
   │   - Battery < 5% (critical)                           │
   │   - Battery < 10% (low power mode)                    │
   │                                                        │
   │ ✓ Motor 2 (main brush) OFF if:                        │
   │   - Wheel drop detected                               │
   │   - Robot velocity = 0 for > 2 seconds                │
   │   - Battery < 5% (critical)                           │
   │   - Battery < 10% (low power mode)                    │
   │                                                        │
   │ ✓ All motors (including wheels) OFF if:               │
   │   - Wheel drop detected (EMERGENCY)                   │
   │                                                        │
   │ ✓ Speed reduced if:                                   │
   │   - Battery < 10% (low power mode)                    │
   └────────────────────────────────────────────────────────┘
"""

BEHAVIOR_TREE_STRUCTURE = """
╔════════════════════════════════════════════════════════════════════════════╗
║                     BEHAVIOR TREE STRUCTURE & PRIORITY                    ║
╚════════════════════════════════════════════════════════════════════════════╝

                          Root Parallel Node
                                 │
                    ┌────────────┴────────────┐
                    │                         │
    ┌───────────────▼────────────┐   ┌───────▼──────────────────┐
    │ Critical Safety Parallel   │   │ Navigation Selector      │
    │ (Highest Priority)          │   │ (Main Behavior)          │
    │                             │   │                          │
    │ ┌─ Wheel Drop Sequence     │   │ ┌─ Low Battery Seq      │
    │ │  └─ Stop & Disable       │   │ │  └─ Reduced Speed      │
    │ │                           │   │ │                        │
    │ └─ Critical Batt Sequence  │   │ ├─ Both Bumpers Seq     │
    │    └─ Stop Completely      │   │ │  └─ 180° Turn          │
    │                             │   │ │                        │
    └─────────────────────────────┘   │ ├─ Left Bumper Seq      │
                                      │ │  └─ Turn Right         │
                                      │ │                        │
                                      │ ├─ Right Bumper Seq     │
                                      │ │  └─ Turn Left          │
                                      │ │                        │
                                      │ ├─ Cliff Detect Seq     │
                                      │ │  └─ Stop+Backup+Turn   │
                                      │ │                        │
                                      │ ├─ Standstill Seq       │
                                      │ │  └─ Disable Motors     │
                                      │ │                        │
                                      │ └─ Forward Movement     │
                                      │    (Default/Fallback)   │
                                      │                         │
                                      └─────────────────────────┘

PRIORITY EXECUTION ORDER:
═══════════════════════════════════════════════════════════════

1️⃣  CRITICAL SAFETY (runs in Parallel - highest interrupt)
    └─ Wheel Drop → Emergency Stop (immediate)
    └─ Battery Critical (< 5%) → Stop completely

2️⃣  NAVIGATION SELECTOR (tries in order, uses FIRST match)
    ├─ Low Battery (< 10%) → Move slow, disable motors
    ├─ Both Bumpers → Stuck! 180° turn + recover
    ├─ Left Bumper → Obstacle left → Turn right
    ├─ Right Bumper → Obstacle right → Turn left
    ├─ Cliff Detected → Danger! Stop + backup + turn
    ├─ Standstill (v=0 >2s) → Save power → Disable motors
    └─ Default → Move forward safely

Note: Later entries are NEVER tried if earlier one succeeds.
      This ensures safe, prioritized behavior execution.
"""

SAFETY_PRIORITY_CHART = """
╔════════════════════════════════════════════════════════════════════════════╗
║                      SAFETY PRIORITY HIERARCHY & ACTIONS                  ║
╚════════════════════════════════════════════════════════════════════════════╝

┏━━━━━━┳━━━━━━━━━━━━━━━━━┳━━━━━━━━━━━━━━━━━━━━━┳━━━━━━━━━━━━━━━━━━━━━━━━━┓
┃ LVL  ┃ CONDITION        ┃ PRIMARY ACTIONS     ┃ SECONDARY ACTIONS      ┃
┡━━━━━━╇━━━━━━━━━━━━━━━━━╇━━━━━━━━━━━━━━━━━━━━━╇━━━━━━━━━━━━━━━━━━━━━━━━━┩
│ 🚨   │ Wheel Drop      │ ⛔ STOP (WHEELS)    │ ⛔ DISABLE ALL MOTORS  │
│ 1️⃣   │ DETECTED        │ ⛔ (Immediate)       │ ⛔ (EMERGENCY)          │
│      │                 │ ❌ VACUUM OFF       │ 🚨 LOG CRITICAL        │
│      │                 │ ❌ BRUSH OFF        │ 🚨 MANUAL RECOVERY     │
├──────┼─────────────────┼─────────────────────┼────────────────────────┤
│ ⛔   │ Battery < 5%    │ 🛑 STOP (ALL)       │ 🚨 LOG CRITICAL        │
│ 2️⃣   │ CRITICAL        │ (Complete stop)     │ 🏠 RETURN TO DOCK      │
│      │                 │ ❌ VACUUM OFF       │                        │
│      │                 │ ❌ BRUSH OFF        │                        │
├──────┼─────────────────┼─────────────────────┼────────────────────────┤
│ ⚠️   │ Cliff Detected  │ 🛑 STOP (WHEELS)    │ ⏮️  BACKUP 1 SEC       │
│ 3️⃣   │                 │ (Immediate)         │ 🔄 TURN AWAY           │
│      │ OR              │ ❌ VACUUM OFF       │ ⏸️  WAIT 0.5 SEC       │
│      │ Battery < 10%   │ ❌ BRUSH OFF        │ ✅ RESUME NAV          │
│      │ LOW             │ 🚇 Reduced speed    │                        │
├──────┼─────────────────┼─────────────────────┼────────────────────────┤
│ ⚡   │ Both Bumpers    │ 🛑 STOP (WHEELS)    │ ⏸️  WAIT 0.5 SEC       │
│ 4️⃣   │ PRESSED         │ 🔄 ROTATE 180°      │ ✅ RESUME NAV          │
│      │                 │ (Stuck recovery)    │                        │
│      │ OR              │ ❌ VACUUM OFF       │                        │
│      │ One Bumper      │ ❌ BRUSH OFF        │                        │
├──────┼─────────────────┼─────────────────────┼────────────────────────┤
│ 🟢   │ Standstill      │ ❌ VACUUM OFF       │ ✅ MAINTAIN POSITION   │
│ 5️⃣   │ v=0 > 2 sec     │ ❌ BRUSH OFF        │ 💾 SAVE POWER          │
│      │                 │ (Power saving)      │ ⏳ RESUME WHEN MOVING  │
├──────┼─────────────────┼─────────────────────┼────────────────────────┤
│ 🟢   │ No Hazards      │ ✅ MOVE FORWARD     │ 🧭 Normal Navigation   │
│ 6️⃣   │ (Default)       │ [0.2 m/s]           │ 🔍 Explore Environment│
│      │                 │ ✅ VACUUM ON (opt)  │                        │
│      │                 │ ✅ BRUSH ON (opt)   │                        │
└──────┴─────────────────┴─────────────────────┴────────────────────────┘

LOGIC FLOW:
══════════════════════════════════════════════════════════════════════════

Tree evaluates in order each cycle (10 Hz ≈ every 100ms):

  1. Check CRITICAL SAFETY in PARALLEL:
     └─ If wheel drop detected → EMERGENCY STOP (interrupt everything)
     └─ If battery < 5% → Stop completely
     
  2. If not critical, try NAVIGATION SELECTOR (first match wins):
     └─ If low battery → Proceed at reduced speed
     └─ Else if both bumpers → 180° turn
     └─ Else if left bumper → Turn right
     └─ Else if right bumper → Turn left
     └─ Else if cliff detected → Stop + backup
     └─ Else if standstill → Disable motors
     └─ Else → Move forward (default)

MOTOR STATE MACHINE:
═══════════════════════════════════════════════════════════════════════════

┌────────────────────────────┐
│  NORMAL OPERATION          │
│  • Vacuum: ON (if active)  │
│  • Brush: ON (if active)   │
│  • Wheels: Forward/Turn    │
└────────────┬───────────────┘
             │
        ┌────┴──────────────────────────────┐
        │ Trigger: If any condition met:   │
        │ • Wheel drop                      │
        │ • Battery < 10%                   │
        │ • Velocity = 0 for > 2s           │
        │                                   │
        │ Action: Disable vacuum + brush    │
        └────┬──────────────────────────────┘
             │
┌────────────▼───────────────┐
│  REDUCED OPERATION         │
│  • Vacuum: OFF             │
│  • Brush: OFF              │
│  • Wheels: Still working   │
└────────────┬───────────────┘
             │
        ┌────┴──────────────────────────────┐
        │ Trigger: If wheel drop detected   │
        │                                   │
        │ Action: EMERGENCY STOP            │
        └────┬──────────────────────────────┘
             │
┌────────────▼───────────────┐
│  EMERGENCY STOP            │
│  • ALL Motors: OFF ❌      │
│  • Wheels: STOP ❌         │
│  • Vacuum: OFF ❌          │
│  • Brush: OFF ❌           │
│ Manual recovery required   │
└────────────────────────────┘
"""

SENSOR_DIAGRAMS = {
    'sensor_flow': SENSOR_FLOW_DIAGRAM,
    'motor_control': MOTOR_CONTROL_DIAGRAM,
    'behavior_tree': BEHAVIOR_TREE_STRUCTURE,
    'safety_priority': SAFETY_PRIORITY_CHART,
}

def print_all_diagrams():
    """Print all architecture diagrams."""
    for name, diagram in SENSOR_DIAGRAMS.items():
        print(diagram)
        print("\n" + "=" * 80 + "\n")

if __name__ == "__main__":
    print_all_diagrams()
