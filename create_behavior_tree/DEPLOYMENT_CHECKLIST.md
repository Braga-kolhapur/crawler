# 🚀 Deployment Checklist & Launch Guide

## Phase 1: Pre-Deployment Setup (Local Development)

### Step 1: Verify Package Structure
```bash
ls -la ~/Documents/ros2_ws/src/crawler/create_behavior_tree/
```

**Expected files:**
- ✅ `sensor_motor_manager.py` (core)
- ✅ `refactored_nodes.py` (core)
- ✅ `refactored_tree_builder.py` (core)
- ✅ `refactored_behavior_tree_node.py` (core)
- ✅ `package.xml` (config)
- ✅ `setup.py` (config)
- ✅ Documentation files (*.md, *.py)

### Step 2: Install Python Dependencies
```bash
pip install py_trees pyyaml
pip install rclpy create-msgs
```

**Verification:**
```python
python3 -c "import py_trees; print('py_trees OK')"
python3 -c "import rclpy; print('rclpy OK')"
python3 -c "import yaml; print('yaml OK')"
```

### Step 3: Build Package
```bash
cd ~/Documents/ros2_ws
colcon build --packages-select create_behavior_tree
```

**Expected output:**
```
Starting >>> create_behavior_tree
Finished <<< create_behavior_tree [x.xxs]
```

**If build fails:**
1. Check error message carefully
2. Reference [QUICKSTART.md](QUICKSTART.md) troubleshooting section
3. Verify all imports exist in .py files
4. Check package.xml dependencies

### Step 4: Source Environment
```bash
source ~/Documents/ros2_ws/install/setup.bash
```

### Step 5: Verify Entry Point
```bash
which refactored_behavior_tree_node
```

**Expected output:** Path to the executable

---

## Phase 2: Pre-Robot Testing (Simulation/Mock)

### Step 6: Test with Mock Sensors (Terminal 1)
```bash
# Start a minimal ROS2 environment
ros2 node list  # Should show nodes available
```

### Step 7: Test with Manual Topic Publishing (Terminal 2)
```bash
# Simulate right bumper pressed
ros2 topic pub --once /bumper create_msgs/Bumper \
  '{header: {frame_id: "base_link", stamp: now}, is_left_pressed: false, is_right_pressed: true}' \
  && sleep 0.5 && echo "Simulated right bumper"
```

**Expected behavior:**
- Robot should turn left
- Check `/cmd_vel` topic should show negative angular velocity

### Step 8: Test Motor Publishing (Terminal 2)
```bash
# Check what /cmd_vel publishes
ros2 topic echo /cmd_vel --max-msgs=5
```

**Expected output:**
```yaml
linear:
  x: 0.1
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: -0.2
```

### Step 9: Test Refactored Node in Isolation (Terminal 1)
```bash
# Start the refactored node
ros2 run create_behavior_tree refactored_behavior_tree_node \
  --ros-args -p tree_update_hz:=10.0
```

**Expected output:**
```
[INFO] [refactored_behavior_tree_node]: Initializing RefactoredBehaviorTreeNode
[INFO] [refactored_behavior_tree_node]: Subscriptions created successfully
[INFO] [refactored_behavior_tree_node]: Publications created successfully
[INFO] [refactored_behavior_tree_node]: Tree construction complete
```

**If you see errors:**
1. Check subscription/publication topics
2. Verify sensor_motor_manager.py imports
3. Check yaml configuration file path

### Step 10: Monitor Internal State (Terminal 2)
While node is running:
```bash
# In another terminal, you can see debug info
# The node prints periodically to verify it's working
```

---

## Phase 3: Hardware Deployment

### Step 11: Start Create2 Driver (Terminal 1)
```bash
# Launch the Create2 hardware interface
ros2 launch create_bringup create_2.launch.py
```

**Expected output:**
```
[INFO] create_driver: [INFO] Setting baudrate to 115200
[INFO] create_driver: [INFO] Serial connection established
[INFO] create_driver: [INFO] Create2 robot initialized
```

**If it fails:**
- Check USB connection to Create2
- Verify `/dev/ttyUSB*` permissions: `ls -l /dev/ttyUSB*`
- Check Create2 battery (minimum 3V)

### Step 12: Verify Sensors are Publishing (Terminal 2)
```bash
# Check all required sensor topics are active
ros2 topic list | grep -E 'bumper|cliff|wheel|battery'
```

**Expected topics:**
- ✅ `/bumper`
- ✅ `/cliff`
- ✅ `/wheeldrop`
- ✅ `/battery/charge_ratio`
- ✅ `/battery/charging_state`
- ✅ `/battery/voltage`

**If topics missing:**
- Wait 5 seconds for driver to fully initialize
- Check create_bringup is running: `ros2 node list | grep create`
- Restart create_bringup

### Step 13: Start Behavior Tree Node (Terminal 3)
```bash
ros2 run create_behavior_tree refactored_behavior_tree_node \
  --ros-args -p tree_update_hz:=10.0
```

**Expected output:**
```
[INFO] [refactored_behavior_tree_node]: Initializing RefactoredBehaviorTreeNode
[INFO] [refactored_behavior_tree_node]: Subscriptions created successfully
[INFO] [refactored_behavior_tree_tree]: Tree construction complete
[INFO] [refactored_behavior_tree_node]: Starting tree update loop
```

### Step 14: Monitor Robot Behavior (Terminal 4)

**Watch velocity commands:**
```bash
ros2 topic echo /cmd_vel
```

**Watch motor commands:**
```bash
ros2 topic echo /main_brush_motor
ros2 topic echo /vacuum_motor
```

**Monitor battery:**
```bash
ros2 topic echo /battery/charge_ratio
```

### Step 15: Test Each Sensor (Manual Testing)

**Bumper Test:**
1. Gently push front bumpers → Robot should turn left or right
2. Both bumpers → Robot should turn around
3. Verify `/cmd_vel` changes

**Cliff Test:**
1. Move robot to cliff edge → Robot should stop and back up
2. Verify `/cliff` topic echoes detection

**Wheel Drop Test:**
1. Carefully lift one wheel → Robot should emergency stop
2. Verify `/wheeldrop` publishes

**Battery Test:**
1. If battery low → Robot should reduce speed or stop
2. Publish mock battery: `ros2 topic pub /battery/charge_ratio std_msgs/Float32 '{data: 0.04}'`

**Charging Test:**
1. If on dock → Verify charging state
2. Check `/battery/charging_state` topic

### Step 16: Test Motor Control

**One by one, verify motors respond:**
```bash
# Vacuum motor (should spin with audible sound)
ros2 topic pub /vacuum_motor create_msgs/MotorSetpoint '{header: now, setpoint: 0.8}'
sleep 2
ros2 topic pub /vacuum_motor create_msgs/MotorSetpoint '{header: now, setpoint: 0.0}'

# Main brush (should spin)
ros2 topic pub /main_brush_motor create_msgs/MotorSetpoint '{header: now, setpoint: 0.8}'
sleep 2
ros2 topic pub /main_brush_motor create_msgs/MotorSetpoint '{header: now, setpoint: 0.0}'

# Side brush (should spin)
ros2 topic pub /side_brush_motor create_msgs/MotorSetpoint '{header: now, setpoint: 0.8}'
sleep 2
ros2 topic pub /side_brush_motor create_msgs/MotorSetpoint '{header: now, setpoint: 0.0}'
```

### Step 17: Safety Tests

**Emergency Stop Test:**
1. Robot moving → Kill behavior tree node (Ctrl+C)
2. Verify robot stops within 1 second

**Low Battery Test:**
1. Publish: `ros2 topic pub /battery/charge_ratio std_msgs/Float32 '{data: 0.08}'`
2. Robot should reduce speed significantly

**Critical Battery Test:**
1. Publish: `ros2 topic pub /battery/charge_ratio std_msgs/Float32 '{data: 0.03}'`
2. Robot should stop completely

**Standstill Test:**
1. Observe robot for 2+ seconds with zero velocity
2. Motors should automatically disable
3. Verify in `/main_brush_motor` and `/vacuum_motor` topics

---

## Phase 4: Validation & Fine-Tuning

### Step 18: Performance Monitoring

**Tree Update Rate:**
```bash
# Monitor how fast tree is updating
# Watch /cmd_vel topic - should get updates at configured rate
ros2 topic hz /cmd_vel
```

**Expected output:**
```
average rate: 10.00
        min: 100.0ms max: 100.0ms std dev: 0.00000ms
```

### Step 19: Sensor Latency Check

**Check response time:**
```bash
# Time from sensor input to motor output
# Should be < 100ms for responsive behavior
```

### Step 20: Battery Life Logging

Monitor battery during deployment:
```bash
# Record battery over time
ros2 run create_behavior_tree record_battery_data.py  # Optional: implement this
```

---

## Quick Deploy Scripts

### Fast Setup (Copy-Paste)
```bash
#!/bin/bash
cd ~/Documents/ros2_ws
source install/setup.bash
colcon build --packages-select create_behavior_tree
source install/setup.bash
echo "✅ Build complete! Ready to deploy."
```

### One-Terminal Demo (For testing)
```bash
#!/bin/bash
# Terminal 1: Everything in one go
cd ~/Documents/ros2_ws
source install/setup.bash

# Start Create driver
ros2 launch create_bringup create_2.launch.py &
sleep 3

# Start behavior tree
ros2 run create_behavior_tree refactored_behavior_tree_node
```

### Multi-Terminal Deploy (Recommended)
```bash
# Terminal 1: Hardware
cd ~/Documents/ros2_ws && source install/setup.bash
ros2 launch create_bringup create_2.launch.py

# Terminal 2: Behavior Tree
cd ~/Documents/ros2_ws && source install/setup.bash
ros2 run create_behavior_tree refactored_behavior_tree_node

# Terminal 3: Monitoring
cd ~/Documents/ros2_ws && source install/setup.bash
ros2 topic echo /cmd_vel
```

---

## Troubleshooting Deployment

### Issue: "refactored_behavior_tree_node not found"
**Solution:**
```bash
source install/setup.bash  # Re-source setup
which refactored_behavior_tree_node  # Verify
colcon build --packages-select create_behavior_tree --force-cmake-configure
```

### Issue: "Cannot import create_msgs"
**Solution:**
```bash
pip install create-msgs
# Or build from source:
colcon build --packages-select create_msgs
source install/setup.bash
```

### Issue: Robot not responding to commands
**Solution:**
1. Check USB connection: `lsusb | grep iRobot`
2. Check serial permissions: `ls -l /dev/ttyUSB0`
3. Verify driver is running: `ros2 node list | grep create_driver`
4. Check `/cmd_vel` is publishing: `ros2 topic echo /cmd_vel --max-msgs=1`

### Issue: Sensors not publishing
**Solution:**
1. Verify driver started: `ros2 topic list | grep -E "bumper|cliff|battery"`
2. Wait 5 seconds after driver start
3. Check driver logs: `ros2 node list` see if create_driver exists
4. Restart driver: Kill and relaunch create_bringup

### Issue: Motors not moving
**Solution:**
1. Check battery voltage: `ros2 topic echo /battery/voltage --max-msgs=1`
2. Check wheel drop status: `ros2 topic echo /wheeldrop --max-msgs=10`
3. Verify /cmd_vel is publishing: `ros2 topic echo /cmd_vel`
4. Try manual motor command (see Step 16)

### Issue: High CPU usage
**Solution:**
1. Reduce tree_update_hz: `--ros-args -p tree_update_hz:=5.0`
2. Check for circular subsciptions
3. Monitor with `top` command

---

## Rollback Procedure

If refactored system has issues:

```bash
# Switch to original system
ros2 run create_behavior_tree behavior_tree_node

# Then investigate:
# 1. Check logs: Check what step fails
# 2. Reference original ARCHITECTURE.py
# 3. Compare with refactored version
# 4. File issue / debug
```

---

## Success Criteria

✅ **Deployment is successful when:**
- [ ] Build completes without errors
- [ ] Nodes start without warnings (some info logs OK)
- [ ] All 6+ sensors publish data
- [ ] Robot responds to bumper contact (turns away)
- [ ] Robot stops at cliff edge
- [ ] Robot stops on wheel drop
- [ ] Motors respond to commands
- [ ] Battery is monitored
- [ ] Emergency stop works (kill node → robot stops < 1s)

---

## Performance Targets

| Metric | Target | Actual |
|--------|--------|--------|
| Tree Update Rate | 10 Hz | _____ |
| Bumper Response Time | < 100ms | _____ |
| Battery Sample Rate | 1 Hz | _____ |
| Motor Command Rate | Variable | _____ |
| CPU Usage | < 5% | _____ |
| Memory Usage | < 50MB | _____ |

---

## Post-Deployment Checklist

- [ ] Document actual performance metrics (see table above)
- [ ] Record any sensor calibrations needed
- [ ] Document any parameter tuning done
- [ ] Create robot-specific launch file if needed
- [ ] Test in multiple environments (carpet, hard floor, obstacles)
- [ ] Verify battery drain rate (should be lowest with standstill disable)
- [ ] Document any custom modifications made
- [ ] Set up monitoring/logging for production
- [ ] Create backup of working configuration

---

**Ready to Deploy!** 🚀

For detailed information while deploying, reference [REFACTORING_GUIDE.md](REFACTORING_GUIDE.md) or [QUICKSTART.md](QUICKSTART.md).

**Last Updated:** February 16, 2026
**Status:** Production Deployment Ready
