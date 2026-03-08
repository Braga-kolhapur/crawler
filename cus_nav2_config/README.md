# cus_nav2_config

Custom ROS2 navigation configuration package for the crawler robot (Create 2 + RPLidar C1).

## Package contents

```
cus_nav2_config/
├── launch/
│   ├── auto_mode.launch.py          # Full autonomy stack (map server + AMCL + nav2 + coverage)
│   ├── localization.launch.py       # AMCL localization only
│   └── pure_pur.launch.py           # Pure pursuit controller
├── maps/
│   ├── map123.pgm / map123.yaml     # Default map
│   └── coverage_plan.json           # Coverage plan for the default map
├── params/
│   ├── localization.yaml            # AMCL + map_server parameters
│   ├── pure_pus.yaml                # Pure pursuit parameters
│   └── mapper_params_online_async.yaml  # SLAM Toolbox mapping parameters
├── paths/
│   └── coverage_plan.json           # Coverage waypoint plan
└── scripts/
    ├── ros2_plan_publisher          # Publishes coverage plan waypoints
    ├── coverage_path_follower       # Follows coverage path via nav2
    ├── odom_pose_publisher          # Republishes odom as human-readable string
    └── odom_test                    # Drives robot fixed distance / angle for testing
```

---

## Build

```bash
cd ~/ros2_ws
colcon build --packages-select cus_nav2_config
source install/setup.bash
```

---

## Nodes

### odom_pose_publisher

Subscribes to `/odom` and publishes position (x, y, z) and orientation (roll, pitch, yaw) in degrees as a plain string.

| Direction | Topic       | Type                | Description                        |
|-----------|-------------|---------------------|------------------------------------|
| Sub       | `/odom`     | `nav_msgs/Odometry` | Robot odometry                     |
| Pub       | `/odom_pose`| `std_msgs/String`   | Position + angles in degrees       |

**Run:**
```bash
ros2 run cus_nav2_config odom_pose_publisher
```

**Monitor output:**
```bash
ros2 topic echo /odom_pose
```

**Example output:**
```
data: pos: x=1.234  y=0.567  z=0.000 | rot(deg): roll=0.0  pitch=0.0  yaw=45.3
```

---

### odom_test

Drives the robot a fixed distance or angle using `/odom` feedback to stop precisely.
Only one test runs at a time — a second trigger is ignored until the current test finishes.

| Direction | Topic            | Type                    | Description                                      |
|-----------|------------------|-------------------------|--------------------------------------------------|
| Sub       | `/odom`          | `nav_msgs/Odometry`     | Position feedback used to detect completion      |
| Sub       | `/test_distance` | `std_msgs/Int32`        | Linear velocity (m/s) — triggers 1 m forward run |
| Sub       | `/test_angle`    | `std_msgs/Int32`        | Angular velocity (deg/s) — triggers 90° turn     |
| Pub       | `/cmd_vel`       | `geometry_msgs/Twist`   | Velocity commands (non-stamped)                  |

**Run:**
```bash
ros2 run cus_nav2_config odom_test
```

**Trigger distance test** — drive 1 metre forward at 0.2 m/s:
```bash
ros2 topic pub --once /test_distance std_msgs/msg/Int32 "data: 0"
```
> Replace `0` with the desired integer velocity in m/s (e.g. `1` for 1 m/s).

**Trigger angle test** — turn 90° CCW at 20 deg/s:
```bash
ros2 topic pub --once /test_angle std_msgs/msg/Int32 "data: 20"
```
> Positive value = counter-clockwise (CCW). Negative value = clockwise (CW).
> Value is angular speed in deg/s (e.g. `20`, `-30`).

**Monitor while running:**
```bash
ros2 topic echo /cmd_vel
ros2 topic echo /odom_pose
```

---

## Launch files

### auto_mode.launch.py

Full autonomous coverage stack — map server, AMCL, controller, plan publisher, path follower, and static TF.

```bash
ros2 launch cus_nav2_config auto_mode.launch.py
```

| Argument         | Default        | Description                               |
|------------------|----------------|-------------------------------------------|
| `map`            | *(required)*   | Path to map `.yaml` file                  |
| `use_sim_time`   | `false`        | Set `true` for Gazebo simulation          |
| `stamped_cmd_vel`| `false`        | Set `true` to publish `TwistStamped`      |
| `plan_file`      | *(optional)*   | Path to coverage plan `.json`             |

**Example:**
```bash
ros2 launch cus_nav2_config auto_mode.launch.py \
  map:=$(ros2 pkg prefix cus_nav2_config)/share/cus_nav2_config/maps/map123.yaml
```

### localization.launch.py

AMCL localization against a saved map.

```bash
ros2 launch cus_nav2_config localization.launch.py \
  map:=/path/to/map.yaml
```

---

## SLAM mapping parameters

Custom SLAM Toolbox parameters are in [params/mapper_params_online_async.yaml](params/mapper_params_online_async.yaml).

Key values tuned for RPLidar C1 + Create 2:

| Parameter                  | Value  | Notes                          |
|----------------------------|--------|--------------------------------|
| `max_laser_range`          | 12.0 m | RPLidar C1 max range           |
| `resolution`               | 0.05 m | Map grid resolution            |
| `minimum_travel_distance`  | 0.2 m  | Min move before new scan saved |
| `minimum_travel_heading`   | 0.2 rad| Min rotation before new scan   |
| `do_loop_closing`          | true   | Enable loop closure            |

Run SLAM manually with custom params:
```bash
ros2 launch slam_toolbox online_async_launch.py \
  slam_params_file:=$(ros2 pkg prefix cus_nav2_config)/share/cus_nav2_config/params/mapper_params_online_async.yaml
```

Save map after mapping:
```bash
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map
```
