# BUET Mars Rover Team — Interplanetar 2026 Recruitment Assignment

## Multi-Modal Quadrotor Control System with TurtleBot3 Swarm

### System Requirements
- **OS**: Ubuntu 24.04 LTS
- **ROS 2**: Jazzy Jalisco
- **Gazebo**: Harmonic
- **Python**: 3.12+

### Features & Marks Distribution

| Component | Points | Description |
|-----------|--------|-------------|
| Voice Control | 30% | Speech → `geometry_msgs/Twist` (Forward/Back/Left/Right/Up/Down/Stop) |
| Telemetry Dashboard | 20% | PyQt5 GUI with position, orientation, velocity, battery, camera |
| Dual-Mode Switching | 25% | Seamless Manual ↔ Voice mode via GUI buttons + service |
| Swarm Setup (Bonus) | 10% | Spawn N TurtleBot3s in unique namespaces |
| TF2 Tracking (Bonus) | 15% | Leader-follower using TF2 transforms instead of raw odometry |

---

### Architecture

```
┌─────────────┐     ┌─────────────┐
│  Microphone │     │  Dashboard  │
└──────┬──────┘     │  (PyQt5)    │
       │            └──────┬──────┘
       ▼                   │
┌──────────────┐           ▼
│  Voice Node  │    ┌─────────────┐
│  (Twist)     │───▶│  Selector   │◀──┐
└──────────────┘    │   Node      │   │
                    └──────┬──────┘   │
┌──────────────┐           │          │
│ Teleop Bridge│───────────┘          │
│(keyboard/GUI)│    Manual / Voice    │
└──────────────┘                      │
                                      │
                    ┌─────────────────┘
                    ▼
            ┌─────────────┐
            │ /model/X3/  │
            │   cmd_vel   │
            └──────┬──────┘
                   ▼
            ┌─────────────┐
            │  Quadcopter │
            │   (Gazebo)  │
            └──────┬──────┘
                   │
        ┌──────────┴──────────┐
        ▼                     ▼
┌──────────────┐      ┌──────────────┐
│ /model/X3/   │      │ /model/X3/   │
│  odometry    │      │   pose       │
└──────┬───────┘      └──────┬───────┘
       │                     │
       ▼                     ▼
┌──────────────┐      ┌──────────────┐
│ Follow Node  │      │ TF Broadcast │
│ (odometry)   │      │   (poses)    │
└──────┬───────┘      └──────┬───────┘
       │                     │
       │              ┌──────┴──────┐
       │              ▼             ▼
       │       ┌─────────┐   ┌──────────┐
       │       │ TF Tree │   │ TF Follow│
       │       │ world→X3│   │  Node    │
       │       │ world→tb3│  │ (bonus)  │
       │       └─────────┘   └────┬─────┘
       │                          │
       └────────────┬─────────────┘
                    ▼
            ┌─────────────┐
            │/model/tb3/  │
            │  cmd_vel    │
            └──────┬──────┘
                   ▼
            ┌─────────────┐
            │ TurtleBot3  │
            │   (Gazebo)  │
            └─────────────┘
```

---

### Building

```bash
cd ~/fuad_ws
colcon build --symlink-install
source install/setup.bash
```

### Running

#### Main Launch (1 Quadcopter + 1 TurtleBot3)
```bash
ros2 launch integration_pkg all.launch.py
```

#### Swarm Launch (1 Quadcopter + N TurtleBot3s)
```bash
ros2 launch integration_pkg swarm.launch.py num_tb3:=3
```

#### Individual Nodes
```bash
# Dashboard GUI
ros2 run integration_pkg dashboard_node

# Voice controller
ros2 run integration_pkg voice_node

# Manual keyboard control
ros2 run integration_pkg teleop_bridge

# Odometry-based follower
ros2 run integration_pkg follow_node

# TF2-based follower (bonus)
ros2 run integration_pkg tf_follow_node

# TF broadcaster (required for TF2 bonus)
ros2 run integration_pkg tf_broadcaster
```

---

### Docker

```bash
docker-compose up --build
```

For GUI support (Linux):
```bash
xhost +local:docker
docker-compose up
```

---

### Voice Commands

| Command  | Twist Action       |
|----------|-------------------|
| forward  | linear.x = +1.0   |
| backward | linear.x = -1.0   |
| left     | angular.z = +1.0  |
| right    | angular.z = -1.0  |
| up       | linear.z = +1.0   |
| down     | linear.z = -1.0   |
| stop     | all zeros         |

---

### ROS 2 Topics

| Topic | Type | Direction |
|-------|------|-----------|
| `/model/X3/cmd_vel` | `geometry_msgs/Twist` | ROS → Gazebo |
| `/model/X3/odometry` | `nav_msgs/Odometry` | Gazebo → ROS |
| `/model/X3/pose` | `geometry_msgs/PoseStamped` | Gazebo → ROS |
| `/model/turtlebot3/cmd_vel` | `geometry_msgs/Twist` | ROS → Gazebo |
| `/model/turtlebot3/odometry` | `nav_msgs/Odometry` | Gazebo → ROS |
| `/model/turtlebot3/scan` | `sensor_msgs/LaserScan` | Gazebo → ROS |
| `/manual_cmd` | `geometry_msgs/Twist` | Dashboard/Teleop → Selector |
| `/voice_cmd` | `geometry_msgs/Twist` | Voice → Selector |
| `/control/mode` | `std_msgs/Bool` | Selector → Dashboard |
| `/drone/battery` | `sensor_msgs/BatteryState` | BatterySim → Dashboard |

---

### Important Notes

1. **Quadcopter Model**: The original quadcopter packages (`quad_control`, `quad_gazebo`, etc.) are **ROS 1 (catkin)** and incompatible with ROS 2 Jazzy. This workspace includes a new **Gazebo Harmonic-compatible SDF model** (`quad_gazebo/models/quadcopter/model.sdf`) using the built-in `MulticopterVelocityControl` plugin.

2. **Topic Verification**: If topics don't match, run:
   ```bash
   gz topic -l                    # List Gazebo topics
   ros2 topic list                # List ROS topics
   ros2 topic echo /model/X3/...  # Inspect messages
   ```

3. **Voice Recognition**: Requires a microphone. Inside Docker, pass `--device /dev/snd` if needed. The node gracefully degrades to a no-op if `speech_recognition` is not installed.

4. **TF2 Bonus**: The `tf_broadcaster` node **must** be running alongside `tf_follow_node`. It bridges Gazebo poses into TF transforms.

---

### Package Structure

```
fuad_ws/
├── src/
│   ├── integration_pkg/          # ROS 2 Python package (your code)
│   │   ├── launch/
│   │   │   ├── all.launch.py
│   │   │   └── swarm.launch.py
│   │   ├── config/
│   │   │   └── params.yaml
│   │   └── integration_pkg/
│   │       ├── voice_node.py
│   │       ├── selector_node.py
│   │       ├── follow_node.py
│   │       ├── tf_follow_node.py
│   │       ├── tf_broadcaster_node.py
│   │       ├── dashboard_node.py
│   │       ├── teleop_bridge.py
│   │       └── battery_sim_node.py
│   ├── quadcopter/              # Original ROS 1 packages (unchanged)
│   │   └── quad_gazebo/
│   │       └── models/
│   │           └── quadcopter/  # ← NEW SDF model for Gazebo Harmonic
│   └── turtlebot3/              # ROS 2 packages (from ROBOTIS)
├── Dockerfile
├── docker-compose.yml
└── README.md
```

---

### Troubleshooting

| Problem | Solution |
|---------|----------|
| Quadcopter doesn't spawn | Check `GZ_SIM_RESOURCE_PATH` includes `quad_gazebo/models` |
| No odometry messages | Verify bridge is running: `ros2 topic echo /model/X3/odometry` |
| TF lookup fails | Ensure `tf_broadcaster` node is running |
| Camera not showing | Check exact Gazebo camera topic with `gz topic -l` |
| Voice not working | Install `python3-speechrecognition` and ensure mic access |
| TurtleBot3 doesn't move | Check `/model/turtlebot3/cmd_vel` is bridged and diff-drive plugin is present |

---

**Author**: Fuad  
**Team**: BUET Mars Rover Team (Interplanetar 2026)
