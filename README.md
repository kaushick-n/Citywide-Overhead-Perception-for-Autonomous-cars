# CARLA 0.9.16 + ROS 2 Jazzy Autonomous Driving Stack

An end-to-end simulation pipeline integrating **CARLA 0.9.16** running natively on Windows 11 with **ROS 2 Jazzy** on Ubuntu 24.04 (WSL2).

---

## 🚀 Quickstart Guide

### 1. Source Environment
```bash
source env_setup.sh
```

### 2. Build Workspace
```bash
colcon build --symlink-install --packages-ignore pcl_recorder
```

### 3. Launch Bridge & Vehicle
- **Terminal 1 (Bridge):**
  ```bash
  source env_setup.sh
  ros2 launch carla_ros_bridge carla_ros_bridge.launch.py host:=172.21.32.1 town:=Town10HD_Opt synchronous_mode:=false timeout:=60.0
  ```
- **Terminal 2 (Spawn Ego Vehicle):**
  ```bash
  source env_setup.sh
  ros2 launch carla_spawn_objects carla_example_ego_vehicle.launch.py host:=172.21.32.1
  ```

---

## 🛠️ Helper Utilities
- **Reset Vehicle:** `python3 scripts/reset_car.py`
- **Follow Camera:** `python3 scripts/spectator_follow.py`
- **Test Connection:** `python3 scripts/test_connection.py`
