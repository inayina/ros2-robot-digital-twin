# Debug Log - Robot State Monitor Gazebo Bridge

## Current Status

- ROS 2 distro: Jazzy
- Gazebo version family: Harmonic / Gazebo Sim
- Bridge interfaces: `ros_gz_interfaces`
- Service bridge: `ros_gz_bridge parameter_bridge`
- IMU input topic: `/imu/filtered`
- Gazebo model: `mpu6050`
- World name: `default`

## Issue Log

### 1. MPU6050 model was not visible

**Symptom**:

- Gazebo ran, but the simulated MPU6050 box could not be seen.

**Root causes**:

- `models/mpu6050.sdf` defined an IMU sensor but did not define a visible `visual`.
- The bridge updated pose but did not reliably spawn the `mpu6050` model first.
- `setup.py` did not install the `models/` directory into the package share path.

**Fixes**:

- Added box `visual` and `collision` to `models/mpu6050.sdf`.
- Updated `robot_gazebo_bridge.py` to call `/world/default/create` before calling `/world/default/set_pose`.
- Added `models/*.sdf` installation in `setup.py`.

**Verification**:

```bash
gz topic -e -t /world/default/pose/info | grep mpu6050
```

### 2. Classic Gazebo service mismatch avoided

**Symptom risk**:

- Using `gazebo_msgs` or `/spawn_entity` would target Gazebo Classic patterns, not this Jazzy + Harmonic setup.
- Waiting directly for ROS 2 services `/world/default/create` and `/world/default/set_pose` fails unless `ros_gz_bridge parameter_bridge` is running.

**Decision**:

- Keep `ros_gz_interfaces.srv.SpawnEntity` and `ros_gz_interfaces.srv.SetEntityPose`.
- Use Gazebo Sim services `/world/default/create` and `/world/default/set_pose`.
- Start `ros_gz_bridge parameter_bridge` for both services in the launch files:
  ```bash
  /world/default/create@ros_gz_interfaces/srv/SpawnEntity
  /world/default/set_pose@ros_gz_interfaces/srv/SetEntityPose
  ```

**Verification**:

```bash
ros2 service list | grep /world/default/create
ros2 service list | grep /world/default/set_pose
```

### 3. `ros2 topic echo /world/default/pose` does not work

**Symptom**:

```text
WARNING: topic [/world/default/pose] does not appear to be published yet
Could not determine the type for the passed topic
```

**Root cause**:

- `/world/default/pose` is not automatically bridged into ROS 2.
- Gazebo Harmonic publishes the native Gazebo Transport pose stream as `/world/default/pose/info`.

**Correct check**:

```bash
gz topic -e -t /world/default/pose/info | grep mpu6050
```

### 3.1 `gz topic ... | grep mpu6050` has no output

**Root cause found on 2026-04-11**:

- `setup.cfg` was missing, so `ros2 launch` could not find the `gazebo_bridge` executable under `lib/robot_state_monitor`.
- After fixing that, `gazebo_bridge` still waited forever for `/world/default/create` because the ROS-Gazebo service bridge was missing.

**Fixes**:

- Added `setup.cfg`:
  ```ini
  [develop]
  script_dir=$base/lib/robot_state_monitor

  [install]
  install_scripts=$base/lib/robot_state_monitor
  ```
- Added `ros_gz_bridge parameter_bridge` nodes to both launch files.

**Verified log**:

```text
Creating ROS->GZ service bridge [/world/default/create ...]
Creating ROS->GZ service bridge [/world/default/set_pose ...]
Spawned Gazebo model: mpu6050
Gazebo bridge node started: /imu/filtered -> mpu6050
```

### 4. Gazebo GUI flashes and exits

**Observed log**:

```text
libEGL warning: egl: failed to create dri2 screen
```

**Likely cause**:

- GUI rendering / EGL / NVIDIA driver path fails before the GUI stays open.
- This does not necessarily mean the Gazebo server, services, or bridge are wrong.

**Mitigation**:

- Use the headless launch first:
  ```bash
  ros2 launch robot_state_monitor mpu6050_gazebo.launch.py
  ```
- Then try GUI separately:
  ```bash
  gz sim -g --render-engine-gui ogre
  ```
- If needed, use the GUI launch:
  ```bash
  ros2 launch robot_state_monitor mpu6050_gazebo_gui.launch.py
  ```

### 5. MPU6050 marker flickers and does not follow hardware tilt

**Symptom**:

- The small Gazebo cube flashes rapidly.
- The model does not clearly follow the real MPU6050 board orientation during demo.

**Root causes**:

- The model was dynamic, so Gazebo physics could fight frequent `/world/default/set_pose` updates.
- The bridge sent pose updates too quickly for a visual demo.
- The ESP32 `/imu/filtered` message currently publishes real acceleration and angular velocity, but its orientation quaternion can still be a placeholder identity quaternion.

**Fixes**:

- Changed `models/mpu6050.sdf` to a static, larger, high-visibility marker.
- Added red, green, and blue axis bars to make rotation direction obvious.
- Reduced default pose update rate to `15 Hz`.
- Added a bridge-side complementary estimator that derives roll/pitch/yaw from MPU6050 acceleration and angular velocity.

**Demo expectation**:

- Roll and pitch should visibly follow real board tilt.
- Yaw may drift because MPU6050 has no magnetometer.
- World position stays fixed intentionally; raw IMU acceleration is not enough for reliable position tracking.

## Runbook

### Build

```bash
cd /home/ina/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select robot_state_monitor
source install/setup.bash
```

### Headless verification

```bash
ros2 launch robot_state_monitor mpu6050_gazebo.launch.py
```

In a second terminal:

```bash
gz topic -e -t /world/default/pose/info | grep mpu6050
```

### GUI attempt

```bash
ros2 launch robot_state_monitor mpu6050_gazebo_gui.launch.py
```

### Static IMU sample

```bash
ros2 topic pub --once /imu/filtered sensor_msgs/msg/Imu "{orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}"
```

## Pending Work

- Confirm GUI rendering on the target machine after EGL/NVIDIA warnings.
- Add a launch option for camera pose or GUI config if the model spawns but is outside the initial view.
- Optionally move the orientation estimator from the Gazebo bridge into the ESP32 firmware later.
