# Robot State Monitor

ROS 2 Jazzy package for visualizing the real MPU6050 stream in Gazebo Harmonic.

The package subscribes to `/imu/filtered`, spawns a visible `mpu6050` marker into Gazebo Sim, and updates the model pose through Gazebo Harmonic services. The launch files start `ros_gz_bridge parameter_bridge` so ROS 2 can call Gazebo Transport services.

For the hardware demo, the Gazebo marker estimates roll/pitch/yaw from MPU6050 acceleration and angular velocity. This avoids relying on the placeholder quaternion currently published by the ESP32 firmware and makes the visible marker follow physical board tilt more clearly.

## V1 Status

V1 has been verified on the real ROS 2 link: STM32 sensor/state frames are forwarded by the ESP32-S3 micro-ROS bridge over WiFi UDP, received by the micro-ROS Agent, and exposed in ROS 2 Jazzy.

Verified ROS 2 topics: `/imu/data`, `/imu/filtered`, and `/robot/state`. The Gazebo bridge consumes `/imu/filtered` and drives the visible MPU6050 marker in Gazebo Harmonic.

This version is intended as the first complete hardware-to-digital-twin demo: live MPU6050 motion enters ROS 2, then Gazebo renders a synchronized orientation marker for debugging and demonstration.

## Layout

- `robot_state_monitor/robot_gazebo_bridge.py`: ROS 2 node that bridges `/imu/filtered` to Gazebo model pose updates
- `models/mpu6050.sdf`: visible static MPU6050 marker with RGB axis bars
- `worlds/empty.sdf`: minimal Gazebo Harmonic world with physics, scene broadcaster, and ground plane
- `launch/mpu6050_gazebo.launch.py`: stable headless Gazebo server, service bridge, and pose bridge
- `launch/mpu6050_gazebo_gui.launch.py`: Gazebo GUI, service bridge, and pose bridge
- `start_static.sh`: headless smoke test script

## Build

```bash
cd /home/ina/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select robot_state_monitor
source install/setup.bash
```

## Run Headless First

Use this path to verify that Gazebo services, model spawning, and the bridge are working without involving GUI rendering.

```bash
cd /home/ina/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch robot_state_monitor mpu6050_gazebo.launch.py
```

In another terminal, verify the model exists in Gazebo:

```bash
gz topic -e -t /world/default/pose/info | grep mpu6050
```

Publish a static orientation if the real ESP32/micro-ROS stream is not running:

```bash
ros2 topic pub --once /imu/filtered sensor_msgs/msg/Imu "{orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}"
```

## Hardware Demo

Start the micro-ROS Agent first:

```bash
cd /home/ina/microros_ws
source /opt/ros/jazzy/setup.bash
./build/micro_ros_agent/micro_ros_agent udp4 --port 8888 -v 6
```

Then start or reset the ESP32-S3 micro-ROS bridge firmware. The firmware should report that the Agent is reachable and that micro-ROS is connected.

In another terminal, confirm the live ROS 2 stream:

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic list | grep imu
ros2 topic echo --once /imu/filtered
ros2 topic echo --once /robot/state
```

Build and source this package if it has not already been built:

```bash
cd /home/ina/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select robot_state_monitor
source install/setup.bash
```

Then launch the headless Gazebo demo:

```bash
cd /home/ina/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch robot_state_monitor mpu6050_gazebo.launch.py
```

The marker should rotate when the real MPU6050 board tilts. Position is intentionally fixed at `(x, y, z)` because raw MPU6050 acceleration is not reliable for world-position tracking without extra localization.

Yaw can drift over time because MPU6050 has no magnetometer. Roll and pitch should be the most stable axes for demonstration.

## Run With GUI

After the headless path works, try the GUI launch:

```bash
cd /home/ina/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch robot_state_monitor mpu6050_gazebo_gui.launch.py
```

If the GUI flashes and exits because of EGL/NVIDIA rendering, keep the headless launch running and attach the GUI separately:

```bash
gz sim -g --render-engine-gui ogre
```

## Expected ROS 2 Topics

With the ESP32 micro-ROS node running, these topics should exist:

```text
/imu/data
/imu/filtered
/robot/state
```

The Gazebo bridge uses `/imu/filtered` by default. To use another topic:

```bash
ros2 launch robot_state_monitor mpu6050_gazebo.launch.py imu_topic:=/imu/data
```

## Useful Checks

```bash
ros2 node list | grep gazebo_bridge
ros2 service list | grep /world/default
gz topic -l | grep /world/default
gz topic -e -t /world/default/pose/info | grep mpu6050
```

Do not use `ros2 topic echo /world/default/pose` unless you have explicitly bridged that Gazebo topic into ROS 2. In Gazebo Harmonic, `/world/default/pose/info` is a Gazebo Transport topic, not a ROS 2 topic by default.

If `gz topic -e -t /world/default/pose/info | grep mpu6050` has no output, check that all three processes are running:

```bash
ros2 node list | grep gazebo_service_bridge
ros2 node list | grep gazebo_bridge
pgrep -a gz
```

## Parameters

- `world_name`: Gazebo world name, default `default`
- `imu_topic`: IMU topic to subscribe, default `/imu/filtered`
- `model_name`: Gazebo entity name, default `mpu6050`
- `model_file`: optional custom SDF path
- `x`, `y`, `z`: spawn and update position, default `(0.0, 0.0, 0.35)`
- `update_rate`: pose update rate, default `15.0`
- `use_imu_estimator`: estimate orientation from acceleration and gyro, default `true`
- `accel_correction`: complementary-filter accelerometer correction, default `0.04`
- `gyro_deadband`: ignore small gyro noise below this value, default `0.015`

To use the quaternion from the ROS message instead of estimating orientation in the bridge:

```bash
ros2 launch robot_state_monitor mpu6050_gazebo.launch.py use_imu_estimator:=false
```
