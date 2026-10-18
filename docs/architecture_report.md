# KushagraBot V21 Architecture Report

> [!NOTE]
> This document serves as the foundational architectural blueprint for KushagraBot. It is based on a deep analysis of the existing workspace, including Arduino firmware, micro-ROS PlatformIO code, and ROS2 `kushagrabot_description` packages.

## 1. Overall Project Architecture
KushagraBot is a differential-drive robot designed for autonomous navigation using ROS2 Humble. The architecture uses a distributed processing model:
- **High-Level Processing (ROS2 Host):** A Linux environment running ROS2 handles heavy computations like TF tree maintenance, Navigation, Path Planning, SLAM, and Teleoperation commands.
- **Low-Level Processing (ESP32):** An ESP32 microcontroller executes real-time hardware operations such as PID motor control, encoder tick counting via hardware interrupts, and polling I2C sensors (MPU6050 and VL53L0X).
- **Middleware (micro-ROS):** The ESP32 communicates with the ROS2 host via micro-ROS over Wi-Fi, bridging the gap between raw hardware and the ROS2 pub/sub graph.

## 2. Directory Structure Explanation
The workspace currently contains two main hubs of code:
- `ROBOT IDE CODES/`: A playground/backup directory containing standalone Arduino IDE sketches. These sketches test individual systems (WiFi teleop, P-controller, IMU, basic odometry) without full ROS2 integration.
- `ROS2/`: The main integration folder. This directory uniquely acts as both a ROS2 Colcon workspace (containing `kushagrabot_description`) and a PlatformIO workspace (containing `platformio.ini` and `src/main.cpp` for the ESP32 firmware).

## 3. Purpose of Every Package
- **`kushagrabot_description`**: A standard ROS2 package responsible for defining the robot's physical structure, visual appearance, and simulation properties. It allows RViz and Gazebo to visualize and simulate the robot before deploying on real hardware.

## 4. Purpose of Every Folder
- **`ROS2/src/`**: Contains the C++ source code (`main.cpp`) for the micro-ROS ESP32 firmware.
- **`ROS2/kushagrabot_description/urdf/`**: Contains XML-based macros (Xacro) defining the robot's links, joints, and Gazebo plugins.
- **`ROS2/kushagrabot_description/launch/`**: Contains Python launch scripts to easily start RViz or Gazebo with the robot model.
- **`ROS2/kushagrabot_description/rviz/`**: Contains pre-configured RViz settings for visualizing the URDF correctly out of the box.
- **`ROBOT IDE CODES/final_robot_code_with_everything/`**: The most mature standalone Arduino code, successfully implementing a web server, PID tuning, heading hold, and encoder-based odometry.
- **`ROBOT IDE CODES/rc_robot_with_.../` & `simple_rc_robot_fast/`**: Earlier iterations of the firmware testing basic functionality.
- **`ROBOT IDE CODES/IMU_ORIENTATION_CUBE/`**: Firmware specifically testing the MPU6050 and 3D visualization.

## 5. Purpose of Every Launch File
- **`display.launch.py`**: Launches the `robot_state_publisher` and RViz. Used to verify the URDF geometry and TF frames without physics simulation.
- **`gazebo.launch.py`**: Spawns the URDF model inside a Gazebo Classic environment, simulating physics and activating the differential drive plugins.

## 6. Purpose of Every URDF/Xacro
- **`kushagrabot.urdf.xacro`**: The core definition of the robot's links (`base_link`, `left_wheel`, `right_wheel`, `castor_wheel`) and joints. It includes calculations for proper inertia and center of mass.
- **`kushagrabot_gazebo.xacro`**: Defines specific properties for Gazebo, like material friction (`mu1`, `mu2`) and includes the `gazebo_ros_diff_drive` and `gazebo_ros_joint_state_publisher` plugins.
- **`materials.xacro`**: Defines color profiles (Blue, Black, Grey) used by the URDF in RViz.

## 7. Purpose of Every Arduino Sketch (C++ files)
- **`final_robot_code_with_everything.ino`**: Standalone proof-of-concept testing logic for odometry calculation, IMU heading hold, and a softAP HTML dashboard. Validates that the underlying hardware and math work.
- **`ROS2/src/main.cpp`**: The **production firmware**. It runs a micro-ROS node subscribing to `/cmd_vel` and publishing `/left_ticks` and `/right_ticks`. It is designed to act as a dumb hardware interface while the ROS2 host does the math.

## 8. Purpose of Every Configuration File
- **`platformio.ini`**: Configures the build environment for the ESP32. It specifies the Jazzy micro-ROS distro, Wi-Fi transport layer, dependencies (Adafruit sensors), and OTA upload settings.
- **`urdf_config.rviz`**: Saves the specific displays (RobotModel, TF, Grid) needed to view KushagraBot immediately upon launching RViz.

## 9. Communication Architecture
- **Hardware Layer:** I2C for MPU6050 and VL53L0X. Hardware interrupts for encoder reading. PWM for motor driver.
- **Middleware Layer:** XRCE-DDS over UDP/Wi-Fi (micro-ROS). The ESP32 connects to an Agent running at `192.168.1.7:8888`.
- **ROS2 Graph:** Standard DDS pub/sub messaging on the host machine.

## 10. Data Flow
1. **Sensors -> ESP32:** MPU6050 and Encoders update volatile state variables via I2C and Interrupts.
2. **ESP32 -> micro-ROS Agent -> ROS2:** The ESP32 publishes `/left_ticks` and `/right_ticks` (and eventually IMU/ToF).
3. **ROS2 Node (Odom/TF):** A host node will consume ticks, calculate velocities, integrate position, and publish to `/odom` and broadcast `odom -> base_footprint` TF.
4. **ROS2 Navigation -> ESP32:** Nav2 or Teleop publishes `geometry_msgs/Twist` to `/cmd_vel`. The ESP32 subscribes, calculates Left/Right wheel speeds, and applies PWM.

## 11. TF Tree (Target)
`map` -> `odom` -> `base_footprint` -> `base_link`
From `base_link`, static transforms go to `left_wheel`, `right_wheel`, `castor_wheel`, `imu_link`, and `tof_links`.

## 12. Expected ROS Graph
- **Nodes:** `/esp32_base_node`, `/robot_state_publisher`, `/odometry_node` (to be written), `/teleop_keyboard`.
- **Topics:** `/cmd_vel` (Twist), `/left_ticks` (Int32), `/right_ticks` (Int32), `/imu/data_raw` (Imu), `/odom` (Odometry), `/tf` & `/tf_static`.

## 13. Current Hardware Architecture
- **Microcontroller:** ESP32 DevKit V1.
- **Motors:** 2x N20 gear motors with magnetic encoders connected to GPIO 33 & 32.
- **Motor Driver:** DRV8833 (AIN/BIN pins mapped to GPIO 25, 26, 27, 14).
- **Sensors:** MPU6050 (I2C SDA: 21, SCL: 22) and VL53L0X (currently physically disconnected, but logic exists).

## 14. Current Software Architecture
The integration is currently half-complete:
- The URDF and simulation package (`kushagrabot_description`) are well-defined.
- The standalone ESP32 code (`final_robot_code...`) has PID and odometry working perfectly, BUT it does this entirely internally.
- The micro-ROS code (`src/main.cpp`) successfully bridges `/cmd_vel` to basic PWM, and publishes ticks, but it **lacks the closed-loop PID control** found in the standalone code.

## 15. Known Limitations
- The `ROS2` directory combines a PlatformIO firmware project with a ROS2 package. This breaks standard Colcon build conventions.
- The micro-ROS firmware (`main.cpp`) applies PWM blindly based on `/cmd_vel`. It does not use the encoders for PID velocity control, meaning the robot will not drive straight in ROS2 mode if the motors are mismatched.
- The VL53L0X is currently bypassed in `main.cpp` because it's physically disconnected.
- Pin 34 was correctly identified as lacking internal pull-ups in `main.cpp` (moved to 33), but `final_robot_code_with_everything.ino` still uses 34.

## 16. Code Quality Analysis
- **Strengths:** Excellent use of non-blocking `millis()` logic. Good use of safety watchdogs (CMD_TIMEOUT_MS) to stop motors if Wi-Fi drops.
- **Weaknesses:** Mixing logic levels (publishing ticks directly instead of actual velocities). Missing PID in the ROS2 firmware.

## 17. Things Implemented Correctly
- The URDF is extremely clean and mathematically sound (correct inertia macros, collision spheres for wheels).
- The use of hardware interrupts with the `IRAM_ATTR` flag is perfectly executed.
- The safety watchdog in `main.cpp` is a crucial robotics best practice implemented early.

## 18. Potential Bugs
- `main.cpp` line 87: `PWM_RES = 8` implies 0-255. Line 106: `int pwm_val = abs(target_speed * 255.0);`. However, `ledcWrite` with ESP32 core 3.x may expect different syntax or ranges depending on exact core version.
- Over-publishing ticks over Wi-Fi without calculating delta/velocity on the ESP32 might cause unnecessary network bandwidth usage and latency issues on the host.

## 19. Possible Improvements
- Calculate real velocities (RPM or m/s) on the ESP32 and use a local PID loop to ensure the motors match the requested `/cmd_vel`.
- Publish `nav_msgs/Odometry` or a custom `WheelVelocity` message instead of raw `Int32` ticks to offload some state tracking.
- Implement IMU publishing in `main.cpp`.

## 20. Refactoring Suggestions
> [!IMPORTANT]
> **Priority Refactor:** Separation of Concerns
> We should separate the PlatformIO project (`esp32_firmware`) from the ROS2 workspace (`kushagrabot_ws`). Colcon and PlatformIO should not share a root directory to prevent build artifact conflicts.

1. Create `kushagrabot_ws/src/` for all ROS2 packages.
2. Move `platformio.ini` and ESP32 code to a dedicated `kushagrabot_firmware/` directory outside of the colcon workspace.
3. Migrate the highly successful PID tuning and Odometry math from `final_robot_code_with_everything.ino` directly into `src/main.cpp`. The ESP32 must enforce velocity closed-loops locally before ROS2 can reliably navigate.
