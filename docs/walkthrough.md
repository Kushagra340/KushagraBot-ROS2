# KushagraBot Refactoring & Hardware Setup Walkthrough

I have successfully completed the structural refactoring and integrated the dual ToF sensors and PID velocity control into your firmware. Here is a breakdown of the changes and your new hardware pinout.

## 1. Workspace Refactoring
Your code is now cleanly separated, resolving the colcon and PlatformIO build conflicts:
- **`kushagrabot_ws/src/kushagrabot_description`**: This is your standard ROS2 colcon workspace. It contains only your URDF, meshes, launch files, and RViz configs.
- **`kushagrabot_firmware/`**: This is your PlatformIO project for the ESP32. It contains `main.cpp`, `platformio.ini`, and your library dependencies.
- The confusing `ROS2/` folder was safely deleted, leaving a clean environment.

## 2. URDF & Simulation Updates
I updated your URDF model mathematically to match your physical hardware design.
- Added `left_tof_link` and `right_tof_link` positioned at the front of the robot.
- Angled the links at **+12.5°** (0.218 rad) and **-12.5°** (-0.218 rad) from the X-axis.
- Integrated `libgazebo_ros_ray_sensor.so` into both links. When you launch Gazebo, you will see two infrared beams emitting from the front of the robot, accurately simulating your real VL53L0X sensors.

## 3. Dual ToF & I2C Addressing
Because both VL53L0X sensors share the same default I2C address (`0x29`), they cannot be used simultaneously on the same bus without modification. I have implemented a hardware boot sequence using the `XSHUT` pins:

1. The ESP32 pulls both `XSHUT` pins `LOW` on boot (turning both sensors off).
2. The left sensor is turned `HIGH` (woken up) and its I2C address is immediately changed to `0x30`.
3. The right sensor is turned `HIGH` (woken up) and initialized at `0x31` (or left at default).
4. Both sensors can now be polled on the same I2C bus independently.

I selected **GPIO 18** and **GPIO 19** for these `XSHUT` control pins.

## 4. Hardware Pinout Table (ESP32 DevKit V1)

> [!IMPORTANT]
> Please wire your ESP32 exactly according to this table to ensure the new firmware works correctly out of the box.

| Component | Pin Function | ESP32 GPIO | Notes |
| :--- | :--- | :--- | :--- |
| **I2C Bus** | SDA | **21** | Shared by MPU6050, Left ToF, Right ToF |
| | SCL | **22** | Shared by MPU6050, Left ToF, Right ToF |
| **Left ToF (VL53L0X)** | XSHUT (Enable) | **18** | Used to dynamically change I2C address on boot |
| **Right ToF (VL53L0X)**| XSHUT (Enable) | **19** | Used to dynamically change I2C address on boot |
| **Motor Driver** | AIN1 (Left FWD) | **25** | PWM Channel 0 |
| | AIN2 (Left REV) | **26** | PWM Channel 1 |
| | BIN1 (Right FWD)| **27** | PWM Channel 2 |
| | BIN2 (Right REV)| **14** | PWM Channel 3 |
| **Encoders** | Left Encoder A | **33** | Hardware Interrupt (Changed from 34 to 33 for pull-up) |
| | Right Encoder A | **32** | Hardware Interrupt |

## 5. Firmware PID Migration
I have completely rewritten `main.cpp`. The robot no longer blindly passes PWM values. Instead, it operates a high-frequency (20Hz) closed-loop PID controller natively on the ESP32:

1. **Inverse Kinematics:** When `/cmd_vel` is received, the ESP32 calculates the precise target velocity in **meters per second (m/s)** for the left and right wheels based on your `0.136m` wheel separation.
2. **Speed Calculation:** It calculates the real-time m/s of both wheels using the encoder interrupts and your `0.034m` wheel diameter.
3. **PID Loop:** It calculates the error between the target speed and actual speed, accumulates the integral, and dynamically adjusts the PWM output to ensure the motors perfectly match the `/cmd_vel` request, even if one motor is weaker or encounters resistance.
