# KushagraBot Hardware Guide (micro-ROS)

Welcome to the real world! This guide will walk you through exactly how to boot up your physical robot, connect it to your ROS2 laptop over Wi-Fi, and troubleshoot hardware issues.

## 1. Hardware Pinout Table (ESP32 DevKit V1)

> [!IMPORTANT]
> Ensure your wiring exactly matches this table before powering on the ESP32 to prevent damage and ensure the firmware functions correctly.

| Component | Pin / Wire | ESP32 GPIO | Notes |
| :--- | :--- | :--- | :--- |
| **I2C Bus** | SDA | **21** | Shared by MPU6050, Left ToF, Right ToF |
| | SCL | **22** | Shared by MPU6050, Left ToF, Right ToF |
| **Left ToF (VL53L0X)** | XSHUT (Enable) | **18** | Essential for dynamically changing the I2C address to 0x30 |
| | VCC | 3.3V | Connect to 3.3V power |
| | GND | GND | Connect to common ground |
| **Right ToF (VL53L0X)**| XSHUT (Enable) | **19** | Initialized at address 0x31 |
| | VCC | 3.3V | Connect to 3.3V power |
| | GND | GND | Connect to common ground |
| **Left Motor Driver** | AIN1 (Logic FWD) | **25** | PWM Channel 0 |
| | AIN2 (Logic REV) | **26** | PWM Channel 1 |
| | Motor M1 / M+ | - | Connect directly to Left Motor |
| | Motor M2 / M- | - | Connect directly to Left Motor |
| **Right Motor Driver**| BIN1 (Logic FWD)| **27** | PWM Channel 2 |
| | BIN2 (Logic REV)| **14** | PWM Channel 3 |
| | Motor M1 / M+ | - | Connect directly to Right Motor |
| | Motor M2 / M- | - | Connect directly to Right Motor |
| **Left Encoder** | C1 / Encoder A | **33** | Hardware Interrupt (Must use internal pull-up) |
| | C2 / Encoder B | - | Not connected (single channel read) |
| | VCC | 3.3V/5V | Encoder Power |
| | GND | GND | Encoder Ground |
| **Right Encoder**| C1 / Encoder A | **32** | Hardware Interrupt (Must use internal pull-up) |
| | C2 / Encoder B | - | Not connected (single channel read) |
| | VCC | 3.3V/5V | Encoder Power |
| | GND | GND | Encoder Ground |

## 2. Flashing the Firmware

**CRITICAL STEP:** Before connecting to Wi-Fi or ROS2, you must flash the ESP32 with the firmware located in the `kushagrabot_firmware` folder.

1. Open VS Code.
2. Go to **File -> Open Folder** and open the `C:\Users\kush2.KUSHAGRA\OneDrive\Desktop\ROBOT FILES\kushagrabot_firmware` directory.
3. Wait for the **PlatformIO** extension to load (you should see the alien head icon on the left sidebar).
4. Connect your ESP32 to your laptop via USB.
5. Click the **PlatformIO: Upload** button (the right-pointing arrow `→` at the bottom blue status bar).
6. Once it says "SUCCESS", the ESP32 is ready!

## 3. Getting Started: Booting the Hardware

Follow these steps exactly to link your ESP32 to your WSL2 ROS2 environment:

1. **Power on the Robot:** Connect the battery to your ESP32 and motor driver. Ensure it connects to your Wi-Fi network (`Airtel_Siddhatva`).
2. **Open your Terminal (WSL2) on your Laptop.**
3. **Start the micro-ROS Agent:**
   The agent acts as the bridge between your Wi-Fi ESP32 and the local ROS2 graph.
   ```bash
   source /opt/ros/humble/setup.bash
   # Run the docker container mapping port 8888
   docker run -it --rm --net=host microros/micro-ros-agent:humble udp4 --port 8888
   ```
   *Note: Ensure your Windows Firewall allows UDP traffic on port 8888.*

4. **Verify Connection:** Once the ESP32 connects, you will see "Session established" messages in the Docker terminal.
5. **Open a SECOND Terminal** to check the ROS2 topics:
   ```bash
   source /opt/ros/humble/setup.bash
   ros2 topic list
   ```
   You should see `/left_ticks`, `/right_ticks`, `~/left_tof/range`, `~/right_tof/range`, and `/cmd_vel` listed.

## 3. Driving the Physical Robot

Open a **THIRD Terminal** and run the teleop node:
```bash
source /opt/ros/humble/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
Use `i`, `,`, `j`, `l` to drive. The ESP32 will receive `/cmd_vel`, convert the target velocity to RPM, use the encoders to calculate actual RPM, and apply PID logic to turn the physical wheels at the exact required speed.

## 4. Key Parameters & What They Do

If you need to tweak the firmware, open `kushagrabot_firmware/src/main.cpp` using the PlatformIO IDE extension in VS Code.

- **`WHEEL_SEPARATION = 0.136;` & `WHEEL_DIAMETER = 0.034;`**: Kinematic properties.
  - *Purpose:* Used to convert a ROS2 `cmd_vel` command (like "move forward at 0.2 m/s") into specific wheel speeds. 
- **`Kp = 500.0;`, `Ki = 10.0;`, `Kd = 5.0;`**: The PID constants for velocity control.
  - *Purpose:* 
    - **Kp (Proportional):** Provides immediate power to fix the speed error. If the robot stutters or shakes violently, Kp is too high. If it's too sluggish, Kp is too low.
    - **Ki (Integral):** Slowly builds up power if the robot isn't reaching the target speed (e.g., struggling up an incline). If it overshoots the speed wildly after a delay, Ki is too high.
    - **Kd (Derivative):** Dampens the effect to prevent overshooting.
- **`CMD_TIMEOUT_MS = 500;`**: Dead man's switch.
  - *Purpose:* If the ESP32 loses Wi-Fi connection or the teleop node crashes, this shuts down the motors after 0.5 seconds to prevent the robot from driving into a wall infinitely.

## 5. Troubleshooting Hardware Issues

- **Issue:** The micro-ROS agent terminal says nothing when the ESP32 is powered on.
  - *Cause:* The ESP32 is failing to connect to Wi-Fi, or your laptop's IP address is no longer `192.168.1.7`.
  - *Fix:* Check your laptop's IP using `ipconfig` in Windows Command Prompt. Update `agent_ip` in `main.cpp` if it changed, and re-flash the ESP32.

- **Issue:** The robot drives backward when I press 'i' (forward).
  - *Cause:* Motor wires (M+ / M-) are swapped on the DRV8833, or logic pins are inverted.
  - *Fix:* Physically swap the two wires for the offending motor on the DRV8833 output block.

- **Issue:** The robot spins continuously when given a forward command.
  - *Cause:* The encoder channels are swapped, or the encoder reads negative when the motor moves forward. This confuses the PID loop into giving *more* power instead of reducing it.
  - *Fix:* Swap the `LEFT_ENC_PIN` and `RIGHT_ENC_PIN` physical wires, OR swap the logic in `main.cpp` `left_motor_dir` logic.

- **Issue:** I2C devices (MPU6050 or ToF) aren't detected on boot.
  - *Cause:* Wiring issue, missing pull-up resistors on SDA/SCL, or XSHUT sequence timing failed.
  - *Fix:* Check connections. Increase `delay(10);` to `delay(50);` after pulling XSHUT HIGH in `main.cpp`.

---
*Created by Antigravity IDE.*
