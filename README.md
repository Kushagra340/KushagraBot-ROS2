# 🤖 KushagraBot v1.0: Autonomous Mobile Robot (AMR)

[![ROS 2](https://img.shields.io/badge/ROS_2-Humble-3498DB?style=for-the-badge&logo=ros&logoColor=white)](https://docs.ros.org/en/humble/)
[![micro-ROS](https://img.shields.io/badge/micro--ROS-XRCE--DDS-orange?style=for-the-badge)](https://micro.ros.org/)
[![ESP32](https://img.shields.io/badge/ESP32-FreeRTOS-E74C3C?style=for-the-badge&logo=espressif&logoColor=white)](https://www.espressif.com/)
[![Sensors](https://img.shields.io/badge/Perception-8x8_ToF_%2B_6--DOF_IMU-2ECC71?style=for-the-badge)]()
[![Simulation](https://img.shields.io/badge/Gazebo-Classic-9B59B6?style=for-the-badge)](https://gazebosim.org/)
[![License](https://img.shields.io/badge/License-MIT-blue?style=for-the-badge)](LICENSE)

> **A custom-engineered, differential-drive Autonomous Mobile Robot (AMR) bridging low-level embedded hardware and high-level autonomous robotics.**  
> Powered by an **ESP32 running dual-core FreeRTOS** for microsecond-deterministic motor PID control and sensor acquisition, bridged seamlessly over Wi-Fi via **micro-ROS (XRCE-DDS)** to a host PC running **ROS 2 Humble**, **Extended Kalman Filter (EKF) sensor fusion**, **3D ToF point cloud projection**, and **2D SLAM**.

---

## 📹 Video Demo

https://github.com/user-attachments/assets/PLACEHOLDER_REPLACE_WITH_ACTUAL_ASSET_ID

> *Autonomous trajectory tracking: KushagraBot v1.0 executing a precision 0.5m × 0.5m square trajectory profile commanded by `shape_controller.py` with real-time wheel odometry and IMU heading stabilization.*  
> *(Video file located in [`media/demo_square_path.mp4`](media/demo_square_path.mp4) — upload to your GitHub Release/Issue to embed).*

---

## 📑 Table of Contents
1. [System Architecture & The "Brainstem vs. Mind" Analogy](#-system-architecture)
2. [Bill of Materials (BOM) & Hardware Stack](#-bill-of-materials-bom--hardware-stack)
3. [Electrical Schematics & Pinout Mapping](#-electrical-schematics--pinout-mapping)
4. [Embedded Firmware Architecture (`firmware/`)](#-embedded-firmware-architecture)
   - [Dual-Core FreeRTOS Threading Model](#dual-core-freertos-threading-model)
   - [Hardware Pulse Counter (PCNT) Quadrature Decoding](#hardware-pulse-counter-pcnt-quadrature-decoding)
   - [Motor Control Pipeline (PID + Feedforward + Deadband)](#motor-control-pipeline)
5. [ROS 2 High-Level Software Stack (`ros2_ws/`)](#-ros-2-high-level-software-stack)
   - [TF2 Coordinate Frames](#tf2-coordinate-frames)
   - [3D ToF Point Cloud & Virtual LaserScan Pipeline](#3d-tof-point-cloud--virtual-laserscan-pipeline)
   - [EKF Odometry Sensor Fusion (`robot_localization`)](#ekf-odometry-sensor-fusion)
   - [Autonomous Trajectory Engine (`shape_controller.py`)](#autonomous-trajectory-engine)
6. [Gazebo Simulation (Digital Twin)](#-gazebo-simulation-digital-twin)
7. [Quickstart & Bringup Playbook](#-quickstart--bringup-playbook)
8. [Repository Directory Structure](#-repository-directory-structure)
9. [Detailed Engineering Documentation (`docs/`)](#-detailed-engineering-documentation)

---

## 🧠 System Architecture

Rather than relying on brittle serial strings (`printf("speed,12,34\n")`), KushagraBot utilizes standard **Object Management Group (OMG) XRCE-DDS** middleware over UDP Wi-Fi, making the ESP32 a native first-class node in the ROS 2 computational graph.

```
+-----------------------------------------------------------------------------------+
|                           THE CONSCIOUS MIND (ROS 2 Host)                         |
|  - High-level decision making, path planning (Nav2), mapping (SLAM Toolbox).      |
|  - Thinks in terms of: "Drive 30cm forward, turn 90 degrees, avoid that table."   |
+-----------------------------------------------------------------------------------+
                                         │
                         [ Wi-Fi / XRCE-DDS UDP 8888 ]
                                         │
+-----------------------------------------------------------------------------------+
|                      THE REFLEXIVE BRAINSTEM (ESP32 FreeRTOS)                     |
|  - Core 0: Reads sensory organs (Eyes/Inner Ear) at strict 100Hz intervals.       |
|  - Core 1: Translates thoughts into muscle impulses (PID loop at 20Hz).          |
|  - Hardware PCNT: Involuntary reflexes (zero-CPU pulse counting on silicon).      |
+-----------------------------------------------------------------------------------+
           │                                                     │
           ▼                                                     ▼
+-----------------------+                             +--------------------+
|  THE EYES & INNER EAR |                             | THE MUSCLES & FEET |
|  - VL53L5CX (Eyes):   |                             | - DRV8833 (Muscles)|
|    64-zone 3D ToF     |                             | - N20 Motors       |
|  - MPU-6050 (Ears):   |                             | - Rubber Wheels    |
|    Balance & Heading  |                             | - Encoders (Spine) |
+-----------------------+                             +--------------------+
```

---

## 🛠️ Bill of Materials (BOM) & Hardware Stack

| Component | Quantity | Key Specifications | Engineering Selection Rationale |
| :--- | :---: | :--- | :--- |
| **ESP32 DevKit V1 (30-pin)** | 1 | Dual Xtensa LX6 @ 240MHz, 520KB SRAM, 4MB Flash | Dual-core FreeRTOS execution isolates 100Hz sensor polling from Wi-Fi networking. Dedicated hardware Pulse Counter (PCNT) peripherals. |
| **N20 Micro Metal Gearmotors** | 2 | 3V–6V DC, 150 RPM, 30:1 Gear Ratio, Magnetic Hall Encoders (353 CPR) | High torque in a miniature envelope; integrated rear magnetic quadrature encoders provide 353 counts per wheel revolution for millimeter odometry. |
| **DRV8833 Dual H-Bridge Driver** | 1 | 2.7V–10.8V, 1.2A RMS per channel, Ultra-low $R_{DS(on)}$ MOSFETs | MOSFET design eliminates the ~2.0V bipolar junction voltage drop of obsolete L298N drivers, delivering full battery voltage to motors without heatsinks. |
| **VL53L5CX 8×8 Time-of-Flight** | 1 | 8×8 (64-zone) matrix, 45° diagonal FoV, 940nm VCSEL, up to 4m range | Generates a true 3D spatial depth cone rather than a single 1D distance point. Allows detecting overhanging obstacles and projecting 2D virtual scans. |
| **MPU-6050 6-Axis IMU** | 1 | 3-axis Gyroscope ($\pm 250^\circ$/s), 3-axis Accelerometer ($\pm 2$g), Fast I2C | Fused with wheel encoders via EKF to eliminate heading error caused by wheel slippage on smooth floors. |
| **34mm Silicone Wheels** | 2 | 34mm OD, high-traction silicone tire, D-shaft hub | High traction coefficient ($\mu \approx 1.0$) prevents slip during acceleration; small radius maximizes linear driving torque. |
| **Mini Steel Ball Caster** | 1 | 10mm ball bearing caster | Smooth 3-point kinematic support with near-zero friction ($\mu \approx 0.001$). Prevents dragging the chassis during differential turns. |
| **MT3608 DC-DC Step-Up Boost** | 1 | 2A Max, 2V–24V in to 5V–28V out | Delivers a clean, regulated 5V power bus to motor driver `VM` and ESP32 `VIN` across the battery's discharge curve. |
| **18650 Li-Ion Cell & Shield** | 1 | 3.7V, 2200mAh high-discharge cell + charging module | High transient current capability without microcontroller brownout during rapid motor acceleration. |
| **100µF Low-ESR Capacitors** | 2 | 35V Aluminum Electrolytic | Installed across DRV8833 `VM` and `GND` to suppress motor inductive kickback voltage spikes. |

---

## ⚡ Electrical Schematics & Pinout Mapping

```
+---------------------------------------------------------------------------------------+
|                                KUSHAGRABOT WIRING NETLIST                             |
+---------------------------------------------------------------------------------------+

  +-----------------------+
  | 18650 BATTERY SHIELD  |
  |  [ +5V OUT ]  [ GND ] |
  +------┬-----------┬----+
         │           │
         │           +---------------------------------------------+
         │                                                         │ (Common Ground Bus)
         ▼                                                         ▼
  +--------------+  +5V Bus   +------------------+          +---------------+
  |  MT3608      |───────────>| DRV8833 DRIVER   |          | ESP32 DEVKIT  |
  |  BOOST (5V)  |            | VM               |          | VIN           |
  +--------------+            | GND ─────────────┼─────────>| GND           |
                              +--------┬---------+          |               |
                                       │                    | 3.3V (VCC)────┼─────+ (3.3V Bus)
                                       │ Motor Outputs      |               |     │
                                       │                    |               |     ▼
                   +───────────────────┴───────────────────+|               |  +------------+
                   │                                       │| GPIO 21 (SDA)─┼─>| MPU6050 &  |
                   ▼                                       ▼| GPIO 22 (SCL)─┼─>| VL53L5CX   |
            +-------------+                         +-------------+|               |  | (Shared    |
            | LEFT MOTOR  |                         | RIGHT MOTOR || GPIO 18 (LPn) ┼─>|  I2C Bus)  |
            | M+ <── OUT1 |                         | M+ <── OUT3 || GPIO 19 (INT)─┼─<|            |
            | M- <── OUT2 |                         | M- <── OUT4 |+---------------+  +------------+
            +------┬------+                         +------┬------+       ▲
                   │                                       │              │
                   │ Encoder Feedback                      │              │
                   +───────────────────────────────────────┴──────────────+
                     Left Enc A  -> GPIO 33 (PCNT Unit 0 Pulse)
                     Left Enc B  -> GPIO 34 (PCNT Unit 0 Ctrl)
                     Right Enc A -> GPIO 32 (PCNT Unit 1 Pulse)
                     Right Enc B -> GPIO 35 (PCNT Unit 1 Ctrl)
```

### Pin Allocation Table

| ESP32 Pin | Direction | Connected Device | Function | Engineering Notes |
| :---: | :---: | :--- | :--- | :--- |
| **VIN** | Power In | Boost Converter (+5V) | Main System Power | Feeds internal linear low-dropout regulator (LDO). |
| **GND** | Power | Star Ground Bus | Reference 0V | Common ground across battery, ESP32, driver, and sensors. |
| **3.3V** | Power Out | Sensors & Encoders | +3.3V Logic Rail | Powers MPU-6050, VL53L5CX, and Hall sensor boards. |
| **GPIO 21** | Bi-Dir | MPU-6050 & VL53L5CX | I2C SDA | 400kHz Fast I2C bus. |
| **GPIO 22** | Output | MPU-6050 & VL53L5CX | I2C SCL | 400kHz Fast I2C clock. |
| **GPIO 18** | Output | VL53L5CX | LPn (Enable) | Pulled LOW to reset, HIGH to enable ToF sensor. |
| **GPIO 19** | Input | VL53L5CX | INT (Interrupt) | Signals when a new 8×8 depth frame is ready. |
| **GPIO 26** | Output (PWM) | DRV8833 IN1 | Left Motor Fwd | LEDC Channel 0 (5kHz, 8-bit resolution). |
| **GPIO 25** | Output (PWM) | DRV8833 IN2 | Left Motor Rev | LEDC Channel 1 (5kHz, 8-bit resolution). |
| **GPIO 14** | Output (PWM) | DRV8833 IN3 | Right Motor Fwd | LEDC Channel 2 (5kHz, 8-bit resolution). |
| **GPIO 27** | Output (PWM) | DRV8833 IN4 | Right Motor Rev | LEDC Channel 3 (5kHz, 8-bit resolution). |
| **GPIO 33** | Input | Left Motor Enc A | Pulse Input | PCNT Unit 0 Pulse input. |
| **GPIO 34** | Input | Left Motor Enc B | Direction Ctrl | **Input-Only GPIO!** Uses N20 external 10k pull-up. |
| **GPIO 32** | Input | Right Motor Enc A | Pulse Input | PCNT Unit 1 Pulse input. |
| **GPIO 35** | Input | Right Motor Enc B | Direction Ctrl | **Input-Only GPIO!** Uses N20 external 10k pull-up. |

> [!IMPORTANT]
> **The GPIO 34/35 Input-Only Hardware Trap:**  
> GPIOs 34, 35, 36, and 39 on the ESP32 lack internal software pull-up resistors (`pinMode(pin, INPUT_PULLUP)` fails silently). In KushagraBot, because the N20 encoder PCBs feature onboard hardware 10k pull-up resistors, GPIO 34 and 35 reliably serve as direction control lines for the hardware pulse counter.

---

## 💻 Embedded Firmware Architecture

The ESP32 firmware is developed in C++ using **PlatformIO** under the Arduino core, leveraging the native **FreeRTOS** kernel embedded within ESP-IDF.

### Dual-Core FreeRTOS Threading Model

```
     CORE 0 (PRO_CPU) - Priority 1                 CORE 1 (APP_CPU) - Priority 1
  ┌─────────────────────────────────┐           ┌──────────────────────────────────┐
  │     sensorTaskCode (100Hz)      │           │     Arduino loop() (20Hz / Idle) │
  │                                 │           │                                  │
  │ 1. Read MPU6050 via I2C         │           │ 1. Handle ArduinoOTA packets     │
  │ 2. Integrate Gyro Yaw (dt=10ms) │           │ 2. Spin micro-ROS executor       │
  │ 3. Check VL53L5CX dataReady()   │           │ 3. Read Hardware PCNT Encoders   │
  │ 4. Read 8x8 Depth Array         │           │ 4. Compute Closed-Loop PID       │
  │ 5. Lock Mutex -> Write Globals  │           │ 5. Apply PWM to DRV8833          │
  │ 6. vTaskDelayUntil(10ms)        │           │ 6. Publish Odometry & Ranges     │
  └─────────────────────────────────┘           └──────────────────────────────────┘
                   │                                              │
                   └──────────────[ SemaphoreMutex ]──────────────┘
```

- **Core 0 (100Hz Strict Real-Time)**: Integrates gyro rates $\omega_z$ with zero timing jitter ($\Delta t = 10\text{ms} \pm 50\mu\text{s}$) to avoid orientation integration drift.
- **Core 1 (20Hz Control & Networking)**: Executes micro-ROS subscription callbacks (`/cmd_vel`), PID computation, and publish routines (`/odom`, `/imu/data_raw`, `/vl53l5cx/ranges`).
- **Thread Safety**: Inter-core data transfer is guarded by FreeRTOS binary semaphores (`xSemaphoreTake` / `xSemaphoreGive`).

### Hardware Pulse Counter (PCNT) Quadrature Decoding
Traditional microcontroller encoder reading uses CPU interrupts (`attachInterrupt`). At high motor speeds, thousands of interrupts per second choke the CPU. KushagraBot configures the ESP32's **hardware silicon pulse counters (PCNT Unit 0 & Unit 1)**:
- Rising edge on Phase A increments/decrements counter based on Phase B state.
- Reading encoder ticks is reduced to a single silicon register read (`pcnt_get_counter_value`), consuming **0.0% CPU overhead**.

### Motor Control Pipeline
To combat static friction and non-linear motor response, KushagraBot combines **PID**, **feedforward**, and **deadband compensation**:

```
PWM = sgn(target_vel) * [ PWM_MIN + K_ff * |target_vel| + PID(target_vel, measured_vel) ]
```
- `PWM_MIN = 60`: Overcomes gearbox and tire static friction (stiction).
- `K_ff`: Feedforward velocity gain providing instant open-loop baseline.
- `PID`: Corrects remaining steady-state errors and load variations.

---

## 🚀 ROS 2 High-Level Software Stack

### TF2 Coordinate Frames
The transform tree ensures consistent spatial awareness across navigation and perception nodes:
```
map ──> odom ──> base_footprint ──> base_link ──┬──> left_wheel_link
                                                ├──> right_wheel_link
                                                ├──> castor_link
                                                ├──> imu_link
                                                └──> tof_link
```

### 3D ToF Point Cloud & Virtual LaserScan Pipeline
The forward-facing VL53L5CX provides 64 distance measurements in an 8×8 grid.
1. **Mathematical Projection (`tof_to_pointcloud`)**:
   Converts each zone $(u, v)$ with distance $d$ into 3D Cartesian coordinates $(X, Y, Z)$ using the sensor's optical field of view ($45^\circ \times 45^\circ$):
   $$X = \frac{Z \cdot (u - c_x)}{f_x}, \quad Y = \frac{Z \cdot (v - c_y)}{f_y}, \quad Z = d$$
   Publishes standard `sensor_msgs/msg/PointCloud2`.
2. **Planar Slicing (`pointcloud_to_laserscan`)**:
   Projects the 3D cone into a virtual 2D planar laser scan (`sensor_msgs/msg/LaserScan`) across a horizontal window, enabling standard 2D costmaps and SLAM without expensive rotating LiDAR.

### EKF Odometry Sensor Fusion
Configured in `config/ekf.yaml` via `robot_localization`:
- **Wheel Odometry**: Provides high-accuracy $X, Y$ linear translation, but drifts in heading when wheels slip.
- **MPU-6050 IMU**: Provides high-rate, slip-independent angular yaw velocity ($\dot{\psi}$).
- **EKF Output**: Clean, filtered `odometry/filtered` published to the TF tree (`odom -> base_footprint`).

### Autonomous Trajectory Engine (`shape_controller.py`)
A custom Python trajectory generator supporting:
- **Square Trajectory**: Executes 4 straight segments and 4 precision 90° in-place rotations.
- **Circle Trajectory**: Commands synchronized linear and angular velocities ($v = \omega \cdot R$).
- **Closed-Loop Odometry Feedback**: Monitors `/odom` to automatically correct trajectory deviations in real time.

---

## 🌐 Gazebo Simulation (Digital Twin)

KushagraBot features a digital twin simulation environment in Gazebo Classic:
- Exact physical dimensions matching 3D CAD chassis and wheel placement.
- Realistic inertial tensors ($I_{xx}, I_{yy}, I_{zz}$) calculated from material density.
- Surface friction coefficients ($\mu_1 = 1.0$ for rubber drive wheels, $\mu_1 = 0.001$ for steel caster).
- Differential drive plugin (`gazebo_ros_diff_drive`) simulating encoder feedback and wheel physics.

```bash
ros2 launch kushagrabot_description gazebo.launch.py
```

---

## ⚡ Quickstart & Bringup Playbook

### 1. Prerequisites
- Host PC running **Ubuntu 22.04 LTS** (or WSL2) with **ROS 2 Humble**.
- VS Code with **PlatformIO IDE** extension.
- micro-ROS Agent installed on host:
  ```bash
  sudo apt install ros-humble-micro-ros-agent
  ```

### 2. Flashing the ESP32 Firmware
```bash
cd firmware
# Build and upload via USB
pio run --target upload
# Or flash wirelessly via OTA (after first USB flash)
pio run --target upload -e esp32dev --upload-port KushagraBot.local
```

### 3. Running the micro-ROS Agent
Connect the robot and host PC to the same 2.4GHz Wi-Fi network, then start the XRCE-DDS UDP agent:
```bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

### 4. Launching Robot Description & EKF
```bash
source /opt/ros/humble/setup.bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash

# Launch URDF state publisher and EKF fusion
ros2 launch kushagrabot_description hardware.launch.py
```

### 5. Running Autonomous Square Demo
```bash
ros2 run kushagrabot_description shape_controller.py
```

### 6. Online SLAM Mapping
```bash
ros2 launch kushagrabot_description slam.launch.py
```

---

## 📂 Repository Directory Structure

```text
├── .gitignore
├── README.md                           # Master engineering overview & documentation
├── firmware/                           # PlatformIO ESP32 dual-core FreeRTOS firmware
│   ├── platformio.ini                  # micro-ROS, libraries & build settings
│   └── src/
│       └── main.cpp                    # Dual-core tasks, PCNT encoder, PID, micro-ROS
├── ros2_ws/                            # Colcon workspace for ROS 2 Humble packages
│   └── src/
│       ├── kushagrabot_description/    # URDF, Xacro kinematics & Gazebo simulation
│       │   ├── CMakeLists.txt
│       │   ├── package.xml
│       │   ├── urdf/                   # kushagrabot.urdf.xacro, materials.xacro
│       │   ├── launch/                 # display, gazebo, hardware, slam launch files
│       │   ├── rviz/                   # Pre-configured RViz visualization profiles
│       │   ├── config/                 # ekf.yaml and controller parameters
│       │   ├── worlds/                 # Gazebo simulation worlds
│       │   └── scripts/                # Python trajectory, avoider, and calibration tools
│       └── kushagrabot_hardware/       # ROS 2 hardware interfaces & pointcloud conversion
│           ├── CMakeLists.txt
│           ├── package.xml
│           ├── include/
│           └── src/
├── hardware/                           # Physical design and electrical assets
│   ├── cad/                            # SolidWorks SLDPRT files and chassis v2.STL
│   ├── bom/                            # Robotics_Project_BOM_Final.xlsx & bom_dump.txt
│   └── schematics/                     # robot_full_netlist.csv pin connection database
├── docs/                               # Comprehensive engineering deep-dives
│   ├── KushagraBot_Complete_Project_Guide.md # 700+ line master reference guide
│   ├── architecture_report.md          # Architectural decisions & system tradeoffs
│   ├── controller_tuning_guide.md      # Motor PID tuning and deadband compensation
│   ├── hardware_guide.md               # Power distribution & assembly manual
│   ├── ros2_workflow_guide.md          # Setup guide for Linux, WSL2 & Docker
│   ├── simulation_guide.md             # Gazebo physics tuning and plugin setup
│   └── how_to_test.md                  # Unit testing and diagnostic verification
└── media/                              # Media assets for README and documentation
    ├── demo_square_path.mp4            # Working video of autonomous trajectory
    ├── tf_tree.pdf                     # TF tree diagram
    └── tf_tree.gv                      # Graphviz source for coordinate frames
```

---

## 📚 Detailed Engineering Documentation

For deep technical dives into individual engineering subsystems, refer to the guides in [`docs/`](docs/):
- **[KushagraBot Complete Project Guide](docs/KushagraBot_Complete_Project_Guide.md)**: Exhaustive 700+ line architectural blueprint and engineering manual.
- **[Controller Tuning Guide](docs/controller_tuning_guide.md)**: Step-by-step procedure for tuning $K_p, K_i, K_d$ and deadband limits.
- **[Hardware & Power Guide](docs/hardware_guide.md)**: Inductive decoupling, star grounding, and power rail diagnostics.
- **[Simulation Guide](docs/simulation_guide.md)**: Gazebo Classic digital twin configuration and friction modeling.
- **[ROS 2 Workflow Guide](docs/ros2_workflow_guide.md)**: Environment setup for native Ubuntu and Windows WSL2.

---

## 📜 License
This project is open-source under the [MIT License](LICENSE).
