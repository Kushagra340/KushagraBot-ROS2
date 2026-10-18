# 🤖 KushagraBot v1.0: The Complete Engineering Guide & Architectural Blueprint
> **A Comprehensive, Ground-Up Guide to Building an Autonomous Differential-Drive Mobile Robot Powered by ESP32, FreeRTOS, micro-ROS, and ROS 2 Humble.**

---

## 📑 Table of Contents
1. [Executive Summary & The "Big Picture" Analogy](#1-executive-summary--the-big-picture-analogy)
2. [Complete Bill of Materials (BOM) & Component Selection Rationale](#2-complete-bill-of-materials-bom--component-selection-rationale)
3. [Hardware & Electronics Stack: Schematics & Pinout](#3-hardware--electronics-stack-schematics--pinout)
4. [Embedded Firmware Architecture (`kushagrabot_firmware`)](#4-embedded-firmware-architecture-kushagrabot_firmware)
   - Dual-Core FreeRTOS Execution Model
   - Hardware Pulse Counter (PCNT) Quadrature Decoding
   - Sensor Drivers: MPU-6050 IMU & VL53L5CX 8x8 ToF Matrix
   - Motor Control Pipeline: PID + Feedforward + Deadband Compensation
   - Over-The-Air (OTA) Wireless Firmware Flashing
5. [The micro-ROS Middleware Bridge](#5-the-micro-ros-middleware-bridge)
6. [ROS 2 High-Level Software Stack (`kushagrabot_ws`)](#6-ros-2-high-level-software-stack-kushagrabot_ws)
   - Workspace Architecture & Package Structure
   - Mathematical Projection: 8x8 Depth Array to 3D `PointCloud2`
   - PointCloud to 2D `LaserScan` Conversion
   - Autonomous Trajectory Engine: `shape_controller.py`
   - Interactive Sensor Calibration Nodes
7. [Coordinate Frames (TF2), Odometry & Sensor Fusion (EKF)](#7-coordinate-frames-tf2-odometry--sensor-fusion-ekf)
8. [SLAM (Simultaneous Localization and Mapping) & RViz Visualization](#8-slam-simultaneous-localization-and-mapping--rviz-visualization)
9. [Simulation & Digital Twin in Gazebo Classic](#9-simulation--digital-twin-in-gazebo-classic)
10. [Step-by-Step Build & Bringup Playbook](#10-step-by-step-build--bringup-playbook)
11. [Troubleshooting Guide & Hardware War Stories](#11-troubleshooting-guide--hardware-war-stories)

---

## 1. Executive Summary & The "Big Picture" Analogy

### What is KushagraBot?
**KushagraBot** is a custom-engineered, fully integrated, differential-drive autonomous mobile robot (AMR). It bridges the gap between low-level embedded hardware (microcontrollers, motor drivers, quadrature encoders, I2C bus sensors) and high-level artificial intelligence / robotics software (ROS 2 Humble, Extended Kalman Filter sensor fusion, 2D SLAM, and 3D perception).

Rather than being a simple toy car controlled via Bluetooth, KushagraBot is built to the architectural standards of commercial robotics platforms. It runs a distributed compute model: an **ESP32 microcontroller** acts as the real-time embedded hardware controller, while an **Ubuntu PC (running ROS 2 Humble natively or under WSL2)** acts as the cognitive navigation brain, bridged seamlessly over Wi-Fi using **micro-ROS (XRCE-DDS)**.

---

### The Biological Robotics Analogy
To understand why KushagraBot is designed the way it is, imagine the robot as a biological organism:

```
+-----------------------------------------------------------------------------------+
|                           THE CONSCIOUS MIND (ROS 2 Host)                         |
|  - High-level decision making, path planning (Nav2), mapping (SLAM Toolbox).      |
|  - Thinks in terms of: "Drive 30cm forward, turn 90 degrees, avoid that table."   |
+-----------------------------------------------------------------------------------+
                                         │
                         [ Wi-Fi / XRCE-DDS (Nerves) ]
                                         │
+-----------------------------------------------------------------------------------+
|                      THE REFLEXIVE BRAINSTEM (ESP32 FreeRTOS)                     |
|  - Core 0: Reads sensory organs (Eyes/Inner Ear) at strict 100Hz intervals.       |
|  - Core 1: Translates thoughts into muscle impulses (PID loop at 20Hz).          |
|  - Hardware PCNT: Involuntary reflexes (counts heartbeats/encoder pulses).        |
+-----------------------------------------------------------------------------------+
           │                                                     │
           ▼                                                     ▼
+-----------------------+                             +--------------------+
|  THE EYES & INNER EAR |                             | THE MUSCLES & FEET |
|  - VL53L5CX (Eyes):   |                             | - DRV8833 (Muscles)|
|    64-pixel 3D sight  |                             | - N20 Motors       |
|  - MPU-6050 (Ears):   |                             | - Rubber Wheels    |
|    Balance & Heading  |                             | - Encoders (Spine) |
+-----------------------+                             +--------------------+
```

1. **The Conscious Mind (ROS 2 on Linux/WSL2):** 
   High-level computations such as building a 2D floorplan of a room, calculating inverse kinematics for global paths, and projecting 3D point clouds require substantial memory and floating-point power. An embedded microcontroller would choke on these tasks. Thus, the PC handles this high-level deliberation.
2. **The Reflexive Spine & Brainstem (ESP32 with FreeRTOS):**
   When your foot slips on ice, your spine reacts before your conscious mind registers the slip. Similarly, motor speed regulation (PID control) and encoder tick counting cannot wait for Wi-Fi packets or Linux OS task scheduling. The ESP32 handles real-time deterministic tasks in microsecond loops.
3. **The Nervous System (micro-ROS over UDP Wi-Fi):**
   Instead of archaic, brittle, comma-separated serial strings (e.g., `printf("speed,12,34\n")`), the robot uses standard Object Management Group (OMG) DDS pub/sub messaging. The ESP32 joins the ROS 2 computational graph as a native first-class node.
4. **The Sensory Organs (Sensors):**
   - **VL53L5CX 8x8 Time-of-Flight:** Unlike simple single-beam distance sensors, this is a multizone solid-state LiDAR matrix. It fires an invisible 940nm laser grid across 64 individual zones, generating a live 3D depth cone.
   - **MPU-6050 6-DOF IMU:** Acts as the robot's inner ear. Wheels slip on smooth floors; the IMU measures actual angular rotation rate (Yaw), providing absolute heading truth.
   - **Magnetic Encoders:** Provide proprioception (knowing how much the body has moved by counting shaft rotations).

---

## 2. Complete Bill of Materials (BOM) & Component Selection Rationale

Every component in KushagraBot was chosen to satisfy specific mechanical, electrical, and computational requirements:

| Component | Quantity | Key Specifications | Engineering Rationale / Selection Justification |
| :--- | :---: | :--- | :--- |
| **ESP32 DevKit V1 (30-pin)** | 1 | Dual Xtensa LX6 @ 240MHz, 520KB SRAM, 4MB Flash, Wi-Fi 802.11 b/g/n | Provides true dual-core processing (FreeRTOS) to isolate sensor polling from networking. Includes dedicated hardware Pulse Counter (PCNT) peripherals. |
| **N20 Micro Metal Gearmotors with Encoders** | 2 | 3V–6V DC, 150 RPM, 30:1 Gear Ratio, Integrated Magnetic Hall Encoders (353 CPR) | Compact form factor with high torque. Crucially, the rear-mounted dual-channel Hall effect encoders allow precise velocity and position tracking. |
| **DRV8833 Dual H-Bridge Motor Driver** | 1 | 2.7V–10.8V operating voltage, 1.2A continuous current per channel, Ultra-low RDS(on) MOSFETs | Far superior to the obsolete L298N. Uses modern MOSFETs instead of BJTs, avoiding the 2V drop and excessive heat generation, allowing full battery power delivery to the motors. |
| **VL53L5CX Multizone Time-of-Flight Sensor** | 1 | 8x8 (64 zone) matrix, 45° diagonal FoV, 940nm VCSEL emitter, up to 4m range, I2C interface | Generates a true 3D spatial array rather than a single 1D point. Allows detection of overhanging obstacles, ground drop-offs, and horizontal slices for SLAM. |
| **MPU-6050 6-Axis IMU** | 1 | 3-axis Gyroscope (±250°/s to ±2000°/s), 3-axis Accelerometer (±2g to ±16g), I2C | Solves wheel-slip odometry degradation. Allows high-rate integration of yaw velocity on Core 0. |
| **34mm Rubber Wheels & D-Shaft Mounts** | 2 | 34mm Outer Diameter, High-grip silicone rubber tire | High traction coefficient ($\mu \approx 1.0$) prevents slip during acceleration; small diameter provides excellent torque response. |
| **Mini Ball Caster Wheel** | 1 | 10mm Steel ball bearing caster | Provides frictionless three-point balance. In simulation and reality, a caster acts with near-zero friction ($\mu \approx 0.001$) so it turns smoothly without dragging the robot off course. |
| **MT3608 DC-DC Step-Up Boost Module** | 1 | 2A Max Output, 2V–24V Input to 5V–28V Output | Boosts single-cell or degraded battery voltages to a clean, regulated 5V power rail for the motor driver and ESP32 VIN. |
| **18650 Li-Ion Cells (3.7V, 2200mAh) + Shield** | 1 | High-discharge Li-Ion cell with integrated charging & 5V/3A step-up output | Provides high transient current capability during motor stall without browning out the microcontroller. |
| **100µF 35V Electrolytic Capacitor** | 2 | Low-ESR aluminum electrolytic | Placed directly across the DRV8833 `VM` and `GND` power terminals to absorb motor inductive kickback voltage spikes. |
| **Hardware Fasteners & Chassis** | - | M3 Standoffs, M3 CSK Screws, Brass heat-set inserts, 3D printed / acrylic plate | Rigid structural base with accurate geometric offsets matching the URDF model. |

---

## 3. Hardware & Electronics Stack: Schematics & Pinout

### Physical Architecture & Grounding Philosophy
In mixed-signal mobile robotics (combining sensitive 3.3V digital I2C lines, high-frequency Wi-Fi antennas, and noisy inductive DC motor coils), **power distribution is the #1 point of failure**.

KushagraBot adheres to three strict electrical design rules:
1. **Single Common Ground Plane:** All grounds (Battery GND, ESP32 GND, DRV8833 GND, Sensor GND, Motor Encoder GND) terminate at a common star junction. A floating ground creates floating voltage references, causing catastrophic I2C bus lockups (`i2cWriteReadNonStop returned Error -1`).
2. **Inductive Decoupling:** DC motor brushes spark and send voltage spikes backwards into the power rail. A 100µF capacitor is installed directly across the DRV8833 power terminals to filter out this noise.
3. **Dedicated Core Voltage Rails:** Motors draw heavy current from the boosted 5V rail (`VM`), while the sensitive digital logic and sensors run on the ESP32's regulated 3.3V rail.

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

---

### Complete Pin Assignment Matrix

| ESP32 Pin | Direction | Connected Device | Function / Signal Type | Hardware Details & Quirks |
| :---: | :---: | :---: | :---: | :--- |
| **VIN** | Power In | Battery / Boost Output | +5V Main System Power | Feeds internal linear low-dropout regulator (LDO). |
| **GND** | Power | Star Ground Bus | Reference 0V | Must connect to all components. |
| **3.3V** | Power Out | Sensors & Encoders | +3.3V Regulated Rail | Powers MPU6050, VL53L5CX, and Hall Sensors. |
| **GPIO 21** | Bi-Directional | MPU-6050 & VL53L5CX | I2C SDA (Data Line) | 400kHz Fast I2C. Shared across both sensors. |
| **GPIO 22** | Output | MPU-6050 & VL53L5CX | I2C SCL (Clock Line) | Synchronous clock pulse driven by ESP32. |
| **GPIO 18** | Output | VL53L5CX | LPn / Enable Pin | Pulled LOW to reset, HIGH to boot ToF imager. |
| **GPIO 19** | Input | VL53L5CX | INT (Interrupt) | Triggered FALLING when a new 8x8 frame is ready. |
| **GPIO 26** | Output (PWM) | DRV8833 IN1 | Left Motor Forward | LEDC PWM Channel 0 (5kHz, 8-bit resolution). |
| **GPIO 25** | Output (PWM) | DRV8833 IN2 | Left Motor Reverse | LEDC PWM Channel 1 (5kHz, 8-bit resolution). |
| **GPIO 14** | Output (PWM) | DRV8833 IN3 | Right Motor Forward | LEDC PWM Channel 2 (5kHz, 8-bit resolution). |
| **GPIO 27** | Output (PWM) | DRV8833 IN4 | Right Motor Reverse | LEDC PWM Channel 3 (5kHz, 8-bit resolution). |
| **GPIO 33** | Input | Left Motor Encoder A | Pulse Counting (Phase A) | PCNT Unit 0 Pulse input. |
| **GPIO 34** | Input | Left Motor Encoder B | Direction Sense (Phase B) | **Input-Only GPIO!** Has no internal pullup. |
| **GPIO 32** | Input | Right Motor Encoder A | Pulse Counting (Phase A) | PCNT Unit 1 Pulse input. |
| **GPIO 35** | Input | Right Motor Encoder B | Direction Sense (Phase B) | **Input-Only GPIO!** Has no internal pullup. |

> [!WARNING]
> **The GPIO 34/35/36/39 Input-Only Trap:**
> On the ESP32, GPIOs 34, 35, 36, and 39 are input-only pins connected to the internal ADC. They **do not have internal software pull-up or pull-down resistors (`INPUT_PULLUP` will fail silently)**. 
> In KushagraBot, because the N20 encoder boards have their own onboard hardware 10k pull-up resistors on both channel A and B, GPIO 34 and 35 work properly as direction sensing control pins for the hardware pulse counter peripheral. Never connect standard pushbuttons or floating lines to GPIO 34/35 without an external physical pull-up resistor!

---

## 4. Embedded Firmware Architecture (`kushagrabot_firmware`)

The ESP32 firmware is written in C++ using **PlatformIO** under the Arduino core, leveraging the native **FreeRTOS** operating system that runs under the hood of the ESP-IDF.

### Dual-Core FreeRTOS Execution Model
Standard Arduino sketches execute code in a single sequential `loop()`. In a ROS 2 robot, this is unacceptable:
- If micro-ROS is waiting for a Wi-Fi packet, or processing a serial buffer, a traditional `delay()` will freeze the motor PID loop and sensor readings.
- The VL53L5CX 8x8 sensor outputs 64 integer distances over I2C, which takes roughly 5-10ms to transfer.
- The MPU-6050 gyroscope must be integrated at an exact, deterministic frequency (100Hz). Any timing jitter ($\Delta t$ fluctuations) results in severe orientation drift.

To solve this, KushagraBot splits the firmware across the ESP32's two physical processing cores using FreeRTOS tasks and a mutual exclusion semaphore (`SemaphoreHandle_t`):

```
+------------------------------------------------------------------------------------+
|                         ESP32 DUAL-CORE FREERTOS THREADING                         |
+------------------------------------------------------------------------------------+

     CORE 0 (PRO_CPU) - Priority 1                 CORE 1 (APP_CPU) - Priority 1
  ┌─────────────────────────────────┐           ┌──────────────────────────────────┐
  │     sensorTaskCode (100Hz)      │           │     Arduino loop() (20Hz / Idle) │
  │                                 │           │                                  │
  │ 1. Read MPU6050 via I2C         │           │ 1. Handle ArduinoOTA packets     │
  │    (dt = 10ms strict timer)     │           │ 2. Command Watchdog (500ms)      │
  │ 2. Integrate Yaw with Deadzone  │           │ 3. PID Velocity Control Loop     │
  │ 3. If ToF INT triggered:        │           │    - Read Hardware PCNT counters │
  │    Read 64-zone distance matrix │           │    - Low-Pass Filter speed       │
  │                                 │           │    - Compute PID + Deadband      │
  │               │                 │           │    - Output PWM to DRV8833       │
  │               ▼                 │           │ 4. micro-ROS Executor Spin       │
  │        xSemaphoreTake           │           │ 5. Publish Topics via Wi-Fi      │
  │   Write to Shared Globals       │           │               │                  │
  │        xSemaphoreGive           │           │               ▼                  │
  │                                 │           │        xSemaphoreTake            │
  │                                 │           │   Read from Shared Globals       │
  │                                 │           │        xSemaphoreGive            │
  └─────────────────────────────────┘           └──────────────────────────────────┘
```

---

### Hardware Pulse Counter (PCNT) Quadrature Decoding
Traditional microcontroller robotics code attaches a software interrupt (`attachInterrupt(pin, ISR, RISING)`) to encoder pins. At 150 RPM with a 353 CPR gearbox, each motor generates over 880 interrupts per second. When both motors spin at top speed, the CPU spends nearly 20% of its time entering and exiting Interrupt Service Routines (ISRs), leading to dropped ticks, Wi-Fi packet drops, and jitter.

KushagraBot bypasses the CPU entirely by utilizing the **ESP32's built-in silicon Pulse Counter (PCNT) hardware**:
- **Unit 0** is assigned to the Left Motor; **Unit 1** is assigned to the Right Motor.
- **`pulse_gpio_num`** connects to Encoder Phase A; **`ctrl_gpio_num`** connects to Encoder Phase B.
- When Phase A transitions, hardware logic instantly checks Phase B:
  - If Phase B is LOW, it increments the internal 16-bit register.
  - If Phase B is HIGH, it decrements the register.
- An integrated digital glitch filter (`pcnt_set_filter_value(PCNT_UNIT_0, 100)`) ignores electrical noise spikes shorter than 100 clock cycles.
- The CPU only reads the hardware registers once every 50ms (20Hz), clears the counter, and continues. Zero CPU interrupts are fired during driving.

```cpp
// Hardware PCNT Configuration (Zero CPU Overhead)
pcnt_config_t pcnt_config_left = {
    .pulse_gpio_num = LEFT_ENC_PIN,    // GPIO 33
    .ctrl_gpio_num = LEFT_ENC_OUTB,    // GPIO 34
    .lctrl_mode = PCNT_MODE_KEEP,      // Reverse counting if control pin is HIGH
    .hctrl_mode = PCNT_MODE_REVERSE,
    .pos_mode = PCNT_COUNT_INC,        // Count up on rising edge
    .neg_mode = PCNT_COUNT_DEC,        // Count down on falling edge
    .counter_h_lim = 32767,
    .counter_l_lim = -32768,
    .unit = PCNT_UNIT_0,
    .channel = PCNT_CHANNEL_0,
};
pcnt_unit_config(&pcnt_config_left);
pcnt_set_filter_value(PCNT_UNIT_0, 100); // 100 clock cycle glitch filter
pcnt_filter_enable(PCNT_UNIT_0);
```

---

### Sensor Drivers: MPU-6050 & VL53L5CX

#### 1. MPU-6050 Drift Elimination via Deadband Integration
Low-cost MEMS gyroscopes suffer from **static bias drift**: when the robot is completely stationary, the gyro outputs a tiny non-zero reading (e.g., $0.3^\circ/\text{s}$). If you blindly integrate this reading over time ($\theta = \int \omega_z dt$), your robot's orientation in ROS 2 will slowly rotate on the screen even while parked.

To fix this, Core 0 runs an inline deadzone filter:
```cpp
sensors_event_t a, g, temp;
mpu.getEvent(&a, &g, &temp);
float gz_deg = g.gyro.z * 57.2958; // Convert rad/s to deg/s

// Deadzone Filter: Any vibration below 1.5 deg/s is treated as zero!
if (abs(gz_deg) < 1.5) gz_deg = 0.0;

xSemaphoreTake(sensorMutex, portMAX_DELAY);
esp32_yaw += gz_deg * dt; // True absolute yaw integrated internally
xSemaphoreGive(sensorMutex);
```
> [!TIP]
> **Why Integrate Yaw on the ESP32 instead of the PC?**
> If you publish raw angular velocity over Wi-Fi to a PC, network latency variations (jitter) mean packet arrival times fluctuate ($10\text{ms} \pm 15\text{ms}$). Integrating fluctuating $\Delta t$ timestamps on a PC compounds mathematical error. By integrating yaw directly on the ESP32 in a rigid, hardware-timed 100Hz FreeRTOS loop, network latency has **zero effect** on heading accuracy.

#### 2. VL53L5CX Multizone Matrix Ranging
The VL53L5CX features 64 independent ranging zones. Initializing this device requires loading a massive ~85KB binary firmware blob into the sensor over I2C at bootup:
1. The ESP32 asserts `TOF_EN_PIN` (GPIO 18) LOW for 10ms, then HIGH to cleanly reboot the sensor's internal microprocessor.
2. `myImager.begin()` streams the firmware and configures the sensor into **8x8 resolution mode**.
3. Ranging is set to autonomous mode, and the sensor's physical interrupt line is tied to `TOF_INT_PIN` (GPIO 19).
4. Whenever the sensor completes a laser measurement across all 64 zones, it pulls GPIO 19 LOW (`FALLING`), triggering `tof_isr()` which notifies Core 0 to transfer the 64-element distance array over I2C.
5. Invalidation Filtering: Distance targets with status codes other than `5` (Target Valid) or `9` (Wrap-around Valid) are marked as `-1.0` meters to prevent invalid laser reflections from polluting the SLAM map.

---

### Motor Control Pipeline: PID + Feedforward + Deadband Compensation

Converting a requested wheel velocity ($\text{rad/s}$) into physical rotation requires solving three physical challenges:
1. **Static Friction (Stiction):** Small DC motors have mechanical gears and brushes that require a minimum threshold voltage just to start turning (the "deadband"). If you send a PWM of 15 out of 255, the motor buzzes and does not move.
2. **Speed-to-PWM Non-linearity:** PWM controls voltage, but motor speed depends on back-EMF, friction, and battery discharge curves.
3. **Discrete Encoder Quantization:** At low speeds, an encoder might only tick once or twice per control cycle, creating massive velocity spikes when computing $\Delta \text{ticks} / \Delta t$.

KushagraBot implements a **four-stage closed-loop velocity controller** running at 20Hz:

```
Target Speed (rad/s)
         │
         ├───> [ Feedforward: Kf * Target ] ──────────────┐
         │                                                │
         ▼                                                │
  [ + ] Error = Target - Filtered_Actual                  ▼
    │                                              [ + ] Summed Effort
    ├───> [ Proportional: Kp * Error ] ───────────>   │
    ├───> [ Integral: Ki * ∫ Error dt ] ──────────>   │
    └───> [ Derivative: Kd * d(Error)/dt ] ───────>   │
                                                      ▼
                                       [ Deadband Compensation: ±45 PWM ]
                                                      │
                                                      ▼
                                          [ Constrain: 0 to 255 ]
                                                      │
                                                      ▼
                                             apply_pwm() to DRV8833
```

#### The Math & Equations:
1. **Velocity Filtering (Exponential Moving Average):**
   $$\omega_{\text{filtered}}[k] = 0.7 \cdot \omega_{\text{filtered}}[k-1] + 0.3 \cdot \omega_{\text{raw}}[k]$$
   This acts as a low-pass filter, eliminating the high-frequency 20Hz vibration caused by discrete encoder resolution.
2. **Feedforward + PID Output:**
   $$\text{PWM}_{\text{raw}} = (K_f \cdot \omega_{\text{target}}) + (K_p \cdot e) + (K_i \int e \, dt) + (K_d \frac{de}{dt})$$
   - **$K_f = 25.0$:** Feedforward handles 85% of the motor work immediately without waiting for error to accumulate.
   - **$K_p = 5.0$:** Corrects small speed discrepancies between the left and right wheels.
   - **$K_i = 20.0$:** Eliminates steady-state error (e.g., when driving across a thick carpet that adds drag).
   - **$K_d = 0.0$:** Dampening is handled naturally by the gearbox inertia; keeping $K_d = 0$ prevents noisy amplification of encoder jitter.
3. **Deadband Injection:**
   ```cpp
   int deadband = 45; // Minimum PWM required to break static gear friction
   if (target_left_speed > 0) {
       pwm_left += deadband;
       pwm_left = constrain(pwm_left, 0, 255);
   } else if (target_left_speed < 0) {
       pwm_left -= deadband;
       pwm_left = constrain(pwm_left, -255, 0);
   } else {
       pwm_left = 0;
       integral_left = 0; // Prevent integral windup when stopped!
   }
   ```
4. **Safety Watchdog (Dead-Man's Switch):**
   If the Wi-Fi router reboots, or the laptop disconnects, the robot must not drive endlessly forward. Every incoming velocity command updates `last_cmd_time = millis()`. If `millis() - last_cmd_time > 500ms`, the controller immediately forces target speeds to zero and stops the motors.

---

### Over-The-Air (OTA) Wireless Firmware Flashing
During real-world robot development, plugging and unplugging a USB cable every time you adjust a PID parameter ruins calibration, snags on wheels, and causes mechanical stress.

KushagraBot integrates **ArduinoOTA**:
1. When the ESP32 boots and connects to Wi-Fi, it starts the OTA daemon listening on port 3232:
   ```cpp
   ArduinoOTA.setHostname("KushagraBot");
   ArduinoOTA.begin();
   ```
2. In the `loop()`, `ArduinoOTA.handle()` listens for incoming binary upload requests.
3. To flash wirelessly from your laptop terminal via PlatformIO, simply uncomment these two lines in `platformio.ini`:
   ```ini
   upload_protocol = espota
   upload_port = KushagraBot.local
   ```
4. Running `pio run -t upload` compiles the binary, locates the robot via mDNS (`KushagraBot.local`), and streams the new firmware over Wi-Fi in under 8 seconds.

---

## 5. The micro-ROS Middleware Bridge

### XRCE-DDS Client-Agent Architecture
Standard ROS 2 uses **DDS (Data Distribution Service)**, which relies on multi-threaded UDP discovery mechanisms that require megabytes of RAM and substantial networking stack overhead. The ESP32 cannot run full DDS natively.

**micro-ROS** solves this using the **eProsima Micro XRCE-DDS** protocol:
- The ESP32 runs an extremely lightweight **Client library** that communicates over standard POSIX UDP sockets.
- The Host PC runs the **`micro-ros-agent`** in Docker (or natively in ROS 2).
- The Agent acts as an interpreter: it accepts lightweight binary UDP packets from the ESP32 on port 8888, converts them into full standard ROS 2 DDS messages, and injects them directly into the ROS 2 computational graph.

```
+-----------------------------------------------------------------------------------------+
|                               MICRO-ROS COMMUNICATION GRAPH                             |
+-----------------------------------------------------------------------------------------+

   [ ESP32 Microcontroller ]                           [ Host PC (WSL2 / Linux) ]
   Node: /esp32_base_node                               
   ┌───────────────────────┐                                ┌──────────────────────┐
   │ Subscriptions:        │      XRCE-DDS over UDP         │ Docker Container:    │
   │ - /left_wheel_cmd     │ <────────────────────────────> │ micro-ros-agent      │
   │ - /right_wheel_cmd    │        Port: 8888              │ (udp4 --port 8888)   │
   │                       │                                └──────────┬───────────┘
   │ Publications:         │                                           │ Native ROS 2 DDS
   │ - /left_ticks         │                                           ▼
   │ - /right_ticks        │                                ┌──────────────────────┐
   │ - /imu/data_raw       │                                │ ROS 2 Topics:        │
   │ - /tof_depth_array    │                                │ - /left_ticks        │
   └───────────────────────┘                                │ - /right_ticks       │
                                                            │ - /imu/data_raw      │
                                                            │ - /tof_depth_array   │
                                                            └──────────────────────┘
```

---

## 6. ROS 2 High-Level Software Stack (`kushagrabot_ws`)

### Workspace Architecture & Package Structure
The high-level software resides in `kushagrabot_ws`, organized cleanly as a standard ROS 2 Colcon package:

```
kushagrabot_ws/
└── src/
    └── kushagrabot_description/
        ├── CMakeLists.txt              # Build configuration & executable targets
        ├── package.xml                 # Package manifest & dependencies
        ├── config/
        │   ├── controllers.yaml        # ros2_control diff_drive parameters
        │   └── ekf.yaml                # robot_localization sensor fusion configuration
        ├── launch/
        │   ├── display.launch.py       # Visualizes URDF model in RViz2
        │   ├── gazebo.launch.py        # Spawns robot model in Gazebo Classic world
        │   ├── hardware.launch.py      # Starts robot_state_publisher & controllers
        │   └── slam_launch.py          # Complete SLAM, PointCloud & LaserScan pipeline
        ├── rviz/
        │   └── urdf_config.rviz        # Pre-configured RViz display perspectives
        ├── scripts/
        │   ├── tof_pointcloud_node.py  # Converts 64-zone array into 3D PointCloud2
        │   ├── shape_controller.py     # Closed-loop autonomous shape driver (GUI)
        │   ├── safety_controller.py    # Autonomous obstacle emergency stop multiplexer
        │   ├── obstacle_avoider.py     # Autonomous reactive obstacle avoidance FSM
        │   ├── calibrate_encoders.py   # Interactive physical encoder CPR calibration
        │   └── calibrate_imu.py        # Interactive 90-degree gyro calibration
        └── urdf/
            ├── kushagrabot.urdf.xacro  # Kinematic links, joints, inertia & geometry
            ├── kushagrabot_gazebo.xacro# Gazebo physics, friction & sensor plugins
            └── materials.xacro         # RViz color shaders
```

---

### Mathematical Projection: 8x8 Depth Array to 3D `PointCloud2`

The raw data streaming from the ESP32 on `/tof_depth_array` is an unstructured flat 64-element floating-point array representing an $8 \times 8$ grid of distances in meters.

To turn this into a 3D point cloud that RViz and SLAM can understand, `tof_pointcloud_node.py` performs spherical-to-Cartesian trigonometry for every single pixel in real-time:

```
                     VL53L5CX Sensor Lens
                             (0,0,0)
                               /│\
                              / │ \
                             /  │  \
                            /   │   \  Horizontal FoV = 45°
                           /    │    \ Vertical FoV   = 45°
                          /     │     \
                         ▼      ▼      ▼
                        [ 8x8 Zone Matrix ]
```

#### Trigonometric Transformation Equations:
- The optical field-of-view of the VL53L5CX is $45^\circ \times 45^\circ$ ($\text{FoV}_{\text{rad}} = 0.785398\text{ rad}$).
- With 8 pixels across, each zone occupies an angular width of:
  $$\Delta \theta = \frac{\text{FoV}_{\text{rad}}}{8} \approx 0.09817\text{ rad } (5.625^\circ)$$
- For a pixel located at matrix row $r \in [0, 7]$ and column $c \in [0, 7]$, its angular deviations from the optical center axis are:
  $$\theta_h = (3.5 - c) \cdot \Delta \theta \quad \text{(Horizontal Azimuth)}$$
  $$\theta_v = (3.5 - r) \cdot \Delta \theta \quad \text{(Vertical Elevation)}$$
- Using standard ROS optical conventions (REP 103: $X = \text{Forward}$, $Y = \text{Left}$, $Z = \text{Up}$), the 3D Cartesian coordinates $(x, y, z)$ are calculated from the radial measured distance $d$:
  $$x = d \cdot \cos(\theta_v) \cdot \cos(\theta_h)$$
  $$y = d \cdot \cos(\theta_v) \cdot \sin(\theta_h)$$
  $$z = d \cdot \sin(\theta_v)$$

The node packs these $(x, y, z)$ coordinates into binary floating-point buffers and publishes a fully compliant `sensor_msgs/PointCloud2` message timestamped to the `left_tof_link` frame.

---

### PointCloud to 2D `LaserScan` Conversion
2D SLAM algorithms (such as Cartographer and SLAM Toolbox) expect a flat 2D `sensor_msgs/LaserScan` like that from an expensive $300 rotating planar LiDAR.

In `slam_launch.py`, KushagraBot utilizes the ROS 2 `pointcloud_to_laserscan` package to take the 3D point cloud and slice it horizontally:
- **`min_height: -1.0`**, **`max_height: 1.0`**: Captures all points within the robot's vertical envelope.
- **`angle_min: -0.392`** ($-22.5^\circ$), **`angle_max: +0.392`** ($+22.5^\circ$): Matches the $45^\circ$ sensor cone.
- **`angle_increment: 0.098`** ($5.625^\circ$): Bins the 64 3D points into a virtual 8-beam planar laser scan published to `/scan`.

---

### Autonomous Trajectory Engine: `shape_controller.py`
To prove the precision of the physical robot's odometry and IMU integration without needing full Nav2 path planning, KushagraBot includes an autonomous shape driving application featuring an interactive Tkinter graphical interface.

```
+───────────────────────────────────────────────────────────+
|               KUSHAGRABOT SHAPE CONTROLLER                |
+───────────────────────────────────────────────────────────+
|  [ Execute Square (30cm) ]   [ Execute Circle (1m dia) ]  |
|  [ Execute Triangle ]        [ EMERGENCY STOP ]           |
|                                                           |
|  Live Telemetry:                                          |
|  Odometry X:  0.298 m        Yaw: 89.92 deg               |
|  Odometry Y:  0.002 m        Status: HOLDING HEADING      |
+───────────────────────────────────────────────────────────+
```

#### How it achieves mm-level precision:
1. **Euclidean Distance Integration:** Rather than driving for a blind duration of seconds (which fails as the battery drains), the controller tracks starting coordinates $(x_0, y_0)$ and calculates actual displacement:
   $$\text{Dist} = \sqrt{(x - x_0)^2 + (y - y_0)^2}$$
2. **Active Heading-Hold P-Controller:** When driving straight, mechanical imperfections or surface bumps will cause the robot to drift slightly. The controller executes an internal closed-loop heading hold:
   $$e_{\theta} = \theta_{\text{target}} - \theta_{\text{current}}$$
   $$\omega_z = e_{\theta} \cdot 0.05 \text{ rad/s}$$
   This actively steers the wheels to keep the robot locked dead-straight on a line!
3. **Kinetic Inertia & Coasting Compensation:** When you tell a robot moving at 0.3 m/s to stop, the motor gears and chassis momentum carry it forward another 7 to 8 centimeters. The controller incorporates predictive deceleration thresholds:
   - For linear distance: Cuts motor power **8cm before the target**, allowing kinetic friction to coast the robot to the exact millimeter mark.
   - For turning: Cuts rotation **12° before the target**, coasting to a perfect 90° corner.

---

### Interactive Calibration Scripts

#### 1. Encoder Calibration (`calibrate_encoders.py`)
Theoretical gearbox math often differs from reality due to tire compression under robot weight:
1. The user places the robot against a ruler on the floor and starts the script.
2. The user pushes the robot forward by hand exactly **30.0 cm**.
3. The script counts total pulses registered ($\Delta \text{ticks}$) and computes the exact empirical Counts-Per-Revolution:
   $$\text{Wheel Circumference} = \pi \cdot 3.4\text{ cm} = 10.6814\text{ cm}$$
   $$\text{Revolutions} = \frac{30.0\text{ cm}}{10.6814\text{ cm}} \approx 2.8086\text{ rev}$$
   $$\text{Calibrated CPR} = \frac{\Delta \text{ticks}}{\text{Revolutions}} = \mathbf{353}$$

#### 2. IMU Calibration (`calibrate_imu.py`)
Eliminates scale factor errors in the MEMS gyroscope silicon:
1. The script guides the user through 5 sequential manual $90^\circ$ turns.
2. It compares physical rotation ($90.0^\circ$) against the integrated gyro reading and computes the exact calibration multiplier:
   $$\text{Factor} = \frac{90.0^\circ}{\text{Average Measured Angle}} = \mathbf{1.0475}$$
3. This factor is hardcoded into `shape_controller.py`, guaranteeing that a commanded 90° turn produces a mathematically perfect right angle in the real world.

---

## 7. Coordinate Frames (TF2), Odometry & Sensor Fusion (EKF)

### The Coordinate Transform Tree (REP 105)
In ROS 2, every sensor, wheel, and frame must know its spatial relationship to every other part of the robot. KushagraBot maintains a clean, fully connected TF tree compliant with standard ROS robotics conventions:

```
                            map
                             │  [ Published by SLAM Toolbox: Map Correction ]
                             ▼
                            odom
                             │  [ Published by diff_drive_controller / EKF ]
                             ▼
                       base_footprint  (Projected directly onto ground plane)
                             │
                             ▼  [ z_offset = +0.04m ]
                         base_link     (Geometric center of physical body)
       ┌─────────────────────┼─────────────────────┬───────────────────┐
       ▼                     ▼                     ▼                   ▼
  left_wheel            right_wheel           castor_wheel         imu_link
  (Continuous)          (Continuous)          (Fixed)              (Fixed)
                                                   │
                                                   ▼
                                             left_tof_link
                                      (Fixed: x=+0.10, y=+0.04, z=+0.04, yaw=+12.5°)
```

- **`map -> odom`**: Represents the robot's long-term global drift correction computed by SLAM scan matching.
- **`odom -> base_footprint`**: The high-rate, smooth dead-reckoning coordinate frame computed from wheel encoders and IMU integration.
- **`base_footprint -> base_link`**: Links the ground plane to the 3D center of mass.
- **`base_link -> left_tof_link`**: Identifies the physical position and angled tilt of the VL53L5CX laser sensor relative to the chassis center.

---

### Sensor Fusion via `robot_localization` (EKF)
In `config/ekf.yaml`, KushagraBot configures an **Extended Kalman Filter (EKF)** node:
- **`odom0` (`/diff_drive_controller/odom`)**: Fuses linear velocities ($v_x, v_y$).
- **`imu0` (`/imu/data_raw`)**: Fuses angular yaw velocity ($\dot{\psi}$) and linear acceleration ($a_x$).
- If the wheels slip on a slick surface, the encoder odometry lies, but the IMU detects the lack of rotational momentum. The Kalman filter statistically weights the covariance matrices of both sensors to compute a drift-free estimate of the robot's state.

---

## 8. SLAM (Simultaneous Localization and Mapping) & RViz Visualization

### Autonomous Mapping via SLAM Toolbox
KushagraBot uses **SLAM Toolbox** running in **Asynchronous Online Mode** (`async_slam_toolbox_node`).

```
+───────────────────────────────────────────────────────────────────────────────────────────+
|                                    SLAM PIPELINE IN ACTION                                |
+───────────────────────────────────────────────────────────────────────────────────────────+

  /tof_depth_array ──> [ tof_pointcloud_node ] ──> /tof_pointcloud
                                                          │
                                                          ▼
  /scan <───────────── [ pointcloud_to_laserscan ] ───────┘
    │
    ▼
  [ slam_toolbox ] <── Fuses /scan with /tf (odom -> base_link)
    │
    ├──> Publishes /map (2D Occupancy Grid)
    └──> Broadcasts map -> odom transform correction
```

1. The virtual 2D laser scan from the VL53L5CX streams into `slam_toolbox`.
2. As the robot drives, the **Ceres Scan Matcher** correlates incoming laser scans with the existing map history.
3. When the robot revisits a previously mapped area, loop closure is detected, snapping the entire floorplan into global alignment and removing accumulated odometry drift.

---

### RViz 3D Setup & Displays
To observe KushagraBot operating live in 3D:
1. Set **Global Options -> Fixed Frame** to `map`.
2. **RobotModel Display:** Points to `robot_description` topic to render the 3D chassis, rubber wheels, and sensors.
3. **TF Display:** Renders the dynamic coordinate axis frames moving through 3D space.
4. **PointCloud2 Display:** Set topic to `/tof_pointcloud`, Style to `Boxes`, Size to `0.02m`, Color Transformer to `AxisColor`. You will see a vibrant 3D depth cone of colored pixels floating in front of the robot.
5. **Map Display:** Set topic to `/map`. The white floorplan lines and black obstacle outlines expand dynamically in real-time as the robot explores.

---

## 9. Simulation & Digital Twin in Gazebo Classic

Before deploying software to the physical robot, the entire system is tested in a virtual simulation environment.

```
       REAL HARDWARE                           GAZEBO SIMULATION
 ┌────────────────────────┐               ┌────────────────────────┐
 │ ESP32 Hardware + PCNT  │               │ libgazebo_ros_diff_    │
 │ DRV8833 Motor Driver   │ <==PARITY==>  │ drive.so Plugin        │
 │ VL53L5CX Physical ToF  │               │ libgazebo_ros_ray_     │
 │ MPU6050 Physical IMU   │               │ sensor.so Plugin       │
 └────────────────────────┘               └────────────────────────┘
             │                                        │
             ▼                                        ▼
   Published to ROS 2 Topics:               Published to ROS 2 Topics:
   - /cmd_vel                               - /cmd_vel
   - /odom                                  - /odom
   - /scan                                  - /scan
```

### Physical Simulation Properties (`kushagrabot_gazebo.xacro`)
1. **Mathematical Inertia Matrices:** Every link (body, wheels, caster) has its inertia tensor accurately calculated via Xacro macros (`box_inertia`, `cylinder_inertia`, `sphere_inertia`). This ensures realistic mass distribution ($0.7\text{kg}$ body, $0.05\text{kg}$ wheels).
2. **Friction Parameters (`mu1`, `mu2`):**
   - Drive wheels are given high friction ($\mu_1 = 1.0, \mu_2 = 1.0$) to simulate rubber-on-floor traction.
   - The caster wheel is assigned near-zero friction ($\mu_1 = 0.001, \mu_2 = 0.001$) so it slides smoothly in any direction without sticking or tipping the chassis.
3. **Sensor Noise Injection:** Ray sensor plugins in Gazebo inject synthetic Gaussian noise, ensuring that high-level navigation code is robust against sensor imperfections.

---

## 10. Step-by-Step Build & Bringup Playbook

Follow this exact sequence whenever booting up and running KushagraBot:

### Step 1: Physical Assembly & Flashing
1. Complete wiring according to the [Pinout Matrix](#complete-pin-assignment-matrix).
2. Connect the ESP32 to your PC via USB.
3. In PlatformIO, build and flash the firmware:
   ```bash
   cd kushagrabot_firmware
   pio run -t upload
   ```
4. Disconnect USB and power the robot from its internal 18650 battery shield.

---

### Step 2: Start the micro-ROS Agent (WSL2 / Linux Terminal 1)
Launch the micro-ROS Docker agent to bridge UDP Wi-Fi communication:
```bash
docker run -it --rm --net=host microros/micro-ros-agent:humble udp4 --port 8888
```
*Hit the physical `EN` (Reset) button on the ESP32. Within 2 seconds, the terminal will log: `[XRCE-DDS] Session established`.*

---

### Step 3: Launch Robot State & Hardware Nodes (Terminal 2)
Navigate to your workspace, source the environment, and start the robot state publisher:
```bash
cd ~/OneDrive/Desktop/ROBOT_FILES/kushagrabot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch kushagrabot_description hardware.launch.py
```

---

### Step 4: Launch SLAM & Point Cloud Processing (Terminal 3)
Start the 3D-to-2D depth cloud converter, laser scan virtualizer, and SLAM Toolbox:
```bash
cd ~/OneDrive/Desktop/ROBOT_FILES/kushagrabot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch kushagrabot_description slam_launch.py
```

---

### Step 5: Launch RViz Visualization (Terminal 4)
Open RViz to observe the live 3D sensor streams and mapping progress:
```bash
source /opt/ros/humble/setup.bash
rviz2 -d ~/OneDrive/Desktop/ROBOT_FILES/kushagrabot_ws/src/kushagrabot_description/rviz/urdf_config.rviz
```

---

### Step 6: Drive the Robot (Terminal 5)
Choose one of two driving modes:
- **Option A (Manual Teleop):**
  ```bash
  ros2 run teleop_twist_keyboard teleop_twist_keyboard
  ```
  *(Drive using `i`, `,`, `j`, `l`)*
- **Option B (Autonomous Shape Execution):**
  ```bash
  ros2 run kushagrabot_description shape_controller.py
  ```
  *(Click "Execute Square" or "Execute Circle" in the GUI window!)*

---

## 11. Troubleshooting Guide & Hardware War Stories

Robotics development is filled with subtle edge cases where software assumptions clash with physics. Here are the documented "war stories" and proven solutions encountered while building KushagraBot:

### 1. The USB Power Brownout Crash during Flashing
- **Symptom:** Flashing the ESP32 via `esptool.py` fails randomly at 24% or 53% with the error: `Fatal Error: The chip stopped responding`.
- **Root Cause:** Erasing and writing flash memory causes high current spikes. Because the MPU-6050, Encoders, and **VL53L5CX (which fires an infrared laser array)** were connected to the ESP32's 3.3V rail, the USB port browned out, dropping voltage below 2.7V and rebooting the chip mid-write.
- **Solution:** 
  1. Temporarily unplug the 5V power wire to the sensors during flashing, OR
  2. Lower the upload baud rate from `460800` to `115200` in `platformio.ini`:
     ```bash
     python esptool.py --chip esp32 --port COM3 --baud 115200 write_flash ...
     ```

### 2. I2C Bus Lockup: `i2cWriteReadNonStop returned Error -1`
- **Symptom:** The ESP32 serial monitor floods with I2C communication errors, and sensor readings freeze.
- **Root Cause:** If an I2C slave device (like the VL53L5CX) loses power momentarily, it can hold the `SDA` line LOW indefinitely, preventing the master from sending a STOP condition.
- **Solution:** Add an explicit I2C bus timeout in `setup()` so the ESP32 automatically clears the bus if a transaction hangs:
  ```cpp
  Wire.begin(21, 22);
  Wire.setTimeOut(150); // Prevents ESP32 from freezing if a sensor hiccups
  ```

### 3. The "Spinning in Place" PID Phase Inversion
- **Symptom:** You send a forward command (`/cmd_vel` linear = +0.2), but the robot spins frantically in circles.
- **Root Cause:** Motor driver polarity and encoder channels were out of phase. The motor turned forward, but the encoder read negative ticks. The PID controller thought the robot was moving backward, so it added *even more forward power*, triggering an runaway positive feedback loop.
- **Solution:** Either swap the physical motor terminal wires (OUT1/OUT2) on the DRV8833 screw terminals, or invert the counting logic in firmware.

### 4. Docker Socket Permission Denied in WSL2
- **Symptom:** Running the micro-ROS agent in Docker outputs: `failed to connect to the docker API at unix:///var/run/docker.sock`.
- **Root Cause:** Docker Desktop was not running on the Windows host, or WSL2 integration was not toggled on for the Ubuntu distro.
- **Solution:** Launch Docker Desktop in Windows -> **Settings -> Resources -> WSL Integration** -> Ensure "Ubuntu-22.04" is checked.

### 5. High-Frequency 20Hz Motor Shudder
- **Symptom:** Motors hum loudly and shudder during low-speed driving.
- **Root Cause:** Calculating derivative error ($K_d$) on discrete integer encoder counts amplifies mathematical noise ($d(\text{error})/dt$ fluctuates wildly).
- **Solution:** Set $K_d = 0.0$ and pass raw speed through the Exponential Moving Average filter ($\alpha = 0.7$) to create silky smooth speed feedback.

---

## 🏁 Conclusion & Future Roadmap

KushagraBot demonstrates that a professional-grade, SLAM-capable autonomous mobile robot does not require thousands of dollars in industrial hardware. By combining:
- The real-time deterministic power of **FreeRTOS & Hardware PCNT**,
- The clean networking layer of **micro-ROS over Wi-Fi**,
- Solid-state 3D matrix perception via the **VL53L5CX ToF**, and
- The robust navigation, TF, and mapping algorithms of **ROS 2 Humble**,

the platform achieves autonomous driving, SLAM mapping, and closed-loop trajectory tracking with high reliability.

### Future Roadmap:
- [ ] **Onboard Single Board Computer:** Mount a Raspberry Pi 5 directly on the chassis to eliminate reliance on an external laptop.
- [ ] **Full Nav2 Stack Integration:** Load costmaps and autonomous path planning using the Regulated Pure Pursuit (RPP) controller.
- [ ] **MoveIt 2 Robotic Arm:** Mount a 3-DOF micro-manipulator on top of the chassis to expand from mobility into mobile manipulation!

---
*Created as part of the KushagraBot Autonomous Systems Architecture Portfolio.*
