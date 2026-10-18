#include <Arduino.h>
#include <WiFi.h>
#include <micro_ros_platformio.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32_multi_array.h>

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <SparkFun_VL53L5CX_Library.h>
#include "driver/pcnt.h" // Hardware pulse counter!
#include <ArduinoOTA.h>

#include <sensor_msgs/msg/imu.h>

// ==========================================
// ROS 2 OBJECTS
// ==========================================
rcl_subscription_t left_cmd_subscriber;
std_msgs__msg__Float32 left_cmd_msg;
rcl_subscription_t right_cmd_subscriber;
std_msgs__msg__Float32 right_cmd_msg;

rcl_publisher_t left_enc_pub;
rcl_publisher_t right_enc_pub;
rcl_publisher_t tof_pub;
rcl_publisher_t imu_pub;

std_msgs__msg__Int32 left_enc_msg;
std_msgs__msg__Int32 right_enc_msg;
std_msgs__msg__Float32MultiArray tof_msg;
sensor_msgs__msg__Imu imu_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop(){
  while(1){
    delay(100); 
  }
}

// ==========================================
// SENSOR CONFIGURATION (DUAL ToF & IMU)
// ==========================================
Adafruit_MPU6050 mpu;

// VL53L5CX Setup
#define TOF_EN_PIN 18
#define TOF_INT_PIN 19

SparkFun_VL53L5CX myImager;
VL53L5CX_ResultsData measurementData;

bool tof_connected = false;
volatile bool tof_data_ready = false;

// Hardware Interrupt ISR for ToF
void IRAM_ATTR tof_isr() {
  tof_data_ready = true;
}

const int STOP_DISTANCE_MM = 150; // Obstacle trigger distance
bool obstacle_detected = false;

// ==========================================
// SAFETY WATCHDOG (DEAD MAN'S SWITCH)
// ==========================================
unsigned long last_cmd_time = 0;
const unsigned long CMD_TIMEOUT_MS = 500; // Stop if no command for 0.5 seconds

// ==========================================
// MOTOR CONFIGURATION (DRV8833)
// ==========================================
// Swapped FWD and REV to fix reversed driving direction!
const int LEFT_FWD = 26;
const int LEFT_REV = 25;
const int RIGHT_FWD = 14;
const int RIGHT_REV = 27;

// ESP32 PWM Channels
const int PWM_FREQ = 5000;
const int PWM_RES = 8; // 0-255
const int CH_LEFT_FWD = 0;
const int CH_LEFT_REV = 1;
const int CH_RIGHT_FWD = 2;
const int CH_RIGHT_REV = 3;

// ==========================================
// ENCODER & KINEMATICS CONFIGURATION
// ==========================================
const int LEFT_ENC_PIN = 33; 
const int LEFT_ENC_OUTB = 34; // For hardware quadrature
const int RIGHT_ENC_PIN = 32;
const int RIGHT_ENC_OUTB = 35; // For hardware quadrature

volatile long left_ticks = 0;
volatile long right_ticks = 0;
// PCNT natively tracks direction, but we keep these for debugging
int left_motor_dir = 1;  
int right_motor_dir = 1;
const float ENCODER_CPR = 353.0; 
float esp32_yaw = 0.0; // Global Absolute Yaw

volatile float global_tof_array[64];

// Mutex for safe Core 0 to Core 1 data sharing
SemaphoreHandle_t sensorMutex;

// FreeRTOS Task Handle
TaskHandle_t SensorTask;

// ==========================================
// SENSOR TASK (RUNS ON CORE 0 AT STRICT 100Hz)
// ==========================================
void sensorTaskCode(void * pvParameters) {
  unsigned long last_time = micros();
  for(;;) {
    unsigned long current_time = micros();
    float dt = (current_time - last_time) / 1000000.0;
    last_time = current_time;

    // Read IMU
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    float gz_deg = g.gyro.z * 57.2958;
    if (abs(gz_deg) < 1.5) gz_deg = 0.0; // Deadzone

    // Safely update globals
    xSemaphoreTake(sensorMutex, portMAX_DELAY);
    esp32_yaw += gz_deg * dt;
    xSemaphoreGive(sensorMutex);

    // Read VL53L5CX ToF Sensor
    if (tof_connected && tof_data_ready) {
      if (myImager.getRangingData(&measurementData)) {
        xSemaphoreTake(sensorMutex, portMAX_DELAY);
        for(int i = 0; i < 64; i++) {
          if(measurementData.target_status[i] == 5 || measurementData.target_status[i] == 9) {
            global_tof_array[i] = measurementData.distance_mm[i] / 1000.0; // Meters
          } else {
            global_tof_array[i] = -1.0; // Invalid / Out of Range
          }
        }
        xSemaphoreGive(sensorMutex);
      }
      tof_data_ready = false; // Reset interrupt flag
    }

    // Strict 10ms delay (100Hz)
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

// ==========================================
// PID CONTROLLER
// ==========================================
float target_left_speed = 0.0; // rad/s
float target_right_speed = 0.0; // rad/s

float current_left_speed = 0.0;
float current_right_speed = 0.0;

// PID Tuning Parameters
// We have a Feed-Forward + Deadband term taking care of the heavy lifting.
// High Kp/Kd causes high-frequency 20Hz vibration due to encoder quantization!
float Kp = 5.0;
float Ki = 20.0;
float Kd = 0.0;

float integral_left = 0.0;
float integral_right = 0.0;
float prev_error_left = 0.0;
float prev_error_right = 0.0;

// ==========================================
// MOTOR CONTROL LOGIC
// ==========================================
void apply_pwm(int ch_fwd, int ch_rev, int pwm_val) {
  if (pwm_val > 0) { 
    ledcWrite(ch_fwd, constrain(pwm_val, 0, 255));
    ledcWrite(ch_rev, 0);
  } else if (pwm_val < 0) { 
    ledcWrite(ch_fwd, 0);
    ledcWrite(ch_rev, constrain(-pwm_val, 0, 255));
  } else { 
    ledcWrite(ch_fwd, 0);
    ledcWrite(ch_rev, 0);
  }
}

// ==========================================
// ROS 2 CALLBACKS
// ==========================================
void left_cmd_callback(const void * msgin) {
  last_cmd_time = millis(); 
  const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
  target_left_speed = msg->data;
  
  if (target_left_speed > 0.0) left_motor_dir = 1;
  else if (target_left_speed < 0.0) left_motor_dir = -1;
}

void right_cmd_callback(const void * msgin) {
  last_cmd_time = millis(); 
  const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
  target_right_speed = msg->data;
  
  if (target_right_speed > 0.0) right_motor_dir = 1;
  else if (target_right_speed < 0.0) right_motor_dir = -1;
}

// ==========================================
// INITIALIZATION FUNCTIONS
// ==========================================
void init_tof() {
  Serial.println("Starting VL53L5CX Initialization...");
  
  pinMode(TOF_EN_PIN, OUTPUT);
  digitalWrite(TOF_EN_PIN, LOW); // Reset
  delay(10);
  digitalWrite(TOF_EN_PIN, HIGH); // Enable
  delay(100);
  
  if (myImager.begin()) {
    Serial.println("VL53L5CX Initialized Successfully");
    // Set 8x8 resolution
    myImager.setResolution(8 * 8);
    // Enable Data Ready Interrupt
    pinMode(TOF_INT_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(TOF_INT_PIN), tof_isr, FALLING);
    myImager.startRanging();
    tof_connected = true;
  } else {
    Serial.println("CRITICAL ERROR: VL53L5CX failed to boot");
  }
}

// ==========================================
// MAIN SETUP
// ==========================================
void setup() {
  Serial.begin(115200);

  // 1. Configure WiFi & Micro-ROS Agent
  // ⚠️  Replace the values below with YOUR network credentials and agent IP before flashing!
  IPAddress agent_ip(192, 168, X, X);   // <- Your ROS 2 host machine IP
  set_microros_wifi_transports("YOUR_SSID", "YOUR_PASSWORD", agent_ip, 8888);

  ArduinoOTA.setHostname("KushagraBot");
  ArduinoOTA.begin();

  // 2. Hardware Safety
  pinMode(LEFT_FWD, OUTPUT); digitalWrite(LEFT_FWD, LOW);
  pinMode(LEFT_REV, OUTPUT); digitalWrite(LEFT_REV, LOW);
  pinMode(RIGHT_FWD, OUTPUT); digitalWrite(RIGHT_FWD, LOW);
  pinMode(RIGHT_REV, OUTPUT); digitalWrite(RIGHT_REV, LOW);

  // 3. Setup Motor PWM Channels
  ledcSetup(CH_LEFT_FWD, PWM_FREQ, PWM_RES);
  ledcAttachPin(LEFT_FWD, CH_LEFT_FWD);
  ledcSetup(CH_LEFT_REV, PWM_FREQ, PWM_RES);
  ledcAttachPin(LEFT_REV, CH_LEFT_REV);
  ledcSetup(CH_RIGHT_FWD, PWM_FREQ, PWM_RES);
  ledcAttachPin(RIGHT_FWD, CH_RIGHT_FWD);
  ledcSetup(CH_RIGHT_REV, PWM_FREQ, PWM_RES);
  ledcAttachPin(RIGHT_REV, CH_RIGHT_REV);

  apply_pwm(CH_LEFT_FWD, CH_LEFT_REV, 0);
  apply_pwm(CH_RIGHT_FWD, CH_RIGHT_REV, 0);

  // Setup Hardware PCNT (Pulse Counter) for Encoders
  // USING HARDWARE QUADRATURE DECODING (OUT A and OUT B)
  pcnt_config_t pcnt_config_left = {
      .pulse_gpio_num = LEFT_ENC_PIN,
      .ctrl_gpio_num = LEFT_ENC_OUTB,
      .lctrl_mode = PCNT_MODE_KEEP,
      .hctrl_mode = PCNT_MODE_REVERSE,
      .pos_mode = PCNT_COUNT_INC,   
      .neg_mode = PCNT_COUNT_DEC,   
      .counter_h_lim = 32767,
      .counter_l_lim = -32768,
      .unit = PCNT_UNIT_0,
      .channel = PCNT_CHANNEL_0,
  };
  pcnt_unit_config(&pcnt_config_left);
  pcnt_set_filter_value(PCNT_UNIT_0, 100); 
  pcnt_filter_enable(PCNT_UNIT_0);
  pcnt_counter_pause(PCNT_UNIT_0);
  pcnt_counter_clear(PCNT_UNIT_0);
  pcnt_counter_resume(PCNT_UNIT_0);

  pcnt_config_t pcnt_config_right = {
      .pulse_gpio_num = RIGHT_ENC_PIN,
      .ctrl_gpio_num = RIGHT_ENC_OUTB,
      .lctrl_mode = PCNT_MODE_KEEP,
      .hctrl_mode = PCNT_MODE_REVERSE,
      .pos_mode = PCNT_COUNT_INC,   
      .neg_mode = PCNT_COUNT_DEC,   
      .counter_h_lim = 32767,
      .counter_l_lim = -32768,
      .unit = PCNT_UNIT_1,
      .channel = PCNT_CHANNEL_0,
  };
  pcnt_unit_config(&pcnt_config_right);
  pcnt_set_filter_value(PCNT_UNIT_1, 100); 
  pcnt_filter_enable(PCNT_UNIT_1);
  pcnt_counter_pause(PCNT_UNIT_1);
  pcnt_counter_clear(PCNT_UNIT_1);
  pcnt_counter_resume(PCNT_UNIT_1);

  // 4. Initialize I2C Sensors
  Wire.begin(21, 22);
  Wire.setTimeOut(150); // Prevent ESP32 from freezing if sensors are disconnected!
  if (!mpu.begin()) Serial.println("MPU6050 NOT FOUND!");
  
  init_tof(); // Start the new VL53L5CX
  
  // Initialize Sensor Mutex
  sensorMutex = xSemaphoreCreateMutex();

  // Create Sensor Task on Core 0 (PRO_CPU)
  // This isolates IMU polling from WiFi and ROS 2 spin delays!
  xTaskCreatePinnedToCore(
    sensorTaskCode,   // Task function
    "SensorTask",     // Name
    4096,             // Stack size
    NULL,             // Parameters
    1,                // Priority
    &SensorTask,      // Handle
    0                 // Core 0 (Core 1 is default for Arduino/WiFi)
  );

  delay(2000); // Give micro-ROS agent time to connect

  // 5. Setup micro-ROS Node and Entities
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "esp32_base_node", "", &support));

  RCCHECK(rclc_subscription_init_default(
    &left_cmd_subscriber, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/left_wheel_cmd"
  ));

  RCCHECK(rclc_subscription_init_default(
    &right_cmd_subscriber, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/right_wheel_cmd"
  ));

  RCCHECK(rclc_publisher_init_default(
    &left_enc_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "/left_ticks"
  ));

  RCCHECK(rclc_publisher_init_default(
    &right_enc_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "/right_ticks"
  ));

  RCCHECK(rclc_publisher_init_default(
    &tof_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "/tof_depth_array"
  ));

  RCCHECK(rclc_publisher_init_default(
    &imu_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "/imu/data_raw"
  ));

  // 6. Setup Executor (2 handles for 2 subscriptions)
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &left_cmd_subscriber, &left_cmd_msg, &left_cmd_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &right_cmd_subscriber, &right_cmd_msg, &right_cmd_callback, ON_NEW_DATA));

  // Init ToF message header for MultiArray
  tof_msg.data.capacity = 64;
  tof_msg.data.size = 64;
  tof_msg.data.data = (float*) malloc(tof_msg.data.capacity * sizeof(float));
  
  tof_msg.layout.dim.capacity = 1;
  tof_msg.layout.dim.size = 1;
  tof_msg.layout.dim.data = (std_msgs__msg__MultiArrayDimension*) malloc(sizeof(std_msgs__msg__MultiArrayDimension));
  
  tof_msg.layout.dim.data[0].label.data = (char*)"depth";
  tof_msg.layout.dim.data[0].label.size = strlen("depth");
  tof_msg.layout.dim.data[0].size = 64;
  tof_msg.layout.dim.data[0].stride = 64;
  tof_msg.layout.data_offset = 0;

  imu_msg.header.frame_id.data = (char*)"imu_link";
  imu_msg.header.frame_id.size = strlen(imu_msg.header.frame_id.data);

  last_cmd_time = millis();
}

// ==========================================
// MAIN LOOP
// ==========================================
void loop() {
  ArduinoOTA.handle();
  unsigned long current_time = millis();

  // 1. Command Watchdog
  if (current_time - last_cmd_time > CMD_TIMEOUT_MS) {
    target_left_speed = 0;
    target_right_speed = 0;
  }

  // 2. PID Velocity Control Loop (Run at ~20Hz)
  static unsigned long last_pid_time = 0;
  if (current_time - last_pid_time >= 50) {
    float dt = (current_time - last_pid_time) / 1000.0;
    last_pid_time = current_time;

    // IMU is now integrated on Core 0 flawlessly at 100Hz!
    // We just read the hardware PCNT counters here.

    // --- READ HARDWARE PCNT COUNTERS ---
    int16_t count_l = 0;
    int16_t count_r = 0;
    pcnt_get_counter_value(PCNT_UNIT_0, &count_l);
    pcnt_get_counter_value(PCNT_UNIT_1, &count_r);
    pcnt_counter_clear(PCNT_UNIT_0);
    pcnt_counter_clear(PCNT_UNIT_1);
    
    // Accumulate ticks (direction is now handled perfectly in hardware!)
    // Note: If a wheel spins backwards relative to forward command, just swap its A/B wires physically.
    left_ticks += count_l;
    right_ticks += count_r;

    // Calculate actual speeds
    static long prev_left_ticks = 0;
    static long prev_right_ticks = 0;

    long d_left = left_ticks - prev_left_ticks;
    long d_right = right_ticks - prev_right_ticks;
    prev_left_ticks = left_ticks;
    prev_right_ticks = right_ticks;

    // Convert encoder counts per delta time into rad/s
    // Using global ENCODER_CPR
    float raw_left_speed = (d_left / ENCODER_CPR) * 2.0 * PI / dt;
    float raw_right_speed = (d_right / ENCODER_CPR) * 2.0 * PI / dt;
    
    // Low-Pass Filter (Exponential Moving Average) to kill quantization noise!
    current_left_speed = 0.7 * current_left_speed + 0.3 * raw_left_speed;
    current_right_speed = 0.7 * current_right_speed + 0.3 * raw_right_speed;

    // Feed-forward term (approximate PWM needed for given speed)
    float Kf = 25.0; 
    
    // PID Left
    float error_left = target_left_speed - current_left_speed;
    integral_left += error_left * dt;
    float derivative_left = (error_left - prev_error_left) / dt;
    int pwm_left = (Kf * target_left_speed) + (Kp * error_left) + (Ki * integral_left) + (Kd * derivative_left);
    prev_error_left = error_left;

    // PID Right
    float error_right = target_right_speed - current_right_speed;
    integral_right += error_right * dt;
    float derivative_right = (error_right - prev_error_right) / dt;
    int pwm_right = (Kf * target_right_speed) + (Kp * error_right) + (Ki * integral_right) + (Kd * derivative_right);
    prev_error_right = error_right;

    // Deadband compensation (Increased to 45 to overcome static friction and fix the 0.5s turn slip!)
    int deadband = 45;

    // Zero-point correction and Anti-Runaway Constraint
    if (target_left_speed > 0) {
      pwm_left += deadband;
      pwm_left = constrain(pwm_left, 0, 255);
    } else if (target_left_speed < 0) {
      pwm_left -= deadband;
      pwm_left = constrain(pwm_left, -255, 0);
    } else {
      pwm_left = 0;
      integral_left = 0;
    }

    if (target_right_speed > 0) {
      pwm_right += deadband;
      pwm_right = constrain(pwm_right, 0, 255);
    } else if (target_right_speed < 0) {
      pwm_right -= deadband;
      pwm_right = constrain(pwm_right, -255, 0);
    } else {
      pwm_right = 0;
      integral_right = 0;
    }

    apply_pwm(CH_LEFT_FWD, CH_LEFT_REV, pwm_left);
    apply_pwm(CH_RIGHT_FWD, CH_RIGHT_REV, pwm_right);
  }

  // 3. ToF Sensor Polling & Obstacle Safety
  static unsigned long last_tof_pub_time = 0;
  if (current_time - last_tof_pub_time >= 100) {
    last_tof_pub_time = current_time;
    obstacle_detected = false;

    xSemaphoreTake(sensorMutex, portMAX_DELAY);
    for(int i = 0; i < 64; i++) {
        tof_msg.data.data[i] = global_tof_array[i];
    }
    xSemaphoreGive(sensorMutex);

    if (tof_connected) RCSOFTCHECK(rcl_publish(&tof_pub, &tof_msg, NULL));
  }

  // 4. Publish Encoders and IMU
  static unsigned long last_pub_time = 0;
  if (current_time - last_pub_time > 100) { 
    left_enc_msg.data = left_ticks;
    right_enc_msg.data = right_ticks;
    RCSOFTCHECK(rcl_publish(&left_enc_pub, &left_enc_msg, NULL));
    RCSOFTCHECK(rcl_publish(&right_enc_pub, &right_enc_msg, NULL));

    // IMU
    xSemaphoreTake(sensorMutex, portMAX_DELAY);
    imu_msg.angular_velocity.x = 0.0;
    imu_msg.angular_velocity.y = 0.0;
    imu_msg.angular_velocity.z = esp32_yaw; 
    xSemaphoreGive(sensorMutex);
    
    RCSOFTCHECK(rcl_publish(&imu_pub, &imu_msg, NULL));

    last_pub_time = current_time;
  }

  // Spin ROS Executor
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
}