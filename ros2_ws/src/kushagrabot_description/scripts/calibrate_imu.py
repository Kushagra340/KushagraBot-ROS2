#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import threading
import time

class IMUCalibrator(Node):
    def __init__(self):
        super().__init__('imu_calibrator')
        self.subscription = self.create_subscription(
            Imu,
            '/imu/data_raw',
            self.imu_callback,
            10)
            
        self.current_yaw = 0.0
        self.last_imu_time = None
        self.is_running = True
        
    def imu_callback(self, msg):
        current_time = self.get_clock().now().nanoseconds / 1e9
        if self.last_imu_time is not None:
            # The ESP32 now sends the ABSOLUTE yaw in this field, bypassing network jitter!
            self.current_yaw = msg.angular_velocity.z
        self.last_imu_time = current_time

def interactive_loop(node):
    print("\n========================================")
    print("      INTERACTIVE IMU CALIBRATION       ")
    print("========================================")
    print("Keep the robot perfectly still on the floor.")
    time.sleep(2)
    
    trials = 5
    measured_angles = []
    
    for i in range(trials):
        print(f"\n--- TRIAL {i+1}/{trials} ---")
        
        # Wait for latest reading
        time.sleep(0.5)
        start_yaw = node.current_yaw
        print(f"Current absolute yaw: {start_yaw:.2f}")
        
        input("Physically rotate the robot EXACTLY 90 degrees to the LEFT, then press Enter...")
        
        # Wait for latest reading after rotation
        time.sleep(0.5)
        end_yaw = node.current_yaw
        
        # The angle turned is the difference
        yaw_diff = end_yaw - start_yaw
        print(f"IMU Registered: {yaw_diff:.2f} degrees")
        measured_angles.append(yaw_diff)
        
    print("\n========================================")
    print("          CALIBRATION COMPLETE          ")
    print("========================================")
    avg_registered = sum(measured_angles) / trials
    print(f"Average IMU Reading for 90-degree turns: {avg_registered:.2f} degrees")
    
    if avg_registered == 0:
        print("ERROR: IMU registered 0 degrees. Is the ROS 2 node running?")
    else:
        calibration_factor = 90.0 / avg_registered
        print(f"\n==> YOUR NEW imu_calibration_factor = {calibration_factor:.4f} <==")
        print("Copy this value into shape_controller.py!")
        
    node.is_running = False

def main(args=None):
    rclpy.init(args=args)
    calibrator = IMUCalibrator()
    
    thread = threading.Thread(target=interactive_loop, args=(calibrator,))
    thread.start()
    
    while calibrator.is_running:
        rclpy.spin_once(calibrator, timeout_sec=0.1)
        
    calibrator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
