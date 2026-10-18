#!/usr/bin/env python3
import dataclasses
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
from nav_msgs.msg import Odometry
import tkinter as tk
from tkinter import ttk
import math
import threading
import time

def euler_from_quaternion(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    
    return roll_x, pitch_y, yaw_z

class SimplePID:
    def __init__(self, kp, ki, kd, min_val, max_val):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.min_val = min_val
        self.max_val = max_val
        
        self.integral = 0.0
        self.prev_error = 0.0
        self.last_time = time.time()
        
    def compute(self, error):
        current_time = time.time()
        dt = current_time - self.last_time
        if dt <= 0.0: dt = 0.001
        
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        
        # Anti-windup and clamp
        if output > self.max_val:
            output = self.max_val
            self.integral -= error * dt # Anti-windup
        elif output < self.min_val:
            output = self.min_val
            self.integral -= error * dt # Anti-windup
            
        self.prev_error = error
        self.last_time = current_time
        return output

from sensor_msgs.msg import Imu

class ShapeControllerNode(Node):
    def __init__(self):
        super().__init__('shape_controller')
        self.cmd_pub = self.create_publisher(Twist, '/diff_drive_controller/cmd_vel_unstamped', 10)
        
        # Subscribe directly to the IMU to bypass EKF tf tree conflicts!
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data_raw',
            self.imu_callback,
            10
        )
        
        # Also need wheel odometry for the forward distance!
        self.odom_sub = self.create_subscription(
            Odometry,
            '/diff_drive_controller/odom',
            self.odom_callback,
            10
        )
        
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.last_imu_time = None
        self.odom_received = False
        self.imu_received = False
        
        self.target_shape = None
        self.is_running = False
        self.thread = None

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.odom_received = True

    def imu_callback(self, msg):
        current_time = self.get_clock().now().nanoseconds / 1e9
        if self.last_imu_time is not None:
            dt = current_time - self.last_imu_time
            # 1. Read the ABSOLUTE Yaw that the ESP32 integrated internally!
            # This completely bypasses WiFi network jitter!
            raw_yaw = msg.angular_velocity.z
            
            # 2. Apply Arduino inversion and Calibration factor
            imu_calibration_factor = 1.0475
            self.current_yaw = raw_yaw * imu_calibration_factor
            
        self.last_imu_time = current_time
        self.imu_received = True

    def publish_twist(self, linear, angular):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.cmd_pub.publish(msg)

    def drive_distance(self, target_dist_m, tolerance_m=0.005):
        if not self.odom_received or not self.imu_received:
            self.get_logger().warn("Waiting for odometry or IMU...")
            return
            
        start_x = self.current_x
        start_y = self.current_y
        target_yaw = self.current_yaw
        
        while self.is_running:
            # Calculate actual Euclidean distance traveled
            dist_traveled = math.sqrt((self.current_x - start_x)**2 + (self.current_y - start_y)**2)
            error = target_dist_m - dist_traveled
            
            # INERTIA COMPENSATION: Stop 8cm early (up from 7cm)
            if error < tolerance_m + 0.08:
                self.publish_twist(0.0, 0.0)
                break
                
            # Creep in even slower (0.1 m/s) to minimize the coasting distance
            speed = max(0.1, min(0.6, error * 2.0))
            
            # 2. Heading Hold Correction (Arduino 'Kp_heading' logic)
            error_heading = target_yaw - self.current_yaw
            
            # P-controller for heading (0.05 rad/s correction per degree of error)
            angular_correction = error_heading * 0.05
            
            self.publish_twist(speed, angular_correction)
            time.sleep(0.05)
            
    def turn_angle(self, target_angle_deg, tolerance_deg=10.0):
        if not self.imu_received:
            return
            
        start_yaw = self.current_yaw
        target_yaw = start_yaw + target_angle_deg
        
        while self.is_running:
            error = target_yaw - self.current_yaw
            
            # INERTIA COMPENSATION: Stop 12.0 degrees early (up from 10.5)
            if abs(error) < tolerance_deg + 12.0:
                self.get_logger().info(f"Target reached! Coasting the last few degrees. Stopping.")
                self.publish_twist(0.0, 0.0)
                break
                
            # Min speed 0.3 ensures smooth, accurate stopping with less inertia.
            speed = max(0.3, min(1.0, abs(error) * 0.03))
            if error < 0: speed = -speed
            self.publish_twist(0.0, speed)
            time.sleep(0.05)

    def execute_square(self):
        self.is_running = True
        for _ in range(4):
            if not self.is_running: break
            self.get_logger().info("Square: Driving forward 30cm")
            self.drive_distance(0.3)
            time.sleep(0.5)
            if not self.is_running: break
            self.get_logger().info("Square: Turning 90 degrees")
            self.turn_angle(90)
            time.sleep(0.5)
        self.is_running = False

    def execute_circle(self):
        self.is_running = True
        if not self.odom_received: return
        start_yaw = self.current_yaw
        
        # Drive in arc until yaw completes 360 (with some padding for exact stop)
        accumulated_yaw = 0
        last_yaw = start_yaw
        
        self.get_logger().info("Circle: Executing")
        while self.is_running:
            self.publish_twist(0.2, 0.5) # v = 0.2m/s, w = 0.5rad/s -> r = 0.4m
            time.sleep(0.05)
            
            dyaw = self.current_yaw - last_yaw
            if dyaw > 180: dyaw -= 360
            elif dyaw < -180: dyaw += 360
            accumulated_yaw += dyaw
            last_yaw = self.current_yaw
            
            if abs(accumulated_yaw) >= 358: # Stop just shy to account for momentum
                break
                
        self.publish_twist(0.0, 0.0)
        self.is_running = False

    def execute_figure_eight(self):
        self.is_running = True
        if not self.odom_received: return
        
        self.get_logger().info("Figure-8: Loop 1 (Left)")
        # Left circle
        accumulated_yaw = 0
        last_yaw = self.current_yaw
        while self.is_running and abs(accumulated_yaw) < 358:
            self.publish_twist(0.2, 0.5)
            time.sleep(0.05)
            dyaw = self.current_yaw - last_yaw
            if dyaw > 180: dyaw -= 360
            elif dyaw < -180: dyaw += 360
            accumulated_yaw += dyaw
            last_yaw = self.current_yaw
            
        if not self.is_running: return
        self.publish_twist(0.0, 0.0)
        time.sleep(0.5)
        
        self.get_logger().info("Figure-8: Loop 2 (Right)")
        # Right circle
        accumulated_yaw = 0
        last_yaw = self.current_yaw
        while self.is_running and abs(accumulated_yaw) < 358:
            self.publish_twist(0.2, -0.5)
            time.sleep(0.05)
            dyaw = self.current_yaw - last_yaw
            if dyaw > 180: dyaw -= 360
            elif dyaw < -180: dyaw += 360
            accumulated_yaw += dyaw
            last_yaw = self.current_yaw
            
        self.publish_twist(0.0, 0.0)
        self.is_running = False

class AppGUI:
    def __init__(self, ros_node):
        self.node = ros_node
        self.root = tk.Tk()
        self.root.title("KushagraBot Precision Shape Controller")
        self.root.geometry("400x300")
        
        style = ttk.Style()
        style.theme_use('clam')
        
        ttk.Label(self.root, text="Select Shape to Execute", font=('Helvetica', 16, 'bold')).pack(pady=20)
        
        self.btn_square = ttk.Button(self.root, text="Square (30cm x 90°)", command=self.start_square)
        self.btn_square.pack(fill='x', padx=50, pady=5)
        
        self.btn_circle = ttk.Button(self.root, text="Circle", command=self.start_circle)
        self.btn_circle.pack(fill='x', padx=50, pady=5)
        
        self.btn_fig8 = ttk.Button(self.root, text="Figure-8", command=self.start_fig8)
        self.btn_fig8.pack(fill='x', padx=50, pady=5)
        
        self.btn_stop = ttk.Button(self.root, text="EMERGENCY STOP", command=self.stop_robot)
        self.btn_stop.pack(fill='x', padx=50, pady=20)
        
        self.status_var = tk.StringVar()
        self.status_var.set("Status: Idle | Waiting for Sensors...")
        ttk.Label(self.root, textvariable=self.status_var, font=('Helvetica', 10, 'italic')).pack(side='bottom', pady=10)
        
        self.update_status()

    def update_status(self):
        if self.node.is_running:
            self.status_var.set(f"Status: Executing... | Yaw: {self.node.current_yaw:.1f}°")
        elif self.node.odom_received and self.node.imu_received:
            self.status_var.set(f"Status: Ready | Yaw: {self.node.current_yaw:.1f}°")
        else:
            self.status_var.set("Status: Waiting for Odom & IMU...")
        self.root.after(100, self.update_status)

    def start_square(self):
        if self.node.is_running or not (self.node.odom_received and self.node.imu_received): return
        self.node.thread = threading.Thread(target=self.node.execute_square)
        self.node.thread.start()

    def start_circle(self):
        if self.node.is_running or not (self.node.odom_received and self.node.imu_received): return
        self.node.thread = threading.Thread(target=self.node.execute_circle)
        self.node.thread.start()

    def start_fig8(self):
        if self.node.is_running or not (self.node.odom_received and self.node.imu_received): return
        self.node.thread = threading.Thread(target=self.node.execute_figure_eight)
        self.node.thread.start()

    def stop_robot(self):
        self.node.is_running = False
        self.node.publish_twist(0.0, 0.0)

import signal
import sys

def main(args=None):
    rclpy.init(args=args)
    shape_node = ShapeControllerNode()
    
    executor_thread = threading.Thread(target=rclpy.spin, args=(shape_node,), daemon=True)
    executor_thread.start()
    
    app = AppGUI(shape_node)

    def signal_handler(sig, frame):
        print("\nCtrl+C detected! Shutting down...")
        shape_node.is_running = False
        shape_node.publish_twist(0.0, 0.0)
        app.root.quit()
        app.root.destroy()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    
    # Check for signals periodically so Tkinter doesn't block them
    def check():
        app.root.after(100, check)
    app.root.after(100, check)
    
    app.root.mainloop()
    
    shape_node.is_running = False
    rclpy.shutdown()

if __name__ == '__main__':
    main()
