#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
import time
import math
import threading

class ObstacleAvoider(Node):
    def __init__(self):
        super().__init__('obstacle_avoider')
        
        self.cmd_pub = self.create_publisher(Twist, '/diff_drive_controller/cmd_vel_unstamped', 10)
        
        self.left_sub = self.create_subscription(Range, '/left_tof/range', self.sensor_cb, 10)
        self.right_sub = self.create_subscription(Range, '/right_tof/range', self.sensor_cb, 10)
        
        self.front_dist = 999.0
        
        # FSM States
        self.STATE_DRIVING = 0
        self.STATE_TURNING = 1
        
        self.state = self.STATE_DRIVING
        
        # Configuration
        self.STOP_DIST_M = 0.30
        self.FLAT_WALL_MARGIN_M = 0.005 # 5mm
        self.DRIVE_SPEED = 0.3
        self.TURN_SPEED = 0.5
        
        self.is_running = True
        self.thread = threading.Thread(target=self.control_loop)
        self.thread.start()
        
    def sensor_cb(self, msg):
        # Print every 10th message to not spam
        if not hasattr(self, 'msg_count'): self.msg_count = 0
        self.msg_count += 1
        if self.msg_count % 10 == 0:
            self.get_logger().info(f"Received ToF: {msg.range:.3f}m")
            
        if msg.range > 0:
            self.front_dist = msg.range
            
    def publish_twist(self, linear, angular):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.cmd_pub.publish(msg)
        
    def control_loop(self):
        self.get_logger().info("Starting Obstacle Avoider FSM...")
        time.sleep(1.0) # Wait for initial sensor readings
        
        while self.is_running and rclpy.ok():
            if self.state == self.STATE_DRIVING:
                if self.front_dist < self.STOP_DIST_M:
                    self.get_logger().warn(f"Obstacle detected! Dist: {self.front_dist:.3f}m")
                    self.publish_twist(0.0, 0.0)
                    self.state = self.STATE_TURNING
                else:
                    self.publish_twist(self.DRIVE_SPEED, 0.0)
                    
            elif self.state == self.STATE_TURNING:
                # Turn Right (negative angular velocity) to avoid the obstacle
                self.publish_twist(0.0, -self.TURN_SPEED)
                
                # Turn until the sensor sees clear space ahead
                if self.front_dist > (self.STOP_DIST_M + 0.15):
                    self.get_logger().info("Path clear! Resuming forward drive.")
                    self.publish_twist(0.0, 0.0)
                    time.sleep(0.5) # Pause to stop momentum before driving
                    self.state = self.STATE_DRIVING
                    
            time.sleep(0.05) # 20Hz control loop
            
def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoider()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.is_running = False
        node.thread.join()
        node.publish_twist(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
