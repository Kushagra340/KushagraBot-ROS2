#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class SafetyController(Node):
    def __init__(self):
        super().__init__('safety_controller')
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.left_tof_sub = self.create_subscription(LaserScan, '/left_tof/range', self.left_tof_callback, qos_profile_sensor_data)
        self.right_tof_sub = self.create_subscription(LaserScan, '/right_tof/range', self.right_tof_callback, qos_profile_sensor_data)
        
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_safe', 10)
        
        self.left_range = 1.0
        self.right_range = 1.0
        self.STOP_DISTANCE = 0.20 # 200mm limit as requested

    def left_tof_callback(self, msg):
        if len(msg.ranges) > 0:
            self.left_range = msg.ranges[0]

    def right_tof_callback(self, msg):
        if len(msg.ranges) > 0:
            self.right_range = msg.ranges[0]

    def cmd_callback(self, msg):
        out_msg = Twist()
        out_msg.angular.z = msg.angular.z
        
        # If moving forward and obstacle is too close, stop forward motion
        if msg.linear.x > 0.0 and (self.left_range < self.STOP_DISTANCE or self.right_range < self.STOP_DISTANCE):
            self.get_logger().warn('Obstacle closer than 200mm! Safety stop activated.')
            out_msg.linear.x = 0.0
        else:
            out_msg.linear.x = msg.linear.x
            
        self.cmd_pub.publish(out_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SafetyController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
