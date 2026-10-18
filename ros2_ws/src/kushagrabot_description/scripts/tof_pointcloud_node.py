#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header
import math
import numpy as np

class TofPointCloudNode(Node):
    def __init__(self):
        super().__init__('tof_pointcloud_node')
        
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/tof_depth_array',
            self.depth_callback,
            10)
            
        self.publisher = self.create_publisher(PointCloud2, '/tof_pointcloud', 10)
        
        # VL53L5CX Parameters
        self.fov_deg = 45.0
        self.fov_rad = math.radians(self.fov_deg)
        self.grid_size = 8
        self.angle_per_pixel = self.fov_rad / self.grid_size

    def depth_callback(self, msg):
        if len(msg.data) != 64:
            self.get_logger().warn(f"Expected 64 elements, got {len(msg.data)}")
            return

        points = []
        
        # The VL53L5CX returns an 8x8 grid. 
        # Index = row * 8 + col
        for row in range(8):
            for col in range(8):
                idx = row * 8 + col
                distance = msg.data[idx]
                
                # -1.0 is our invalid/out-of-range marker
                if distance <= 0.0 or distance > 4.0:
                    continue
                    
                # Calculate angles (assuming center of grid is 0 degrees)
                # X axis is forward, Y is left, Z is up (ROS REP 103)
                angle_h = (3.5 - col) * self.angle_per_pixel
                angle_v = (3.5 - row) * self.angle_per_pixel
                
                # Trigonometry to project radial distance into Cartesian coordinates
                # In spherical coordinates where X is forward:
                x = distance * math.cos(angle_v) * math.cos(angle_h)
                y = distance * math.cos(angle_v) * math.sin(angle_h)
                z = distance * math.sin(angle_v)
                
                points.append([x, y, z])

        # Create PointCloud2 message
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'left_tof_link'
        
        pc2_msg = pc2.create_cloud_xyz32(header, points)
        self.publisher.publish(pc2_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TofPointCloudNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
