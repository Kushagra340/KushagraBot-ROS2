import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. Custom Node: Convert 64-point array to PointCloud2
        Node(
            package='kushagrabot_description',
            executable='tof_pointcloud_node.py',
            name='tof_pointcloud_node',
            output='screen'
        ),
        
        # 2. Convert PointCloud2 to 2D LaserScan
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            remappings=[
                ('cloud_in', '/tof_pointcloud'),
                ('scan', '/scan')
            ],
            parameters=[{
                'target_frame': 'left_tof_link',
                'transform_tolerance': 0.01,
                'min_height': -1.0,
                'max_height': 1.0,
                'angle_min': -0.392, # -22.5 degrees (Half of 45 deg FOV)
                'angle_max': 0.392,  # +22.5 degrees
                'angle_increment': 0.098, # 45 degrees / 8 pixels = 5.625 deg
                'scan_time': 0.1,
                'range_min': 0.03,
                'range_max': 4.0,
                'use_inf': True,
                'inf_epsilon': 1.0
            }]
        ),
        
        # 3. SLAM Toolbox (Online Async Mode for best performance)
        Node(
            parameters=[
                {'odom_frame': 'odom'},
                {'map_frame': 'map'},
                {'base_frame': 'base_link'},
                {'scan_topic': '/scan'},
                {'use_sim_time': False},
                {'mode': 'mapping'}
            ],
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen'
        )
    ])
