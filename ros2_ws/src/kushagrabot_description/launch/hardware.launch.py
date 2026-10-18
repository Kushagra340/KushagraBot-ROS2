import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
import launch_ros.descriptions
import launch.substitutions
import xacro

def generate_launch_description():
    pkg_dir = get_package_share_directory('kushagrabot_description')

    # Process Xacro
    xacro_file = os.path.join(pkg_dir, 'urdf', 'kushagrabot.urdf.xacro')
    robot_description = {'robot_description': launch_ros.descriptions.ParameterValue(
        launch.substitutions.Command(['xacro ', xacro_file]), value_type=str)}

    # Controller Config
    controllers_file = os.path.join(pkg_dir, 'config', 'controllers.yaml')

    # Nodes
    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controllers_file],
        output='screen'
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )

    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller', '--controller-manager', '/controller_manager'],
    )

    return LaunchDescription([
        robot_state_pub_node,
        controller_manager,
        joint_state_broadcaster_spawner,
        diff_drive_controller_spawner,
    ])
