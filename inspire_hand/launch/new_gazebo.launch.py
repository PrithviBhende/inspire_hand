import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    control_yaml = '/home/psb/ros2_ws/src/inspire_hand/config/control.yaml'
    urdf_path = '/home/psb/ros2_ws/src/inspire_hand/urdf/inspire_hand_left.urdf'

    return LaunchDescription([
        # Publishes TF for links of the robot without joints
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            arguments=[urdf_path]
        ),

        # Publishes TF for joints only links
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
        ),

        # Start Controller Manager
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            name='controller_manager',
            output='screen',
            parameters=[control_yaml]
        ),

        # Spawn controllers
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'],
            output='screen',
        ),

        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_trajectory_controller'],
            output='screen',
        ),

        # Initialize Joint Positions
        Node(
            package='inspire_hand',
            executable='joint_initializer',
            name='joint_initializer',
            output='screen'
        ),
    ])
