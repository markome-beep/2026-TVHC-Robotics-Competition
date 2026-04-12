"""
display.launch.py
Visualise the RC car in RViz2 without Gazebo.
Good for quickly checking the URDF geometry.

  ros2 launch rc_car_description display.launch.py
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('rc_car_description')
    xacro_file  = os.path.join(pkg, 'urdf', 'rc_car.urdf.xacro')
    rviz_config = os.path.join(pkg, 'rviz', 'rc_car.rviz')

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': Command(['xacro ', xacro_file])}],
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
        ),
    ])
