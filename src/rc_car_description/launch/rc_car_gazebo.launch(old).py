"""
Usage:
  ros2 launch rc_car_description cat_robotics.launch.py

Optional args:
  rviz:=true|false      (default: true)
  world:=<path>         (default: cat_robotics.world)
  x:=0 y:=0 z:=0.1     spawn pose
"""

import os
import subprocess
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription,
    TimerAction, OpaqueFunction, ExecuteProcess
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    pkg = get_package_share_directory('rc_car_description')
    
    use_rviz  = LaunchConfiguration('rviz').perform(context)
    world_arg = LaunchConfiguration('world').perform(context)
    x_pos     = LaunchConfiguration('x').perform(context)
    y_pos     = LaunchConfiguration('y').perform(context)
    z_pos     = LaunchConfiguration('z').perform(context)

    world_path = world_arg if world_arg else \
        os.path.join(pkg, 'worlds', 'cat_robotics.world')

    xacro_file = os.path.join(pkg, 'urdf', 'rc_car.urdf.xacro')
    
    # Expand xacro → URDF (for robot_state_publisher)
    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)
    
        
    # Path to your YAML
    config_file = os.path.join(pkg, 'config', 'ros2_control.yaml')

    # ── Robot State Publisher ──────────────────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': True},
        ],
    )


    # Launch Gazebo - Added the config file to the command
    gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', '--verbose', '-r', world_path], 
        output='screen',
    )

    # ── Spawn robot into Ignition ──────────────────────────────
    create_entity = TimerAction(
        period=3.0,
        actions=[Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-topic', 'robot_description',
                '-name', 'rc_car',
                '-allow_renaming', 'false',
                '-x', LaunchConfiguration('x'),
                '-y', LaunchConfiguration('y'),
                '-z', LaunchConfiguration('z'),
            ],
            output='screen',
        )],
    )

    # ── ROS-Ignition Bridge ────────────────────────────────────
    # Maps Ignition topics → ROS2 topics
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ign_ros_bridge',
        arguments=[
            # Clock
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            # Odometry89
            # '/ackermann_steering_controller/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry',

            # Ground truth poses (NEW)
            '/model/rc_car/pose@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',

            # 3D LiDAR — PointCloud2
            '/lidar/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
            # IMU
            '/imu/data@sensor_msgs/msg/Imu[ignition.msgs.IMU',
            # Camera image
            '/camera/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image',
            # Camera info
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
        ],
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    tf_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='tf_bridge',
        arguments=[
            '/model/rc_car/pose@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V'
        ],
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    ground_truth_tf = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ground_truth_tf',
        arguments=[
            '/model/rc_car/pose@geometry_msgs/msg/PoseStamped[ignition.msgs.Pose',
        ],
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # ── Controller spawners (delayed — wait for sim to load) ──
    joint_state_broadcaster = TimerAction(
        period=4.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
            output='screen',
            parameters=[{'use_sim_time': True}],
        )],
    )

    ackermann_controller = TimerAction(
        period=6.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=['ackermann_steering_controller', 
                '--controller-manager', '/controller_manager',
                '--ros-args',
                '--remap', 'ackermann_steering_controller/reference_unstamped:=/cmd_vel'],
            output='screen',
            parameters=[{'use_sim_time': True}],
        )],
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'odom0': '/ackermann_steering_controller/odometry',
            'odom0_config': [True, True, False,
                            False, False, True,
                            False, False, False,
                            False, False, True,
                            False, False, False],
            'publish_tf': True,
            'odom_frame': 'odom',
            'base_link_frame': 'base_footprint',
            'world_frame': 'odom',
            'two_d_mode': True,
        }],
    )

    cmd_vel_relay = Node(
        package='topic_tools',
        executable='relay',
        name='cmd_vel_to_ackermann',
        arguments=[
            '/cmd_vel',
            '/ackermann_steering_controller/reference_unstamped'
        ],
        output='screen'
    )

    # Bridge the rc_car pose from Gazebo as a ROS Pose message
    pose_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='pose_bridge',
        arguments=[
            '/world/cat_robotics_world/pose/info@geometry_msgs/msg/PoseArray[ignition.msgs.Pose_V',
        ],
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    odom_tf = Node(
        package='rclpy',
        executable='python3',
        arguments=['-c', '''
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import PoseStamped, TransformStamped
    from tf2_ros import TransformBroadcaster

    class R(Node):
        def __init__(self):
            super().__init__('ground_truth_tf')

            self.br = TransformBroadcaster(self)

            self.sub = self.create_subscription(
                PoseStamped,
                '/model/rc_car/pose',
                self.cb,
                10
            )

        def cb(self, msg):
            t = TransformStamped()
            t.header = msg.header
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_footprint'

            t.transform.translation.x = msg.pose.position.x
            t.transform.translation.y = msg.pose.position.y
            t.transform.translation.z = msg.pose.position.z

            t.transform.rotation = msg.pose.orientation

            self.br.sendTransform(t)

    rclpy.init()
    rclpy.spin(R())
    '''],
        output='screen'
    )

    # ── Static TF: map → odom (placeholder) ───────────────────
    static_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[{'use_sim_time': True}],
    )

    static_odom_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_base_footprint',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_footprint'],
        parameters=[{'use_sim_time': True}],
    )

    # ── RViz2 ─────────────────────────────────────────────────
    rviz_config = os.path.join(pkg, 'rviz', 'rc_car.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        condition=None if use_rviz == 'true' else None,
        output='screen',
    )

    nodes = [
        robot_state_publisher,
        gazebo,
        create_entity,
        bridge,
        tf_bridge,
        pose_bridge,
        joint_state_broadcaster,
        ackermann_controller,
        #cmd_vel_relay,
        #odom_tf,
        #ekf_node,
        static_map_odom,
        #static_odom_base
    ]
    if use_rviz == 'true':
        nodes.append(rviz_node)

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('rviz',  default_value='true',  description='Launch RViz2'),
        DeclareLaunchArgument('world', default_value='',      description='Path to SDF world'),
        DeclareLaunchArgument('x',     default_value='-2.74', description='Spawn X'),
        DeclareLaunchArgument('y',     default_value='-0.46', description='Spawn Y'),
        DeclareLaunchArgument('z',     default_value='0.1',   description='Spawn Z'),
        OpaqueFunction(function=launch_setup),
    ])
