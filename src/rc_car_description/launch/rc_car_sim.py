import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, TimerAction,
    OpaqueFunction, ExecuteProcess
)
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    pkg = get_package_share_directory("rc_car_description")

    use_rviz  = LaunchConfiguration("rviz").perform(context)
    world_arg = LaunchConfiguration("world").perform(context)
    x_pos     = LaunchConfiguration("x").perform(context)
    y_pos     = LaunchConfiguration("y").perform(context)
    z_pos     = LaunchConfiguration("z").perform(context)

    world_path = world_arg if world_arg else \
        os.path.join(pkg, "worlds", "cat_robotics.world")

    xacro_file = os.path.join(pkg, "urdf", "rc_car.urdf.xacro")
    robot_description = ParameterValue(
        Command(["xacro ", xacro_file]), value_type=str
    )

    # ── 1. Robot State Publisher ───────────────────────────────
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_description},
            {"use_sim_time": True},
        ],
    )

    # ── 2. Gazebo ──────────────────────────────────────────────
    gazebo = ExecuteProcess(
        cmd=["ign", "gazebo", "--verbose", "-r", world_path],
        output="screen",
    )

    # ── 3. Spawn robot (delayed 3s for Gazebo to start) ────────
    spawn = TimerAction(
        period=3.0,
        actions=[Node(
            package="ros_gz_sim",
            executable="create",
            arguments=[
                "-topic", "robot_description",
                "-name",  "rc_car",
                "-x", x_pos,
                "-y", y_pos,
                "-z", z_pos,
            ],
            output="screen",
        )],
    )

    # ── 4. ROS <-> Gazebo bridge ───────────────────────────────
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
            "/imu/data@sensor_msgs/msg/Imu[ignition.msgs.IMU",
            "/ackermann_steering_controller/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry",
            "/world/cat_robotics_world/pose/info@geometry_msgs/msg/PoseArray[ignition.msgs.Pose_V",

            # 3D LiDAR — PointCloud2
            '/lidar/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
            # IMU
            '/imu/data@sensor_msgs/msg/Imu[ignition.msgs.IMU',
            # Camera image
            '/camera/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image',
            # Camera info
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
        ],
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    # ── 5. Controllers ─────────────────────────────────────────
    joint_state_broadcaster = TimerAction(
        period=6.0,
        actions=[Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster",
                       "--controller-manager", "/controller_manager"],
            output="screen",
        )],
    )

    ackermann_controller = TimerAction(
        period=8.0,
        actions=[Node(
            package="controller_manager",
            executable="spawner",
            arguments=["ackermann_steering_controller",
                       "--controller-manager", "/controller_manager"],
            output="screen",
        )],
    )

    # ── 6. Static TF: map -> odom ──────────────────────────────
    map_to_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_to_odom",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
        parameters=[{"use_sim_time": True}],
    )

    # ── 7. Ground truth TF: odom -> base_footprint ─────────────
    ground_truth_tf = ExecuteProcess(
        cmd=["python3", "-c",
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, TransformStamped
from tf2_ros import TransformBroadcaster

class GzTF(Node):
    def __init__(self):
        super().__init__('gz_ground_truth_tf')
        self.br = TransformBroadcaster(self)
        self.rc_car_index = None
        self.create_subscription(PoseArray,
            '/world/cat_robotics_world/pose/info', self.cb, 10)

    def cb(self, msg):
        # Auto-detect rc_car index by finding the pose closest
        # to spawn position (-2.74, -0.46) on first message
        if self.rc_car_index is None:
            best = None
            best_dist = 9999
            for i, p in enumerate(msg.poses):
                d = ((p.position.x - (-2.74))**2 +
                     (p.position.y - (-0.46))**2) ** 0.5
                if d < best_dist:
                    best_dist = d
                    best = i
            if best_dist < 1.0:
                self.rc_car_index = best
                self.get_logger().info(
                    f'rc_car found at index {best} dist={best_dist:.3f}')
            return

        if len(msg.poses) <= self.rc_car_index:
            return

        p = msg.poses[self.rc_car_index]
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = p.position.x
        t.transform.translation.y = p.position.y
        t.transform.translation.z = 0.0
        t.transform.rotation = p.orientation
        self.br.sendTransform(t)

rclpy.init()
rclpy.spin(GzTF())
"""],
        output="screen",
    )

    # ── 8. RViz ────────────────────────────────────────────────
    rviz_config = os.path.join(pkg, "rviz", "rc_car.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    nodes = [
        robot_state_publisher,
        gazebo,
        spawn,
        bridge,
        joint_state_broadcaster,
        ackermann_controller,
        map_to_odom,
        ground_truth_tf,
    ]
    if use_rviz == "true":
        nodes.append(rviz_node)

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("rviz",  default_value="true",
            description="Launch RViz2"),
        DeclareLaunchArgument("world", default_value="",
            description="Path to SDF world"),
        DeclareLaunchArgument("x",     default_value="-2.74",
            description="Spawn X"),
        DeclareLaunchArgument("y",     default_value="-0.46",
            description="Spawn Y"),
        DeclareLaunchArgument("z",     default_value="0.1",
            description="Spawn Z"),
        OpaqueFunction(function=launch_setup),
    ])
