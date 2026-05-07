import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('cone_follower_simulation')
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'fsds_config.rviz')

    return LaunchDescription([
        # FSDS ROS 2 Bridge
        Node(
            package='fsds_ros2_bridge',
            executable='fsds_ros2_bridge',
            name='fsds_ros2_bridge',
            output='screen',
        ),
        # FSDS Track Bridge (Simulator -> ROS format)
        Node(
            package='cone_follower_simulation',
            executable='fsds_track_bridge',
            name='fsds_track_bridge',
            output='screen',
        ),
        # Centerline Generator
        Node(
            package='cone_follower_planning',
            executable='centerline_generator_node',
            name='centerline_generator_node',
            output='screen',
        ),
        # Pure Pursuit Controller
        Node(
            package='cone_follower_control',
            executable='pure_pursuit_node',
            name='pure_pursuit_node',
            output='screen',
        ),
        # Visualization Node
        Node(
            package='cone_follower_simulation',
            executable='cone_visualization_node',
            name='cone_visualization_node',
            output='screen',
        ),
        # Static Transform Publisher (world -> fsds/map)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'fsds/map'],
            output='screen',
        ),
        # RViz 2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],
            output='screen',
        )
    ])
