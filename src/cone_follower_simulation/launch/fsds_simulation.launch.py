import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('cone_follower_simulation')
    bridge_share = get_package_share_directory('fsds_ros2_bridge')

    bridge_launch_path = os.path.join(bridge_share, 'launch', 'fsds_ros2_bridge.launch.py')

    use_camera_viz = LaunchConfiguration('use_camera_viz')

    rviz_config_path = PythonExpression([
        "'", os.path.join(pkg_share, 'rviz', 'cameras.rviz'), "' if '", 
        use_camera_viz, "' == 'true' else '", 
        os.path.join(pkg_share, 'rviz', 'fsds_config.rviz'), "'"
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_camera_viz',
            default_value='true',
            description='Use camera-focused RViz config if true, else use default FSDS config'
        ),
        # FSDS ROS 2 Bridge (Including cameras)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bridge_launch_path)
        ),
        # FSDS Track Bridge (Simulator -> ROS format)
        Node(
            package='cone_follower_simulation',
            executable='fsds_track_bridge',
            name='fsds_track_bridge',
            output='screen',
            remappings=[
                ('/testing_only/track', '/fsds/testing_only/track')
            ]
        ),
        # Centerline Generator
        Node(
            package='cone_follower_planning',
            executable='centerline_generator_node',
            name='centerline_generator_node',
            output='screen',
            remappings=[
                ('/testing_only/odom', '/fsds/testing_only/odom')
            ]
        ),
        # Pure Pursuit Controller
        Node(
            package='cone_follower_control',
            executable='pure_pursuit_node',
            name='pure_pursuit_node',
            output='screen',
            remappings=[
                ('/testing_only/odom', '/fsds/testing_only/odom'),
                ('/control_command', '/fsds/control_command')
            ]
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
