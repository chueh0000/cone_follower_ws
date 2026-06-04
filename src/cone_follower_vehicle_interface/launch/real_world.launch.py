import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    pkg_simulation = get_package_share_directory('cone_follower_simulation')
    
    # Launch arguments
    use_rviz = LaunchConfiguration('use_rviz')
    odom_topic = LaunchConfiguration('odom_topic')

    rviz_config_path = os.path.join(pkg_simulation, 'rviz', 'zed_perception.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz if true'
        ),
        DeclareLaunchArgument(
            'odom_topic',
            default_value='/zed/zed_node/odom',
            description='Topic for odometry input (e.g., from ZED SDK)'
        ),

        # 1. Perception: ZED YOLO TF Node
        Node(
            package='cone_follower_perception',
            executable='zed_yolo_tf_node',
            name='zed_yolo_tf',
            output='screen',
        ),

        # 2. Planning: Centerline Generator
        Node(
            package='cone_follower_planning',
            executable='centerline_generator_node',
            name='centerline_generator_node',
            output='screen',
            remappings=[('/testing_only/odom', odom_topic)]
        ),

        # 3. Control: Pure Pursuit
        Node(
            package='cone_follower_control',
            executable='pure_pursuit_node',
            name='pure_pursuit_node',
            output='screen',
            remappings=[('/testing_only/odom', odom_topic)]
        ),

        # 4. Vehicle Interface: SUV ECU Integration
        Node(
            package='cone_follower_vehicle_interface',
            executable='vehicle_interface_node',
            name='vehicle_interface_node',
            output='screen',
        ),

        # 5. Visualization: Marker Publisher
        Node(
            package='cone_follower_simulation',
            executable='cone_visualization_node',
            name='cone_visualization_node',
            output='screen',
        ),

        # 6. Static TF: world -> fsds/map (used by RViz config)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'fsds/map'],
            output='screen',
        ),

        # 7. RViz 2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],
            output='screen',
            condition=IfCondition(use_rviz)
        )
    ])
