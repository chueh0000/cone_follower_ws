import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_perception = get_package_share_directory('cone_follower_perception')
    pkg_planning = get_package_share_directory('cone_follower_planning')
    pkg_simulation = get_package_share_directory('cone_follower_simulation')
    
    rviz_config_path = os.path.join(pkg_simulation, 'rviz', 'zed_perception.rviz')

    return LaunchDescription([
        # ZED YOLO TF Node (Perception)
        Node(
            package='cone_follower_perception',
            executable='zed_yolo_tf_node',
            name='zed_yolo_tf',
            output='screen',
        ),
        # Centerline Generator (Planning)
        Node(
            package='cone_follower_planning',
            executable='centerline_generator_node',
            name='centerline_generator_node',
            output='screen',
        ),
        # Visualization Node (Markers for RViz)
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
