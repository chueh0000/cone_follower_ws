import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('cone_follower_perception')
    yolo_model_path = os.path.join(pkg_share, 'weights', 'yolov8n_fsoco.pt')
    
    return LaunchDescription([
        Node(
            package='cone_follower_perception',
            executable='unified_perception_node',
            name='unified_perception_node',
            output='screen',
            parameters=[
                {'target_frame': 'base_link'}, # ZED vehicle frame
                {'confidence_threshold': 0.5},
                {'yolo_model_path': yolo_model_path},
            ],
            remappings=[
                ('/camera/rgb/image_rect_color', '/zed/zed_node/rgb/image_rect_color'),
                ('/camera/depth/depth_registered', '/zed/zed_node/depth/depth_registered'),
                ('/camera/rgb/camera_info', '/zed/zed_node/rgb/camera_info'),
                ('/cones', '/perception/cones'),
            ]
        ),
        Node(
            package='cone_follower_perception',
            executable='local_map_node',
            name='local_map_node',
            output='screen',
            parameters=[
                {'vehicle_frame': 'base_link'},
                {'global_frame': 'odom'},
                {'merge_distance': 0.5},
                {'prune_distance_behind': 15.0},
            ],
            remappings=[
                ('/odom', '/zed/zed_node/odom')
            ]
        )
    ])
