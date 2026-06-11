from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cone_follower_perception',
            executable='unified_perception_node',
            name='unified_perception_node',
            output='screen',
            parameters=[
                {'target_frame': 'base_link'},
                {'confidence_threshold': 0.5},
                {'yolo_model_path': '/home/b10902076/cone_follower_ws/runs/detect/fsoco_model/weights/best.pt'},
            ],
            remappings=[
                ('/camera/rgb/image_rect_color', '/zed/zed_node/rgb/image_rect_color'),
                ('/camera/depth/depth_registered', '/zed/zed_node/depth/depth_registered'),
                ('/camera/rgb/camera_info', '/zed/zed_node/rgb/camera_info'),
            ]
        )
    ])
