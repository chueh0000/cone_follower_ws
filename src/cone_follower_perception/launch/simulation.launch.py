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
                {'target_frame': 'fsds/FSCar'}, # Typically FSDS vehicle frame
                {'confidence_threshold': 0.5},
                {'yolo_model_path': '/home/b10902076/cone_follower_ws/runs/detect/fsoco_model/weights/best.pt'},
            ],
            remappings=[
                ('/camera/rgb/image_rect_color', '/fsds/cam1/image_color'),
                ('/camera/depth/depth_registered', '/fsds/depth_cam/image_color'),
                ('/camera/rgb/camera_info', '/fsds/cam1/camera_info'),
            ]
        )
    ])
