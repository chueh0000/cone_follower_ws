import rclpy
from rclpy.node import Node
import pytest
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
from cone_follower_msgs.msg import ConeArray
from cv_bridge import CvBridge
import numpy as np

from cone_follower_perception.unified_perception_node import UnifiedPerceptionNode

class MockTestNode(Node):
    def __init__(self):
        super().__init__('mock_test_node')
        self.cone_array = None
        self.sub = self.create_subscription(
            ConeArray,
            '/cones',
            self.cone_cb,
            10)
        self.rgb_pub = self.create_publisher(Image, '/camera/rgb/image_rect_color', 10)
        self.depth_pub = self.create_publisher(Image, '/camera/depth/depth_registered', 10)
        self.info_pub = self.create_publisher(CameraInfo, '/camera/rgb/camera_info', 10)

    def cone_cb(self, msg):
        self.cone_array = msg

@pytest.fixture
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()

def test_node_initialization(ros_context):
    node = UnifiedPerceptionNode()
    assert node is not None
    assert node.get_name() == 'unified_perception_node'
    # Check if subscribers are created
    assert hasattr(node, 'rgb_sub')
    assert hasattr(node, 'depth_sub')
    assert hasattr(node, 'info_sub')
    # Check if publisher is created
    assert hasattr(node, 'cone_pub')

