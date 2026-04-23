import rclpy
from rclpy.node import Node
from cone_follower_msgs.msg import Cone, ConeArray

class MockTrackPublisher(Node):
    def __init__(self):
        super().__init__('mock_track_publisher')
        self.publisher_ = self.create_publisher(ConeArray, '/cones', 10)
        self.timer = self.create_timer(1.0, self.publish_mock_track)
        self.get_logger().info('Mock Track Publisher has been started.')

    def publish_mock_track(self):
        msg = ConeArray()
        
        # Simple straight track
        for i in range(10):
            # Left cones (blue)
            left_cone = Cone()
            left_cone.x = float(i * 2.0)
            left_cone.y = 2.0
            left_cone.z = 0.0
            left_cone.color = 'blue'
            msg.cones.append(left_cone)
            
            # Right cones (yellow)
            right_cone = Cone()
            right_cone.x = float(i * 2.0)
            right_cone.y = -2.0
            right_cone.z = 0.0
            right_cone.color = 'yellow'
            msg.cones.append(right_cone)
            
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing mock track with 20 cones.')

def main(args=None):
    rclpy.init(args=args)
    node = MockTrackPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
