import rclpy
from rclpy.node import Node
import numpy as np
from cone_follower_msgs.msg import Cone, ConeArray

class MockTrackPublisher(Node):
    def __init__(self):
        super().__init__('mock_track_publisher')
        self.publisher_ = self.create_publisher(ConeArray, '/cones', 10)
        self.timer = self.create_timer(1.0, self.publish_mock_track)
        self.get_logger().info('Mock Track Publisher has been started.')

    def publish_mock_track(self):
        msg = ConeArray()
        
        # Curved track
        for i in range(20):
            angle = float(i) * 0.2
            radius = 10.0
            
            # Center of the curve
            cx = radius * np.sin(angle)
            cy = radius * (1.0 - np.cos(angle))
            
            # Normal vector for track width (perpendicular to tangent)
            # Tangent is (cos(angle), sin(angle)), so normal is (-sin(angle), cos(angle))
            nx = -np.sin(angle)
            ny = np.cos(angle)
            
            track_half_width = 2.0
            
            # Left cones (blue) - shifted "outward" or "inward"
            left_cone = Cone()
            left_cone.x = cx + track_half_width * nx
            left_cone.y = cy + track_half_width * ny
            left_cone.z = 0.0
            left_cone.color = 'blue'
            msg.cones.append(left_cone)
            
            # Right cones (yellow) - shifted opposite to left
            right_cone = Cone()
            right_cone.x = cx - track_half_width * nx
            right_cone.y = cy - track_half_width * ny
            right_cone.z = 0.0
            right_cone.color = 'yellow'
            msg.cones.append(right_cone)
            
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing curved mock track with {len(msg.cones)} cones.')

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
