import rclpy
from rclpy.node import Node
from cone_follower_msgs.msg import ConeArray
from visualization_msgs.msg import Marker, MarkerArray

class ConeVisualizationNode(Node):
    def __init__(self):
        super().__init__('cone_visualization_node')
        
        # Subscriber
        self.cone_sub = self.create_subscription(
            ConeArray,
            '/cones',
            self.cone_callback,
            10
        )
        
        # Publisher
        self.marker_pub = self.create_publisher(MarkerArray, '/cone_markers', 10)
        
        self.get_logger().info('Cone Visualization Node has been started.')

    def cone_callback(self, msg: ConeArray):
        marker_array = MarkerArray()
        
        for i, cone in enumerate(msg.cones):
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'cones'
            marker.id = i
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            
            # Position
            marker.pose.position.x = cone.x
            marker.pose.position.y = cone.y
            marker.pose.position.z = 0.25 # Center of cylinder height
            
            # Scale (standard FS cone is ~0.3m wide, 0.5m high)
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.5
            
            # Color
            if cone.color == 'blue':
                marker.color.r = 0.0
                marker.color.g = 0.0
                marker.color.b = 1.0
                marker.color.a = 1.0
            elif cone.color == 'yellow':
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.color.a = 1.0
            else: # Unknown color (white)
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 1.0
                marker.color.a = 1.0
                
            marker_array.markers.append(marker)
            
        self.marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = ConeVisualizationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
