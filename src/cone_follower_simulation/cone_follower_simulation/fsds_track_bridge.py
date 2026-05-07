import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from fs_msgs.msg import Track as FSDSTrack
from cone_follower_msgs.msg import ConeArray, Cone

class FSDSTrackBridge(Node):
    def __init__(self):
        super().__init__('fsds_track_bridge')
        
        # QoS Profile: Transient Local is required to receive the one-time track publish
        qos_profile = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        
        # Subscriber
        self.track_sub = self.create_subscription(
            FSDSTrack,
            '/testing_only/track',
            self.track_callback,
            qos_profile
        )
        
        # Publisher: Also use Transient Local so late-joining nodes (like RViz) get the map
        self.cone_pub = self.create_publisher(
            ConeArray, 
            '/cones', 
            qos_profile
        )
        
        self.get_logger().info('FSDS Track Bridge has been started.')

    def track_callback(self, msg: FSDSTrack):
        self.get_logger().info(f'BRIDGE: Received track with {len(msg.track)} cones.')
        cone_array = ConeArray()
        
        for fsds_cone in msg.track:
            cone = Cone()
            # Map FSDS frame to our internal frame (using fsds/map as default)
            cone.x = fsds_cone.location.x
            cone.y = fsds_cone.location.y
            cone.z = fsds_cone.location.z
            
            # Map FSDS color enum to string
            # BLUE=0, YELLOW=1, ORANGE_BIG=2, ORANGE_SMALL=3, UNKNOWN=4
            if fsds_cone.color == 0:
                cone.color = 'blue'
            elif fsds_cone.color == 1:
                cone.color = 'yellow'
            elif fsds_cone.color in [2, 3]:
                cone.color = 'orange'
            else:
                cone.color = 'unknown'
                
            cone_array.cones.append(cone)
            
        self.cone_pub.publish(cone_array)
        self.get_logger().info(f'BRIDGE: Successfully published {len(cone_array.cones)} cones to /cones')

def main(args=None):
    rclpy.init(args=args)
    node = FSDSTrackBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
