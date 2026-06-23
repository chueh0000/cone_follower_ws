import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped, TransformStamped
from nav_msgs.msg import Odometry
from cone_follower_msgs.msg import ConeArray, Cone

class LocalMapNode(Node):
    def __init__(self):
        super().__init__('local_map_node')
        
        # Parameters
        self.declare_parameter('vehicle_frame', 'fsds/FSCar')
        self.declare_parameter('global_frame', 'fsds/map')
        self.declare_parameter('merge_distance', 0.5)
        self.declare_parameter('prune_distance_behind', 15.0)
        
        self.vehicle_frame = self.get_parameter('vehicle_frame').value
        self.global_frame = self.get_parameter('global_frame').value
        self.merge_distance = self.get_parameter('merge_distance').value
        self.prune_distance_behind = self.get_parameter('prune_distance_behind').value
        
        # State: Store cones in global_frame. Each entry is a dict: {'x': float, 'y': float, 'z': float, 'color': str}
        self.global_cones = []
        
        # TF2 (Private Buffer, NO listener to avoid /tf conflicts)
        self.tf_buffer = tf2_ros.Buffer()
        
        qos_profile = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        
        # Subscribers and Publishers
        self.cone_sub = self.create_subscription(
            ConeArray,
            '/perception/cones',
            self.cone_callback,
            qos_profile
        )
        
        self.cone_pub = self.create_publisher(
            ConeArray,
            '/cones',
            qos_profile
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        self.get_logger().info(f'Local Map Node initialized. Odometry-based TF override.')

    def odom_callback(self, msg: Odometry):
        # Inject odometry directly into our private TF buffer
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = self.global_frame
        t.child_frame_id = self.vehicle_frame
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        
        self.tf_buffer.set_transform(t, "odom_topic")

    def cone_callback(self, msg: ConeArray):
        if not msg.cones:
            # Still publish the local map even if nothing is seen currently
            self.publish_local_map()
            return
            
        try:
            # Transform from vehicle_frame to global_frame
            transform_v2g = self.tf_buffer.lookup_transform(
                self.global_frame,
                self.vehicle_frame,
                rclpy.time.Time()
            )
        except tf2_ros.TransformException as ex:
            self.get_logger().warn(f'Could not transform {self.vehicle_frame} to {self.global_frame}: {ex}')
            return
            
        # 1. Transform incoming cones to global frame and merge
        for incoming_cone in msg.cones:
            pt_veh = PointStamped()
            pt_veh.header.frame_id = self.vehicle_frame
            pt_veh.point.x = incoming_cone.x
            pt_veh.point.y = incoming_cone.y
            pt_veh.point.z = incoming_cone.z
            
            pt_global = tf2_geometry_msgs.do_transform_point(pt_veh, transform_v2g)
            
            # Check for duplicates in the global map
            is_duplicate = False
            for mapped_cone in self.global_cones:
                dist = math.sqrt((pt_global.point.x - mapped_cone['x'])**2 + (pt_global.point.y - mapped_cone['y'])**2)
                if dist < self.merge_distance:
                    is_duplicate = True
                    # Optionally average the position to refine it over time
                    mapped_cone['x'] = (mapped_cone['x'] + pt_global.point.x) / 2.0
                    mapped_cone['y'] = (mapped_cone['y'] + pt_global.point.y) / 2.0
                    break
                    
            if not is_duplicate:
                self.global_cones.append({
                    'x': pt_global.point.x,
                    'y': pt_global.point.y,
                    'z': pt_global.point.z,
                    'color': incoming_cone.color
                })
                
        # 2. Prune old cones and publish map in vehicle frame
        self.publish_local_map()
        
    def publish_local_map(self):
        try:
            # Transform from global_frame back to vehicle_frame
            transform_g2v = self.tf_buffer.lookup_transform(
                self.vehicle_frame,
                self.global_frame,
                rclpy.time.Time()
            )
        except tf2_ros.TransformException as ex:
            return
            
        output_msg = ConeArray()
        pruned_global_cones = []
        
        for mapped_cone in self.global_cones:
            pt_global = PointStamped()
            pt_global.header.frame_id = self.global_frame
            pt_global.point.x = mapped_cone['x']
            pt_global.point.y = mapped_cone['y']
            pt_global.point.z = mapped_cone['z']
            
            pt_veh = tf2_geometry_msgs.do_transform_point(pt_global, transform_g2v)
            
            # Prune cones that are too far behind the vehicle (negative X)
            if pt_veh.point.x < -self.prune_distance_behind:
                continue # Do not keep this cone in memory
                
            pruned_global_cones.append(mapped_cone)
            
            out_cone = Cone()
            out_cone.x = pt_veh.point.x
            out_cone.y = pt_veh.point.y
            out_cone.z = pt_veh.point.z
            out_cone.color = mapped_cone['color']
            
            output_msg.cones.append(out_cone)
            
        self.global_cones = pruned_global_cones
        self.cone_pub.publish(output_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LocalMapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
