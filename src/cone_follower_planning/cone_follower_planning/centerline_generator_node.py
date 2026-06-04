import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
import numpy as np
from scipy.spatial import Delaunay
from cone_follower_msgs.msg import ConeArray, Cone
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
import math

class CenterlineGeneratorNode(Node):
    def __init__(self):
        super().__init__('centerline_generator_node')
        
        qos_profile = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        
        # State
        self.current_pose = None
        self.last_cones = None
        
        # Subscribers
        self.cone_sub = self.create_subscription(
            ConeArray,
            '/cones',
            self.cone_callback,
            qos_profile
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/testing_only/odom',
            self.odom_callback,
            10
        )
        
        # Publishers
        self.path_pub = self.create_publisher(
            Path, 
            '/centerline', 
            qos_profile
        )
        
        self.get_logger().info('Centerline Generator Node has been started.')

    def odom_callback(self, msg: Odometry):
        self.current_pose = msg.pose.pose
        # Continuously regenerate path to ensure it always starts at the vehicle
        if self.last_cones is not None:
            self.generate_path()

    def get_yaw(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def cone_callback(self, msg: ConeArray):
        self.last_cones = msg
        if self.current_pose is not None:
            self.generate_path()

    def generate_path(self):
        if self.last_cones is None or self.current_pose is None:
            return

        points = []
        colors = []
        for cone in self.last_cones.cones:
            points.append([cone.x, cone.y])
            colors.append(cone.color)
        
        points = np.array(points)
        try:
            tri = Delaunay(points)
        except:
            return

        midpoints = []
        edges = set()
        for simplex in tri.simplices:
            for i in range(3):
                edge = tuple(sorted((simplex[i], simplex[(i+1)%3])))
                edges.add(edge)
        
        for edge in edges:
            p1_idx, p2_idx = edge
            if (colors[p1_idx] == 'blue' and colors[p2_idx] == 'yellow') or \
               (colors[p1_idx] == 'yellow' and colors[p2_idx] == 'blue'):
                p1, p2 = points[p1_idx], points[p2_idx]
                dist = np.linalg.norm(p1 - p2)
                if 1.0 <= dist <= 15.0:
                    midpoints.append((p1 + p2) / 2.0)
        
        if not midpoints: return

        # DYNAMIC RE-SORTING: Always start path from closest point in front of car
        sorted_midpoints = []
        remaining_pts = [np.array(mp) for mp in midpoints]
        
        vx, vy = self.current_pose.position.x, self.current_pose.position.y
        v_yaw = self.get_yaw(self.current_pose.orientation)
        
        # Find best starting point (closest in front)
        best_start_idx = -1
        min_start_dist = float('inf')
        for i, pt in enumerate(remaining_pts):
            dx, dy = pt[0] - vx, pt[1] - vy
            lx = dx * math.cos(v_yaw) + dy * math.sin(v_yaw)
            dist = math.sqrt(dx**2 + dy**2)
            if lx > 0 and dist < min_start_dist:
                min_start_dist = dist
                best_start_idx = i
        
        if best_start_idx == -1:
            dists = [np.linalg.norm(p - np.array([vx, vy])) for p in remaining_pts]
            best_start_idx = np.argmin(dists)

        current_pt = remaining_pts.pop(best_start_idx)
        sorted_midpoints.append(current_pt)

        # Greedy sort the rest
        while remaining_pts:
            distances = [np.linalg.norm(current_pt - p) for p in remaining_pts]
            nearest_idx = np.argmin(distances)
            current_pt = remaining_pts.pop(nearest_idx)
            sorted_midpoints.append(current_pt)
        
        # Publish
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'fsds/map'
        for mp in sorted_midpoints:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x, pose.pose.position.y = float(mp[0]), float(mp[1])
            path_msg.poses.append(pose)
            
        self.path_pub.publish(path_msg)

def main(args=None):
    rclpy.init(args=args)
    node = CenterlineGeneratorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt: pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
