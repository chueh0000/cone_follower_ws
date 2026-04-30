import rclpy
from rclpy.node import Node
import numpy as np
from scipy.spatial import Delaunay
from cone_follower_msgs.msg import ConeArray, Cone
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class CenterlineGeneratorNode(Node):
    def __init__(self):
        super().__init__('centerline_generator_node')
        
        # Subscribers
        self.cone_sub = self.create_subscription(
            ConeArray,
            '/cones',
            self.cone_callback,
            10
        )
        
        # Publishers
        self.path_pub = self.create_publisher(Path, '/centerline', 10)
        
        self.get_logger().info('Centerline Generator Node has been started.')

    def cone_callback(self, msg: ConeArray):
        if len(msg.cones) < 3:
            return

        # Extract coordinates and colors
        points = []
        colors = []
        for cone in msg.cones:
            points.append([cone.x, cone.y])
            colors.append(cone.color)
        
        points = np.array(points)
        
        try:
            # Perform Delaunay Triangulation
            tri = Delaunay(points)
        except Exception as e:
            self.get_logger().error(f'Delaunay triangulation failed: {e}')
            return

        midpoints = []
        
        # Iterate through all edges of the triangles
        # tri.simplices contains indices of points forming each triangle
        edges = set()
        for simplex in tri.simplices:
            for i in range(3):
                edge = tuple(sorted((simplex[i], simplex[(i+1)%3])))
                edges.add(edge)
        
        for edge in edges:
            p1_idx, p2_idx = edge
            c1 = colors[p1_idx]
            c2 = colors[p2_idx]
            
            # Filter: only edges connecting blue and yellow cones
            if (c1 == 'blue' and c2 == 'yellow') or (c1 == 'yellow' and c2 == 'blue'):
                p1 = points[p1_idx]
                p2 = points[p2_idx]
                
                # Filter: edge length (avoid too long connections)
                dist = np.linalg.norm(p1 - p2)
                if 2.0 <= dist <= 10.0:  # Adjust thresholds as needed
                    midpoint = (p1 + p2) / 2.0
                    midpoints.append(midpoint)
        
        if not midpoints:
            return
            
        # Greedy sorting: start from origin (0,0) and pick the nearest next point
        sorted_midpoints = []
        current_pt = np.array([0.0, 0.0])
        remaining_pts = [np.array(mp) for mp in midpoints]
        
        while remaining_pts:
            # Find the index of the nearest point in remaining_pts
            distances = [np.linalg.norm(current_pt - p) for p in remaining_pts]
            nearest_idx = np.argmin(distances)
            
            # Move to the nearest point
            current_pt = remaining_pts.pop(nearest_idx)
            sorted_midpoints.append(current_pt)
        
        # Publish Path
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'
        
        for mp in sorted_midpoints:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = float(mp[0])
            pose.pose.position.y = float(mp[1])
            pose.pose.position.z = 0.0
            # Orientations are left at identity for now
            path_msg.poses.append(pose)
            
        self.path_pub.publish(path_msg)
        self.get_logger().info(f'Published centerline with {len(sorted_midpoints)} points.')

def main(args=None):
    rclpy.init(args=args)
    node = CenterlineGeneratorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
