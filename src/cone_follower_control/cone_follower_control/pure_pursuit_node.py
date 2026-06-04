import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
import numpy as np
import math
from nav_msgs.msg import Path, Odometry
from fs_msgs.msg import ControlCommand
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker

class PurePursuitNode(Node):
    def __init__(self):
        super().__init__('pure_pursuit_node')
        
        # Parameters
        self.declare_parameter('wheelbase', 2.7)
        self.declare_parameter('lookahead_k', 0.5)
        self.declare_parameter('lookahead_min', 4.0)
        self.declare_parameter('lookahead_max', 10.0)
        self.declare_parameter('lookahead_curvature_k', 1.5)
        self.declare_parameter('lookahead_error_k', 0.2)
        self.declare_parameter('target_velocity', 3.0) 
        self.declare_parameter('steering_gain', -1.0)
        
        self.L = self.get_parameter('wheelbase').value
        self.K = self.get_parameter('lookahead_k').value
        self.Lmax = self.get_parameter('lookahead_max').value
        
        qos_profile = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        
        # State
        self.path = None
        self.current_pose = None
        self.current_vel = 0.0
        
        # Subscribers
        self.path_sub = self.create_subscription(Path, '/centerline', self.path_callback, qos_profile)
        self.odom_sub = self.create_subscription(Odometry, '/testing_only/odom', self.odom_callback, 10)
        
        # Publishers
        self.control_pub = self.create_publisher(ControlCommand, '/control_command', 10)
        self.marker_pub = self.create_publisher(Marker, '/lookahead_marker', 10)
        
        self.timer = self.create_timer(0.02, self.control_loop)
        self.get_logger().info('Pure Pursuit Node has been started.')

    def path_callback(self, msg: Path):
        self.path = msg

    def odom_callback(self, msg: Odometry):
        self.current_pose = msg.pose.pose
        self.current_vel = math.sqrt(msg.twist.twist.linear.x**2 + msg.twist.twist.linear.y**2)

    def get_yaw(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def control_loop(self):
        if self.path is None or self.current_pose is None: return

        # 1. Find Closest Point
        elat, kappa, idx_min = self.calculate_path_metrics()

        # 2. Calculate Adaptive Lookahead Distance
        ld_base = self.K * self.current_vel
        k_c = self.get_parameter('lookahead_curvature_k').value
        k_e = self.get_parameter('lookahead_error_k').value
        ld = (ld_base / (1.0 + k_c * kappa)) + min(2.0, k_e * elat)
        ld = max(self.get_parameter('lookahead_min').value, min(self.Lmax, ld))
        
        # 3. Find Target Point with Interpolation
        target_pt = self.find_interpolated_target(ld, idx_min)
        if target_pt is None:
            # self.get_logger().warn('No target point found')
            return

        # 4. Visualization
        self.publish_markers(target_pt[0], target_pt[1], self.path.poses[idx_min].pose.position)

        # 5. Steering Calculation
        vx, vy = self.current_pose.position.x, self.current_pose.position.y
        v_yaw = self.get_yaw(self.current_pose.orientation)
        dx, dy = target_pt[0] - vx, target_pt[1] - vy
        
        # Transform to local vehicle frame
        target_x_v = dx * math.cos(v_yaw) + dy * math.sin(v_yaw)
        target_y_v = -dx * math.sin(v_yaw) + dy * math.cos(v_yaw)
        
        # Use actual distance to target for more stable Pure Pursuit
        actual_ld = math.sqrt(target_x_v**2 + target_y_v**2)
        actual_ld = max(0.1, actual_ld) # Prevent div by zero
        
        steering_angle = math.atan2(2.0 * self.L * target_y_v, actual_ld**2)
        normalized_steering = max(-1.0, min(1.0, (steering_angle / 0.52) * self.get_parameter('steering_gain').value))
        
        # 6. Speed Control
        v_limit = self.get_parameter('target_velocity').value / (1.0 + 5.0 * kappa)
        speed_error = v_limit - self.current_vel
        
        control_msg = ControlCommand()
        control_msg.header.stamp = self.get_clock().now().to_msg()
        # Increased base throttle for simulator stability
        control_msg.throttle = min(1.0, 0.25 + 0.1 * speed_error) if speed_error > 0 else 0.0
        control_msg.brake = min(1.0, 0.3 * abs(speed_error)) if speed_error < -0.5 else 0.0
        control_msg.steering = float(normalized_steering)
        
        self.get_logger().info(f"CTRL: Steer: {control_msg.steering:.2f} | Thr: {control_msg.throttle:.2f} | Brk: {control_msg.brake:.2f} | Vel: {self.current_vel:.2f}", throttle_duration_sec=1.0)
        
        self.control_pub.publish(control_msg)

    def calculate_path_metrics(self):
        vx, vy = self.current_pose.position.x, self.current_pose.position.y
        idx_min = 0
        min_dist = float('inf')
        
        for i, pose in enumerate(self.path.poses):
            p = pose.pose.position
            dist = math.sqrt((p.x - vx)**2 + (p.y - vy)**2)
            if dist < min_dist:
                min_dist = dist
                idx_min = i
        
        kappa = 0.0
        if idx_min < len(self.path.poses) - 10:
            p1, p2, p3 = self.path.poses[idx_min].pose.position, self.path.poses[idx_min+5].pose.position, self.path.poses[idx_min+10].pose.position
            t1 = math.atan2(p2.y-p1.y, p2.x-p1.x)
            t2 = math.atan2(p3.y-p2.y, p3.x-p2.x)
            kappa = abs((t2 - t1 + math.pi) % (2*math.pi) - math.pi) / 2.0
            
        return min_dist, kappa, idx_min

    def find_interpolated_target(self, ld, idx_min):
        vx, vy = self.current_pose.position.x, self.current_pose.position.y
        v_yaw = self.get_yaw(self.current_pose.orientation)
        
        for i in range(idx_min, len(self.path.poses) - 1):
            p1 = self.path.poses[i].pose.position
            p2 = self.path.poses[i+1].pose.position
            
            d1 = math.sqrt((p1.x - vx)**2 + (p1.y - vy)**2)
            d2 = math.sqrt((p2.x - vx)**2 + (p2.y - vy)**2)
            
            if d1 <= ld <= d2:
                t = (ld - d1) / (d2 - d1) if (d2 != d1) else 0.0
                tx = p1.x + t * (p2.x - p1.x)
                ty = p1.y + t * (p2.y - p1.y)
                
                # Check if in front
                if ((tx - vx) * math.cos(v_yaw) + (ty - vy) * math.sin(v_yaw)) > 0:
                    return (tx, ty)
        
        # Fallback to point closest to lookahead distance in front
        best_pt = None
        min_diff = float('inf')
        for i in range(idx_min, len(self.path.poses)):
            p = self.path.poses[i].pose.position
            dx, dy = p.x - vx, p.y - vy
            if (dx * math.cos(v_yaw) + dy * math.sin(v_yaw)) > 0:
                dist = math.sqrt(dx**2 + dy**2)
                diff = abs(dist - ld)
                if diff < min_diff:
                    min_diff = diff
                    best_pt = (p.x, p.y)
        
        return best_pt

    def publish_markers(self, tx, ty, cp):
        now = self.get_clock().now().to_msg()
        
        # Target Point (Green)
        m1 = Marker()
        m1.header.frame_id, m1.header.stamp, m1.ns, m1.id = 'fsds/map', now, 'target', 0
        m1.type, m1.action = Marker.SPHERE, Marker.ADD
        m1.pose.position.x, m1.pose.position.y, m1.pose.position.z = tx, ty, 1.0 # Higher Z
        m1.scale.x = m1.scale.y = m1.scale.z = 0.6
        m1.color.g, m1.color.a = 1.0, 1.0
        self.marker_pub.publish(m1)
        
        # Closest Point (Yellow)
        m2 = Marker()
        m2.header.frame_id, m2.header.stamp, m2.ns, m2.id = 'fsds/map', now, 'closest', 1
        m2.type, m2.action = Marker.SPHERE, Marker.ADD
        m2.pose.position.x, m2.pose.position.y, m2.pose.position.z = cp.x, cp.y, 1.0 # Higher Z
        m2.scale.x = m2.scale.y = m2.scale.z = 0.5
        m2.color.r, m2.color.g, m2.color.a = 1.0, 1.0, 1.0
        self.marker_pub.publish(m2)

def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__': main()
