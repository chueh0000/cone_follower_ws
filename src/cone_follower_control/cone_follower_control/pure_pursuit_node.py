import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
import numpy as np
import math
from nav_msgs.msg import Path, Odometry
from fs_msgs.msg import ControlCommand
from geometry_msgs.msg import PoseStamped

class PurePursuitNode(Node):
    def __init__(self):
        super().__init__('pure_pursuit_node')
        
        # Parameters
        self.declare_parameter('wheelbase', 2.7)
        self.declare_parameter('lookahead_k', 0.5)
        self.declare_parameter('lookahead_min', 2.0)
        self.declare_parameter('lookahead_max', 10.0)
        self.declare_parameter('lookahead_curvature_k', 1.0)
        self.declare_parameter('lookahead_error_k', 0.5)
        self.declare_parameter('target_velocity', 1.0) # m/s
        self.declare_parameter('max_steering', 1.0) # Normalized max
        self.declare_parameter('steering_gain', -1.0) # Inversion gain
        
        self.L = self.get_parameter('wheelbase').value
        self.K = self.get_parameter('lookahead_k').value
        self.Lmin = self.get_parameter('lookahead_min').value
        self.Lmax = self.get_parameter('lookahead_max').value
        self.v_target = self.get_parameter('target_velocity').value
        self.steering_gain = self.get_parameter('steering_gain').value
        
        # QoS Profile: Must match the publisher (Transient Local)
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
        
        # Control Loop Timer (50Hz)
        self.timer = self.create_timer(0.02, self.control_loop)
        
        self.get_logger().info('Pure Pursuit Node has been started.')

    def path_callback(self, msg: Path):
        self.get_logger().info(f'CONTROL: Received path with {len(msg.poses)} points.')
        self.path = msg

    def odom_callback(self, msg: Odometry):
        if self.current_pose is None:
            self.get_logger().info('CONTROL: Received first Odom message.')
        self.current_pose = msg.pose.pose
        self.current_vel = math.sqrt(msg.twist.twist.linear.x**2 + msg.twist.twist.linear.y**2)

    def get_yaw(self, q):
        # Quaternion to Yaw
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def control_loop(self):
        if self.path is None or self.current_pose is None:
            return

        # 1. Calculate Path Metrics (Lateral Error and Curvature)
        elat, kappa, idx_min = self.calculate_path_metrics()

        # 2. Calculate Adaptive Lookahead Distance
        # ld = (Kv * v) / (1 + Kc * kappa) + Ke * |elat|
        ld_base = self.K * self.current_vel
        k_c = self.get_parameter('lookahead_curvature_k').value
        k_e = self.get_parameter('lookahead_error_k').value
        
        ld_curv = ld_base / (1.0 + k_c * kappa)
        ld = ld_curv + (k_e * elat)
        
        # Clamp to bounds
        ld = max(self.Lmin, min(self.Lmax, ld))
        
        # 3. Find Target Point on Path (Forward Only)
        target_pt = self.find_target_point(ld, idx_min)
        if target_pt is None:
            return

        # 4. Transform target point to vehicle frame
        # Vehicle position
        vx = self.current_pose.position.x
        vy = self.current_pose.position.y
        v_yaw = self.get_yaw(self.current_pose.orientation)
        
        # Relative coordinates
        dx = target_pt[0] - vx
        dy = target_pt[1] - vy
        
        # Rotate to vehicle frame (X forward, Y left)
        # In FSDS/AirSim, it might be different, but let's assume standard ROS ENU for map
        # and vehicle frame X-forward.
        target_x_v = dx * math.cos(v_yaw) + dy * math.sin(v_yaw)
        target_y_v = -dx * math.sin(v_yaw) + dy * math.cos(v_yaw)
        
        # 4. Calculate steering angle
        # alpha is the angle between vehicle heading and lookahead vector
        # alpha = atan2(target_y_v, target_x_v)
        # In Pure Pursuit: delta = atan(2 * L * sin(alpha) / ld)
        # Since sin(alpha) = target_y_v / dist_to_target and we use ld as dist
        # delta = atan(2 * L * target_y_v / ld^2)
        
        steering_angle = math.atan2(2.0 * self.L * target_y_v, ld**2)
        
        # 5. Normalize steering for FSDS (-1 to 1)
        # We'll assume max steering angle is around 30 degrees (0.52 rad) for normalization
        # if the SUV allows more, this can be tuned.
        max_delta = 0.52 
        normalized_steering = (steering_angle / max_delta) * self.steering_gain
        normalized_steering = max(-1.0, min(1.0, normalized_steering))
        
        # 6. Simple Speed Control (P-controller on throttle)
        control_msg = ControlCommand()
        control_msg.header.stamp = self.get_clock().now().to_msg()
        
        # Basic throttle logic: if velocity < target, throttle up
        speed_error = self.v_target - self.current_vel
        if speed_error > 0:
            control_msg.throttle = min(1.0, 0.2 + 0.02 * speed_error)
            control_msg.brake = 0.0
        else:
            control_msg.throttle = 0.0
            control_msg.brake = min(1.0, 0.1 * abs(speed_error))
            
        control_msg.steering = float(normalized_steering)
        
        self.control_pub.publish(control_msg)
        self.get_logger().info(f'CONTROL: Ld: {ld:.2f} | Curv: {kappa:.3f} | Error: {elat:.2f} | Steer: {control_msg.steering:.2f}')

    def calculate_path_metrics(self):
        """
        Calculates lateral error and path curvature.
        Returns: (lateral_error, curvature, idx_min)
        """
        if self.path is None or len(self.path.poses) == 0:
            return 0.0, 0.0, 0

        vx = self.current_pose.position.x
        vy = self.current_pose.position.y

        # 1. Find closest point P1 and idx_min
        idx_min = 0
        min_dist = float('inf')
        for i, pose in enumerate(self.path.poses):
            dist = math.sqrt((pose.pose.position.x - vx)**2 + (pose.pose.position.y - vy)**2)
            if dist < min_dist:
                min_dist = dist
                idx_min = i
        
        elat = min_dist
        p1 = self.path.poses[idx_min].pose.position

        # 2. Find P2 and P3 for curvature calculation (Robust Change in Heading)
        # We accumulate distance along the path to find points at ~1.0m and ~2.0m ahead.
        p2_idx = -1
        p3_idx = -1
        d1 = 0.0
        d2 = 0.0

        # Find P2
        curr_idx = idx_min
        while curr_idx < len(self.path.poses) - 1:
            next_idx = curr_idx + 1
            dist = math.sqrt((self.path.poses[next_idx].pose.position.x - self.path.poses[curr_idx].pose.position.x)**2 +
                             (self.path.poses[next_idx].pose.position.y - self.path.poses[curr_idx].pose.position.y)**2)
            d1 += dist
            if d1 >= 1.0:
                p2_idx = next_idx
                break
            curr_idx = next_idx

        # Find P3
        if p2_idx != -1:
            curr_idx = p2_idx
            while curr_idx < len(self.path.poses) - 1:
                next_idx = curr_idx + 1
                dist = math.sqrt((self.path.poses[next_idx].pose.position.x - self.path.poses[curr_idx].pose.position.x)**2 +
                                 (self.path.poses[next_idx].pose.position.y - self.path.poses[curr_idx].pose.position.y)**2)
                d2 += dist
                if d2 >= 1.0:
                    p3_idx = next_idx
                    break
                curr_idx = next_idx

        # 3. Calculate Curvature
        kappa = 0.0
        if p3_idx != -1:
            p2 = self.path.poses[p2_idx].pose.position
            p3 = self.path.poses[p3_idx].pose.position

            # Headings of P1->P2 and P2->P3
            theta1 = math.atan2(p2.y - p1.y, p2.x - p1.x)
            theta2 = math.atan2(p3.y - p2.y, p3.x - p2.x)

            # Safe Angular Difference (Wrap to [-pi, pi])
            delta_theta = (theta2 - theta1 + math.pi) % (2 * math.pi) - math.pi
            
            # kappa = delta_theta / average_segment_length
            kappa = abs(delta_theta) / (0.5 * (d1 + d2))
        
        return elat, kappa, idx_min

    def find_target_point(self, ld, start_idx=0):
        if not self.path or not self.path.poses:
            return None
            
        vx = self.current_pose.position.x
        vy = self.current_pose.position.y
        
        best_pt = None
        min_dist_err = float('inf')
        
        # Search forward only from start_idx
        for i in range(start_idx, len(self.path.poses)):
            pose = self.path.poses[i]
            px = pose.pose.position.x
            py = pose.pose.position.y
            dist = math.sqrt((px - vx)**2 + (py - vy)**2)
            
            dist_err = abs(dist - ld)
            if dist_err < min_dist_err:
                min_dist_err = dist_err
                best_pt = (px, py)
                
        return best_pt

def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
