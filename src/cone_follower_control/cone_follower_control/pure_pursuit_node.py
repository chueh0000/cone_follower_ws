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

        # 1. Calculate Adaptive Lookahead Distance
        ld = max(self.Lmin, min(self.Lmax, self.K * self.current_vel))
        
        # 2. Find Target Point on Path
        target_pt = self.find_target_point(ld)
        if target_pt is None:
            return

        # 3. Transform target point to vehicle frame
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
        self.get_logger().info(f'CONTROL: Published cmd - Steering: {control_msg.steering:.2f}, Throttle: {control_msg.throttle:.2f}')

    def find_target_point(self, ld):
        if not self.path.poses:
            return None
            
        vx = self.current_pose.position.x
        vy = self.current_pose.position.y
        
        best_pt = None
        min_dist_err = float('inf')
        
        for pose in self.path.poses:
            px = pose.pose.position.x
            py = pose.pose.position.y
            dist = math.sqrt((px - vx)**2 + (py - vy)**2)
            
            # We look for the point closest to the lookahead distance
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
