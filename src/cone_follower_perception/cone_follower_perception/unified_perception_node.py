import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point
import message_filters
from cv_bridge import CvBridge
import numpy as np
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped

from cone_follower_msgs.msg import ConeArray, Cone
from cone_follower_perception.core_logic import YOLOInferenceWrapper, extract_depth_roi, project_pixel_to_3d

class UnifiedPerceptionNode(Node):
    def __init__(self):
        super().__init__('unified_perception_node')
        
        self.declare_parameter('yolo_model_path', 'yolov8n.pt')
        self.declare_parameter('target_frame', 'base_link')
        self.declare_parameter('confidence_threshold', 0.5)
        
        model_path = self.get_parameter('yolo_model_path').get_parameter_value().string_value
        self.target_frame = self.get_parameter('target_frame').get_parameter_value().string_value
        self.conf_thresh = self.get_parameter('confidence_threshold').get_parameter_value().double_value

        self.bridge = CvBridge()
        self.yolo_wrapper = YOLOInferenceWrapper(model_path)
        
        # TF2 Setup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Camera info setup
        self.camera_intrinsics = None
        self.info_sub = self.create_subscription(
            CameraInfo,
            '/camera/rgb/camera_info',
            self.camera_info_callback,
            10
        )

        # Message synchronization (RGB and Depth)
        self.rgb_sub = message_filters.Subscriber(self, Image, '/camera/rgb/image_rect_color')
        self.depth_sub = message_filters.Subscriber(self, Image, '/camera/depth/depth_registered')
        
        # Using ApproximateTimeSynchronizer to handle depth lag
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub], 
            queue_size=10, 
            slop=0.05
        )
        self.ts.registerCallback(self.sync_callback)

        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE
        )
        self.cone_pub = self.create_publisher(ConeArray, '/cones', qos_profile)
        self.get_logger().info("Unified Perception Node Initialized")

    def camera_info_callback(self, msg):
        # Extract intrinsics (fx, fy, cx, cy) from CameraInfo matrix K
        # K = [fx,  0, cx]
        #     [ 0, fy, cy]
        #     [ 0,  0,  1]
        self.camera_intrinsics = {
            'fx': msg.k[0],
            'fy': msg.k[4],
            'cx': msg.k[2],
            'cy': msg.k[5],
            'frame_id': msg.header.frame_id
        }
        # Unsubscribe after receiving first valid info (assume static intrinsics)
        self.destroy_subscription(self.info_sub)
        self.get_logger().info(f"Received camera intrinsics. Frame: {self.camera_intrinsics['frame_id']}")

    def sync_callback(self, rgb_msg, depth_msg):
        if self.camera_intrinsics is None:
            self.get_logger().warn("Camera intrinsics not yet received, skipping frame.", throttle_duration_sec=2.0)
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
            # Usually depth is 32FC1 or 16UC1
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        except Exception as e:
            self.get_logger().error(f"CV Bridge error: {e}")
            return
            
        # Convert depth to meters if it's in millimeters (16UC1)
        if depth_msg.encoding == '16UC1':
            depth_image = depth_image.astype(np.float32) / 1000.0

        detections = self.yolo_wrapper.detect(cv_image)
        
        cone_array_msg = ConeArray()

        try:
            # ROS 2 strictness: frame_ids cannot start with '/'
            source_frame = rgb_msg.header.frame_id.lstrip('/')
            
            # Look up transform from camera frame to target frame
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                source_frame,
                rclpy.time.Time() # Get latest available
            )
        except tf2_ros.TransformException as ex:
            self.get_logger().warn(f"Could not transform {rgb_msg.header.frame_id} to {self.target_frame}: {ex}", throttle_duration_sec=2.0)
            return

        for det in detections:
            if det['confidence'] < self.conf_thresh:
                continue
                
            bbox = det['bbox'] # [x_min, y_min, x_max, y_max]
            class_id = det['class_id']
            
            # Robust Depth Extraction
            depth = extract_depth_roi(depth_image, bbox, roi_ratio=0.5)
            
            if np.isnan(depth) or np.isinf(depth):
                continue
                
            # Use bottom-center of bounding box for ground plane projection
            x_min, y_min, x_max, y_max = bbox
            u_center = x_min + (x_max - x_min) / 2.0
            v_bottom = y_max
            
            # 3D Projection in Camera Frame
            x_cam, y_cam, z_cam = project_pixel_to_3d(
                u_center, v_bottom, depth,
                self.camera_intrinsics['fx'], self.camera_intrinsics['fy'],
                self.camera_intrinsics['cx'], self.camera_intrinsics['cy']
            )
            
            if np.isnan(x_cam):
                continue
                
            # Transform to Vehicle Frame
            pt_cam = PointStamped()
            pt_cam.header.frame_id = source_frame
            pt_cam.point.x = x_cam
            pt_cam.point.y = y_cam
            pt_cam.point.z = z_cam
            
            pt_base = tf2_geometry_msgs.do_transform_point(pt_cam, transform)
            
            cone = Cone()
            cone.x = pt_base.point.x
            cone.y = pt_base.point.y
            cone.z = pt_base.point.z
            
            # Map YOLO class_id to cone_follower_msgs color
            # Based on FSOCO Kaggle dataset: 0: yellow, 1: blue, 2: orange, 3: large_orange, 4: unknown
            if class_id == 0:
                cone.color = 'yellow'
            elif class_id == 1:
                cone.color = 'blue'
            elif class_id == 2 or class_id == 3:
                cone.color = 'orange'
            else:
                cone.color = 'unknown'
                
            cone_array_msg.cones.append(cone)
            
        self.cone_pub.publish(cone_array_msg)

def main(args=None):
    rclpy.init(args=args)
    node = UnifiedPerceptionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
