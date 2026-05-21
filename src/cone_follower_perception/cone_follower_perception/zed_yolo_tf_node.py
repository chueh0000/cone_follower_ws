#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, DurabilityPolicy

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from zed_msgs.msg import ObjectsStamped

from cv_bridge import CvBridge
import numpy as np

from cone_follower_msgs.msg import Cone, ConeArray

import cv2

class ZedYoloTF(Node):
    def __init__(self):
        super().__init__('zed_yolo_tf')

        #qos setting from cone follower centerline generator node
        qos_profile = QoSProfile( 
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        self.bridge = CvBridge()

        self.create_subscription(
            Image,
            '/zed/zed_node/left/color/rect/image',
            self.rgb_callback,
            10)

        self.create_subscription(
            CameraInfo,
            '/zed/zed_node/rgb/camera_info',
            self.caminfo_callback,
            10)
         
        self.create_subscription(
            ObjectsStamped,
            '/zed/zed_node/obj_det/objects',
            self.Obj_detect_callback,
            10)

        self.conePub = self.create_publisher(
            ConeArray,
            '/cones',
            qos_profile
        )

        self.rgb_image = None
        self.last_stamp = None

    # ===== 相機參數 =====
    def caminfo_callback(self, msg):
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]

    # ===== RGB =====
    def rgb_callback(self, msg):
        self.rgb_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.last_stamp = msg.header.stamp
        # self.get_logger().info(f"{self.rgb_image.shape}")

    def Obj_detect_callback(self, msg):

        if self.rgb_image is None:
            self.get_logger().info("no rgb image")
            return

        obj_num = len(msg.objects)

        self.get_logger().info(f"Detected objects: {obj_num}")

        if obj_num < 1:
            return

        coneArr_msg = ConeArray()

        for i, obj in enumerate(msg.objects):
            y1 = obj.bounding_box_2d.corners[0].kp[1]
            y2 = obj.bounding_box_2d.corners[2].kp[1]
            x1 = obj.bounding_box_2d.corners[0].kp[0]
            x2 = obj.bounding_box_2d.corners[2].kp[0]

            crop = self.rgb_image[(y1+y2)//2:y2, (2*x1+x2)//3:(x1+2*x2)//3]
            # cv2.imshow("crop", crop)
            # cv2.waitKey(1)
            # self.get_logger().info(f"{crop.shape}, confidence: {obj.confidence}")
            # self.get_logger().info(f"state: {obj.tracking_state}")
            # self.get_logger().info(f"velocity: {obj.velocity}")
        
            hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
            mean_h = np.mean(hsv[:, :, 0])

            cone = Cone()
            cone.x = float(obj.position[0]) # converting float32 to float64
            cone.y = float(obj.position[1])
            cone.z = float(obj.position[2])

            if 90 <= mean_h <= 120:
                cone.color = 'blue'
            elif 25 <= mean_h <= 45:
                cone.color = 'yellow'
            else:
                self.get_logger().warn('cone color unknown')
                cone.color = 'unknown'
            self.get_logger().info(
                f"Object {i} {cone.color}, x:{obj.position[0]} y:{obj.position[1]} z:{obj.position[2]}"
            )
            coneArr_msg.cones.append(cone)
        
        self.conePub.publish(coneArr_msg)
        self.get_logger().info("Published cone array")

    # ===== 主流程 =====
    # def process(self):

def main(args=None):
    rclpy.init(args=args)
    node = ZedYoloTF()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
