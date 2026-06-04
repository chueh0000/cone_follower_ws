# Cone Follower Perception

This package handles the detection and 3D localization of cones for the autonomous navigation stack.

## Current Status: `zed_yolo_tf_node`

The current implementation (`zed_yolo_tf_node.py`) is designed as a **post-processor for the ZED SDK**.

### Features
- Subscribes to `/zed/zed_node/obj_det/objects` (ZED SDK Object Detection).
- Subscribes to `/zed/zed_node/left/color/rect/image` for color classification.
- Performs HSV-based color filtering on cropped cone images to distinguish between **blue** and **yellow** cones.
- Publishes `cone_follower_msgs/ConeArray` to the `/cones` topic.

### Limitations
- **Hardware Dependent:** It requires the ZED SDK's Object Detection module to be running, which is only available on x86-64 systems with NVIDIA GPUs and the ZED camera connected.
- **Simulator Incompatibility:** The FSDS simulator provides raw RGB and Depth images but does not include the ZED SDK. Therefore, this node cannot be used in simulation environments as-is.

---

## Future Plan: Unified Perception Node

To support both the real-world electric SUV and the FSDS simulator, we plan to implement a new, more flexible perception node.

### Design Goals
1. **Standalone Detection:** Integrate a YOLO (e.g., YOLOv8) detector directly into the ROS node to remove dependency on the proprietary ZED Object Detection API.
2. **Flexible Localization:**
   - **Real-World:** Use ZED SDK spatial mapping or point-cloud data if available.
   - **Simulation:** Use the FSDS Depth camera (`ImageType: 2`) to project 2D detections into 3D coordinates.
3. **Unified Interface:** Use ROS 2 remappings to switch between simulation camera topics and ZED camera topics seamlessly.

### Proposed Architecture
- **Input:** `sensor_msgs/Image` (RGB) and `sensor_msgs/Image` (Depth or Disparity).
- **Inference:** YOLO-based 2D bounding box detection.
- **Localization:** 3D Projection using Camera Intrinsics + Depth Map.
- **Classification:** Improved CNN or HSV classifier for cone color.

### Roadmap
- **Phase 2 (Simulation):** Implement the node to process FSDS `/fsds/camera/cam1` (RGB) and `/fsds/camera/depth_cam` (Depth) topics.
- **Phase 3 (Hardware):** Deploy on the vehicle laptop, using remappings to point to the ZED 2i camera streams.
