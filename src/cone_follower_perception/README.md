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
- **Inference & Classification:** Multi-class YOLO detection (e.g., YOLOv8) to simultaneously output 2D bounding boxes and cone color (class_id).
- **Localization:** 3D Projection using Camera Intrinsics + Depth Map (with ROI filtering for robust depth extraction).

### Test-Driven Development (TDD) Implementation Plan

To ensure robustness and compatibility across simulation and hardware, we will follow a rigorous Test-Driven Development (TDD) approach. The development will be broken down into discrete testable components.

#### Phase 1: Core Logic (Unit Tests)
In this phase, we build the underlying mathematical and inference models independently of ROS 2.
1. **2D Bounding Box Detection & Classification (YOLO):**
   - *Test:* Provide a dummy image and verify the wrapper returns expected bounding box dimensions and the correct `class_id` (color).
   - *Implementation:* Wrap the YOLOv8 inference API, natively handling multi-class detection (e.g., 0: blue_cone, 1: yellow_cone, 2: orange_cone).
2. **3D Depth Projection (Robust ROI Filtering):**
   - *Test:* Feed the projection function a dummy depth map where the exact center pixel is `NaN` or `inf` (common in stereo depth), but surrounding pixels have valid depth. Assert that the function still returns an accurate distance using median filtering.
   - *Implementation:* Do not use a single center pixel. Extract a Region of Interest (ROI) from the depth map (e.g., inner 50% of the box, or focusing on the bottom-center `[y_max, x_center]` for ground contact) and take the median or a trimmed mean (e.g., 20th percentile) depth value. Then apply the projection math: `X = (x - cx) * Z / fx`, `Y = (y - cy) * Z / fy`.

#### Phase 2: ROS 2 Node Integration (Integration Tests)
In this phase, we integrate the core logic into a ROS 2 node and test the publish/subscribe interfaces.
1. **Message Synchronization:**
   - *Test:* Publish mock `sensor_msgs/Image` (RGB) and `sensor_msgs/Image` (Depth) with slight timestamp offsets. Verify the node's internal callback triggers with synchronized data.
   - *Implementation:* Create `message_filters::ApproximateTimeSynchronizer` for RGB and Depth topics. Set queue size and slop correctly (e.g., queue size 10, slop 0.05 seconds) to handle depth computation lag.
2. **TF2 Frame Integration:**
   - *Test:* Publish a mock static transform between `camera_link` and `base_link`. Feed a cone located at `(0, 0, 5)` in the camera frame (5 meters straight ahead). Assert that the published `ConeArray` outputs the cone at `(5, 0, 0)` in the `base_link` frame.
   - *Implementation:* Integrate `tf2_ros` to look up the static transform from the Camera Optical Frame to the Vehicle Frame (e.g., `base_link`). Multiply the calculated 3D points by this transform.
3. **ConeArray Publication:**
   - *Test:* Feed synchronized mock images, depth maps, and TF data. Assert that the node publishes a `cone_follower_msgs/ConeArray` with the correctly transformed 3D coordinates and colors.
   - *Implementation:* Connect the complete pipeline (YOLO -> Projection -> TF2 Transform) within the synchronized callback and publish the computed `ConeArray`.

#### Phase 3: System Verification (Simulation & Hardware)
1. **Simulation (FSDS):**
   - *Test:* Run the node against a `rosbag` recorded from FSDS. Verify that detection and projection function correctly in the FSDS environment.
   - *Implementation:* Create a launch file for simulation with appropriate remappings (`/fsds/camera/cam1` for RGB, `/fsds/camera/depth_cam` for Depth) and FSDS camera intrinsics.
2. **Hardware (ZED 2i):**
   - *Test:* Run the node with live ZED 2i camera or recorded `rosbag` from the field.
   - *Implementation:* Create a hardware launch file remapping to ZED topics (e.g., `/zed/zed_node/rgb/image_rect_color`, `/zed/zed_node/depth/depth_registered`) and loading real camera intrinsics.
