# Usage by Use Case

### 1. Mock Track Evaluation (Centerline Only)
*Use this to test the Delaunay triangulation and path smoothing logic without the simulator or camera.*
- **Command:** `just run-simulation`
- **What it does:** Publishes a static set of 3D cone points. You should also run `just run-planning` and `just run-viz` to see the generated centerline in RViz.

### 2. Simulator Driving (Ground Truth Cones)
*Use this to evaluate the Adaptive Pure Pursuit controller and path planning in a closed-loop environment.*
- **Step 1 (Simulator):** `just run-fsds TrainingMap`
- **Step 2 (Stack):** `just launch-sim false`
- **What it does:** Uses the simulator's internal "map" to provide perfect cone coordinates to the planner. Bypasses perception to isolate control/planning performance.
- Demo Video: [YouTube link](https://youtu.be/IipHGM0J3D0?si=R8dTyqCfrm4ses3f) / [Google Drive link](https://drive.google.com/file/d/1FfGchQ7Cq-Om5PTYDIYxig15oGlC0ZDh/view?usp=sharing)

### 3. Simulator Full Stack (Camera Perception) - *[UNDER DEVELOPMENT]*
*Use this to test the end-to-end pipeline, including virtual camera processing and 3D localization.*
- **Step 1 (Simulator):** `just run-fsds TrainingMap`
- **Step 2 (Stack):** `just launch-sim true`
- **What it does:** Enables the virtual camera stream and depth mapping. The system must detect and localize cones itself before planning a path.

### 4. Real-World Perception (Camera Only)
*Use this to validate the ZED YOLO TF node and cone localization using live or recorded data.*
- **Live/Rosbag:** Play a ZED `rosbag` or connect the camera.
- **Perception Launch:** `just launch-zed`
- **What it does:** Runs the YOLO detector and spatial mapping. Visualizes localized 3D cones in RViz.

### 5. Real-World Deployment (Vehicle Integration)
*Use this for final deployment on the physical electric SUV or for logic testing via Dry Run.*
- **Handshake:** Ensure the steering wheel **Trip** button is ready (dead-man switch).
- **Full Stack Launch:** `just launch-real-world`
- **Dry Run (No Vehicle):** `just real_dry_run=true launch-real-world`

#### CLI Options
You can customize the real-world launch by passing variables before the recipe:
| Variable | Default | Description |
| :--- | :--- | :--- |
| `real_dry_run` | `false` | If `true`, bypasses vehicle connection and logs drive commands. |
| `real_perception` | `true` | Set to `false` to skip the ZED YOLO perception node (e.g., if using mock data). |
| `real_viz` | `true` | Set to `false` to disable RViz. |
| `real_odom` | `/zed/zed_node/odom` | The odometry topic to use for planning and control. |

**Example:** `just real_dry_run=true real_viz=false launch-real-world`

- **What it does:** Consolidates perception, planning, control, and vehicle interface into a single command.
 Maps ROS steering/speed commands to the SUV's ECU via DoIP/UDS. Includes mandatory safety handshakes and torque/angle limits. In **Dry Run** mode, hardware communication is bypassed, allowing full stack validation on development laptops.
