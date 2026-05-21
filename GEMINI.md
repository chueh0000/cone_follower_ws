# GEMINI.md - Project Architectural Context

## Project Core: Electric SUV Cone Following

This project focuses on implementing an autonomous navigation stack for an electric SUV to follow a track defined by cones. It uses a ZED 2i camera for perception and classical control algorithms for path following.

---

## Technical Stack & Architecture

### 1. Perception Module (`src/cone_follower_perception`)
- **YOLO Detector:** Trained on FSOCO v2 dataset. Generates 2D bounding boxes of cones.
- **ZED YOLO TF Node:** Custom node to localize 3D $(X, Y, Z)$ coordinates from ZED Object Detection data.
- **Data Input:** Supports live ZED 2i camera feed and ZED `.svo` file playback (for offline testing).
- **Depth Integration:** Utilizes FSDS Depth perspective (ImageType 2) for simulation-based spatial validation.

### 2. Planning Module (`src/cone_follower_planning`)
- **Delaunay Triangulation:** Connects 3D cone points to map the track boundaries.
- **Centerline Generation:** Filters edges and calculates midpoints to create a continuous trajectory (smoothed via B-splines or moving averages).

### 3. Control Module (`src/cone_follower_control`)
- **Adaptive Pure Pursuit:** Calculates steering and speed commands based on an adaptive lookahead distance.
- **Kinematic Mapping:** Converts target vehicle steering angle to the specific electric SUV's steering wheel angle and speed.

### 4. Vehicle Interface Module (`src/cone_follower_vehicle_interface`)
- **Proprietary Integration:** Interfaces with the vehicle's ECU via DoIP/UDS using the `foxtronpi-pyclient` library.
- **APS Control Strategy:** Due to hardware limitations, movement is controlled via the **APS Speed Control** mode (DID 0x1001), limited to 7 km/h.
- **Steering Handshake:** Implements the mandatory 5-step reset sequence to gain control authority and maps ROS steering to raw wheel angles (±360 deg).
- **Binary Dependencies:** Requires x86-64 architecture to run the pre-compiled `.so` communication modules.

### 5. Simulation & Validation (`FSDS`)
- **Formula Student Driverless Simulator (FSDS):** Unreal Engine-based simulator for closed-loop testing.
- **ROS 2 Bridge:** Facilitates communication between the autonomous stack and the simulator.
- **AirLib Physics:** Provides realistic vehicle dynamics for control law validation.

### 6. Custom Messages (`src/cone_follower_msgs`)
- Centralized definitions for perception and control data structures to ensure consistency across nodes.

---

## Development & Deployment Lifecycle

### Environment Strategy
- **Mac / Remote Workstation:** Primary development for Phases 1 and 2. Uses FSDS, mock data, and recorded ZED `.svo` files.
- **Vehicle Laptop:** Final deployment and Phase 3. ROS 2 on Ubuntu with direct sensor access.

### Phased Workflow
1.  **Phase 1 (Weeks 1-3): Logic & Simulation.** Work with mock 3D points in RViz 2 and FSDS.
2.  **Phase 2 (Weeks 4-5): Perception Integration.** Transition from mock points to FSDS virtual cameras and ZED `.svo` playback with YOLO.
3.  **Phase 3 (Weeks 6-7): Field Deployment.** Hardware handshake and real-world track testing.

---

## Development Conventions
- **Tooling:** Use `just` for command automation and `direnv` for automatic ROS 2 environment sourcing. 
- **Setup:** `just setup` performs a robust initialization:
    - Shallow clones FSDS with submodules.
    - Symlinks project `settings.json` to `~/Documents/AirSim/settings.json` to keep simulator and ROS bridge in sync.
    - Configures Eigen and build optimizations (`COLCON_IGNORE`).
- **Simulation First:** Verify all algorithmic changes using FSDS before testing on hardware.
- **Sensor Configuration:** The simulator is configured with a RGB camera and a central depth camera (`ImageType: 2`) to support Phase 2 perception development.
