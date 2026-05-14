# Cone Follower - Electric SUV Autonomous Navigation

This project implements an autonomous cone-following system for an electric SUV using ROS 2, computer vision (YOLO + ZED SDK), and classical path planning/control algorithms.

## Project Overview

The goal is to enable an electric SUV to navigate through a course defined by cones. The system detects cones in 3D space, generates a optimal centerline path, and calculates the necessary steering and speed commands to follow that path.

### System Architecture

```mermaid
graph LR
    subgraph Inputs ["Perception Inputs"]
        direction TB
        subgraph Simulation ["Simulator Data"]
            SimMap["Ground Truth Map<br/>(fsds_track_bridge)"]
            SimCam["Virtual Camera<br/>(FSDS Virtual Cam)"]
        end
        ZED_Camera["ZED 2i Camera<br/>(Live Feed / SVO)"]
    end

    Perception["<b>Perception Node</b><br/>(YOLO + 3D Localization)"]
    Planning["<b>Planning Node</b><br/>(Delaunay + Centerline)"]
    Control["<b>Control Node</b><br/>(Pure Pursuit + Kinematics)"]

    subgraph Outputs ["Control Outputs"]
        direction TB
        Drive_Sim["FSDS Simulator<br/>(Throttle, Brake, Steering Angle)"]
        FoxtronPi["SUV Interface<br/>(Speed, Steering Wheel Angle)"]
    end

    SimMap --> Perception
    SimCam --> Perception
    ZED_Camera --> Perception

    Perception -- "ConeArray" --> Planning
    Planning -- "Path" --> Control

    Control --> Drive_Sim
    Control --> FoxtronPi
```

### Hardware & Deployment
- **Development Environment:** Mac or Remote Ubuntu Workstation with GPU (no sensor attached).
- **Deployment Platform:** Ubuntu Laptop + ROS 2 + GPU (mounted on the vehicle).
- **Primary Sensor:** ZED 2i camera for 2D bounding box generation and 3D spatial coordinate extraction.
### Software Stack
- **Perception:** YOLO (trained on FSOCO v2) for 2D detection + ZED Object Detection API (Custom Detector) for 3D localization.
- **Planning:** Delaunay Triangulation for track mapping and centerline generation.
- **Control:** Adaptive Pure Pursuit for trajectory following and steering wheel angle calculation.
- **Simulation:** Formula Student Driverless Simulator (FSDS) for high-fidelity vehicle dynamics and sensor emulation.
- **Actuation:** Proprietary Python package for low-level vehicle control (steering wheel angle and speed).

---

## 7-Week Development Roadmap

### Phase 1: Logic & Simulation (Weeks 1-3) - [COMPLETED]
*Goal: Build the "Perfect World" logic in software.*
- **[x] Week 1: Mock Data & Path Planning**
  - Implement Delaunay Triangulation to generate track centerlines from synthetic 3D points.
- **[x] Week 2: Control System & Kinematics**
  - Implement Adaptive Pure Pursuit and map vehicle steering to steering wheel angles. *(Note: Implementation complete; fine-tuning in progress)*
- **[x] Week 3: ROS 2 Architecture & Visualization**
  - Integrate nodes with FSDS and RViz 2 to verify control and planning in a closed-loop simulation.

### Phase 2: Perception & Reality (Weeks 4-5)
*Goal: Handle "Messy World" sensor data.*
- **Week 4: 2D Perception (YOLO)**
  - Train YOLO on FSOCO v2 dataset; validate using FSDS virtual camera streams.
- **Week 5: 3D Perception (ZED API Integration)**
  - Process ZED `.svo` files and FSDS spatial data to bridge 2D boxes into 3D coordinates.

---

## Development Workflow
Use the provided `justfile` and `direnv` (.envrc) to manage the ROS 2 environment and build processes. (Install `justfile` and `direnv` first)

1. **Setup Workspace:** Run `just setup` to clone the FSDS repository, configure dependencies (e.g., Eigen, COLCON_IGNORE), and symlink the `settings.json` to `~/Documents/AirSim`.
2. **Download Simulator:** Run `just download-fsds` to fetch the pre-compiled FSDS binary (Linux).
3. **Build:** Run `just build` to compile the workspace, including the `fsds_ros2_bridge`.

### Running the Simulation
To run the full autonomous stack in simulation (FSDS):

1. **Terminal 1 (FSDS):** `just run-fsds TrainingMap` (or `SmallTrack`, `Skidpad`)
2. **Terminal 2 (Stack):** `just launch-sim [viz]`

The `just launch-sim` command handles the ROS bridge, track bridge, planning, control, and visualization nodes.

**Visualization Options:**
- `just launch-sim true` (default): Opens RViz with a camera-focused view including Cam1 (color) and Depth Camera (scaled 0-20m).
- `just launch-sim false`: Opens RViz with the standard track visualization (cones and centerline).

**In RViz 2:**
- The configuration is automatically loaded based on your `viz` choice.
- **Depth Visualization:** The depth camera image is pre-scaled in the `cameras.rviz` config. If viewing manually, ensure **Normalize Range** is off and set the range to 0.0 - 20.0 for best contrast.

---

### Camera Configuration
The system is configured with a depth camera for perception testing. The `settings.json` (found in `tools/FSDS/`) includes:
- `cam1`: Standard RGB camera.
- `depth_cam`: Depth camera (ImageType 2) mounted centrally for spatial mapping.

### Legacy/Individual Node Execution
If you need to verify specific components or run the mock track (no FSDS required):

- **Mock Track (Standalone):** `just run-simulation`
- **Individual Nodes:** See `justfile` for `run-planning`, `run-control`, `run-viz`, etc.

---

### Phase 3: Hardware Handshake & Field Testing (Weeks 6-7)
*Goal: Deploy to the vehicle and optimize.*
- **Week 6: Deployment & Actuation**
  - Migrate to the vehicle laptop and connect the `vehicle_interface_node` to the proprietary control package.
- **Week 7: Field Testing & Tuning**
  - Perform real-world track testing, latency profiling, and parameter refinement.
