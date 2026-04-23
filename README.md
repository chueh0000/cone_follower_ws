# Cone Follower - Electric SUV Autonomous Navigation

This project implements an autonomous cone-following system for an electric SUV using ROS 2, computer vision (YOLO + ZED SDK), and classical path planning/control algorithms.

## Project Overview

The goal is to enable an electric SUV to navigate through a course defined by cones. The system detects cones in 3D space, generates a optimal centerline path, and calculates the necessary steering and speed commands to follow that path.

### Hardware & Deployment
- **Development Environment:** Mac or Remote Ubuntu Workstation with GPU (no sensor attached).
- **Deployment Platform:** Ubuntu Laptop + ROS 2 + GPU (mounted on the vehicle).
- **Primary Sensor:** ZED 2i camera for 2D bounding box generation and 3D spatial coordinate extraction.

### Software Stack
- **Perception:** YOLO (trained on FSOCO v2) for 2D detection + ZED Object Detection API (Custom Detector) for 3D localization.
- **Planning:** Delaunay Triangulation for track mapping and centerline generation.
- **Control:** Adaptive Pure Pursuit for trajectory following and steering wheel angle calculation.
- **Actuation:** Proprietary Python package for low-level vehicle control (steering wheel angle and speed).

---

## 7-Week Development Roadmap

### Phase 1: Logic & Simulation (Weeks 1-3)
*Goal: Build the "Perfect World" logic in software.*
- **Week 1: Mock Data & Path Planning**
  - Implement Delaunay Triangulation to generate track centerlines from synthetic 3D points.
- **Week 2: Control System & Kinematics**
  - Implement Adaptive Pure Pursuit and map vehicle steering to steering wheel angles.
- **Week 3: ROS 2 Architecture & Visualization**
  - Integrate planning and control nodes with RViz 2 and a kinematic bicycle model.

### Phase 2: Perception & Reality (Weeks 4-5)
*Goal: Handle "Messy World" sensor data.*
- **Week 4: 2D Perception (YOLO)**
  - Train YOLO on FSOCO v2 dataset for cone detection.
- **Week 5: 3D Perception (ZED API Integration)**
  - Process ZED `.svo` files to bridge 2D boxes into 3D coordinates using the ZED Custom Object Detection API.

### Phase 3: Hardware Handshake & Field Testing (Weeks 6-7)
*Goal: Deploy to the vehicle and optimize.*
- **Week 6: Deployment & Actuation**
  - Migrate to the vehicle laptop and connect the `vehicle_interface_node` to the proprietary control package.
- **Week 7: Field Testing & Tuning**
  - Perform real-world track testing, latency profiling, and parameter refinement.

---

## Development Workflow
Use the provided `justfile` and `direnv` (.envrc) to manage the ROS 2 environment and build processes across different machines.
