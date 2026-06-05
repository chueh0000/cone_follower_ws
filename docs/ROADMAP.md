# 7-Week Development Roadmap

### Phase 1: Logic & Simulation (Weeks 1-3) - [COMPLETED]
*Goal: Build the "Perfect World" logic in software.*
- **[x] Week 1: Mock Data & Path Planning**
- **[x] Week 2: Control System & Kinematics** (PID Speed Control & Adaptive Lookahead)
- **[x] Week 3: ROS 2 Architecture & Visualization**

### Phase 2: Perception & Reality (Week 4-5) - [COMPLETED]
*Goal: Handle "Messy World" sensor data.*
- **[ ] Week 4: 2D Perception (YOLO)** - *[SKIPPED: Transitioned directly to 3D integration]*
  - Train YOLO on FSOCO v2 dataset; validate using FSDS virtual camera streams.
- **[x] Week 5: 3D Perception (ZED API Integration)**
  - Implemented `zed_yolo_tf_node` for real-time 3D cone localization.

### Phase 3: Hardware Handshake & Field Testing (Weeks 6-7) - [IN PROGRESS]
- **[x] Week 6: Deployment & Actuation**
  - Integrated Steering Activation Handshake and safety delta guards.
- **[ ] Week 7: Field Testing & Tuning**
  - Parameter refinement and real-world track testing.
