# cone_follower_control

The `cone_follower_control` package implements the lateral and longitudinal control logic for the autonomous cone-following vehicle. It specifically uses an **Adaptive Pure Pursuit** algorithm to track the generated centerline trajectory.

## Nodes

### pure_pursuit_node

This node subscribes to the planned centerline and vehicle odometry to compute the optimal steering angle and speed commands.

#### Subscribed Topics

| Topic | Type | Description |
|---|---|---|
| `/centerline` | `nav_msgs/msg/Path` | The target trajectory to follow. |
| `/testing_only/odom` | `nav_msgs/msg/Odometry` | Current pose and velocity of the vehicle. |

#### Published Topics

| Topic | Type | Description |
|---|---|---|
| `/control_command` | `fs_msgs/msg/ControlCommand` | Actuation commands including steering, throttle, and brake. |
| `/lookahead_marker` | `visualization_msgs/msg/Marker` | RViz markers for the interpolated target point and the closest point on path. |

#### Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `wheelbase` | `double` | `2.7` | The distance between the front and rear axles of the vehicle (meters). |
| `lookahead_k` | `double` | `0.5` | Velocity-proportional lookahead gain ($L_d = k \cdot v$). |
| `lookahead_min` | `double` | `4.0` | Minimum allowable lookahead distance (meters). |
| `lookahead_max` | `double` | `10.0` | Maximum allowable lookahead distance (meters). |
| `lookahead_curvature_k`| `double` | `1.5` | Gain for decreasing lookahead distance in sharp turns. |
| `lookahead_error_k` | `double` | `0.2` | Gain for increasing lookahead distance based on lateral error to smooth recovery. |
| `target_velocity` | `double` | `3.0` | Base target cruise velocity (m/s). |
| `steering_gain` | `double` | `-1.0` | Final multiplier for the steering command (used for direction correction). |

## Algorithm: Adaptive Pure Pursuit

The node implements an enhanced Pure Pursuit algorithm where the lookahead distance ($L_d$) is dynamically adjusted to balance stability and tracking performance.

### 1. Dynamic Lookahead Scaling
The base lookahead distance is calculated as $L_d = K \cdot V$. This is further modified by path curvature ($\kappa$) and lateral error ($e_{lat}$):

$$L_d = \max\left(L_{min}, \min\left(L_{max}, \frac{K \cdot V}{1 + k_c \cdot \kappa} + k_e \cdot e_{lat}\right)\right)$$

- **Curvature Compensation ($k_c$):** Reduces $L_d$ on sharp turns to prevent corner cutting.
- **Error Compensation ($k_e$):** Increases $L_d$ when the vehicle is far from the path to ensure a smooth, non-oscillatory approach back to the centerline.

### 2. Curvature Estimation
Path curvature ($\kappa$) is estimated using a "Robust Change in Heading" method, looking at the angular difference between segments formed by three consecutive points on the path ahead of the closest point.

### 3. Longitudinal Control
The target speed is automatically reduced in high-curvature sections:
$$V_{limit} = \frac{V_{target}}{1 + 5.0 \cdot \kappa}$$
A simple proportional controller then maps the velocity error to throttle and brake commands for the simulator.

## TODOs
- [ ] **PID Speed Control:** Replace the current P-controller with a full PID (Proportional-Integral-Derivative) loop to eliminate steady-state velocity error and improve acceleration smoothness.
- [ ] **Dynamic Gains:** Implement gain scheduling for the PID controller based on the current velocity and track surface conditions.

