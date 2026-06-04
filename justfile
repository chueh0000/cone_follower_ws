set shell := ["bash", "-c"]

# Variable for sourcing the ROS 2 workspace
ros_env := "source install/setup.bash && "

# --- Default ---

# List available commands
default:
    @just --list

# --- Setup & Build ---

# Install ROS dependencies
deps:
    sudo apt-get update
    rosdep update
    rosdep install --from-paths src --ignore-src -r -y

# Setup the workspace and FSDS simulator
setup: deps
    # Clone FSDS with submodules, skipping heavy LFS assets
    if [ ! -d "src/fsds_simulator" ]; then \
        GIT_LFS_SKIP_SMUDGE=1 git clone --depth 1 --recurse-submodules https://github.com/FS-Driverless/Formula-Student-Driverless-Simulator.git src/fsds_simulator; \
    fi
    # Ignore non-ROS/heavy directories to speed up colcon build
    touch src/fsds_simulator/ros/COLCON_IGNORE
    touch src/fsds_simulator/AirSim/COLCON_IGNORE
    touch src/fsds_simulator/python/COLCON_IGNORE
    touch src/fsds_simulator/simulator/COLCON_IGNORE
    touch src/fsds_simulator/UE4Project/COLCON_IGNORE
    # Setup Eigen dependency for AirLib
    mkdir -p src/fsds_simulator/AirSim/AirLib/deps
    ln -sfn /usr/include/eigen3 src/fsds_simulator/AirSim/AirLib/deps/eigen3
    # Link settings.json to both locations expected by the simulator and the bridge
    mkdir -p ~/Documents/AirSim
    mkdir -p ~/Formula-Student-Driverless-Simulator
    ln -sfn {{justfile_directory()}}/tools/FSDS/settings.json ~/Documents/AirSim/settings.json
    ln -sfn {{justfile_directory()}}/tools/FSDS/settings.json ~/Formula-Student-Driverless-Simulator/settings.json

# Download FSDS binaries
download-fsds:
    mkdir -p tools
    wget https://github.com/FS-Driverless/Formula-Student-Driverless-Simulator/releases/download/v2.2.0/fsds-v2.2.0-linux.zip -O tools/FSDS_Linux.zip
    unzip -n tools/FSDS_Linux.zip -d tools/FSDS
    rm tools/FSDS_Linux.zip

# Build the workspace
build:
    colcon build --symlink-install
    @touch .envrc # Forces direnv to reload the new install space

# Clean the workspace
clean:
    rm -rf build/ install/ log/

# Run tests
test:
    colcon test

# Print source command
source:
    @echo "Run 'source install/setup.bash' to source the local workspace."

# --- Simulation & Tools ---

# Run the FSDS simulator
run-fsds map="":
    cd tools/FSDS && ./FSDS.sh {{map}} -windowed -ResX=1280 -ResY=720

# --- Individual ROS Nodes ---

# Run mock track publisher
run-simulation:
    {{ros_env}} ros2 run cone_follower_simulation mock_track_publisher

# Run FSDS ROS bridge
run-ros-bridge:
    {{ros_env}} ros2 launch fsds_ros2_bridge fsds_ros2_bridge.launch.py

# Run FSDS track bridge
run-track-bridge:
    {{ros_env}} ros2 run cone_follower_simulation fsds_track_bridge

# Run centerline generator planning node
run-planning:
    {{ros_env}} ros2 run cone_follower_planning centerline_generator_node

# Run pure pursuit control node
run-control:
    {{ros_env}} ros2 run cone_follower_control pure_pursuit_node

# Run visualization nodes (TF and Markers)
run-viz:
    {{ros_env}} (ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 world fsds/map & ros2 run cone_follower_simulation cone_visualization_node)

# Run camera-focused RViz
run-viz-cameras:
    {{ros_env}} rviz2 -d src/cone_follower_simulation/rviz/cameras.rviz

# --- Launchers ---

# Launch simulation stack
launch-sim viz="true":
    {{ros_env}} ros2 launch cone_follower_simulation fsds_simulation.launch.py use_camera_viz:={{viz}}

# Launch ZED perception node
launch-zed:
    {{ros_env}} ros2 launch cone_follower_perception zed_perception.launch.py

# Launch real-world vehicle stack
launch-real-world viz="true" odom="/zed/zed_node/odom" perception="true":
    {{ros_env}} ros2 launch cone_follower_vehicle_interface real_world.launch.py use_rviz:={{viz}} odom_topic:={{odom}} use_perception:={{perception}}
