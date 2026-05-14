default:
    @just --list

setup:
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
    ln -sf /usr/include/eigen3 src/fsds_simulator/AirSim/AirLib/deps/eigen3
    # Link settings.json to the location expected by the simulator
    mkdir -p ~/Documents/AirSim
    ln -sf {{justfile_directory()}}/tools/FSDS/settings.json ~/Documents/AirSim/settings.json

download-fsds:
    mkdir -p tools
    wget https://github.com/FS-Driverless/Formula-Student-Driverless-Simulator/releases/download/v2.2.0/fsds-v2.2.0-linux.zip -O tools/FSDS_Linux.zip
    unzip -n tools/FSDS_Linux.zip -d tools/FSDS
    rm tools/FSDS_Linux.zip




build:
    colcon build --symlink-install
    @touch .envrc # Forces direnv to reload the new install space

clean:
    rm -rf build/ install/ log/

test:
    colcon test

source:
    @echo "Run 'source install/setup.bash' to source the local workspace."


run-simulation:
    bash -c "source install/setup.bash && ros2 run cone_follower_simulation mock_track_publisher"


run-fsds map="":
    cd tools/FSDS && ./FSDS.sh {{map}} -windowed -ResX=1280 -ResY=720

run-ros-bridge:
	bash -c "source install/setup.bash && ros2 launch fsds_ros2_bridge fsds_ros2_bridge.launch.py"

run-track-bridge:
    bash -c "source install/setup.bash && ros2 run cone_follower_simulation fsds_track_bridge"

run-planning:
    bash -c "source install/setup.bash && ros2 run cone_follower_planning centerline_generator_node"

run-viz:
    bash -c "source install/setup.bash && (ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 world fsds/map & ros2 run cone_follower_simulation cone_visualization_node)"

run-viz-cameras:
    bash -c "source install/setup.bash && rviz2 -d src/cone_follower_simulation/rviz/cameras.rviz"

run-control:
    bash -c "source install/setup.bash && ros2 run cone_follower_control pure_pursuit_node"

launch-sim viz="true":
    bash -c "source install/setup.bash && ros2 launch cone_follower_simulation fsds_simulation.launch.py use_camera_viz:={{viz}}"
