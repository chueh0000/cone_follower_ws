setup:
    # Clone FSDS with submodules, skipping heavy LFS assets
    GIT_LFS_SKIP_SMUDGE=1 git clone --depth 1 --recurse-submodules https://github.com/FS-Driverless/Formula-Student-Driverless-Simulator.git src/fsds_simulator
    # Ignore non-ROS/heavy directories to speed up colcon build
    touch src/fsds_simulator/ros/COLCON_IGNORE
    touch src/fsds_simulator/AirSim/COLCON_IGNORE
    touch src/fsds_simulator/rpc/COLCON_IGNORE
    # Setup Eigen dependency for AirLib
    mkdir -p src/fsds_simulator/AirSim/AirLib/deps
    ln -s /usr/include/eigen3 src/fsds_simulator/AirSim/AirLib/deps/eigen3

download-fsds:
    mkdir -p tools
    wget https://github.com/FS-Driverless/Formula-Student-Driverless-Simulator/releases/download/v2.2.0/fsds-v2.2.0-linux.zip -O tools/FSDS_Linux.zip
    unzip tools/FSDS_Linux.zip -d tools/FSDS
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
