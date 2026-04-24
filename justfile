build:
    colcon build --symlink-install
    @touch .envrc # Forces direnv to reload the new install space

clean:
    rm -rf build/ install/ log/

test:
    colcon test

source:
    @echo "Run 'source install/setup.bash' to source the local workspace."
