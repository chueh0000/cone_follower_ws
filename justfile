build:
    colcon build --symlink-install

clean:
    rm -rf build/ install/ log/

test:
    colcon test

source:
    @echo "Run 'source install/setup.bash' to source the local workspace."
