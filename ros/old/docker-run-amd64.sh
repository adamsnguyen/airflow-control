#!/bin/bash

# give docker root user X11 permissions
sudo xhost +si:localuser:root

# run the container
sudo docker run -it --rm --network host -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix/:/tmp/.X11-unix \
    --privileged \
    --device=/dev/ttyACM0 \
    --device=/dev/ttyACM1 \
    --mount type=bind,source=$PWD/../../ros-workspace,target=/workspace \
    -w /workspace/dev_ws \
    amd64_ros2_foxy 
