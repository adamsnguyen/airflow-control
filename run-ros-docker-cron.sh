#!/bin/bash

# give docker root user X11 permissions
xhost +si:localuser:root

docker kill ros_jetson

# run the container
docker run --runtime nvidia --rm --network host -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix/:/tmp/.X11-unix \
    --privileged \
    --device=/dev/ttyACM0 \
    --device=/dev/ttyACM1 \
    --device=/dev/ttyUSB0 \
    --device=/dev/ttyUSB1 \
    --device=$(readlink -f /dev/RS485) \
    -e RS485=$(readlink -f /dev/RS485) \
    --mount type=bind,source=/mnt/usb-drive/workspace/airflow-control/ros-workspace,target=/workspace \
    -w /workspace/dev_ws \
    --name ros_jetson \
    jetson_ros2_foxy 
