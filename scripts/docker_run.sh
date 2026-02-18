#!/bin/bash
sudo docker run -it --rm \
  --privileged \
  --network host \
  --device /dev/ttyUSB_LIDAR \
  --device /dev/ttyUSB_SERIAL \
  -v /dev/ttyUSB_LIDAR:/dev/ttyUSB_LIDAR \
  -v /dev/ttyUSB_SERIAL:/dev/ttyUSB_SERIAL \
  --group-add dialout \
  -v /home/greatsheep/crawler:/home/ubuntu/ros2_ws/src \
  working:2
