#!/bin/bash
sudo docker run --rm --detach \
  --name crawler-container \
  --privileged \
  --network host \
  --device /dev/ttyUSB_LIDAR \
  --device /dev/ttyUSB_SERIAL \
  -v /dev/ttyUSB_LIDAR:/dev/ttyUSB_LIDAR \
  -v /dev/ttyUSB_SERIAL:/dev/ttyUSB_SERIAL \
  --group-add dialout \
  -v /home/greatsheep/crawler:/home/ubuntu/ros2_ws/src \
  crawler:latest \
  bash -c "source /opt/ros/jazzy/setup.bash && \
           source /home/ubuntu/ros2_ws/install/setup.bash && \
           ros2 launch web_hmi hmi_server.launch.py"
