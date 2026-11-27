#!/usr/bin/env bash
# 自动识别：在容器内就直接编译；在宿主机就 docker exec 进容器编译。
set -e
if [ -f /opt/ros/noetic/setup.bash ] && [ -d /home/ros/catkin_ws ]; then
  echo "[build] inside container"
  source /opt/ros/noetic/setup.bash
  cd /home/ros/catkin_ws
  source devel/setup.bash 2>/dev/null || true
  catkin_make
else
  echo "[build] on host -> docker exec ros-noetic"
  docker start ros-noetic >/dev/null 2>&1 || true
  sleep 1
  docker exec ros-noetic bash -lc 'source /opt/ros/noetic/setup.bash && cd /home/ros/catkin_ws && catkin_make'
fi
