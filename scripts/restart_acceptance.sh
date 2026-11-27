#!/usr/bin/env bash
# 改完代码后：停旧链路 -> 编译 -> 重新启动 red_4 + 调试图 + RViz
set -e
source /opt/ros/noetic/setup.bash
cd /home/ros/catkin_ws
source devel/setup.bash 2>/dev/null || true
bash /home/ros/catkin_ws/src/vision_final_test/scripts/stop_day1.sh >/dev/null 2>&1 || true
echo "[restart] catkin_make ..."
catkin_make
echo "[restart] build ok, starting red_4 ..."
exec bash /home/ros/catkin_ws/src/vision_final_test/scripts/acceptance_all.sh /home/ros/catkin_ws/bags/red_4.bag 5
