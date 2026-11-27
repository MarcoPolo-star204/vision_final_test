#!/usr/bin/env bash
# 现场验收：启动检测/跟踪/选择 + 播 bag
set -e
source /opt/ros/noetic/setup.bash
cd /home/ros/catkin_ws
source devel/setup.bash
BAG="${1:-/home/ros/catkin_ws/bags/red_4.bag}"
START="${2:-5}"
cleanup() {
  kill "$LP" "$BP" 2>/dev/null || true
  pkill -f "roslaunch ft_bringup vision_final.launch" 2>/dev/null || true
  pkill -f "rosbag play --clock" 2>/dev/null || true
}
trap cleanup INT TERM EXIT
roslaunch ft_bringup vision_final.launch rviz:=false use_sim_time:=true >/tmp/acc_launch.log 2>&1 &
LP=$!
sleep 5
rosbag play --clock -s "$START" --rate 0.5 -l "$BAG" >/tmp/acc_bag.log 2>&1 &
BP=$!
echo "验收已启动：bag=$BAG"
echo "调试图：任务『验收: 调试图』"
echo "停止：任务『验收: 停止』"
wait
