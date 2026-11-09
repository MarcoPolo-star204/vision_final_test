#!/usr/bin/env bash
# Day1 一键启动：检测 + 跟踪 + 选择 + bag
set -e
source /opt/ros/noetic/setup.bash
cd /home/ros/catkin_ws
source devel/setup.bash

BAG="${1:-/home/ros/catkin_ws/bags/blue_1.bag}"
START="${2:-8}"

cleanup() {
  kill "$LAUNCH_PID" "$BAG_PID" 2>/dev/null || true
  pkill -f "roslaunch ft_bringup vision_final.launch" 2>/dev/null || true
  pkill -f "rosbag play -s" 2>/dev/null || true
}
trap cleanup INT TERM EXIT

roslaunch ft_bringup vision_final.launch rviz:=false use_sim_time:=false >/tmp/day1_launch.log 2>&1 &
LAUNCH_PID=$!
sleep 5

rosbag play -s "$START" --rate 0.5 -l "$BAG" >/tmp/day1_bag.log 2>&1 &
BAG_PID=$!

echo
echo "已启动："
echo "  launch PID=$LAUNCH_PID"
echo "  bag    PID=$BAG_PID"
echo "  bag    =$BAG"
echo
echo "在 VS Code 里继续点：ROS: Day1 3 看调试图 / ROS: Day1 4 看检测频率"
echo "停止：点 ROS: Day1 5 停止，或在这个终端按 Ctrl+C"
wait
