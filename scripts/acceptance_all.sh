#!/usr/bin/env bash
# 现场一键：launch + bag + rqt 调试图 + RViz
set -e
source /opt/ros/noetic/setup.bash
cd /home/ros/catkin_ws
source devel/setup.bash
BAG="${1:-/home/ros/catkin_ws/bags/red_4.bag}"
START="${2:-5}"
cleanup() {
  kill "$LP" "$BP" 2>/dev/null || true
  pkill -f "roslaunch ft_bringup vision_final.launch" 2>/dev/null || true
  pkill -9 -x play 2>/dev/null || true
  pkill -9 -x rqt_image_view 2>/dev/null || true
  pkill -9 -x rviz 2>/dev/null || true
}
trap cleanup INT TERM EXIT
roslaunch ft_bringup vision_final.launch rviz:=false use_sim_time:=true >/tmp/acc_all_launch.log 2>&1 &
LP=$!
sleep 5
rosbag play --clock -s "$START" --rate 0.5 -l "$BAG" >/tmp/acc_all_bag.log 2>&1 &
BP=$!
sleep 4
rqt_image_view /ft/debug/image >/tmp/acc_all_rqt.log 2>&1 &
sleep 2
rosrun rviz rviz -d /home/ros/catkin_ws/src/vision_final_test/src/ft_bringup/rviz/vision_final.rviz >/tmp/acc_all_rviz.log 2>&1 &
echo "已启动：调试图 + RViz + red_4"
echo "看 detection：任务『验收: 3』；看 track：任务『验收: 4』"
echo "停止：任务『验收: 7』"
wait
