#!/usr/bin/env bash
# 停止 Day1 所有 ROS 进程
pkill -f "roslaunch ft_bringup vision_final.launch" 2>/dev/null || true
pkill -9 -x play 2>/dev/null || true
pkill -f "ft_detector_node" 2>/dev/null || true
pkill -f "ft_tracker_node" 2>/dev/null || true
pkill -f "ft_selector_node" 2>/dev/null || true
pkill -9 -x rqt_image_view 2>/dev/null || true
pkill -9 -x rviz 2>/dev/null || true
pkill -9 -f '[v]ision_panel.py' 2>/dev/null || true
sleep 1
echo "已停止 Day1 进程"
