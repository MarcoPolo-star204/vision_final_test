# vision_final_test

装甲板自瞄系统考核（ROS1 Noetic + OpenCV）。

## 当前状态

- [x] 仓库骨架、消息包
- [x] 图像接入骨架（compressed + camera_info，header 使用图像时间戳）
- [x] 单目标 Kalman 跟踪 + 提前量 + RViz Marker
- [ ] HSV 装甲板识别 / 灯条配对
- [ ] solvePnP / TF / projectPoints
- [ ] dynamic_reconfigure 阈值在线调
- [ ] bag 调参与演示视频
- [ ] 进阶：ONNX / 旋转模型 / 全链路

## 结构

```text
src/
├── ft_msgs/       # DetectionArray / Track / TargetSelect
├── ft_detector/   # 图像接入 + 识别 + PnP
├── ft_tracker/    # 跟踪 + 提前量
├── ft_selector/   # 进阶：目标选择
└── ft_bringup/    # launch / YAML / rviz
```

## 编译

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## 运行

```bash
roslaunch ft_bringup vision_final.launch
rosbag play --clock -l ~/bags/blue_1.bag
```

## 说明

检测部分目前只跑通图像接入与调试图，HSV/PnP 待素材到位后实现。
