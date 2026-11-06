# vision_final_test

装甲板自瞄系统考核（ROS1 Noetic + OpenCV 4.2）。

## 当前进度

- [x] 仓库骨架、消息包（DetectionArray / Track / TargetSelect）
- [x] 图像接入：compressed + camera_info，header 使用图像时间戳
- [x] HSV 红/蓝分割、灯条轮廓、灯条配对、大小装甲板判定
- [x] solvePnP、TF、projectPoints 坐标轴、调试图
- [x] `/ft/detection`、`/ft/debug/image`
- [x] YAML 参数 + dynamic_reconfigure 阈值
- [x] 单目标 Kalman 跟踪、提前量、RViz Marker、`/ft/track`
- [x] `/ft/target` 基础选择节点
- [ ] 用真实 bag 调 HSV/配对/尺寸参数
- [ ] 2–3 分钟演示视频
- [ ] 进阶：ONNX / 旋转模型 / 全链路
- [ ] README 完整参数表、ai_usage、tag

## 依赖

```bash
sudo apt install ros-noetic-cv-bridge ros-noetic-image-transport \
  ros-noetic-image-transport-plugins ros-noetic-dynamic-reconfigure \
  ros-noetic-tf2-ros ros-noetic-visualization-msgs
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
# 另开终端
rosbag play --clock -l ~/bags/blue_1.bag
```

查看：

```bash
rqt_image_view /ft/debug/image
rostopic echo /ft/detection
rostopic echo /ft/track
rviz   # 已由 launch 启动
```

## 合成图像测试（无 bag 时）

```bash
rosrun ft_detector ft_detector_node &
rosrun ft_tracker ft_tracker_node &
rosrun ft_selector ft_selector_node &
python3 test/fake_camera.py
rostopic echo /ft/detection
```

合成图是两根基线段，用于验证图像接入、配对、PnP、消息和跟踪链路，不代表真实识别效果。

## 顶点定义

采用“装甲板本体四角”定义：

- 左上：左灯条最左侧上端点
- 右上：右灯条最右侧上端点
- 右下：右灯条最右侧下端点
- 左下：左灯条最左侧下端点

物体点顺序固定为：左上 → 右上 → 右下 → 左下；原点在左上，x 向右，y 向下，z 垂直板面向外。单位米。

## 已知限制

- 还没有真实 bag，HSV、配对、`large_gap_ratio`、尺寸都需要用 `blue_1.bag` / `red_4.bag` 重新标定。
- `solvePnP` 使用 `SOLVEPNP_IPPE`，平面目标；如果 OpenCV 版本不支持会回退/报错。
- 跟踪只做单目标匀速模型；换面、旋转、多车选择属于进阶。
