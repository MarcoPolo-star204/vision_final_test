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
- [x] 合成图像链路验证（检测、TF、动态参数、跟踪）
- [ ] 用真实 bag 调 HSV/配对/尺寸参数
- [ ] 2–3 分钟演示视频
- [ ] 进阶：ONNX / 旋转模型 / 全链路
- [ ] 完整参数表、ai_usage、tag、zip

## 结构

```text
src/
├── ft_msgs/       # Detection / DetectionArray / Track / TargetSelect
├── ft_detector/   # 图像接入 + 识别 + PnP + debug
├── ft_tracker/    # 跟踪 + 提前量
├── ft_selector/   # 目标选择
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
# 另开终端
rosbag play --clock -l ~/bags/blue_1.bag
```

查看：

```bash
rqt_image_view /ft/debug/image
rostopic echo /ft/detection
rostopic echo /ft/track
rostopic echo /ft/target
```

## 合成图像测试（无 bag）

```bash
roscore
rosrun ft_detector ft_detector_node
rosrun ft_tracker ft_tracker_node
rosrun ft_selector ft_selector_node
python3 test/fake_camera.py
```

已验证输出：

```text
/ft/detection: id=0, is_red=true, is_large_armor=false, z≈0.343 m
/ft/track: tracking=true, position/velocity/position_predict 正常
/tf: camera → armor_0, translation [-0.040, -0.062, 0.343]
dynamic_reconfigure: 修改 red_h_max/min_area 生效
```

合成图只验证链路，不代表真实识别效果。

## 参数

### detector.yaml

| 参数 | 含义 |
| --- | --- |
| `image_topic` / `camera_info_topic` | 输入话题 |
| `queue_size` | 图像队列，默认 2 |
| `large_armor_width/height` | 235×127 mm |
| `small_armor_width/height` | 120×125 mm |
| `large_gap_ratio` | 大/小装甲板间距/灯条高度阈值 |
| `min_aspect` / `max_aspect` | 灯条高宽比范围 |
| `min_fill` | 轮廓面积/外接矩形面积 |
| `red_h_min/max` + `red_h_min2/max2` | 红色两段 H |
| `blue_h_min/max` | 蓝色 H |
| `s_min` / `v_min` | S/V 下限 |
| `min_area` | 最小轮廓面积 |
| `max_angle_diff` | 配对灯条最大角度差 |
| `max_height_diff` | 配对灯条最大高度差比例 |
| `max_gap` | 配对最大间距（像素） |
| `fallback_camera_matrix` / `fallback_distortion` | 没有 camera_info 时的回退值 |

### tracker.yaml

| 参数 | 含义 |
| --- | --- |
| `T` | 提前量，默认 0.1 s |
| `gate` | 数据关联门限，默认 0.4 m |
| `q_pos` / `q_vel` | 过程噪声 |
| `r_meas` | 观测噪声 |
| `max_lost` | 连续丢失删除时间，默认 0.5 s |

## 顶点定义

采用“装甲板本体四角”定义：

- 左上：左灯条最左侧上端点
- 右上：右灯条最右侧上端点
- 右下：右灯条最右侧下端点
- 左下：左灯条最左侧下端点

物体点顺序固定为：左上 → 右上 → 右下 → 左下；原点在左上，x 向右，y 向下，z 垂直板面向外。单位米。

## 已知限制

- 还没有真实 bag，HSV、配对、`large_gap_ratio`、尺寸需要重新标定。
- `solvePnP` 使用 `SOLVEPNP_IPPE`，平面目标。
- 跟踪只做单目标匀速模型；换面、旋转、多车选择属于进阶。
- 检测 ID 只保证帧内唯一，不做跨帧关联。
