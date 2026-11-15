# AI 使用记录

姓名：许驭程  
学号：3126003452

## 工具和模型

- DeepSeek Harness（Web GUI）
- 模型：deepseek-flash

## 哪几块用了 AI

1. 仓库骨架：catkin 包、CMakeLists、package.xml、launch/YAML 样板；
2. 消息定义：Detection / DetectionArray / Track / TargetSelect；
3. 检测节点初版：图像接入、HSV 分割、灯条配对、PnP、TF、调试图；
4. 跟踪节点初版：Kalman、提前量、Marker；
5. 报错排查：catkin 找不到嵌套包、tf2 依赖、Marker 类型不匹配等；
6. 文档与 PPT 排版：README、逐行注释、学习计划。

## AI 给了什么，我改了什么

### 1. 红色阈值改成两段

AI 最初给的是单段红色 `inRange`。实际 bag 里红色在 HSV 色环跨 0°，一半红色会漏。我改成 `[red_h_min, red_h_max]` 和 `[red_h_min2, red_h_max2]` 两段，再 `bitwise_or` 合并。

### 2. 反光过滤

AI 初版没有过滤地面反光，`red_4.bag` 里地板会被当成装甲板。我加了配对框内平均亮度 `max_body_v`，超过阈值就丢弃；这个参数放进 YAML 和 dynamic_reconfigure。

### 3. 时间戳和 use_sim_time

AI 初版示例用了 `ros::Time::now()`。bag 回放时会和图像时间对不上，我改成所有消息沿用 `msg->header.stamp`，launch 默认 `use_sim_time=true`，回放加 `--clock`。

### 4. 跟踪丢检重置

AI 初版一直 predict，目标丢了位置会越飘越远。我加了 `max_lost=0.5s`、连续丢检 10 帧、外推超过 2m 就 `reset()`。

## 被否掉的 AI 方案

- 直接用现成 YOLO 检测框架：考核明确禁止直接调用现成 YOLO 框架，改为传统 HSV + 灯条几何。
- 把所有阈值写死在代码里：考核要求 YAML + dynamic_reconfigure，改为参数服务器读取。
- 用 `ros::Time::now()` 做时间戳：会导致 bag 回放时间不一致，改为使用图像 `header.stamp`。
- 用 OpenCV DNN 跑 FLOAT16 ONNX：OpenCV 4.2 不支持，改为预留 ONNX Runtime 骨架，进阶部分未接入主链路。
- 无限 predict：会漂移，改为丢检超时/外推过远 reset。

## 自检

- [x] 检测链路每行代码能解释（有逐行注释版和挖空练习）
- [x] PnP 物体点/图像点顺序能解释（README 顶点定义）
- [x] 跟踪 Q/R、门限、T 能解释（tracker.yaml + README）
- [x] 没有让 AI 读私有 bag 真值
- [x] 没有整包生成后原样提交（自己跑 bag、调参、改时间戳/反光/重置）
- [x] AI 生成的代码已 review，能指出对应文件和行号

## 实际使用记录（2026-11-06 至 11-15）

- 用 AI 生成 catkin 包骨架、消息定义、CMake/package.xml、launch/YAML 样板；
- 用 AI 实现 HSV 分割、灯条配对、PnP、TF、调试图、Kalman 跟踪初版；
- 用 AI 排查 catkin 嵌套包、tf2 依赖、Marker 类型不匹配、bag 循环时间戳回退；
- 真实 bag 调参由本人根据 `/ft/debug/image` 和 `/ft/detection` 结果决定，AI 给出候选阈值；
- 被否掉的方案：直接用现成 YOLO 框架（考核禁止）、用 OpenCV DNN 跑 FLOAT16 ONNX（版本不支持）、无限 predict（会漂移）。

## 对话/记录

- 主要对话在 DeepSeek Harness 中完成；
- 代码对应提交历史见 GitHub 仓库；
- 逐行注释版在 `docs/annotated/`，可逐行核对。
