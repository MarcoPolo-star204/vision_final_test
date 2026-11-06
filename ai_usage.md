# AI 使用记录

## 工具和模型

- DeepSeek Harness（Web GUI）
- 模型：deepseek-flash

## 哪几块用了 AI

1. 仓库骨架：catkin 包、CMakeLists、package.xml、launch/YAML 样板；
2. 消息定义：Detection / DetectionArray / Track / TargetSelect；
3. 检测节点初版：图像接入、HSV 分割、灯条配对、PnP、TF、调试图；
4. 跟踪节点初版：Kalman、提前量、Marker；
5. 报错排查：catkin 找不到嵌套包、tf2 依赖、Marker 类型不匹配等。

## AI 给了什么，我改了什么

（待本人补充 review 记录：至少写 3 处自己改动/确认的地方，例如阈值、坐标系、参数默认值、代码结构。）

## 被否掉的 AI 方案

- 直接用现成 YOLO 检测框架：考核明确禁止直接调用现成 YOLO 框架，改为传统 HSV + 灯条几何。
- 把所有阈值写死在代码里：考核要求 YAML + dynamic_reconfigure，改为参数服务器读取。
- 用 `ros::Time::now()` 做时间戳：会导致 bag 回放时间不一致，改为使用图像 header.stamp。

## 自检

- [ ] 检测链路每行代码能解释
- [ ] PnP 物体点/图像点顺序能解释
- [ ] 跟踪 Q/R、门限、T 能解释
- [ ] 没有让 AI 读私有 bag 真值
- [ ] 没有整包生成后原样提交
