# VS Code 头文件报错怎么修

## 现象

打开 `ft_detector_node.cpp`、`ft_tracker_node.cpp` 时，`#include <ros/ros.h>`、`#include <cv_bridge/cv_bridge.h>` 下面有红波浪线，提示：

```text
cannot open source file "ros/ros.h"
```

但容器里执行 `catkin_make` 是 100% 通过的。

## 原因

ROS Noetic 装在 Docker 容器里，不在宿主机：

```text
容器：/opt/ros/noetic/include/ros/ros.h   ✅
宿主机：/opt/ros/noetic/include/ros/ros.h ❌
```

VS Code 默认在宿主机做 IntelliSense，所以找不到容器里的头文件。**这不是代码错误，是编辑器路径错误。**

## 正确修法：用 Dev Container 打开

1. 在 VS Code 打开**项目根目录**，不是 `src/vision_final_test`：

```bash
code ~/ros-noetic-docker
```

2. 左下角点 `><`（Remote 按钮），选：

```text
Dev Containers: Reopen in Container
```

3. 等容器启动，VS Code 左下角会显示：

```text
Dev Container: ROS Noetic (smb)
```

4. 在容器里打开：

```text
/home/ros/catkin_ws/src/vision_final_test
```

这时 `/opt/ros/noetic/include` 存在，头文件红线会消失。

5. 编译用：

```bash
cd /home/ros/catkin_ws
catkin_make
```

## 已经补好的配置

仓库里新增：

```text
.vscode/c_cpp_properties.json
.vscode/settings.json
.vscode/tasks.json
```

- `c_cpp_properties.json`：includePath 指向 `/opt/ros/noetic/include/**`
- `compileCommands`：指向 `catkin_ws/build/compile_commands.json`
- `tasks.json`：一键 `catkin_make`

容器里已经生成：

```text
/home/ros/catkin_ws/build/compile_commands.json
```

## 如果暂时不想用容器

代码在容器里编译是正常的，红波浪线只是编辑器提示。可以临时：

```json
"C_Cpp.errorSquiggles": "disabled"
```

但不建议长期这样，会失去补全和跳转。

## 常见误区

| 做法 | 结果 |
| --- | --- |
| 在宿主机 `sudo apt install ros-noetic-...` | Ubuntu 26.04 没有 ROS Noetic 包 |
| 把 `/opt/ros` 从容器复制到宿主机 | 路径能骗过 IntelliSense，但系统头文件版本不一致，仍可能报错 |
| 直接忽略红线 | 能编译，但没有补全/跳转 |
| 用 Dev Container 打开 | 头文件、补全、调试都正常，推荐 |
