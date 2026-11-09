# 在 VS Code 里完成 Day 1

## 先打开对的工作区

1. VS Code 打开 `~/ros-noetic-docker`；
2. 左下角 `><` → **Dev Containers: Reopen in Container**；
3. 在容器里打开：

```text
/home/ros/catkin_ws
```

## 运行任务

菜单：**Terminal → Run Task**，或 `Ctrl+Shift+P` 输入：

```text
Tasks: Run Task
```

按顺序点：

| 顺序 | 任务 | 作用 |
| --- | --- | --- |
| 1 | `ROS: Day1 1 编译` | `catkin_make` |
| 2 | `ROS: Day1 2 启动 blue_1` | 启动检测/跟踪 + 播 blue_1 |
| 3 | `ROS: Day1 3 看调试图` | 打开 rqt_image_view |
| 4 | `ROS: Day1 4 看检测频率` | `rostopic hz /ft/detection` |
| 5 | `ROS: Day1 5 停止` | 停掉全部 |
| 6 | `ROS: Day1 6 提交` | git add/commit/push |

换 `red_4`：点 `ROS: Day1 2b 启动 red_4`。

## 任务没出现

- 确认 VS Code 打开的是 `catkin_ws`，不是 `vision_final_test`；
- `Ctrl+Shift+P` → `Developer: Reload Window`；
- 确认左下角显示 `Dev Container: ROS Noetic (smb)`。

## 看到什么算成功

- 编译：`[100%] Built target ft_detector_node`；
- 启动：终端出现 `process[ft_detector-1]: started`；
- 频率：`average rate: ...` 有数字；
- 调试图：框落在装甲板上，不是地板上。

## 停止

先点 `ROS: Day1 5 停止`，再关终端；不要直接关 VS Code，否则容器里进程可能还在。
