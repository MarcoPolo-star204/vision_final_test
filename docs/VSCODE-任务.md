# 在 VS Code 里完成 Day 1

## 打开方式

VS Code 直接打开：

```text
/home/marco/ros-noetic-docker/catkin_ws
```

不需要进 Dev Container。任务会用 `docker exec` 自动进 `ros-noetic` 容器执行。

## 运行任务

`Ctrl+Shift+P` → 输入：

```text
Tasks: Run Task
```

按顺序点：

| 顺序 | 任务 | 作用 |
| --- | --- | --- |
| 1 | `ROS: Day1 1 编译` | 容器内 `catkin_make` |
| 2 | `ROS: Day1 2 启动 blue_1` | 启动节点 + 播 blue_1 |
| 3 | `ROS: Day1 3 看调试图` | 容器里开 `rqt_image_view` |
| 4 | `ROS: Day1 4 看检测频率` | `rostopic hz /ft/detection` |
| 5 | `ROS: Day1 5 停止` | 停掉全部 |
| 6 | `ROS: Day1 6 提交` | git add/commit/push |

换 red_4：点 `ROS: Day1 2b 启动 red_4`。

## 看到什么算成功

- 编译：`[100%] Built target ft_detector_node`；
- 启动：终端显示 `已启动`，bag 在跑；
- 频率：`average rate: ...` 有数字；
- 调试图：框落在装甲板上。

## 任务没出现

`Ctrl+Shift+P` → `Developer: Reload Window`。

## 停止

一定先点 `ROS: Day1 5 停止`，再关 VS Code。

## 日志

```text
/tmp/day1_launch.log
/tmp/day1_bag.log
```

任务一闪就退时，先看这两个文件。
