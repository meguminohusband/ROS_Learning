# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 项目背景

无人方程式社团（FSAE）入社任务：在 **macOS + Apple Silicon** 上用 **ROS 1 Noetic**（RoboStack/micromamba 安装）写一个 C++ 节点 `cone_color_guesser`，输入 `timu.bag` 里的无色锥桶点云，「猜」出颜色（赛道规则：**左红右蓝**）并发布到 `/cones/colored`。

`ROS自学与锥桶分色实战指南.md` 是核心参考文档（3300+ 行），包含：数据解剖结论、环境搭建全过程、算法设计（§5）、节点完整实现与参数文件（§6）、命令速查表（附录 B）、报错排查表（附录 C）。改代码前先查对应章节，不要凭空设计。

## 两套独立的 Python 环境（勿混用）

1. **micromamba 环境 `ros_noetic`** —— 编译/运行 ROS 与 C++ 节点用
2. **uv 虚拟环境 `.venv`**（Python 3.14）—— 仅用于 `tools/` 下的 bag 分析脚本（依赖 `rosbags`，无需 ROS）

## 常用命令

### C++ / catkin（ROS 侧）

```bash
micromamba activate ros_noetic                  # 每个新终端都要先激活
cd catkin_ws && catkin_make                     # 全部编译
catkin_make --pkg <包名>                        # 只编译一个包
source devel/setup.zsh                          # ★ 编译后必须 source，否则 rospack 找不到包

roscore                                         # 终端 1
rosbag play timu.bag --loop --pause             # 终端 2，在项目根目录播放
rosrun <包名> <节点名>                          # 终端 3
```

### Python 分析（.venv 侧）

```bash
uv sync                                         # 安装依赖
./.venv/bin/python tools/inspect_bag.py         # 看 bag 话题/点云结构
./.venv/bin/python tools/stats_bag.py           # 点云统计、杂点特征分析
```

两个脚本都用相对路径读 `timu.bag`，必须在项目根目录运行。

## 关键数据事实（均为实测，见指南 §1.3/§4.3）

- 话题：`/only_lidar_points_pub`（PointCloud2，10 Hz，600 帧）和 **`/tf`**（题目描述写的 `/tf2` 是错的）
- 点云 frame_id 是 `imu_map`（地图系），字段只有 x/y/z，每帧 10~96 个点
- **一个点 = 一个锥桶**（间距约 2.5 m）——绝不要用欧氏聚类/降采样等稠密点云套路
- 杂点识别：帧内最近邻 < 0.5 m 即杂点；z > 0.9 m 丢弃
- 左右判断必须先经 TF 转到 `imu_link`（此 bag 没有 base_link），REP-103：y > 0 = 车左 = 红
- 赛道宽约 3.07 m；所有阈值参数放 `config/params.yaml`，不要硬编码

## 环境踩坑（改动环境前必读）

- **RoboStack 包版本钉死**：`ros-noetic-desktop=1.5.0=np126py311h7b59bab_22` + python 3.11。2026 年的 py312 新构建在 Apple Silicon 上会让 `rosrun`/`rospack` 崩溃（`dyld: symbol not found ... __Py_NoneStruct`），不要升级或删版本号
- **cmake 必须 < 4**，否则 `catkin_make` 在 `cmake_minimum_required` 处报错
- **~/.zshrc 必须设** `ROS_MASTER_URI`/`ROS_HOSTNAME`/`ROS_IP` 均为 `127.0.0.1`（macOS 上不设会报 `RLException: Unable to contact my own server`）
- `source devel/setup.zsh` 必须放在 `micromamba activate` **之后**
- C++ 编译器必须来自 conda 环境（`compilers` 包），与系统 Apple Clang 混用会出 `Undefined symbols for architecture arm64`
- 终端必须是原生 arm64（`uname -m` 验证），Rosetta 模式装出来的是 x86 版

## 目录结构要点

- `catkin_ws/src/hello_ros/` —— 已跑通的里程碑示例包（roscpp，实测可收到点云回调），可作为新包模板
- `cone_color_guesser` —— **最终交付包，尚未创建**，完整设计在指南 §6；算法迭代路线：v1 中位数切分 → v2 k-means → v3 配对+赛道方向 → v4 时序投票（§5.4）
- `tools/` —— rosbags 离线分析脚本（不依赖 ROS）；`figures/` —— 分析图
- `src/ros/` —— uv 脚手架生成的占位 Python 包，与 ROS C++ 工作无关
- `y/micromamba` —— 安装时下载的 micromamba 二进制，未被 .gitignore 覆盖；`catkin_ws/build|devel` 为生成物，git 只需跟踪 `catkin_ws/src`
