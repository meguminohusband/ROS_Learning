# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 项目背景

无人方程式社团（FSAE）入社任务：在 **macOS + Apple Silicon** 上用 **ROS 1 Noetic**（RoboStack/micromamba 安装）写一个 C++ 节点 `cone_color_guesser`，输入 `timu.bag` 里的无色锥桶点云，「猜」出颜色（赛道规则：**左红右蓝**）并发布到 `/cones/colored`。

`ROS自学与锥桶分色实战指南.md` 是核心参考文档（3300+ 行），包含：数据解剖结论、环境搭建全过程、算法设计（§5）、节点完整实现与参数文件（§6）、命令速查表（附录 B）、报错排查表（附录 C）。改代码前先查对应章节，不要凭空设计。

## 两套独立的 Python 环境（勿混用）

1. **micromamba 环境 `ros_noetic`** —— 编译/运行 ROS 与 C++ 节点用
2. **uv 虚拟环境 `.venv`**（Python 3.12，见 `.python-version`）—— 仅用于 `tools/` 下的 bag 分析脚本（依赖 `rosbags`，无需 ROS）

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

### 一键启动（推荐）

```bash
roslaunch cone_color_guesser guess.launch              # 参数 + 播 bag + 节点 + RViz
roslaunch cone_color_guesser guess.launch rviz:=false  # 纯终端验证，不开 RViz
roslaunch cone_color_guesser guess.launch loop:=true   # 循环播放，方便反复观察
```

`guess.launch` 已带 `use_sim_time`、`--clock`、`--wait-for-subscribers`（等订阅者连上再播，
避免丢开头几帧）。改参数只改 `config/params.yaml`，不用重新编译。

### Python 分析（.venv 侧）

```bash
uv sync                                         # 安装依赖
./.venv/bin/python tools/inspect_bag.py         # 看 bag 话题/点云结构
./.venv/bin/python tools/stats_bag.py           # 点云统计、杂点特征分析
./.venv/bin/python tools/diag_geometry.py       # 赛道几何/轨迹/锥桶间距实测
./.venv/bin/python tools/sim_algorithms.py      # 因果回放，对比各左右判定算法的准确率与翻转率
SWEEP=1 ./.venv/bin/python tools/sim_algorithms.py   # 额外跑参数扫描，确认算法对参数不敏感
```

这些脚本都用相对路径读 `timu.bag`，必须在项目根目录运行；中间产物写在 `tools/_*.npz`（已 gitignore）。

### 端到端实跑验证（抓取节点真实输出）

```bash
micromamba activate ros_noetic
bash tools/run_live_test.sh /tmp/cone_live      # roscore + 节点 + 播 bag，抓取 /cones/colored
```

`tools/capture_colored.py` 会把 `/cones/colored` 的每一帧覆盖写入 `colored.txt`，
并追加一行 `时间 总数 红 蓝` 到 `colored.txt.trend`——后者用来判断输出是否已经收敛稳定。

## 关键数据事实（均为实测，见指南 §1.3/§4.3）

- 话题：`/only_lidar_points_pub`（PointCloud2，10 Hz，600 帧）和 **`/tf`**（题目描述写的 `/tf2` 是错的）
- 点云 frame_id 是 `imu_map`（地图系），字段只有 x/y/z，每帧 10~96 个点
- **一个点 = 一个锥桶**（间距约 2.5 m）——绝不要用欧氏聚类/降采样等稠密点云套路
- ⚠️ **锥桶沿赛道的间隔（实测中位 2.31 m）比赛道宽（3.07 m）还小**，两个尺度重叠。
  所以「按帧内距离就近配对」是不可靠的：实测 74% 的最近邻是**同一侧**的相邻锥桶，
  配对中点根本不在赛道中线上。判左右不要走这条路（指南 §5.4 的 v3 方案在本数据上有这个坑）
- ⚠️ **「车头朝向」判左右只在 8 m 内可靠**：实测准确率 0–5 m = 97.4%、5–8 m = 81.1%、
  8–12 m 只有 **53.1%（近乎随机）**。所以用朝向给前方锥桶上「暂定色」的 `head_range` 只能取 8 m，
  再远宁可不显示。测量脚本：`tools/measure_head_range.py`
- 杂点识别：帧内最近邻 < 0.5 m 即杂点；z > 0.9 m 丢弃
- 左右判断必须先经 TF 转到 `imu_link`（此 bag 没有 base_link），REP-103：y > 0 = 车左 = 红
- 赛道宽约 3.07 m；所有阈值参数放 `config/params.yaml`，不要硬编码
- **bag 时钟错位**：点云 stamp（2025-09）比 `/tf` stamp（2025-03）快约 171.78 天（录制机器未对时）。跑节点必须 `rosbag play --clock` + `/use_sim_time=true`，且 `transformToBody()` 按 stamp 查询失败时回退 `ros::Time(0)`（已实现，详见指南 §6.7「bag 时钟错位」）

## 环境踩坑（改动环境前必读）

- **RoboStack 包版本钉死**：`ros-noetic-desktop=1.5.0=np126py311h7b59bab_22` + python 3.11。2026 年的 py312 新构建在 Apple Silicon 上会让 `rosrun`/`rospack` 崩溃（`dyld: symbol not found ... __Py_NoneStruct`），不要升级或删版本号
- **cmake 必须 < 4**，否则 `catkin_make` 在 `cmake_minimum_required` 处报错
- **~/.zshrc 必须设** `ROS_MASTER_URI`/`ROS_HOSTNAME`/`ROS_IP` 均为 `127.0.0.1`（macOS 上不设会报 `RLException: Unable to contact my own server`）
- `source devel/setup.zsh` 必须放在 `micromamba activate` **之后**
- C++ 编译器必须来自 conda 环境（`compilers` 包），与系统 Apple Clang 混用会出 `Undefined symbols for architecture arm64`
- 终端必须是原生 arm64（`uname -m` 验证），Rosetta 模式装出来的是 x86 版
- **TF 查询超时会拖垮帧率**：本 bag 的 TF stamp 比点云慢 171.78 天，按 stamp 查询必然失败。
  若这一跳给 `ros::Duration(0.2)` 超时，就会**每帧白等 200 ms**，节点只能跑到 5 Hz 跟不上 10 Hz 的点云（实测每帧耗时 205 ms）。必须用 `ros::Duration(0.0)` 非阻塞尝试再回退
- **`pcl::PointXYZRGB` 的 alpha 字节必须显式初始化**（`p.a = 255`）：`rgb` 是一个 float，
  a 字节若保持未初始化的随机值，打包出的 float 可能是 NaN/Inf，任何用 `skip_nans=True` 读点云的下游都会把这些点**整批丢掉**（实测表现为红色点全部消失、只剩蓝色点）
- **macOS 上跑 ROS 命令建议用 `micromamba run -p <prefix> bash -c '...'`**：`micromamba shell hook -s zsh` 在非交互 zsh 里会因 `compdef` 报错；且 `devel/setup.bash` 引用了未定义变量，脚本里不能开 `set -u`
- **launch：`<rosparam command="load">` 必须写在 `<node>` 标签【里面】**，才会加载到节点的私有命名空间
  （`~` = `/cone_color_guesser`）。放在 `<launch>` 顶层会加载到全局 `/`，而节点用 `nh_("~")` 读参数，
  结果是「参数文件根本不生效，节点静默退回代码内置默认值」。可用 `rosparam list | grep cone_color_guesser` 自检
- **XML 注释里不能出现 `--`**：写 `<!-- ---------- 分节 ---------- -->` 或注释里提 `--clock` 都会让
  roslaunch 报 `Invalid roslaunch XML syntax: not well-formed`。分节线用 `=====` 代替
- **`rosbag play` 的 output 用 `log` 不要用 `screen`**：它的进度条靠 `\r` 原地刷新，在 roslaunch 里会被
  逐行加前缀变成刷屏（实测 22 秒刷出 233 KB），把节点自己的日志淹掉

## 目录结构要点

- `catkin_ws/src/hello_ros/` —— 已跑通的里程碑示例包（roscpp，实测可收到点云回调），可作为新包模板
- `cone_color_guesser` —— **最终交付包，已实现并验证通过**（节点 `cone_color_guesser_node`）。
  算法不是指南 §5.4 的「配对 + 相邻中点方向」，而是**以「车辆在地图系下走过的路径」的切向**
  作为行进方向的基准：把锥桶投影到路径上，左侧 = z × d，侧向偏移符号即颜色；
  路径自地图原点（车辆出发点）起算。配合加权投票（只有投影落在「已驶过路径」内部的观测
  才算满权票）。
  **显示分三级**（`UNKNOWN → PROVISIONAL → LOCKED`），做到「边行驶边上色」：
  前方 ≤ `head_range`(8 m) 的锥桶用累积多数票显示**降亮半透明暂定色**，驶过后升级为
  **实心全亮锁定色**（永久不变）。这样不会出现「要等车开过去锥桶才出现颜色」。
  实测：锁定色显示正确率 100%、暂定色 99.16%、零判错、全程仅 1 次翻转，赛道锥桶 62/62 覆盖。
  设计依据与实测数据见 `锥桶分色算法优化方案.md`；参数全在 `config/params.yaml`
- `tools/` —— rosbags 离线分析脚本 + 实跑验证脚本（不依赖 ROS，除 `capture_colored.py`）；`figures/` —— 分析图
- `src/ros/` —— uv 脚手架生成的占位 Python 包，与 ROS C++ 工作无关
- `y/micromamba` —— 安装时下载的 micromamba 二进制，未被 .gitignore 覆盖；`catkin_ws/build|devel` 为生成物，git 只需跟踪 `catkin_ws/src`
