# 项目长期记忆 · ROS 锥桶分色

## 交付目标
FSAE 无人方程式入社任务：macOS + Apple Silicon 上用 ROS 1 Noetic 写 C++ 节点
`cone_color_guesser`，输入 `timu.bag` 的无色锥桶点云，按赛道规则「左红右蓝」猜色，
发布到 `/cones/colored`。**任务已完成并验证通过。**

## 项目约定
- 改代码前先查 `ROS自学与锥桶分色实战指南.md` 对应章节，不要凭空设计。
- 所有阈值参数放 `config/params.yaml`，不要硬编码。
- 两套 Python 环境严禁混用：micromamba `ros_noetic`（ROS/C++）与 uv `.venv`（仅 `tools/` 脚本）。
- **对现有文件做外科式修改，不要推倒重写**（用户明确偏好）。
- **未经明确要求，不要生成任何产物文件**（压缩包、报告、导出文件等）。用户会自己决定要不要打包——
  只在被要求时动手；"回答问题"和"产出文件"是两件事。

## 交付 / 打包约定
- **交付对象 = `catkin_ws/src/cone_color_guesser/` 这一个功能包**（6 个文件 / 64 KB：
  `CMakeLists.txt`、`package.xml`、`src/cone_color_guesser_node.cpp`、`config/params.yaml`、
  `launch/guess.launch`、`rviz/cone.rviz`）。
- **接收方是 Linux**（Ubuntu + ROS 1 Noetic，`catkin_make` 后直接跑），不是 macOS。
- **明确不要打包**：指南/方案等文档、`figures/`、`tools/`、`.venv`/uv 层、`catkin_ws/build|devel`、
  `catkin_ws/src/CMakeLists.txt`（本机 micromamba 符号链接；Linux 侧自己 `catkin_init_workspace`）。
- Linux 兼容性实测：`CMakeLists.txt` 的 Apple 块有 `if(APPLE AND ...)` 守卫，Linux 上自动跳过，**可移植**；
  `package.xml` 声明齐全（roscpp/sensor_msgs/pcl_ros/pcl_conversions/visualization_msgs/tf2_ros/tf2_sensor_msgs），
  但 launch 用到的 `rosbag`、`rviz` 未声明（desktop-full 自带，无需处理）。
- `launch/guess.launch:14` 原本硬编码 macOS 路径 `$(env HOME)/Documents/Python_Project/ROS/timu.bag`，
  是**唯一的跨平台障碍**。

### 最终交付形态（已定稿并实测）
- **zip 顶层就是功能包本身**（不是 `catkin_ws`，也不套一层），对方解压后把 `cone_color_guesser/` 整个
  拷进自己的 `catkin_ws/src/`。`catkin_ws/build|devel` 一律不带（`CMakeCache.txt` 含本机绝对路径、
  `devel` 里是 arm64 Mach-O，带到 Linux 必坏事）。
- **`timu.bag` 放进包内** `bag/timu.bag`（4.2 MB），launch 默认值改为
  `default="$(find cone_color_guesser)/bag/timu.bag"` → 对方零参数 `roslaunch` 即可跑。
  **该改动只做在交付副本上，项目里的 `guess.launch` 保持原样未动**（避免影响本机 `tools/` 工作流）。
- **`include/cone_color_guesser/` 是承重目录，不能丢**：实测删掉它 `catkin_package(INCLUDE_DIRS include)`
  直接报 `include dir 'include' does not exist` → 编译失败。空目录容易被 zip/解压工具丢掉，
  故在包内放 `include/cone_color_guesser/.gitkeep` 占位。包内那个空的 `tools/` 目录无用，已删。
- 交付包内附 `README.md`。**用户明确要求极简：只写目录结构**（22 行 / 766 B），
  不含依赖/编译/话题/实现要点/已知限制等章节——那些被用户视为「奇怪的东西」。
  措辞按「交作业给学长」的分寸（开头打招呼、用「您」）。
- 打包用 `zip -r -X`（`-X` 去掉 macOS 扩展属性），并先清 `.DS_Store`；包内文件名全 ASCII，
  避免 Linux 解压乱码。

## 关键技术结论（实测，可复用于同类问题）
1. **锥桶间隔 2.31 m < 赛道宽 3.07 m** → 「按距离就近配对」判左右必然不可靠（74% 配成同侧）。
   正确做法：用**车辆自身走过的路径切向**作为行进方向基准。
2. **投票锁定必须锚定可信观测**：只要观测满 N 次就永久锁定，会被最早几帧（数据最差时）锁死。
   要区分「实测切向的满权票」与「近似的降权票」，并要求满权票数量。
3. **`pcl::PointXYZRGB` 必须 `p.a = 255`**，否则 rgb float 可能是 NaN/Inf，
   下游 `skip_nans=True` 会整批丢点。
4. **ROS 的 TF 查询不要给长超时**：时钟不同步时按 stamp 查询必然失败，`Duration(0.2)`
   会让每帧白等 200 ms；用 `Duration(0.0)` 非阻塞尝试再回退。
5. macOS 上跑 ROS 用 `micromamba run -p <prefix> bash -c '...'`；
   `devel/setup.bash` 引用未定义变量，脚本里不能开 `set -u`。
6. **「边行驶边上色」靠显示分级实现**：`UNKNOWN → PROVISIONAL → LOCKED`。
   锁定要求满权票（只在驶过后产生），若"只发 locked"就会出现「要等车开过去才上色」；
   必须把**显示**与**锁定**解耦：未锁定但票够的用累积多数票显示降亮暂定色。
7. **「车头朝向」判左右的可靠范围只有 8 m**（实测 0-5 m 97%、5-8 m 81%、8-12 m 53%），
   所以前方暂定色的 `head_range` 只能是 8 m，再远宁可不显示。
8. **轨迹关联门限要大于观测抖动**：0.9 m 会把同一个锥桶拆成多条轨迹（输出重复点），
   1.2 m 合适（锥桶间最近也隔约 2 m，不会误并）。

## 验证工作流（本项目已沉淀为脚本）
1. `tools/diag_geometry.py` —— 实测赛道几何/轨迹/锥桶间距。
2. `tools/sim_algorithms.py` —— 用 `rosbags` 离线**因果回放**，对比多种算法的准确率/翻转率，
   改 C++ 之前先在这里确定算法和参数（`SWEEP=1` 跑参数扫描）。
3. `tools/run_live_test.sh` —— roscore + 节点 + 播 bag，`tools/capture_colored.py`
   抓 `/cones/colored`，`colored.txt.trend` 用来看输出是否收敛稳定。
