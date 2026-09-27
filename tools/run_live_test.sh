#!/usr/bin/env bash
# ============================================================
#  端到端实跑验证：roscore + cone_color_guesser + 播放 timu.bag，
#  并抓取节点实际发布的 /cones/colored，供离线核对。
#
#  用法（需先进入 ROS 环境）：
#     micromamba activate ros_noetic
#     bash tools/run_live_test.sh [输出目录]
# ============================================================
set +u    # devel/setup.bash 会引用未定义变量（ZSH_VERSION 等），不能开 -u

PROJ="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS="$PROJ/catkin_ws"
OUT="${1:-/tmp/cone_live}"

mkdir -p "$OUT"
# shellcheck disable=SC1091
source "$WS/devel/setup.bash"

export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_HOSTNAME=127.0.0.1
export ROS_IP=127.0.0.1

CORE_PID=""
NODE_PID=""
CAP_PID=""
BAG_PID=""

cleanup() {
    [ -n "$BAG_PID" ]  && kill "$BAG_PID"  2>/dev/null
    [ -n "$CAP_PID" ]  && kill "$CAP_PID"  2>/dev/null
    [ -n "$NODE_PID" ] && kill "$NODE_PID" 2>/dev/null
    [ -n "$CORE_PID" ] && kill "$CORE_PID" 2>/dev/null
    sleep 1
}
trap cleanup EXIT

echo "==> 启动 roscore"
roscore > "$OUT/roscore.log" 2>&1 &
CORE_PID=$!

echo "==> 等待 master 就绪"
for _ in $(seq 1 60); do
    if rostopic list >/dev/null 2>&1; then break; fi
    sleep 1
done
if ! rostopic list >/dev/null 2>&1; then
    echo "!! master 起不来，见 $OUT/roscore.log"; exit 1
fi

# bag 的时间戳比 /tf 快 171.78 天，必须用仿真时间
rosparam set /use_sim_time true

echo "==> 启动 cone_color_guesser 节点"
rosrun cone_color_guesser cone_color_guesser_node > "$OUT/node.log" 2>&1 &
NODE_PID=$!
sleep 3

echo "==> 启动抓取器（/cones/colored）"
python "$PROJ/tools/capture_colored.py" "$OUT/colored.txt" > "$OUT/capture.log" 2>&1 &
CAP_PID=$!
sleep 3

echo "==> 播放 timu.bag（--clock，单次）"
rosbag play --clock "$PROJ/timu.bag" > "$OUT/bag.log" 2>&1 &
BAG_PID=$!
wait "$BAG_PID"

echo "==> bag 播放结束，等待 3 s 收尾"
sleep 3

echo "==> 结果："
echo "    节点日志 : $OUT/node.log"
echo "    抓取结果 : $OUT/colored.txt"
wc -l "$OUT/colored.txt" 2>/dev/null || true
