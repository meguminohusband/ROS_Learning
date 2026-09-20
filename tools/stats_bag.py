"""统计 timu.bag 点云的空间分布，并找出杂点的特征。

用法：
    cd ~/Documents/Python_Project/ROS
    ./.venv/bin/python tools/stats_bag.py
"""
from pathlib import Path
import numpy as np
from rosbags.highlevel import AnyReader

BAG = Path("timu.bag")


def parse_points(msg):
    """把 PointCloud2 的二进制数据解析成 N x 3 的数组。

    每个点占 16 字节 = 4 个 float32（x, y, z + 1 个填充位），
    我们只要前三个。
    """
    arr = np.frombuffer(msg.data, dtype=np.float32).reshape(-1, 4)
    return arr[:, :3].astype(np.float64)


with AnyReader([BAG]) as reader:
    conns = [c for c in reader.connections
             if c.topic == "/only_lidar_points_pub"]
    clouds = []
    for conn, ts, raw in reader.messages(connections=conns):
        clouds.append(parse_points(reader.deserialize(raw, conn.msgtype)))

print(f"总帧数: {len(clouds)}")

# ---------- 1. 每帧点数 ----------
counts = np.array([len(c) for c in clouds])
print(f"每帧点数: 最少 {counts.min()}, 最多 {counts.max()}, "
      f"平均 {counts.mean():.0f}")

# ---------- 2. 坐标范围 ----------
ALL = np.vstack(clouds)
print("\n全部点的坐标范围:")
for i, name in enumerate("xyz"):
    v = ALL[:, i]
    print(f"  {name}: [{v.min():7.2f}, {v.max():7.2f}]  平均 {v.mean():6.2f}")

# ---------- 3. ★ 帧内最近邻距离（最重要！）----------
print("\n帧内最近邻距离（每个点到同帧最近点的距离）:")
nn_list = []
for c in clouds:
    if len(c) < 2:
        continue
    d = np.linalg.norm(c[:, None, :] - c[None, :, :], axis=2)
    np.fill_diagonal(d, np.inf)          # 排除自己
    nn_list.append(d.min(axis=1))

nn = np.concatenate(nn_list)
print(f"  样本数 {len(nn)}")
for q in [5, 10, 25, 50, 75, 90]:
    print(f"    {q:2d}% 的点，最近邻距离 < {np.percentile(nn, q):6.3f} m")

# ---------- 4. 结论 ----------
close = (nn < 0.5).sum()
print(f"\n最近邻 < 0.5m 的点: {close} 个 "
      f"({100 * close / len(nn):.1f}%)")
print("  ↑ 这群点挤在一起，就是题目说的杂点；")
print("    其余点间距 2~3 m，是正常的赛道锥桶。")
