"""核实 timu.bag 的赛道几何、车辆轨迹与锥桶分布（离线，不依赖 ROS）。

用法：
    cd ~/Documents/Python_Project/ROS
    ./.venv/bin/python tools/diag_geometry.py
"""
from pathlib import Path
import numpy as np
from rosbags.highlevel import AnyReader

BAG = Path("timu.bag")
CLOUD = "/only_lidar_points_pub"
TFS = "/tf"


def parse_cloud(msg):
    arr = np.frombuffer(msg.data, dtype=np.float32).reshape(-1, 4)
    return arr[:, :3].astype(np.float64)


def to_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def quat_to_yaw(x, y, z, w):
    return np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def cluster(pts, thr=0.6):
    """网格哈希 + 邻域合并，把同一锥桶的重复观测并成一个中心。"""
    centers, cnt, grid = [], [], {}
    for p in pts:
        key = (int(np.floor(p[0] / thr)), int(np.floor(p[1] / thr)))
        hit = -1
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for idx in grid.get((key[0] + dx, key[1] + dy), ()):
                    if np.hypot(centers[idx][0] - p[0], centers[idx][1] - p[1]) < thr:
                        hit = idx
                        break
                if hit >= 0:
                    break
            if hit >= 0:
                break
        if hit >= 0:
            c = centers[hit]
            c[:] = (c * cnt[hit] + p) / (cnt[hit] + 1)
            cnt[hit] += 1
        else:
            centers.append(p.astype(float))
            cnt.append(1)
            grid.setdefault(key, []).append(len(centers) - 1)
    return np.array(centers), np.array(cnt)


def nearest_on_polyline(A, B, AB, L2, Q):
    """Q 到折线的最近距离、最近点、该处切向。"""
    best_d = np.full(len(Q), np.inf)
    best_i = np.zeros(len(Q), dtype=int)
    best_t = np.zeros(len(Q))
    for i in range(len(A)):
        ap = Q - A[i]
        t = np.clip((ap * AB[i]).sum(1) / L2[i], 0.0, 1.0)
        proj = A[i] + t[:, None] * AB[i]
        dd = np.hypot(Q[:, 0] - proj[:, 0], Q[:, 1] - proj[:, 1])
        m = dd < best_d
        best_d[m] = dd[m]
        best_i[m] = i
        best_t[m] = t[m]
    near = A[best_i] + best_t[:, None] * AB[best_i]
    tang = AB[best_i] / np.sqrt(L2[best_i])[:, None]
    return best_d, near, tang


with AnyReader([BAG]) as reader:
    cloud_conns = [c for c in reader.connections if c.topic == CLOUD]
    tf_conns = [c for c in reader.connections if c.topic == TFS]

    clouds, c_hdr = [], []
    for conn, ts, raw in reader.messages(connections=cloud_conns):
        msg = reader.deserialize(raw, conn.msgtype)
        clouds.append(parse_cloud(msg))
        c_hdr.append(to_sec(msg.header.stamp))
        frame_id = msg.header.frame_id

    tx, ty, tyaw, t_hdr = [], [], [], []
    for conn, ts, raw in reader.messages(connections=tf_conns):
        msg = reader.deserialize(raw, conn.msgtype)
        for tr in msg.transforms:
            if tr.header.frame_id != "imu_map" or tr.child_frame_id != "imu_link":
                continue
            q, t = tr.transform.rotation, tr.transform.translation
            tx.append(t.x); ty.append(t.y); tyaw.append(quat_to_yaw(q.x, q.y, q.z, q.w))
            t_hdr.append(to_sec(tr.header.stamp))

tx = np.array(tx); ty = np.array(ty); tyaw = np.array(tyaw)
t_hdr = np.array(t_hdr); c_hdr = np.array(c_hdr)

print("=" * 68)
print("1. 话题与时钟")
print("=" * 68)
print(f"  点云帧数              : {len(clouds)}   frame_id = {frame_id}")
print(f"  imu_map->imu_link 条数: {len(tx)}")
print(f"  点云 header stamp 跨度: {c_hdr.max()-c_hdr.min():.1f}s")
print(f"  TF   header stamp 跨度: {t_hdr.max()-t_hdr.min():.1f}s")
print(f"  两话题 header 时钟差   : {(c_hdr[0]-t_hdr[0])/86400.0:.3f} 天  <- 录制机未对时")

print()
print("=" * 68)
print("2. 车辆轨迹（imu_map 下，即地图基准）")
print("=" * 68)
print(f"  起点 (t=0)   : x={tx[0]:+.3f}  y={ty[0]:+.3f}  yaw={np.degrees(tyaw[0]):+.1f}deg")
print(f"  终点 (t=60s) : x={tx[-1]:+.3f}  y={ty[-1]:+.3f}  yaw={np.degrees(tyaw[-1]):+.1f}deg")
print(f"  轨迹包围盒   : x [{tx.min():+.2f}, {tx.max():+.2f}]   y [{ty.min():+.2f}, {ty.max():+.2f}]")
seg = np.hypot(np.diff(tx), np.diff(ty))
print(f"  轨迹总长     : {seg.sum():.1f} m    首尾间距 {np.hypot(tx[-1]-tx[0], ty[-1]-ty[0]):.2f} m")

print()
print("=" * 68)
print("3. 锥桶几何（累积 600 帧 -> 最近邻杂点过滤 -> 网格聚类去重）")
print("=" * 68)
pts = []
tot = 0
for c in clouds:
    tot += len(c)
    if len(c) < 2:
        continue
    d = np.hypot(c[:, None, 0] - c[None, :, 0], c[:, None, 1] - c[None, :, 1])
    np.fill_diagonal(d, np.inf)
    keep = (d.min(1) >= 0.5) & (c[:, 2] <= 0.9)
    pts.append(c[keep])
P_all = np.vstack(pts)
print(f"  原始总点数   : {tot}")
print(f"  过滤后剩余   : {len(P_all)}  ({100*len(P_all)/tot:.1f}%)")

CONES, CNT = cluster(P_all, 1.2)
print(f"  唯一锥桶数   : {len(CONES)}   (观测次数: 中位 {int(np.median(CNT))}, "
      f"最少 {CNT.min()}, 最多 {CNT.max()})")
print(f"  观测<5次的簇 : {int((CNT<5).sum())} 个 (疑似残留杂点)")

d = np.hypot(CONES[:, None, 0] - CONES[None, :, 0], CONES[:, None, 1] - CONES[None, :, 1])
np.fill_diagonal(d, np.inf)
nn = d.min(1)
print(f"  锥桶最近邻   : 中位 {np.median(nn):.2f} m  [25% {np.percentile(nn,25):.2f}, "
      f"75% {np.percentile(nn,75):.2f}, 最大 {nn.max():.2f}]")

P = np.stack([tx, ty], 1)
A, B = P[:-1], P[1:]
AB = B - A
L2 = np.maximum((AB ** 2).sum(1), 1e-9)
Q = CONES[:, :2]
dist_traj, near, tang = nearest_on_polyline(A, B, AB, L2, Q)
left = np.stack([-tang[:, 1], tang[:, 0]], 1)
s = ((Q - near) * left).sum(1)
is_red = s > 0

print(f"  到轨迹距离   : 中位 {np.median(dist_traj):.2f} m  90% {np.percentile(dist_traj,90):.2f} m")
print(f"  左/右侧向偏移: 左(红)中位 {np.median(s[s>0]):.2f} m   右(蓝)中位 {np.median(-s[s<0]):.2f} m")
print(f"  -> 赛道宽度  : {np.median(s[s>0]) + np.median(-s[s<0]):.2f} m")
print(f"  -> 参考分色  : 红 {is_red.sum()} / 蓝 {(~is_red).sum()}")

side = np.where(is_red, 1, -1)
nbr = d.argmin(1)
same = side == side[nbr]
print()
print(f"  * 最近邻同侧占比: {100*same.mean():.1f}%")
print(f"    同侧最近邻距离中位: {np.median(nn[same]):.2f} m")
print(f"    异侧最近邻距离中位: {np.median(nn[~same]):.2f} m")

np.save("tools/_cones.npy", CONES)
np.save("tools/_side.npy", is_red.astype(np.int8))
np.save("tools/_traj.npy", np.stack([tx, ty], 1))
np.save("tools/_yaw.npy", tyaw)
np.save("tools/_count.npy", CNT)
print()
print("  中间结果已存 tools/_cones.npy / _side.npy / _traj.npy / _yaw.npy / _count.npy")
