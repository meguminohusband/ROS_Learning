"""测量「车头朝向」对车前方锥桶判左右的准确率随距离的变化（离线）。

用于确定 head_range：前方锥桶的暂定色只能用「当前车头朝向」近似，
距离越远、弯道越急误差越大。本脚本给出实测的安全距离。

用法：
    cd ~/Documents/Python_Project/ROS
    ./.venv/bin/python tools/measure_head_range.py
"""
from pathlib import Path
import numpy as np
from rosbags.highlevel import AnyReader

BAG = Path("timu.bag")
CLOUD = "/only_lidar_points_pub"
TFS = "/tf"


def parse_cloud(msg):
    return np.frombuffer(msg.data, dtype=np.float32).reshape(-1, 4)[:, :3].astype(np.float64)


def to_sec(s):
    return s.sec + s.nanosec * 1e-9


def yaw_of(q):
    return np.arctan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def cluster(p, thr=1.2):
    cen, cnt, grid = [], [], {}
    for q in p:
        key = (int(np.floor(q[0] / thr)), int(np.floor(q[1] / thr)))
        hit = -1
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for idx in grid.get((key[0] + dx, key[1] + dy), ()):
                    if np.hypot(cen[idx][0] - q[0], cen[idx][1] - q[1]) < thr:
                        hit = idx; break
                if hit >= 0: break
            if hit >= 0: break
        if hit >= 0:
            c = cen[hit]; c[:] = (c * cnt[hit] + q) / (cnt[hit] + 1); cnt[hit] += 1
        else:
            cen.append(q.astype(float)); cnt.append(1)
            grid.setdefault(key, []).append(len(cen) - 1)
    return np.array(cen)


with AnyReader([BAG]) as reader:
    cc = [c for c in reader.connections if c.topic == CLOUD]
    tc = [c for c in reader.connections if c.topic == TFS]
    clouds, c_ts = [], []
    for conn, ts, raw in reader.messages(connections=cc):
        clouds.append(parse_cloud(reader.deserialize(raw, conn.msgtype)))
        c_ts.append(ts)
    tx, ty, tyaw, t_ts = [], [], [], []
    for conn, ts, raw in reader.messages(connections=tc):
        for tr in reader.deserialize(raw, conn.msgtype).transforms:
            if tr.header.frame_id != "imu_map" or tr.child_frame_id != "imu_link":
                continue
            tx.append(tr.transform.translation.x)
            ty.append(tr.transform.translation.y)
            tyaw.append(yaw_of(tr.transform.rotation))
            t_ts.append(ts)

c_ts = np.array(c_ts)
tx, ty, tyaw, t_ts = map(np.array, (tx, ty, tyaw, t_ts))
k = np.ones(len(tx), bool)
k[1:] = np.hypot(np.diff(tx), np.diff(ty)) > 0.02
TX, TY, TYAW, TTS = tx[k], ty[k], tyaw[k], t_ts[k]
o = np.argsort(TTS); TTS, TX, TY, TYAW = TTS[o], TX[o], TY[o], TYAW[o]

# 参考真值：重采样轨迹切向
arc = np.concatenate([[0], np.cumsum(np.hypot(np.diff(TX), np.diff(TY)))])
ss = np.arange(0.0, arc[-1], 0.25)
RX = np.interp(ss, arc, TX); RY = np.interp(ss, arc, TY)
P = np.stack([RX, RY], 1); A, B = P[:-1], P[1:]
AB = B - A; L2 = np.maximum((AB ** 2).sum(1), 1e-12)

pts = []
for c in clouds:
    if len(c) < 2:
        continue
    d = np.linalg.norm(c[:, None] - c[None, :], axis=2)
    np.fill_diagonal(d, np.inf)
    pts.append(c[(d.min(1) >= 0.5) & (c[:, 2] <= 0.9)])
CONES = cluster(np.vstack(pts))

def traj_side(Q):
    bd = np.full(len(Q), np.inf); bi = np.zeros(len(Q), int); bt = np.zeros(len(Q))
    for i in range(len(A)):
        ap = Q - A[i]
        t = np.clip((ap * AB[i]).sum(1) / L2[i], 0, 1)
        pr = A[i] + t[:, None] * AB[i]
        dd = np.hypot(Q[:, 0] - pr[:, 0], Q[:, 1] - pr[:, 1])
        m = dd < bd; bd[m] = dd[m]; bi[m] = i; bt[m] = t[m]
    near = A[bi] + bt[:, None] * AB[bi]
    tang = AB[bi] / np.sqrt(L2[bi])[:, None]
    left = np.stack([-tang[:, 1], tang[:, 0]], 1)
    return ((Q - near) * left).sum(1), bd

s_ref, d_ref = traj_side(CONES[:, :2])
VALID = d_ref < 4.0
TRUTH = np.where(s_ref > 0, 1, 2)

print("=" * 72)
print("「车头朝向」对车前方锥桶判左右的准确率  vs  距离")
print("=" * 72)
bins = [(0, 5), (5, 8), (8, 12), (12, 15), (15, 20), (20, 25), (25, 40)]
tot = {i: [0, 0] for i in range(len(bins))}

ci = np.where(VALID)[0]
for fi, c in enumerate(clouds):
    i = int(np.clip(np.searchsorted(TTS, c_ts[fi]), 1, len(TTS) - 1))
    x, y, yaw = TX[i], TY[i], TYAW[i]
    cs, sn = np.cos(-yaw), np.sin(-yaw)
    d = CONES[:, :2] - np.array([x, y])
    bx = cs * d[:, 0] - sn * d[:, 1]
    by = sn * d[:, 0] + cs * d[:, 1]
    ahead = (bx > 0) & (by > -15) & (by < 15)
    # 前方锥桶：用车头朝向的 y 符号判左右（y>0 左=红）
    pred = np.where(by > 0, 1, 2)
    for bi_i, (lo, hi) in enumerate(bins):
        m = VALID & ahead & (bx >= lo) & (bx < hi)
        if m.any():
            tot[bi_i][0] += int((pred[m] == TRUTH[m]).sum())
            tot[bi_i][1] += int(m.sum())

print(f"{'距离 x (m)':>12} {'准确率':>9} {'样本数':>9}")
for (lo, hi), (ok, n) in zip(bins, tot.values()):
    acc = 100 * ok / n if n else float("nan")
    print(f"  [{lo:2d},{hi:2d})  {acc:8.1f}%  {n:9d}")

print()
print("说明：参考真值 = 车辆轨迹切向定义的左右（左红右蓝）。")
print("「车头朝向」在该距离内准确率越高，越适合用它给前方锥桶定暂定色。")
