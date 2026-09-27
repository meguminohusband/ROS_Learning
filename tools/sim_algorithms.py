"""离线回放 timu.bag，对比「现有算法」与「候选修法」的左右判定可靠性。

关键点：所有算法只使用「当前帧 + 历史」，与在线节点一致（因果回放）。

用法：
    cd ~/Documents/Python_Project/ROS
    ./.venv/bin/python tools/sim_algorithms.py
"""
import os as _os
from pathlib import Path
import numpy as np
from rosbags.highlevel import AnyReader

BAG = Path("timu.bag")
CLOUD = "/only_lidar_points_pub"
TFS = "/tf"

# ---- 与 params.yaml 保持一致 ----
NOISE_NN, NOISE_Z = 0.5, 0.90
MIN_RANGE, MAX_RANGE, MAX_LATERAL = 2.0, 40.0, 15.0
PAIR_MIN, PAIR_MAX = 2.0, 4.5
VOTE_THR, MIN_OBS, ASSOC = 0.8, 3, 0.5
FRONT_X_MIN, FRONT_X_MAX = 0.0, 25.0
PATH_STEP, PATH_TAN_SPAN, PATH_GATE, S_MIN = 0.5, 2, 8.0, 0.40
PATH_MARGIN, HEAD_RANGE, HEAD_W = 6, 12.0, 0.3
MIN_VOTES, MIN_RELIABLE = 6.0, 2
PROV_MIN = 2.0          # 显示暂定色所需的最少加权票

UNKNOWN, RED, BLUE = 0, 1, 2


def parse_cloud(msg):
    return np.frombuffer(msg.data, dtype=np.float32).reshape(-1, 4)[:, :3].astype(np.float64)


def yaw_of(q):
    return np.arctan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


# ================= 载入 =================
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
k[1:] = np.hypot(np.diff(tx), np.diff(ty)) > 0.02      # 去掉原地抖动
TX, TY, TYAW, TTS = tx[k], ty[k], tyaw[k], t_ts[k]
o = np.argsort(TTS)
TTS, TX, TY, TYAW = TTS[o], TX[o], TY[o], TYAW[o]


def pose_at(ts):
    i = int(np.clip(np.searchsorted(TTS, ts), 1, len(TTS) - 1))
    return TX[i], TY[i], TYAW[i]


# ---- 参考真值：按弧长均匀重采样轨迹，消除零长度段的假切向 ----
arc = np.concatenate([[0], np.cumsum(np.hypot(np.diff(TX), np.diff(TY)))])
ss = np.arange(0.0, arc[-1], 0.25)
RX = np.interp(ss, arc, TX)
RY = np.interp(ss, arc, TY)
P = np.stack([RX, RY], 1)
A, B = P[:-1], P[1:]
AB = B - A
L2 = np.maximum((AB ** 2).sum(1), 1e-12)


def traj_project(A, B, AB, L2, Q):
    best_d = np.full(len(Q), np.inf); best_i = np.zeros(len(Q), int); best_t = np.zeros(len(Q))
    for i in range(len(A)):
        ap = Q - A[i]
        t = np.clip((ap * AB[i]).sum(1) / L2[i], 0, 1)
        proj = A[i] + t[:, None] * AB[i]
        dd = np.hypot(Q[:, 0] - proj[:, 0], Q[:, 1] - proj[:, 1])
        m = dd < best_d
        best_d[m] = dd[m]; best_i[m] = i; best_t[m] = t[m]
    near = A[best_i] + best_t[:, None] * AB[best_i]
    tang = AB[best_i] / np.sqrt(L2[best_i])[:, None]
    return best_d, near, tang


# 唯一锥桶
pts = []
for c in clouds:
    if len(c) < 2:
        continue
    d = np.linalg.norm(c[:, None] - c[None, :], axis=2)
    np.fill_diagonal(d, np.inf)
    pts.append(c[(d.min(1) >= NOISE_NN) & (c[:, 2] <= NOISE_Z)])
allp = np.vstack(pts)


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
    return np.array(cen), np.array(cnt)


CONES, CNT = cluster(allp)
d_ref, near_ref, tan_ref = traj_project(A, B, AB, L2, CONES[:, :2])
left_ref = np.stack([-tan_ref[:, 1], tan_ref[:, 0]], 1)
s_ref = ((CONES[:, :2] - near_ref) * left_ref).sum(1)
# 参考真值只认「赛道边界排」：实测边界排到路径 0.00~2.88 m，
# 赛道外还有一排真实物体在 3.59~8.98 m（不该上色），11 m 外是杂点团块。
# 取 3.0 m 作为分界，正好落在 2.88 与 3.59 之间的空档里。
VALID = d_ref < 3.0
TRUTH = np.where(s_ref > 0, RED, BLUE)

print("=" * 74)
print("参考真值（车辆轨迹切向 = 行进方向；左 = z x d）")
print("=" * 74)
print(f"  唯一锥桶 {len(CONES)}，赛道上 {int(VALID.sum())}（到轨迹 < 4 m）")
print(f"  参考分色: 红 {int((TRUTH[VALID]==RED).sum())} / 蓝 {int((TRUTH[VALID]==BLUE).sum())}")
print(f"  重采样轨迹点数 {len(P)}，总弧长 {arc[-1]:.1f} m，"
      f"绕行方向 {'逆时针' if np.sum(RX[:-1]*RY[1:]-RX[1:]*RY[:-1])>0 else '顺时针'}")
print(f"  被排除(离赛道/残留杂点) {int((~VALID).sum())} 个")

dd_all = np.hypot(CONES[:, None, 0] - CONES[None, :, 0], CONES[:, None, 1] - CONES[None, :, 1])
np.fill_diagonal(dd_all, np.inf)
nb = dd_all.argmin(1)
print(f"  * 最近邻同侧占比: {100*np.mean(TRUTH[nb]==TRUTH):.1f}%  "
      f"（同侧间隔中位 {np.median(dd_all[np.arange(len(CONES)),nb][TRUTH[nb]==TRUTH]):.2f} m，"
      f"异侧 {np.median(dd_all[np.arange(len(CONES)),nb][TRUTH[nb]!=TRUTH]):.2f} m）")


def to_body(pmap, x, y, yaw):
    c, s = np.cos(-yaw), np.sin(-yaw)
    dd = pmap[:, :2] - np.array([x, y])
    return np.stack([c * dd[:, 0] - s * dd[:, 1], s * dd[:, 0] + c * dd[:, 1]], 1)


def obs_to_cone(obs):
    if len(obs) == 0:
        return np.array([], int)
    dd = np.hypot(obs[:, None, 0] - CONES[None, :, 0], obs[:, None, 1] - CONES[None, :, 1])
    idx = dd.argmin(1)
    return np.where(dd[np.arange(len(obs)), idx] < 1.5, idx, -1)


# ================= 算法 =================
def algo_current(body):
    n = len(body)
    color = np.full(n, UNKNOWN, np.int8)
    if n < 2:
        return color
    dist = np.linalg.norm(body[:, None] - body[None, :], axis=2)
    iu, ju = np.triu_indices(n, 1)
    d = dist[iu, ju]
    m = (d >= PAIR_MIN) & (d <= PAIR_MAX)
    cand = np.stack([iu[m], ju[m], d[m]], 1)
    cand = cand[np.argsort(cand[:, 2])]
    used = np.zeros(n, bool); pairs = []
    for i, j, _ in cand.astype(int):
        if used[i] or used[j]:
            continue
        used[i] = used[j] = True
        pairs.append((i, j))
    if not pairs:
        return color
    mids = np.array([(body[i] + body[j]) * 0.5 for i, j in pairs])
    mids_s = mids[np.argsort(np.linalg.norm(mids, axis=1))]
    for kk, (i, j) in enumerate(pairs):
        if len(pairs) >= 2:
            k0 = max(0, kk - 1); k1 = min(len(mids_s) - 1, kk + 1)
            dv = mids_s[k1] - mids_s[k0]
            if np.linalg.norm(dv) < 1e-4:
                dv = -mids_s[kk]
        else:
            dv = -mids_s[0]
        dv = np.array([dv[0], dv[1]])
        nv = np.linalg.norm(dv)
        if nv < 1e-9:
            continue
        dv /= nv
        left = np.array([-dv[1], dv[0]])
        if (body[i] - body[j]) @ left > 0:
            color[i], color[j] = RED, BLUE
        else:
            color[i], color[j] = BLUE, RED
    return color


def algo_heading(body, front_only=True):
    color = np.full(len(body), UNKNOWN, np.int8)
    y = body[:, 1]
    rng = np.linalg.norm(body, axis=1)
    ok = np.abs(y) < MAX_LATERAL
    ok &= (body[:, 0] >= FRONT_X_MIN) & (body[:, 0] <= FRONT_X_MAX) if front_only else (rng <= MAX_RANGE)
    color[ok & (y > 0)] = RED
    color[ok & (y < 0)] = BLUE
    return color


def algo_path(pmap, path, heading, car):
    """以地图原点为基准：用「车辆已走过的路径」在锥桶处的切向定义行进方向。

    返回 (颜色数组, 投票权重数组)：
      权重 1.0 = 锥桶投影落在已驶过的路径内部 → 切向是实测的，可信
      权重 HEAD_W = 锥桶在车前方（投影落在路径末端）→ 只能用当前车头朝向近似
      权重 0     = 不投票（离赛道太远 / 贴中线 / 前方过远）
    """
    color = np.full(len(pmap), UNKNOWN, np.int8)
    w = np.zeros(len(pmap))
    if len(path) < 2:
        return color, w
    Pp = np.array(path)
    PA, PB = Pp[:-1], Pp[1:]
    PAB = PB - PA
    PL2 = np.maximum((PAB ** 2).sum(1), 1e-12)
    Q = pmap[:, :2]
    M = len(Q)
    ap = Q[:, None, :] - PA[None, :, :]
    tt = np.clip((ap * PAB[None, :, :]).sum(2) / PL2[None, :], 0.0, 1.0)
    proj = PA[None, :, :] + tt[:, :, None] * PAB[None, :, :]
    dd = np.hypot(Q[:, 0, None] - proj[:, :, 0], Q[:, 1, None] - proj[:, :, 1])
    idx = dd.argmin(1)
    dmin = dd[np.arange(M), idx]
    near = proj[np.arange(M), idx]
    interior = idx <= len(PA) - 1 - PATH_MARGIN
    i0 = np.maximum(0, idx - PATH_TAN_SPAN)
    i1 = np.minimum(len(Pp) - 1, idx + 1 + PATH_TAN_SPAN)
    dv = Pp[i1] - Pp[i0]
    hd = np.array([np.cos(heading), np.sin(heading)])    # 标量朝向 → 广播到 (M,2)
    dv = np.where(interior[:, None], dv, hd)
    nv = np.hypot(dv[:, 0], dv[:, 1])
    dist_car = np.hypot(Q[:, 0] - car[0], Q[:, 1] - car[1])
    left_car = np.array([-np.sin(heading), np.cos(heading)])
    lat_car = np.abs((Q - np.array(car)) @ left_car)
    # 已驶过 → 用「实测横向距离」把关；车前方 → 路径还没铺到，只能用
    # 「距车距离」+「相对车头航向线的横向偏移」把关（否则 head_range 形同虚设）
    gate = np.where(interior, dmin <= PATH_GATE,
                    (dist_car <= HEAD_RANGE) & (lat_car <= PATH_GATE))
    ok = (nv > 1e-6) & gate
    unit = np.where(ok[:, None], dv / np.maximum(nv, 1e-9)[:, None], 0.0)
    left = np.stack([-unit[:, 1], unit[:, 0]], 1)
    s = ((Q - near) * left).sum(1)
    ok &= np.abs(s) >= S_MIN
    color[ok & (s > 0)] = RED
    color[ok & (s < 0)] = BLUE
    w[ok] = np.where(interior[ok], 1.0, HEAD_W)
    return color, w


# ================= 回放 =================
def run(kind):
    tracks_pos, rv, bv, obs_n, locked, nrel, nfull = [], [], [], [], [], [], []
    rrl, bbl = [], []            # 只统计「满权票（实测切向）」的红/蓝票
    track_g = []                 # 每条轨迹关联到的参考锥桶索引
    flip = {}
    fc = ft = 0
    lk_tot = lk_ok = pv_tot = pv_ok = 0     # 显示正确率（逐帧统计所有被显示的轨迹）
    path = []

    # 可选：把某个锥桶的逐帧观测打出来（DEBUG_CONE_XY="x,y"）
    dbg_t = -1
    _xy = _os.environ.get("DEBUG_CONE_XY", "")
    if _xy and "," in _xy:
        _qx, _qy = [float(v) for v in _xy.split(",")]
        dbg_t = int(np.argmin(np.hypot(CONES[:, 0] - _qx, CONES[:, 1] - _qy)))
    dbg = []

    for fi, c in enumerate(clouds):
        x, y, yaw = pose_at(c_ts[fi])
        if not path or np.hypot(x - path[-1][0], y - path[-1][1]) >= PATH_STEP:
            path.append((x, y))
        body = to_body(c, x, y, yaw)
        rng = np.linalg.norm(body, axis=1)
        keep = ((rng <= MAX_RANGE) & (rng >= MIN_RANGE)
                & (np.abs(body[:, 1]) <= MAX_LATERAL))       # 与节点第4步一致
        body, cms = body[keep], c[keep]

        if kind == "A":
            col = algo_current(body)
        elif kind == "B":
            col = algo_heading(body, False)
        elif kind == "C":
            col = algo_heading(body, True)
        else:
            col, _w = algo_path(cms, path, yaw, (x, y))
        if kind in "ABC":
            _w = (col != UNKNOWN).astype(float)

        ci = obs_to_cone(cms)
        ok = (col != UNKNOWN) & (ci >= 0) & VALID[ci]
        if ok.any():
            fc += int((col[ok] == TRUTH[ci[ok]]).sum()); ft += int(ok.sum())

        hits = np.zeros(len(tracks_pos), bool)
        for oi in range(len(body)):
            if _w[oi] <= 0:
                continue
            g = ci[oi]
            if g < 0:
                continue
            best, bd = -1, ASSOC
            for kk in range(len(tracks_pos)):
                if locked[kk] != UNKNOWN and hits[kk]:
                    continue
                dd = np.hypot(tracks_pos[kk][0] - cms[oi, 0], tracks_pos[kk][1] - cms[oi, 1])
                if dd < bd:
                    bd, best = dd, kk
            if best < 0:
                tracks_pos.append(cms[oi, :2].copy()); rv.append(0.0); bv.append(0.0)
                obs_n.append(0); locked.append(UNKNOWN); nrel.append(0.0); nfull.append(0)
                rrl.append(0.0); bbl.append(0.0)
                track_g.append(int(g))
                hits = np.append(hits, False)
                best = len(tracks_pos) - 1
            hits[best] = True
            track_g[best] = int(g)
            tracks_pos[best] = (tracks_pos[best] * obs_n[best] + cms[oi, :2]) / (obs_n[best] + 1)
            obs_n[best] += 1
            flip.setdefault(g, []).append(col[oi])
            if g == dbg_t and kind == "E":
                _q = cms[oi, :2]
                _Pp = np.array(path)
                _PA, _PB = _Pp[:-1], _Pp[1:]
                _PAB = _PB - _PA
                _PL2 = np.maximum((_PAB ** 2).sum(1), 1e-12)
                _ap = _q - _PA
                _t = np.clip((_ap * _PAB).sum(1) / _PL2, 0, 1)
                _pr = _PA + _t[:, None] * _PAB
                _dd = np.hypot(_q[0] - _pr[:, 0], _q[1] - _pr[:, 1])
                _i = int(_dd.argmin())
                _int = _i <= len(_PA) - 1 - PATH_MARGIN
                _i0 = max(0, _i - PATH_TAN_SPAN)
                _i1 = min(len(_Pp) - 1, _i + 1 + PATH_TAN_SPAN)
                _dv = _Pp[_i1] - _Pp[_i0]
                dbg.append((float(c_ts[fi] / 1e9), round(float(_w[oi]), 2), int(col[oi]),
                            round(float(_dd[_i]), 2), _i, len(_PA) - 1, bool(_int),
                            np.round(_PA[_i], 2).tolist(), np.round(_dv, 2).tolist()))
            if locked[best] != UNKNOWN:
                continue
            ww = float(_w[oi])
            nrel[best] += ww
            if ww >= 1.0:
                nfull[best] += 1
            if col[oi] == RED:
                rv[best] += ww
                if ww >= 1.0:
                    rrl[best] += ww
            else:
                bv[best] += ww
                if ww >= 1.0:
                    bbl[best] += ww
            # ★ 锁定判据只看「实测切向」的可信票：前方用车头朝向投的票在这种
            #   急弯处可能一半是错的，混进来会把比例拉到 50/50、永远锁不上。
            rel_tot = rrl[best] + bbl[best]
            if rel_tot > 0:
                lead = rrl[best] / rel_tot
            else:
                lead = None
            if nrel[best] >= MIN_VOTES and nfull[best] >= MIN_RELIABLE and lead is not None:
                if lead >= VOTE_THR:
                    locked[best] = RED
                elif 1 - lead >= VOTE_THR:
                    locked[best] = BLUE

        # ---- 本帧「显示正确率」：按新方案的显示分级统计 ----
        #   锁定 → 实心全亮色；未锁定但票够 → 降亮暂定色（多数票）
        for k in range(len(tracks_pos)):
            if locked[k] != UNKNOWN:
                dcol, prov = int(locked[k]), False
            elif nrel[k] >= PROV_MIN:
                dcol, prov = (RED if rv[k] > bv[k] else BLUE), True
            else:
                continue
            g = track_g[k]
            if g < 0 or not VALID[g]:
                continue
            if prov:
                pv_tot += 1
                if dcol == TRUTH[g]:
                    pv_ok += 1
            else:
                lk_tot += 1
                if dcol == TRUTH[g]:
                    lk_ok += 1

    tp = np.array(tracks_pos)
    if dbg_t >= 0 and kind == "E":
        full = [d for d in dbg if d[1] >= 1.0]
        part = [d for d in dbg if 0 < d[1] < 1.0]
        print()
        print("=" * 74)
        print(f"调试锥桶 #{dbg_t} 位置 {np.round(CONES[dbg_t,:2],2).tolist()}"
              f"  参考侧={'红' if TRUTH[dbg_t]==1 else '蓝'}  到路径 {d_ref[dbg_t]:.2f}")
        print(f"  被上色(vote_w>0)的帧数: {len(dbg)}   满权(实测切向) {len(full)}，降权(前方朝向) {len(part)}")
        for d in dbg:
            print(f"    t={d[0]:.2f} 权重={d[1]:.2f} {'红' if d[2]==1 else '蓝'}  "
                  f"到路径={d[3]:5.2f} 段#{d[4]:3d}/{d[5]:3d} interior={str(d[6]):5s} "
                  f"最近点{d[7]} 中心差分{d[8]}")
        if not dbg:
            print("  （从未被上色：所有观测都被 vote_w=0 丢弃）")
    dd = np.hypot(tp[:, None, 0] - CONES[None, :, 0], tp[:, None, 1] - CONES[None, :, 1])
    gidx = dd.argmin(1)
    near = dd[np.arange(len(tp)), gidx] < 1.0
    lk = np.array(locked)
    m = (lk != UNKNOWN) & near & VALID[gidx]
    acc = float((lk[m] == TRUTH[gidx[m]]).mean()) if m.any() else float("nan")
    wrong = [(np.round(tp[t], 1).tolist(), int(lk[t]), int(TRUTH[gidx[t]]))
             for t in np.where(m & (lk != TRUTH[gidx]))[0]]
    fl = {g: int(np.sum(np.diff(v) != 0)) for g, v in flip.items()}
    return dict(frame_acc=fc / ft if ft else float("nan"), lock_acc=acc,
                n_locked=int(m.sum()), n_tracks=len(tp), n_colored=ft,
                flip_cones=sum(1 for v in fl.values() if v > 0),
                total_flips=sum(fl.values()), wrong=wrong, flips=fl,
                tp=tp, obs_n=np.array(obs_n), rv=np.array(rv), bv=np.array(bv),
                nrel=np.array(nrel), nfull=np.array(nfull), lk=lk, gidx=gidx,
                gdist=dd[np.arange(len(tp)), gidx], near=near,
                lk_tot=lk_tot, lk_ok=lk_ok, pv_tot=pv_tot, pv_ok=pv_ok)


print()
print("=" * 74)
print("回放对比（600 帧，因果在线）")
print("=" * 74)
names = [("A", "现有代码 assignColors（配对+相邻中点方向）"),
         ("B", "车体系 y 符号（全向）"),
         ("C", "车体系 y 符号（仅车前方）"),
         ("E", "地图路径切向 + 投票（本次修法）")]
res = {}
for kk, nm in names:
    r = run(kk)
    res[kk] = r
    print(f"\n【{nm}】")
    print(f"  单帧准确率        : {100*r['frame_acc']:.1f}%")
    print(f"  锁定后准确率      : {100*r['lock_acc']:.1f}%   (锁定 {r['n_locked']} / 轨迹 {r['n_tracks']})")
    print(f"  颜色翻转锥桶数    : {r['flip_cones']} 个，累计翻转 {r['total_flips']} 次")
    if r["wrong"]:
        print(f"  锁定但判错        : {len(r['wrong'])} 个 {r['wrong'][:5]}")

np.savez("tools/_sim.npz", cones=CONES, truth=TRUTH, valid=VALID, cnt=CNT,
         sref=s_ref, dref=d_ref, rx=RX, ry=RY, tx=TX, ty=TY, yaw=TYAW, tts=TTS, cts=c_ts)

# ---------------- 诊断：为什么锁定准确率远低于单帧准确率 ----------------
r = res["E"]
print()
print("=" * 74)
print("诊断 E：锁定轨迹 vs 全局锥桶 的对齐情况")
print("=" * 74)
print(f"  E 上色观测总数 {r['n_colored']}，轨迹数 {r['n_tracks']}，锁定 {int((r['lk']!=UNKNOWN).sum())}")
print(f"  轨迹到最近全局锥桶的距离: 中位 {np.median(r['gdist']):.2f} m, "
      f"90% {np.percentile(r['gdist'],90):.2f} m, >1.0 m 的 {int((r['gdist']>1.0).sum())} 条")
print(f"  观测次数分布: 中位 {int(np.median(r['obs_n']))}, <5 次的轨迹 {int((r['obs_n']<5).sum())} 条")
print(f"  投票纯度: 平均 max(r,b)/total = "
      f"{np.mean(np.maximum(r['rv'],r['bv'])/np.maximum(r['rv']+r['bv'],1)):.3f}")

bad = np.where((r['lk'] != UNKNOWN) & r['near'] & VALID[r['gidx']] & (r['lk'] != TRUTH[r['gidx']]))[0]
print(f"\n  锁定但判错 {len(bad)} 条，逐个看：")
print(f"  {'轨迹位置':>18} {'观测':>5} {'红票':>5} {'蓝票':>5} {'锁定':>4} {'真值':>4} {'到锥桶':>7} {'真值锥桶位置':>18}")
for t in bad[:20]:
    print(f"  {str(np.round(r['tp'][t],1).tolist()):>18} {r['obs_n'][t]:5d} {r['rv'][t]:5d} {r['bv'][t]:5d} "
          f"{'红' if r['lk'][t]==1 else '蓝':>4} {'红' if TRUTH[r['gidx'][t]]==1 else '蓝':>4} "
          f"{r['gdist'][t]:7.2f} {str(np.round(CONES[r['gidx'][t],:2],1).tolist()):>18}")

# ---------------- 参数扫描：确认修法对参数不敏感 ----------------
import os as _os
BASE = dict(PATH_GATE=PATH_GATE, ASSOC=ASSOC, MIN_VOTES=MIN_VOTES, HEAD_W=HEAD_W,
            S_MIN=S_MIN, PATH_MARGIN=PATH_MARGIN, HEAD_RANGE=HEAD_RANGE,
            MIN_RELIABLE=MIN_RELIABLE)


def _restore():
    for k_, v_ in BASE.items():
        globals()[k_] = v_


if _os.environ.get("SWEEP") == "1":
    print()
    print("=" * 74)
    print("参数扫描（算法 E）")
    print("=" * 74)
    print(f"{'PATH_GATE':>10} {'ASSOC':>6} {'MIN_VOTES':>10} {'HEAD_W':>7} {'S_MIN':>6} "
          f"{'PMAR':>5} {'MREL':>5} | {'单帧%':>7} {'锁定%':>7} {'锁定数':>7} {'错':>4} {'翻转':>6}")
    rows = []
    for pg in (4.0, 5.0, 6.0):
        for assoc in (0.5, 0.9):
            for mv in (3.0, 6.0, 12.0):
                for hw in (0.0, 0.3):
                    for mrel in (0, 2):
                        for extra in ({}, {"S_MIN": 0.6}, {"PATH_MARGIN": 2}):
                            _restore()
                            globals()["PATH_GATE"] = pg
                            globals()["ASSOC"] = assoc
                            globals()["MIN_VOTES"] = mv
                            globals()["HEAD_W"] = hw
                            globals()["MIN_RELIABLE"] = mrel
                            for k_, v_ in extra.items():
                                globals()[k_] = v_
                            rr = run("E")
                            rows.append((pg, assoc, mv, hw, globals()["S_MIN"],
                                         globals()["PATH_MARGIN"], mrel,
                                         rr["frame_acc"], rr["lock_acc"], rr["n_locked"],
                                         len(rr["wrong"]), rr["total_flips"]))
    _restore()
    rows.sort(key=lambda z: (z[10], -z[9], z[11]))
    for x in rows[:18]:
        print(f"{x[0]:10.1f} {x[1]:6.1f} {x[2]:10.1f} {x[3]:7.1f} {x[4]:6.1f} {x[5]:5d} "
              f"{x[6]:5d} | {100*x[7]:7.1f} {100*x[8]:7.1f} {x[9]:7d} {x[10]:4d} {x[11]:6d}")
    print(f"\n  共 {len(rows)} 组配置，其中「零判错」的 {sum(1 for x in rows if x[10]==0)} 组")

# ---------------- 最终选定配置（与当前 params.yaml / 节点代码一致） ----------------
_restore()
globals()["PATH_GATE"] = 5.0
globals()["ASSOC"] = 1.2
globals()["MIN_VOTES"] = 6.0
globals()["MIN_RELIABLE"] = 2
globals()["HEAD_W"] = 0.3
globals()["S_MIN"] = 0.40
globals()["PATH_MARGIN"] = 6
globals()["HEAD_RANGE"] = 8.0
rf = run("E")
print()
print("=" * 74)
print("最终选定配置验证")
print("=" * 74)
print(f"  path_gate=5.0  assoc=1.2  min_votes=6.0  min_reliable=2  head_weight=0.3")
print(f"  s_min=0.40  head_range=8.0  provisional_min_votes={PROV_MIN}")
print(f"  单帧准确率 {100*rf['frame_acc']:.1f}%   锁定后准确率 {100*rf['lock_acc']:.1f}%   "
      f"锁定 {rf['n_locked']} 条 / 轨迹 {rf['n_tracks']} 条")
print(f"  判错 {len(rf['wrong'])} 条   翻转 {rf['flip_cones']} 个锥桶 / {rf['total_flips']} 次")
print(f"  ── 显示正确率（逐帧统计所有被显示的锥桶）──")
print(f"     锁定色(实心)   : {100*rf['lk_ok']/max(rf['lk_tot'],1):6.2f}%   ({rf['lk_ok']}/{rf['lk_tot']} 次显示)")
print(f"     暂定色(降亮前方): {100*rf['pv_ok']/max(rf['pv_tot'],1):6.2f}%   ({rf['pv_ok']}/{rf['pv_tot']} 次显示)")
print(f"     综合           : {100*(rf['lk_ok']+rf['pv_ok'])/max(rf['lk_tot']+rf['pv_tot'],1):6.2f}%")
cov = sorted(set(int(g) for g, v in rf["flips"].items() if g >= 0))
v_cov = [g for g in cov if VALID[g]]
print(f"  覆盖到在赛道锥桶: {len(v_cov)} / {int(VALID.sum())} 个"
      f"   （未被任何帧上色的赛道锥桶 {int(VALID.sum())-len(v_cov)} 个）")
uncovered = [np.round(CONES[g, :2], 1).tolist() for g in np.where(VALID)[0] if int(g) not in cov]
if uncovered:
    print(f"  未覆盖: {uncovered}")

print()
print("  path_gate 取值影响（关注：赛道锥桶覆盖率 与 锁定总数）")
print(f"  {'path_gate':>10} {'锁定%':>7} {'锁定数':>7} {'判错':>5} {'覆盖赛道锥桶':>12} {'额外锁定的离群锥桶':>20}")
for _pg in (2.6, 3.0, 3.2, 3.5, 4.0, 5.0):
    globals()["PATH_GATE"] = _pg
    rr2 = run("E")
    cv2 = set(int(g) for g, v in rr2["flips"].items() if g >= 0 and VALID[g])
    off = sum(1 for g, v in rr2["flips"].items() if g >= 0 and not VALID[g])
    print(f"  {_pg:10.1f} {100*rr2['lock_acc']:7.1f} {rr2['n_locked']:7d} {len(rr2['wrong']):5d} "
          f"{len(cv2):8d}/{int(VALID.sum()):<3d} {off:20d}")
globals()["PATH_GATE"] = 5.0


# ============ 逐项审计：「驶过未上色」与「只上一侧」的归属 ============
_restore()
globals()["PATH_GATE"] = 5.0
globals()["ASSOC"] = 1.2
globals()["MIN_VOTES"] = 6.0
globals()["MIN_RELIABLE"] = 2
globals()["HEAD_W"] = 0.3
globals()["S_MIN"] = 0.40
globals()["PATH_MARGIN"] = 6
globals()["HEAD_RANGE"] = 8.0
rA = run("E")

locked_of = {}
for t in range(len(rA["tp"])):
    if rA["gdist"][t] < 1.0 and rA["lk"][t] != UNKNOWN:
        g = int(rA["gidx"][t])
        if g not in locked_of:
            locked_of[g] = int(rA["lk"][t])

VALIDi = np.where(VALID)[0]
missing = [g for g in VALIDi if g not in locked_of]
wrong_c = [g for g in VALIDi if g in locked_of and locked_of[g] != int(TRUTH[g])]
tp = rA["tp"]

print()
print("=" * 74)
print("审计：最终锁定结果 vs 参考真值（当前线上配置）")
print("=" * 74)
print(f"  赛道锥桶 {len(VALIDi)} 个：锁定 {len(locked_of)}，未锁定 {len(missing)}，锁错 {len(wrong_c)}")

if missing:
    print()
    print("  【驶过却未被上色】的锥桶（看观测/投票统计，定位卡在哪个条件）:")
    print(f"    {'位置':>18} {'到路径':>6} {'侧向s':>6} {'观测':>4} {'满权':>4} {'加权':>6} {'红':>5} {'蓝':>5} {'最近轨迹':>7}")
    for g in missing:
        dd = np.hypot(tp[:, 0] - CONES[g, 0], tp[:, 1] - CONES[g, 1])
        t0 = int(dd.argmin())
        print(f"    {str(np.round(CONES[g,:2],1).tolist()):>18} {d_ref[g]:6.2f} {s_ref[g]:6.2f} "
              f"{rA['obs_n'][t0]:4d} {rA['nfull'][t0]:4d} {rA['nrel'][t0]:6.2f} "
              f"{rA['rv'][t0]:5.2f} {rA['bv'][t0]:5.2f} {dd[t0]:7.2f}")
        # 调试：该锥桶附近 3 米内的所有轨迹
        near_t = np.where(dd < 3.0)[0]
        for t in near_t:
            print(f"        轨迹#{t}: 位置 {np.round(tp[t],2).tolist()} 距锥桶 {dd[t]:.2f} "
                  f"gidx={int(rA['gidx'][t])} 锁定={int(rA['lk'][t])} 观测={int(rA['obs_n'][t])} "
                  f"满权={int(rA['nfull'][t])} 加权={rA['nrel'][t]:.2f} 红={rA['rv'][t]:.2f} 蓝={rA['bv'][t]:.2f}")

# 单侧：锁定锥桶的最近异侧锥桶是否也锁定了
print()
oneside = []
for g in VALIDi:
    if g not in locked_of:
        continue
    dd = np.hypot(CONES[:, 0] - CONES[g, 0], CONES[:, 1] - CONES[g, 1])
    cand = [h for h in VALIDi if h != g and TRUTH[h] != TRUTH[g] and dd[h] < 5.0]
    if not cand:
        continue
    h = min(cand, key=lambda x: dd[x])
    if h not in locked_of:
        oneside.append((g, h))
print(f"  【只上一侧】共 {len(oneside)} 处（左侧有颜色、右侧没有，或反之）:")
for g, h in oneside[:10]:
    print(f"    已锁 {np.round(CONES[g,:2],1).tolist()}({'红' if locked_of[g]==1 else '蓝'})  "
          f"对面 {np.round(CONES[h,:2],1).tolist()}({'红' if TRUTH[h]==1 else '蓝'}) 未锁")

# 按侧统计锁定率，判断是否整体偏一侧
locked_valid = [g for g in VALIDi if g in locked_of]
nr = sum(1 for g in locked_valid if TRUTH[g] == 1)
nb = sum(1 for g in locked_valid if TRUTH[g] == 2)
tr = int((TRUTH[VALIDi] == 1).sum()); tb = int((TRUTH[VALIDi] == 2).sum())
print()
print(f"  红侧锁定 {nr}/{tr}（{100*nr/max(tr,1):.0f}%）   蓝侧锁定 {nb}/{tb}（{100*nb/max(tb,1):.0f}%）")


