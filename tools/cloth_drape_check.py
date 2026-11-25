"""
离线复现 VertexPhysics 的布料求解器，验证"水平旗面会自然垂下来"。

几何：与 test.dat 一致的 33x33 网格（顶点间距 0.0827m，总宽 2.65m）。
第 0 行是刚性横杆（R=0），其余 32 行是布料（R=128, G=200, B=255）。

场景：把旗面摆成水平（从横杆向 +Y 伸出），重力 -Z。
期望：布面绕横杆向下翻折，最终大致垂到 -Z 方向。

只依赖 numpy；不是游戏内数据，但求解顺序与 GeometryPhysics.cpp 一致：
  积分 -> 6 次约束/挂点迭代 -> 2 遍 1.35x 拉伸上限 -> 距离限幅
"""

import numpy as np

N = 33                  # 每行 33 个顶点
SPACING = 0.08273       # 顶点间距（由 test.dat 反推）
DT = 1.0 / 60.0
GRAVITY = 10.0
ITERS = 6
STIFFNESS = 0.55        # Cloth 的约束刚度下限
MAX_STEP = 8.0 * DT
CAP_PASSES = 2


def build_grid(horizontal=True):
    # 先在竖直平面里建网格：row 0 = 顶部横杆，row 32 = 最下摆
    pts = np.zeros((N, N, 3))
    for i in range(N):
        for j in range(N):
            x = (j - (N - 1) / 2.0) * SPACING
            y = 0.0
            z = (N - 1 - i) * SPACING
            pts[i, j] = (x, y, z)

    if horizontal:
        # 绕横杆（row 0）转 90°：旗面从水平 +Y 方向伸出
        top_z = (N - 1) * SPACING
        pts[:, :, 1] = top_z - pts[:, :, 2]     # +Y 深度
        pts[:, :, 2] = top_z                    # 全部在同一高度

    return pts


def run(style, verbose=True):
    rest = build_grid(horizontal=True).reshape(-1, 3)
    pos = rest.copy()
    prev = rest.copy()

    # 顶点索引：row 0 是刚性；cloth 从 row 1 开始
    idx = np.arange(N * N).reshape(N, N)
    rigid = idx[0, :]
    cloth_rows = idx[1:, :]

    # 三角形边（只有 cloth-cloth 边参与约束；与刚性行的边不建约束）
    edges = []
    for i in range(1, N):
        for j in range(N):
            if j + 1 < N:
                edges.append((idx[i, j], idx[i, j + 1]))
            if i + 1 < N:
                edges.append((idx[i, j], idx[i + 1, j]))
    edges = np.array(edges)

    rest_len = np.linalg.norm(
        rest[edges[:, 0]] - rest[edges[:, 1]], axis=1)

    # 挂点：cloth 第 1 行钉在横杆上（与 C++ 的 triangles->rigid pin 对应）
    pins = []   # (particle, target_pos, rest_length)
    for j in range(N):
        for dj in (-1, 0, 1):
            jj = j + dj
            if 0 <= jj < N:
                target = rest[idx[0, jj]]
                pins.append((idx[1, j], target,
                             np.linalg.norm(rest[idx[1, j]] - target)))

    # restSpan：到横杆最近顶点的静止距离
    rest_span = np.zeros(N * N)
    for i in range(1, N):
        for j in range(N):
            d = np.linalg.norm(rest[idx[0, :]] - rest[idx[i, j]], axis=1).min()
            rest_span[idx[i, j]] = d

    if style == "old":
        limit = np.minimum(1.5, rest_span + 0.15)
        limit[rest_span == 0] = 0.15
        damping = 0.92
        pull = 0.0
    else:
        limit = np.clip(rest_span * 2.0 + 0.20, 0.20, 10.0)
        damping = 0.98
        pull = 0.0    # 有挂点的布料：不再向静止姿势回拉

    cloth_set = set(cloth_rows.ravel().tolist())
    rigid_set = set(rigid.tolist())

    far = idx[-1, -1]
    trace = []

    for frame in range(360):
        # --- 积分（只动布料） ---
        vel = (pos - prev) * damping
        vel[list(cloth_set)] += np.array([0.0, 0.0, -GRAVITY]) * DT * DT

        step = np.linalg.norm(vel, axis=1)
        mask = step > MAX_STEP
        vel[mask] *= (MAX_STEP / step[mask])[:, None]

        prev = pos.copy()
        pos = pos + vel

        # --- 安全网：离静止位置的最大距离（旧版这里就是 0.15m） ---
        for p in cloth_set:
            d = pos[p] - rest[p]
            ln = np.linalg.norm(d)
            if ln > limit[p] and ln > 1e-9:
                pos[p] = rest[p] + d * (limit[p] / ln)

        # --- 6 次迭代：约束 + 挂点 + 锚定回拉 ---
        for _ in range(ITERS):
            a = edges[:, 0]
            b = edges[:, 1]
            d = pos[b] - pos[a]
            ln = np.linalg.norm(d, axis=1)
            ln[ln < 1e-6] = 1e-6
            diff = (ln - rest_len) / ln
            corr = d * (diff * 0.5 * STIFFNESS)[:, None]
            np.add.at(pos, a, corr)
            np.add.at(pos, b, -corr)

            for p, target, rl in pins:
                d = pos[p] - target
                ln = np.linalg.norm(d)
                if ln > 1e-6:
                    pos[p] = target + d * (rl / ln)

            if pull > 0:
                pos[list(cloth_set)] += (rest[list(cloth_set)] - pos[list(cloth_set)]) * pull

        # --- 1.35x 拉伸上限 ---
        for _ in range(CAP_PASSES):
            a = edges[:, 0]
            b = edges[:, 1]
            d = pos[b] - pos[a]
            ln = np.linalg.norm(d, axis=1)
            over = ln > rest_len * 1.35
            if over.any():
                e = (ln[over] - rest_len[over] * 1.35) / ln[over]
                dir_ = d[over] / ln[over][:, None]
                np.add.at(pos, a[over], dir_ * (e * 0.5)[:, None])
                np.add.at(pos, b[over], -dir_ * (e * 0.5)[:, None])

        # --- 再限幅 ---
        for p in cloth_set:
            d = pos[p] - rest[p]
            ln = np.linalg.norm(d)
            if ln > limit[p] and ln > 1e-9:
                pos[p] = rest[p] + d * (limit[p] / ln)

        if frame % 60 == 0:
            trace.append((frame, pos[far].copy()))

    # 最终：旗面法向 / 下垂程度
    far_pos = pos[far]
    span = np.linalg.norm(far_pos - rest[far])
    # 水平旗面从 +Y 伸出；落到 -Z 时 far 的 y 分量应该接近 0
    droop = np.degrees(np.arctan2(-(far_pos[2] - rest[idx[0, 0], 2]),
                                  far_pos[1] - rest[idx[0, 0], 1]))
    stretch = np.max(np.linalg.norm(pos[edges[:, 0]] - pos[edges[:, 1]], axis=1)
                     / rest_len)

    print(f"[{style:3s}] far 位移 = {span:.2f} m | 下垂角 = {droop:5.1f}° "
          f"| 最大拉伸 = {stretch * 100:5.1f}% | 末位置 = "
          f"({far_pos[0]:.2f}, {far_pos[1]:.2f}, {far_pos[2]:.2f})")
    if verbose:
        for f, p in trace:
            print(f"        t={f / 60:4.1f}s  far=({p[0]:6.2f},{p[1]:6.2f},{p[2]:6.2f})")


if __name__ == "__main__":
    print("水平旗面（从横杆向 +Y 伸出），重力 -Z：")
    run("old", verbose=False)
    run("new", verbose=False)
    print("\\n迭代次数 / 拉伸上限遍数 对最大拉伸与下垂角的影响（new）：")
    for it in (6, 8, 10, 12):
        for cp in (2, 4, 6, 8):
            ITERS = it
            CAP_PASSES = cp
            print(f"  iters={it:2d} capPasses={cp}: ", end="")
            run("new", verbose=False)


