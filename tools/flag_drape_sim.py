"""用 DFF 里真实的旗面网格，离线复现 VertexPhysics 的布料求解。

对比三种挂点方式，看哪种能让"只有一条竖边挂在杆上"的旗面自然垂下来：
  A. 全部挂点（当前实现）
  B. 每个顶点只保留最近的一个挂点
  C. 只保留上/下两个角点（真实旗帜挂在杆上的样子）
"""

import importlib.util
import os
import sys
import types
from collections import defaultdict

import numpy as np

DFF_CORE = (r"C:\Users\Chipachipy\AppData\Roaming\Blender Foundation\Blender"
            r"\5.2\extensions\user_default\inu_tools_gta_sa\core")
DT = 1.0 / 60.0
G = 10.0
ITERS = 10
STIFF = 0.55
DAMP = 0.98
MAX_STEP = 8.0 * DT


def load_dff():
    pkg = types.ModuleType("inucore")
    pkg.__path__ = [DFF_CORE]
    sys.modules["inucore"] = pkg
    for name in ("rwbinary", "dff"):
        spec = importlib.util.spec_from_file_location(
            f"inucore.{name}", os.path.join(DFF_CORE, name + ".py"))
        mod = importlib.util.module_from_spec(spec)
        sys.modules[f"inucore.{name}"] = mod
        spec.loader.exec_module(mod)
    return sys.modules["inucore.dff"]


def classify(c):
    r, g, b = c
    if r == g == b or r == 0:
        return None
    if abs(r - 0x40) <= 4:
        return "hair"
    if abs(r - 0x80) <= 4:
        return "cloth"
    if abs(r - 0xC0) <= 4:
        return "body"
    if r == 0xFE:
        return "soft"
    if abs(r - 3) <= 4:
        return "hair"
    return None


def build(path, pin_mode):
    dff = load_dff()
    clump = dff.read_dff_file(path)
    g = clump.geometries[0]
    n = len(g.vertices)
    colors = [(c.r, c.g, c.b) for c in g.prelit_colors]
    types = [classify(c) for c in colors]

    quant, v2p, pos, ptype = {}, [-1] * n, [], []
    for v in range(n):
        if types[v] is None:
            continue
        key = tuple(int(round(x * 10000.0)) for x in g.vertices[v][:3])
        if key in quant:
            v2p[v] = quant[key]
            continue
        quant[key] = len(pos)
        v2p[v] = len(pos)
        pos.append(np.array(g.vertices[v][:3], dtype=float))
        ptype.append(types[v])

    pos = np.array(pos)
    edges = set()
    pins = [[] for _ in range(len(pos))]

    for t in g.triangles:
        idx = [t.a, t.b, t.c]
        pid = [v2p[i] for i in idx]
        for a in range(3):
            for b in range(a + 1, 3):
                if pid[a] >= 0 and pid[b] >= 0 and pid[a] != pid[b]:
                    edges.add((min(pid[a], pid[b]), max(pid[a], pid[b])))
        for a in range(3):
            if pid[a] < 0:
                continue
            for b in range(3):
                if b == a or pid[b] >= 0:
                    continue
                rp = np.array(g.vertices[idx[b]][:3])
                d = float(np.linalg.norm(pos[pid[a]] - rp))
                if d > 1e-4:
                    pins[pid[a]].append((rp, d))

    # 只看那面 63 顶点的旗（全 cloth 且最大的片）
    parent = list(range(len(pos)))

    def find(x):
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    for a, b in edges:
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[rb] = ra

    comps = defaultdict(list)
    for i in range(len(pos)):
        comps[find(i)].append(i)

    best = None
    for members in comps.values():
        if len(members) < 20:
            continue
        if all(ptype[i] == "cloth" for i in members):
            if best is None or len(members) > len(best):
                best = members
    assert best, "cloth piece not found"

    members = best
    remap = {v: k for k, v in enumerate(members)}
    P = pos[members]
    E = np.array([(remap[a], remap[b]) for a, b in edges
                  if a in remap and b in remap])
    rest_len = np.linalg.norm(P[E[:, 0]] - P[E[:, 1]], axis=1)

    pin_list = []
    for old in members:
        for rp, d in pins[old]:
            pin_list.append([remap[old], rp, d])
    pin_list = np.array(pin_list, dtype=object)

    # restSpan
    span = np.zeros(len(members))
    anchor_pts = [p[1] for p in pin_list]
    for k in range(len(members)):
        span[k] = min(np.linalg.norm(P[k] - a) for a in anchor_pts)
    limit = np.clip(span * 2.0 + 0.2, 0.2, 10.0)

    # 挂点模式
    if pin_mode == "B":
        nearest = {}
        for k, rp, d in pin_list:
            k = int(k)
            if k not in nearest or d < nearest[k][1]:
                nearest[k] = (rp, d)
        pin_list = np.array([(k, rp, d) for k, (rp, d) in nearest.items()],
                            dtype=object)
    elif pin_mode == "C":
        # 只留静止位置 Z 最高和最低的两个挂点顶点
        by_z = sorted({int(k) for k, _rp, _d in pin_list},
                      key=lambda k: P[k][2])
        keep = {by_z[0], by_z[-1]}
        pin_list = np.array([(k, rp, d) for k, rp, d in pin_list
                             if int(k) in keep], dtype=object)

    return P, E, rest_len, pin_list, span, limit, members, ptype


def simulate(P, E, rest_len, pin_list, limit, verbose=False, pin_alpha=1.0, pin_once=False):
    pos = P.copy()
    prev = P.copy()
    trace = []
    n = len(P)

    lo_pin = np.array([P[int(k)] for k, _rp, _d in pin_list]).min(axis=0)
    free_edge = int(np.argmax(P[:, 1])) if P[:, 1].min() < 0 else 0
    # 自由边：离挂点列最远的顶点（Y 最大）
    pin_y = np.mean([P[int(k)][1] for k, _rp, _d in pin_list])
    free_idx = int(np.argmax(np.abs(P[:, 1] - pin_y)))

    for frame in range(600):
        vel = (pos - prev) * DAMP
        vel[:, 2] -= G * DT * DT

        step = np.linalg.norm(vel, axis=1)
        mask = step > MAX_STEP
        vel[mask] *= (MAX_STEP / step[mask])[:, None]

        prev = pos.copy()
        pos = pos + vel

        for k in range(n):
            d = pos[k] - P[k]
            ln = np.linalg.norm(d)
            if ln > limit[k] and ln > 1e-9:
                pos[k] = P[k] + d * (limit[k] / ln)

        for _ in range(ITERS):
            a, b = E[:, 0], E[:, 1]
            d = pos[b] - pos[a]
            ln = np.linalg.norm(d, axis=1)
            ln[ln < 1e-6] = 1e-6
            diff = (ln - rest_len) / ln
            corr = d * (diff * 0.5 * STIFF)[:, None]
            np.add.at(pos, a, corr)
            np.add.at(pos, b, -corr)

            if not pin_once:
                for k, rp, rl in pin_list:
                    k = int(k)
                    d = pos[k] - rp
                    ln = np.linalg.norm(d)
                    if ln > 1e-6:
                        target = rp + d * (rl / ln)
                        pos[k] = pos[k] + (target - pos[k]) * pin_alpha

        if pin_once:
            for k, rp, rl in pin_list:
                k = int(k)
                d = pos[k] - rp
                ln = np.linalg.norm(d)
                if ln > 1e-6:
                    target = rp + d * (rl / ln)
                    pos[k] = pos[k] + (target - pos[k]) * pin_alpha

        for _ in range(2):
            a, b = E[:, 0], E[:, 1]
            d = pos[b] - pos[a]
            ln = np.linalg.norm(d, axis=1)
            over = ln > rest_len * 1.35
            if over.any():
                e = (ln[over] - rest_len[over] * 1.35) / ln[over]
                dir_ = d[over] / ln[over][:, None]
                np.add.at(pos, a[over], dir_ * (e * 0.5)[:, None])
                np.add.at(pos, b[over], -dir_ * (e * 0.5)[:, None])

        for k in range(n):
            d = pos[k] - P[k]
            ln = np.linalg.norm(d)
            if ln > limit[k] and ln > 1e-9:
                pos[k] = P[k] + d * (limit[k] / ln)

        if frame % 60 == 0:
            trace.append((frame, pos[free_idx].copy()))

        if frame == 300:
            mid = pos.mean(axis=0)

    drop = P[free_idx][2] - pos[free_idx][2]     # 自由边下落高度
    spread = np.max(np.abs(pos[:, 1] - P[:, 1]))  # 沿 Y 的位移
    max_stretch = np.max(np.linalg.norm(pos[E[:, 0]] - pos[E[:, 1]], axis=1)
                         / rest_len)
    return pos, drop, spread, max_stretch, trace, free_idx


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else \
        r"A:\Vanilla GTASA\modloader\新建文件夹\carlshou1_lae2.dff"
    cases = [
        ("A", "全部挂点(当前)", 1.0, False),
        ("B", "每点最近挂点", 1.0, False),
        ("C", "只留上下角点", 1.0, False),
        ("A", "全挂点 soft0.5/iter", 0.5, False),
        ("A", "全挂点 soft0.15/iter", 0.15, False),
        ("A", "全挂点 soft0.02/iter", 0.02, False),
        ("A", "全挂点 soft0.5/frame", 0.5, True),
        ("A", "全挂点 soft0.2/frame", 0.2, True),
    ]
    for mode, label, alpha, once in cases:
        P, E, rest_len, pin_list, span, limit, members, ptype = build(path, mode)
        pos, drop, spread, stretch, trace, free_idx = simulate(
            P, E, rest_len, pin_list, limit, pin_alpha=alpha, pin_once=once)
        print(f"[{mode}] {label}: verts={len(P)} pins={len(pin_list)} "
              f"| 自由边下落={drop:.2f}m 沿Y位移={spread:.2f}m "
              f"最大拉伸={stretch*100:.0f}%")
        for f, p in trace[:6]:
            print(f"      t={f/60:4.1f}s free=({p[0]:6.2f},{p[1]:6.2f},{p[2]:6.2f})")
        tag = f"{mode}_{label}".replace(" ", "_").replace("/", "_")
        save_png(P, pos, E, pin_list, tag)


def save_png(P, pos, E, pin_list, mode):
    try:
        from PIL import Image, ImageDraw
    except Exception:
        return
    W = H = 700
    img = Image.new("RGB", (W, H), (18, 18, 22))
    dr = ImageDraw.Draw(img)

    allp = np.vstack([P, pos])
    lo = np.array([allp[:, 1].min(), allp[:, 2].min()])
    hi = np.array([allp[:, 1].max(), allp[:, 2].max()])
    span = np.maximum(hi - lo, 0.01)
    scale = 0.85 * min(W / span[0], H / span[1])

    def px(p):
        return (int((p[1] - lo[0]) * scale + 0.075 * W),
                int(H - ((p[2] - lo[1]) * scale + 0.075 * H)))

    for a, b in E:
        dr.line([px(pos[a]), px(pos[b])], fill=(70, 90, 120), width=1)
    for i in range(len(P)):
        dr.ellipse([px(pos[i])[0] - 2, px(pos[i])[1] - 2,
                    px(pos[i])[0] + 2, px(pos[i])[1] + 2], fill=(120, 220, 255))
    for k, _rp, _d in pin_list:
        p = px(pos[int(k)])
        dr.ellipse([p[0] - 3, p[1] - 3, p[0] + 3, p[1] + 3], fill=(255, 80, 80))

    out = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       f"flag_sim_{mode}.png")
    img.save(out)
    print(f"      shape saved: {out}")


if __name__ == "__main__":
    main()



