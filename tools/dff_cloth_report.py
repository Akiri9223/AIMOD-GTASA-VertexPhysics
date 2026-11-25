"""用 INU Tools 的纯 Python DFF 读取器，离线复现 VertexPhysics 的布料拓扑。

输出：
  * 每个 Geometry 的顶点/三角形/预光照色统计
  * 布料顶点的连通片（component）大小、包围盒、挂点数量
  * 每个连通片的 restSpan 范围（决定自由位移上限）
  * 可选的 PNG 俯视图，直观看旗面形状
"""

import importlib.util
import os
import sys
import types
from collections import Counter, defaultdict

import numpy as np

DFF_CORE = (r"C:\Users\Chipachipy\AppData\Roaming\Blender Foundation\Blender"
            r"\5.2\extensions\user_default\inu_tools_gta_sa\core")


def load_dff_module():
    pkg = types.ModuleType("inucore")
    pkg.__path__ = [DFF_CORE]
    sys.modules["inucore"] = pkg
    for name in ("rwbinary", "dff"):
        path = os.path.join(DFF_CORE, name + ".py")
        spec = importlib.util.spec_from_file_location(f"inucore.{name}", path)
        mod = importlib.util.module_from_spec(spec)
        sys.modules[f"inucore.{name}"] = mod
        spec.loader.exec_module(mod)
    return sys.modules["inucore.dff"]


def classify(c):
    """与 GeometryPhysics.cpp 的 ClassifyVertexColor 一致。"""
    r, g, b = c
    if r == g == b:
        return None
    if r == 0:
        return None

    def near(t):
        return abs(r - t) <= 4

    if near(0x40):
        return "hair"
    if near(0x80):
        return "cloth"
    if near(0xC0):
        return "body"
    if r == 0xFE:
        return "soft"
    if near(0x03):
        return "hair"
    return None


def main(path):
    dff = load_dff_module()
    clump = dff.read_dff_file(path)
    print(f"file={path}")
    print(f"geometries={len(clump.geometries)} atomics={len(clump.atomics)}")

    for gi, g in enumerate(clump.geometries):
        n = len(g.vertices)
        print(f"\n== Geometry {gi}: verts={n} tris={len(g.triangles)} "
              f"prelit={len(g.prelit_colors)} normals={len(g.normals)} ==")

        colors = [(c.r, c.g, c.b) if hasattr(c, "r") else tuple(c)
                  for c in g.prelit_colors]
        print("color histogram top 8:")
        for c, k in Counter(colors).most_common(8):
            print(f"   {c} x{k}")

        types = [classify(c) for c in colors]
        print("types:", Counter(t for t in types if t))

        # 同位置顶点合并（0.1mm 量化），与 InitGeometryPhysics 一致
        quant = {}
        v2p = [-1] * n
        part_pos = []
        part_type = []
        part_src = []
        for v in range(n):
            if types[v] is None:
                continue                      # 刚体顶点不是粒子
            key = tuple(int(round(x * 10000.0)) for x in g.vertices[v][:3])
            if key in quant:
                v2p[v] = quant[key]
                continue
            quant[key] = len(part_pos)
            v2p[v] = len(part_pos)
            part_pos.append(np.array(g.vertices[v][:3], dtype=float))
            part_type.append(types[v])
            part_src.append(v)

        m = len(part_pos)
        print(f"particles after weld: {m}")

        # 约束（三角形中两端都是粒子的边）
        edges = set()
        pins = [[] for _ in range(m)]  # (target index, rest length)

        for t in g.triangles:
            idx = [t.a, t.b, t.c]
            pid = [v2p[i] for i in idx]
            for a in range(3):
                for b in range(a + 1, 3):
                    if pid[a] >= 0 and pid[b] >= 0 and pid[a] != pid[b]:
                        edges.add((min(pid[a], pid[b]), max(pid[a], pid[b])))
            # 物理粒子 <-> 刚体顶点 的挂点
            for a in range(3):
                if pid[a] < 0:
                    continue
                for b in range(3):
                    if b == a or pid[b] >= 0:
                        continue
                    d = float(np.linalg.norm(part_pos[pid[a]] - np.array(g.vertices[idx[b]][:3])))
                    if d > 1e-4:
                        pins[pid[a]].append((idx[b], d))

        # 并查集
        parent = list(range(m))

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
        for i in range(m):
            comps[find(i)].append(i)

        cloth_comps = []
        for root, members in comps.items():
            ct = Counter(part_type[i] for i in members)
            if ct.get("cloth", 0) == 0:
                continue
            pts = np.array([part_pos[i] for i in members])
            lo, hi = pts.min(axis=0), pts.max(axis=0)
            anchored = [i for i in members if pins[i]]
            hard = [i for i in members if part_type[i] is None]
            has_pin = bool(anchored)
            print(f"\n  cloth piece: verts={len(members)} types={dict(ct)}")
            print(f"    bbox min={np.round(lo, 2)} max={np.round(hi, 2)} "
                  f"size={np.round(hi - lo, 2)}")
            print(f"    vertsWithPins={len(anchored)} pinLinks={sum(len(pins[i]) for i in members)} "
                  f"hardAnchors={len(hard)}")
            if anchored:
                ap = np.array([part_pos[i] for i in anchored])
                print(f"    pinned bbox min={np.round(ap.min(axis=0),2)} max={np.round(ap.max(axis=0),2)}")
                print(f"    pinned yz={[tuple(np.round(part_pos[i][1:],2)) for i in anchored[:12]]}")
                tp = np.array([np.array(g.vertices[ti][:3]) for i in anchored for ti, _d in pins[i]])
                if len(tp):
                    print(f"    pin target bbox min={np.round(tp.min(axis=0),2)} max={np.round(tp.max(axis=0),2)} "
                          f"span={np.round(tp.max(axis=0)-tp.min(axis=0),2)}")

            # restSpan = 到最近锚点（pin 目标或 hardAnchor）
            anchor_pts = []
            for i in members:
                for ti, _d in pins[i]:
                    anchor_pts.append(np.array(g.vertices[ti][:3]))
            spans = []
            for i in members:
                if not anchor_pts:
                    break
                d = min(np.linalg.norm(part_pos[i] - a) for a in anchor_pts)
                spans.append(d)
            if spans:
                print(f"    restSpan min={min(spans):.2f} max={max(spans):.2f} "
                      f"freeLimit(max)={min(10.0, max(spans) * 2 + 0.2):.2f}")
            cloth_comps.append((members, part_pos, pins, part_type))

        # 俯视/侧视 PNG：把布料片画出来（红=有挂点，蓝=自由）
        if cloth_comps:
            save_preview(path, gi, cloth_comps)


def save_preview(path, gi, comps):
    try:
        from PIL import Image, ImageDraw
    except Exception:
        return

    W = H = 900
    img = Image.new("RGB", (W, H), (20, 20, 24))
    dr = ImageDraw.Draw(img)

    allpts = np.array([pos[i] for members, pos, _, _ in comps for i in members])
    yz = allpts[:, 1:3]
    lo = np.array([yz[:,0].min(), yz[:,1].min()])
    hi = np.array([yz[:,0].max(), yz[:,1].max()])
    span = np.maximum(hi - lo, 0.01)
    scale = 0.85 * min(W / span[0], H / span[1])

    def to_px(p):
        return (int((p[0] - lo[0]) * scale + 0.075 * W),
                int(H - ((p[1] - lo[1]) * scale + 0.075 * H)))

    # 按 YZ 侧视：Y 右，Z 上（这面旗在 x 常数平面上）
    for members, pos, pins, ptype in comps:
        for i in members:
            x, y = to_px((pos[i][1], pos[i][2], 0.0))
            color = (255, 80, 80) if pins[i] else (80, 180, 255)
            dr.ellipse([x - 2, y - 2, x + 2, y + 2], fill=color)

    out = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       f"cloth_preview_g{gi}.png")
    img.save(out)
    print(f"\n  preview saved: {out}  (red=pinned, blue=free, top view XY)")


if __name__ == "__main__":
    p = sys.argv[1] if len(sys.argv) > 1 else \
        r"A:\Vanilla GTASA\modloader\新建文件夹\carlshou1_lae2.dff"
    main(p)




