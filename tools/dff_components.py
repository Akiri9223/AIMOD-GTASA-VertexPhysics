"""列出 DFF 里所有连通片（含未刷色）的 bbox，定位"屋顶上那面旗"。"""

import importlib.util
import os
import sys
import types
from collections import defaultdict

import numpy as np

DFF_CORE = (r"C:\Users\Chipachipy\AppData\Roaming\Blender Foundation\Blender"
            r"\5.2\extensions\user_default\inu_tools_gta_sa\core")


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
        return "rigid"
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
    return "rigid"


def main(path):
    dff = load_dff()
    g = dff.read_dff_file(path).geometries[0]
    n = len(g.vertices)
    P = np.array([g.vertices[i][:3] for i in range(n)])
    colors = [(c.r, c.g, c.b) for c in g.prelit_colors]
    types = [classify(c) for c in colors]

    print(f"verts={n} tris={len(g.triangles)}")
    print(f"model bbox min={np.round(P.min(0),2)} max={np.round(P.max(0),2)} "
          f"size={np.round(P.max(0)-P.min(0),2)}")

    # 全网格连通片（按三角形顶点）
    parent = list(range(n))

    def find(x):
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    for t in g.triangles:
        a, b, c = find(t.a), find(t.b), find(t.c)
        parent[b] = a
        parent[find(c)] = a

    comps = defaultdict(list)
    for i in range(n):
        comps[find(i)].append(i)

    rows = []
    for members in comps.values():
        pts = P[members]
        lo, hi = pts.min(0), pts.max(0)
        size = hi - lo
        tc = defaultdict(int)
        for i in members:
            tc[types[i]] += 1
        rows.append((len(members), size, lo, hi, dict(tc)))

    rows.sort(key=lambda r: -(r[1][1] * r[1][2]))   # 按 Y*Z 面积排

    print("\n最大的 15 个连通片（按 YZ 面积）：")
    for cnt, size, lo, hi, tc in rows[:15]:
        print(f"  verts={cnt:4d} size=({size[0]:5.2f},{size[1]:5.2f},{size[2]:5.2f}) "
              f"z=[{lo[2]:5.2f},{hi[2]:5.2f}] y=[{lo[1]:6.2f},{hi[1]:6.2f}] "
              f"x=[{lo[0]:6.2f},{hi[0]:6.2f}] {tc}")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else
         r"A:\Vanilla GTASA\modloader\新建文件夹\carlshou1_lae2.dff")
