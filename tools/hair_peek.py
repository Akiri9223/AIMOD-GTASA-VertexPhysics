"""Inspect specific particles/vertices from the [TriSpike] log lines.

Rebuilds particles exactly like InitGeometryPhysics (quantize-0.1mm merge),
then prints for the requested particle indices / geometry vertex indices:
type, piece id, piece stats, pin links, restSpan, and whether an edge
constraint exists between a given pair.

Usage: python tools/hair_peek.py <dff> <particleA> <particleB> [<vertexA> <vertexB> ...]
"""

import importlib.util
import os
import sys
import types
from collections import defaultdict

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
    r, g, b = c
    if r == g == b or r == 0:
        return None

    def near(t):
        return abs(r - t) <= 4

    if near(0x40) or near(0x03):
        return "hair"
    if near(0x80):
        return "cloth"
    if near(0xC0):
        return "body"
    if r == 0xFE:
        return "soft"
    return None


def main(path, pids, vids):
    dff = load_dff_module()
    clump = dff.read_dff_file(path)
    g = max(clump.geometries, key=lambda gg: len(gg.vertices))
    n = len(g.vertices)
    verts = np.array([v[:3] for v in g.vertices], dtype=float)
    colors = [(c.r, c.g, c.b) if hasattr(c, "r") else tuple(c)
              for c in g.prelit_colors]
    types = [classify(c) for c in colors]

    quant, v2p, p_idx = {}, [-1] * n, []
    for v in range(n):
        if types[v] is None:
            continue
        key = tuple(int(round(x * 10000.0)) for x in verts[v])
        if key in quant:
            v2p[v] = quant[key]
            continue
        quant[key] = len(p_idx)
        v2p[v] = len(p_idx)
        p_idx.append(v)

    m = len(p_idx)
    pos = verts[p_idx].copy()
    ptype = [types[v] for v in p_idx]

    edges, pins = set(), [[] for _ in range(m)]
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
                d = float(np.linalg.norm(pos[pid[a]] - verts[idx[b]]))
                if d > 1e-4:
                    pins[pid[a]].append((idx[b], d))

    e = np.array(sorted(edges), dtype=int)
    rest = np.linalg.norm(pos[e[:, 1]] - pos[e[:, 0]], axis=1)
    keep = rest >= 1e-4
    e, rest = e[keep], rest[keep]

    parent = list(range(m))

    def find(x):
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    for a, b in e:
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[rb] = ra
    comps = defaultdict(list)
    for i in range(m):
        comps[find(i)].append(i)
    comp_of = {}
    for k, (root, members) in enumerate(comps.items()):
        for i in members:
            comp_of[i] = k

    # Edge lookup
    edge_len = {}
    for a, b, r in zip(e[:, 0], e[:, 1], rest):
        edge_len[(int(a), int(b))] = float(r)

    print(f"particles={m} edges={len(e)} pieces={len(comps)}")

    print("\n== requested particles ==")
    for p in pids:
        p = int(p)
        if p < 0 or p >= m:
            print(f"  particle {p}: OUT OF RANGE")
            continue
        comp = comps[find(p)]
        pinl = pins[p]
        deg = int(((e[:, 0] == p) | (e[:, 1] == p)).sum())
        near_pins = ", ".join(f"v{vi}:{d*1000:.1f}mm" for vi, d in pinl[:6])
        print(f"  particle {p}: type={ptype[p]} vert={p_idx[p]} "
              f"piece={comp_of[p]} pieceN={len(comp)} deg={deg} "
              f"pins={len(pinl)} [{near_pins}]")

    print("\n== requested geometry vertices ==")
    for v in vids:
        v = int(v)
        if v < 0 or v >= n:
            print(f"  vertex {v}: OUT OF RANGE")
            continue
        p = v2p[v]
        print(f"  vertex {v}: type={types[v]} -> particle {p} "
              f"piece={comp_of.get(p, -1) if p >= 0 else -1} "
              f"pos={np.round(verts[v], 4)}")

    # All pairs of requested particles: edge? restLen?
    print("\n== pairwise edge check ==")
    pl = [int(p) for p in pids if 0 <= int(p) < m]
    for i in range(len(pl)):
        for j in range(i + 1, len(pl)):
            a, b = pl[i], pl[j]
            key = (min(a, b), max(a, b))
            if key in edge_len:
                print(f"  edge ({a},{b}) EXISTS restLen={edge_len[key]*1000:.1f}mm")
            else:
                print(f"  edge ({a},{b}) MISSING  (samePiece={comp_of[a]==comp_of[b]})")

    # Are there triangles whose edge endpoints are in different pieces?
    print("\n== triangles with an edge that has NO constraint between two particles ==")
    bad = 0
    shown = 0
    for t in g.triangles:
        idx = [t.a, t.b, t.c]
        pid = [v2p[i] for i in idx]
        for k in range(3):
            a, b = pid[k], pid[(k + 1) % 3]
            if a < 0 or b < 0 or a == b:
                continue
            key = (min(a, b), max(a, b))
            if key in edge_len:
                continue
            same = comp_of[a] == comp_of[b]
            bad += 1
            if shown < 12:
                shown += 1
                d = float(np.linalg.norm(pos[a] - pos[b]))
                print(f"  tri={t.a}/{t.b}/{t.c} particles=({a},{b}) "
                      f"samePiece={same} dist={d*1000:.1f}mm "
                      f"pieces=({comp_of[a]},{comp_of[b]})")
    print(f"  total such edges: {bad}")


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2:4], sys.argv[4:])
