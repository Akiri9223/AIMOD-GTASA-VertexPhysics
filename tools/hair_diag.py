"""Offline diagnostic for the hair ped model (tk7Chi2).

Reproduces exactly what InitGeometryPhysics builds for the hair particles
(weld-by-quantized-position, edge constraints, rigid pins) and reports:

  * skin plugin stats (bone count, max weights, zero-weight vertices)
  * hair piece topology (size, pins per particle, pin restLength distribution)
  * whether multiple pins on one particle disagree (pin conflict)
  * restSpan distribution used for the per-particle free-motion limit

Usage:  python tools/hair_diag.py <path-to-dff> [target-vertex-count]
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


def main(path, target=None):
    dff = load_dff_module()
    clump = dff.read_dff_file(path)
    print(f"file={path}")
    print(f"geometries={len(clump.geometries)} atomics={len(clump.atomics)}")

    best = None

    for gi, g in enumerate(clump.geometries):
        n = len(g.vertices)
        print(f"\n== Geometry {gi}: verts={n} tris={len(g.triangles)} "
              f"prelit={len(g.prelit_colors)} ==")

        skin = getattr(g, "skin", None)
        if skin is None:
            print("  skin: NONE")
        else:
            print(f"  skin: bones={skin.num_bones} used={skin.num_used} "
                  f"max_weights={skin.max_weights} "
                  f"indices={len(skin.bone_indices)} weights={len(skin.bone_weights)}")

            zero = [v for v in range(n)
                    if sum(skin.bone_weights[v]) <= 1e-6]
            print(f"  zero-weight vertices: {len(zero)}"
                  f"{'  first=' + str(zero[:8]) if zero else ''}")

            nonzero = [v for v in range(n)
                       if sum(skin.bone_weights[v]) > 1e-6]

            if nonzero:
                w = np.array([skin.bone_weights[v] for v in nonzero])
                print(f"  weight sum min/max = {w.sum(axis=1).min():.4f}/"
                      f"{w.sum(axis=1).max():.4f}")
                print(f"  first nonzero vertex={nonzero[0]} "
                      f"weights={skin.bone_weights[nonzero[0]]} "
                      f"indices={skin.bone_indices[nonzero[0]]}")
                print(f"  last  nonzero vertex={nonzero[-1]} "
                      f"weights={skin.bone_weights[nonzero[-1]]} "
                      f"indices={skin.bone_indices[nonzero[-1]]}")

            if zero:
                v0 = zero[0]
                print(f"  zero-weight sample v={v0} "
                      f"weights={skin.bone_weights[v0]} "
                      f"indices={skin.bone_indices[v0]}")
                mx = np.max(np.array(skin.bone_weights, dtype=float), axis=1)
                print(f"  per-vertex max weight: mean={mx.mean():.3f} "
                      f"min={mx.min():.3f} max={mx.max():.3f}")

        if target is None or n == int(target):
            best = (gi, g)

    if best is None:
        print("\nno geometry matched target")
        return

    gi, g = best
    n = len(g.vertices)
    colors = [(c.r, c.g, c.b) if hasattr(c, "r") else tuple(c)
              for c in g.prelit_colors]
    types = [classify(c) for c in colors]

    print(f"\n==== topology for Geometry {gi} (verts={n}) ====")
    print("types:", Counter(t for t in types if t))

    # ---- particle build (same as InitGeometryPhysics) ----
    quant = {}
    v2p = [-1] * n
    part_pos = []
    part_type = []
    part_src = []

    for v in range(n):
        if types[v] is None:
            continue
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
    print(f"particles after weld: {m}  hair={part_type.count('hair')} "
          f"cloth={part_type.count('cloth')}")

    edges = set()
    pins = [[] for _ in range(m)]

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
                d = float(np.linalg.norm(
                    part_pos[pid[a]] - np.array(g.vertices[idx[b]][:3])))
                if d > 1e-4:
                    pins[pid[a]].append((idx[b], d))

    print(f"edge constraints: {len(edges)}")

    # union find
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

    npin = Counter(len(p) for p in pins)
    print("\npins per particle:", dict(sorted(npin.items())))

    hair_pieces = []
    for root, members in comps.items():
        ct = Counter(part_type[i] for i in members)
        if ct.get("hair", 0) == 0:
            continue
        hair_pieces.append((members, ct))

    print(f"\nhair pieces: {len(hair_pieces)}")

    all_spans = []

    for k, (members, ct) in enumerate(hair_pieces):
        pts = np.array([part_pos[i] for i in members])
        lo, hi = pts.min(axis=0), pts.max(axis=0)

        pinned = [i for i in members if pins[i]]
        nlinks = sum(len(pins[i]) for i in members)
        maxpins = max((len(pins[i]) for i in members), default=0)

        anchor_pts = []
        for i in members:
            for ti, _d in pins[i]:
                anchor_pts.append(np.array(g.vertices[ti][:3]))

        spans = []
        if anchor_pts:
            ap = np.array(anchor_pts)
            for i in members:
                d = np.linalg.norm(ap - part_pos[i], axis=1).min()
                spans.append(d)
            all_spans.extend(spans)

        # pin conflict measure: for one particle, the pin targets spread
        worst_spread = 0.0
        worst_len = 0.0
        for i in members:
            if len(pins[i]) < 2:
                continue
            tp = np.array([np.array(g.vertices[ti][:3]) for ti, _d in pins[i]])
            spread = float(np.linalg.norm(tp - tp.mean(axis=0), axis=1).max())
            if spread > worst_spread:
                worst_spread = spread
            lens = [d for _ti, d in pins[i]]
            worst_len = max(worst_len, max(lens))

        print(f"\n hair piece {k}: verts={len(members)} types={dict(ct)}")
        print(f"   bbox size={np.round(hi - lo, 3)}")
        print(f"   pinned={len(pinned)} pinLinks={nlinks} maxPinsPerVertex={maxpins}")
        if spans:
            print(f"   restSpan min={min(spans):.3f} max={max(spans):.3f} m "
                  f"-> freeLimit(max)={min(4.0, max(spans) * 2 + 0.10):.2f} m")
        print(f"   worst pin-target spread among a vertex's pins = "
              f"{worst_spread * 1000:.1f} mm, longest pin restLength = "
              f"{worst_len * 1000:.1f} mm")

    if all_spans:
        s = np.array(all_spans)
        print(f"\nALL hair restSpan: n={len(s)} mean={s.mean():.3f} "
              f"p50={np.percentile(s, 50):.3f} p90={np.percentile(s, 90):.3f} "
              f"max={s.max():.3f}")
        lim = np.minimum(4.0, s * 2.0 + 0.10)
        print(f"ALL hair freeLimit: mean={lim.mean():.3f} max={lim.max():.3f}")


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2] if len(sys.argv) > 2 else None)
