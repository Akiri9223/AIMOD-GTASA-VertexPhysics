"""Reproduce VertexPhysics' hair solver offline on the real ped hair mesh.

Tests three scenarios with the CURRENT algorithm and with the PROPOSED fix:

  A) character standing still   -> does the hair stay on the scalp?
  B) character running at v m/s -> is there a permanent velocity-proportional lag?
  C) character stops           -> does it recover?

The proposed fix is "frame transport": move pos & prev by the anchor's own
per-frame displacement (so the character's rigid motion is NOT seen as hair
velocity) and then subtract the anchor acceleration as a fictitious force.

Usage: python tools/hair_motion_sim.py <dff>
"""

import importlib.util
import os
import sys
import types
from collections import Counter, defaultdict

import numpy as np

DFF_CORE = (r"C:\Users\Chipachipy\AppData\Roaming\Blender Foundation\Blender"
            r"\5.2\extensions\user_default\inu_tools_gta_sa\core")

G = 9.8
DT = 1.0 / 60.0
ITER = 12


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


SOFT = [(0.0, 1.00, 0.50), (64.0, 0.77, 0.38), (128.0, 0.55, 0.26),
        (192.0, 0.32, 0.14), (255.0, 0.10, 0.02)]


def softness(g):
    if g <= SOFT[0][0]:
        return SOFT[0][1], SOFT[0][2]
    if g >= SOFT[-1][0]:
        return SOFT[-1][1], SOFT[-1][2]
    for i in range(len(SOFT) - 1):
        if g <= SOFT[i + 1][0]:
            t = (g - SOFT[i][0]) / (SOFT[i + 1][0] - SOFT[i][0])
            stiff = SOFT[i][1] + (SOFT[i + 1][1] - SOFT[i][1]) * t
            ret = SOFT[i][2] + (SOFT[i + 1][2] - SOFT[i][2]) * t
            return stiff, ret
    return SOFT[-1][1], SOFT[-1][2]


class Mesh:
    pass


def build(path):
    dff = load_dff_module()
    clump = dff.read_dff_file(path)
    g = max(clump.geometries, key=lambda gg: len(gg.vertices))
    n = len(g.vertices)

    colors = [(c.r, c.g, c.b) if hasattr(c, "r") else tuple(c)
              for c in g.prelit_colors]
    types = [classify(c) for c in colors]

    verts = np.array([v[:3] for v in g.vertices], dtype=float)

    quant = {}
    v2p = [-1] * n
    p_idx = []
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

    stiff = np.zeros(m)
    ret = np.zeros(m)
    anchor = np.zeros(m)

    for i, v in enumerate(p_idx):
        r, gg, bb = colors[v]
        s, rr = softness(gg)
        if ptype[i] == "cloth":
            s = max(s, 0.55)
        stiff[i] = s
        ret[i] = rr
        anchor[i] = max(0.0, min(1.0, 1.0 - bb / 255.0))

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
                d = float(np.linalg.norm(pos[pid[a]] - verts[idx[b]]))
                if d > 1e-4:
                    pins[pid[a]].append((verts[idx[b]].copy(), d))

    e = np.array(sorted(edges), dtype=int)
    rest_len = np.linalg.norm(pos[e[:, 1]] - pos[e[:, 0]], axis=1)
    keep = rest_len >= 1e-4
    e = e[keep]
    rest_len = rest_len[keep]
    cstiff = np.clip(0.5 * (stiff[e[:, 0]] + stiff[e[:, 1]]), 0.05, 1.0)

    # restSpan / free limit (same rules as InitGeometryPhysics)
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

    span = np.zeros(m)
    limit = np.zeros(m)

    for root, members in comps.items():
        if not members:
            continue
        anchor_pts = []
        for i in members:
            for target, _d in pins[i]:
                anchor_pts.append(target)
        centroid = pos[members].mean(axis=0)
        radius = np.linalg.norm(pos[members] - centroid, axis=1).max()
        for i in members:
            if anchor_pts:
                ap = np.array(anchor_pts)
                best = np.linalg.norm(ap - pos[i], axis=1).min()
            else:
                best = radius
            span[i] = best
            lead = 0.10 if ptype[i] == "hair" else 0.20
            cap = 4.0 if ptype[i] == "hair" else 10.0
            limit[i] = min(cap, max(lead, span[i] * 2.0 + lead))

    mesh = Mesh()
    mesh.pos0 = pos
    mesh.type = ptype
    mesh.stiff = stiff
    mesh.ret = ret
    mesh.anchor = anchor
    mesh.edges = e
    mesh.rest_len = rest_len
    mesh.cstiff = cstiff
    mesh.pins = pins
    mesh.pintab = PinTable(pins)
    mesh.span = span
    mesh.limit = limit
    mesh.hair = np.array([i for i in range(m) if ptype[i] == "hair"])
    mesh.verts = verts
    mesh.triangles = [(t.a, t.b, t.c) for t in g.triangles]

    print(f"particles={m} hair={len(mesh.hair)} edges={len(e)} "
          f"pinned={sum(1 for p in pins if p)}")
    print(f"hair restSpan: mean={span[mesh.hair].mean():.3f} "
          f"max={span[mesh.hair].max():.3f} "
          f"freeLimit mean={limit[mesh.hair].mean():.3f} "
          f"max={limit[mesh.hair].max():.3f}")
    return mesh


def clamp_anchor(p, anch, lim):
    d = anch - p
    d2 = (d * d).sum(axis=-1)
    bad = d2 > lim * lim
    if bad.any():
        dist = np.sqrt(d2[bad])
        p[bad] = anch[bad] - d[bad] * (lim[bad] / dist)[:, None]
    return p


def limit_stretch(pos, e, rest_len, movable):
    for _ in range(2):
        d = pos[e[:, 1]] - pos[e[:, 0]]
        L = np.linalg.norm(d, axis=1)
        lim = rest_len * 1.35
        over = L > lim
        if not over.any():
            return
        ex = (L[over] - lim[over])
        dirv = d[over] / L[over, None]
        wa = movable[e[over, 0]].astype(float)
        wb = movable[e[over, 1]].astype(float)
        ws = wa + wb
        ok = ws > 0
        if not ok.any():
            return
        idx = np.where(over)[0][ok]
        adj = ex[ok] / ws[ok]
        # scatter (Jacobi-ish, fine for the diagnosis)
        np.add.at(pos, e[idx, 0], dirv[ok] * (adj * wa[ok])[:, None])
        np.add.at(pos, e[idx, 1], -dirv[ok] * (adj * wb[ok])[:, None])


class PinTable:
    """Flat pin links: (particle, target, restLength, layer) with layer = order
    of the link inside that particle.  Lets us replay the sequential (last one
    wins) solver and the consensus solver fully vectorized."""

    def __init__(self, pins):
        part = []
        target = []
        rest = []
        layer = []
        for i, plist in enumerate(pins):
            for k, (t, r) in enumerate(plist):
                part.append(i)
                target.append(t)
                rest.append(r)
                layer.append(k)
        self.part = np.array(part, dtype=int)
        self.target = np.array(target, dtype=float).reshape(-1, 3)
        self.rest = np.array(rest, dtype=float)
        self.layer = np.array(layer, dtype=int)
        self.maxlayer = int(self.layer.max()) + 1 if len(self.layer) else 0
        if len(self.part):
            self.w = 1.0 / np.maximum(self.rest ** 2, 1e-6)
        else:
            self.w = np.zeros(0)

    def solve_seq(self, pos):
        """Current C++ behaviour: hard sphere projection, link by link."""
        for k in range(self.maxlayer):
            sel = self.layer == k
            idx = self.part[sel]
            p = pos[idx]
            t = self.target[sel]
            r = self.rest[sel]
            d = p - t
            L = np.linalg.norm(d, axis=1)
            L[L < 1e-5] = 1e-5
            pos[idx] = t + d * (r / L)[:, None]

    def solve_consensus(self, pos, alpha=1.0):
        """Proposed: one weighted-average target, no order dependence."""
        d = pos[self.part] - self.target
        L = np.linalg.norm(d, axis=1)
        L[L < 1e-5] = 1e-5
        want = self.target + d * (self.rest / L)[:, None]

        acc = np.zeros_like(pos)
        wsum = np.zeros(len(pos))
        np.add.at(acc, self.part, want * self.w[:, None])
        np.add.at(wsum, self.part, self.w)

        hit = wsum > 0
        if not hit.any():
            return
        target_pos = acc[hit] / wsum[hit][:, None]
        pos[hit] = pos[hit] + (target_pos - pos[hit]) * alpha


def run(mesh, frames, speed=0.0, stop_at=None, fix=False, pins_new=False,
        damping=0.92, transport=0.0, verbose=True):
    pos = mesh.pos0.copy()
    prev = pos.copy()
    anchor_prev = pos.copy()

    h = DT
    h2 = h * h
    damp = damping
    maxstep = 8.0 * h
    maxstep2 = maxstep * maxstep
    mov = np.ones(len(pos), bool)
    mov[mesh.type == "body"] = False

    # per-particle frame velocity used for the fictitious force
    fvel = np.zeros_like(pos)

    hist = []

    for f in range(frames):
        t = f * h

        if stop_at is not None and t >= stop_at:
            anc = mesh.pos0 + np.array([speed * stop_at, 0, 0])
        else:
            anc = mesh.pos0 + np.array([speed * t, 0, 0])

        move = anc - anchor_prev
        anchor_prev = anc.copy()

        if fix:
            # frame transport: the rigid motion of the frame is not hair velocity
            pos = pos + move
            prev = prev + move

            avel = move / h
            lowpass = 0.35
            acc = (avel - fvel) / h
            fvel = fvel + (avel - fvel) * lowpass
            alen = np.linalg.norm(acc, axis=1)
            bad = alen > 30.0
            if bad.any():
                acc[bad] *= (30.0 / alen[bad])[:, None]
        else:
            if transport > 0.0:
                ride = move * transport
                pos = pos + ride
                prev = prev + ride
            acc = np.zeros_like(pos)

        vel = (pos - prev) * damp
        vel[:, 2] -= G * h2
        vel = vel - acc * h2

        v2 = (vel * vel).sum(axis=1)
        big = v2 > maxstep2
        if big.any():
            vel[big] *= (maxstep / np.sqrt(v2[big]))[:, None]

        prev = pos.copy()
        pos = pos + vel
        pos = clamp_anchor(pos, anc, mesh.limit)

        for _ in range(ITER):
            d = pos[mesh.edges[:, 1]] - pos[mesh.edges[:, 0]]
            L = np.linalg.norm(d, axis=1)
            L[L < 1e-6] = 1e-6
            diff = (L - mesh.rest_len) / L
            wa = mesh.cstiff * 0.5
            np.add.at(pos, mesh.edges[:, 0], d * (diff * wa)[:, None])
            np.add.at(pos, mesh.edges[:, 1], -d * (diff * wa)[:, None])

            if pins_new:
                mesh.pintab.solve_consensus(pos)
            else:
                mesh.pintab.solve_seq(pos)

            pull = mesh.anchor * mesh.ret
            pull = pull * mesh.anchor           # hair: squared
            m = mov & (pull > 0)
            pos[m] += (anc[m] - pos[m]) * pull[m, None]

        limit_stretch(pos, mesh.edges, mesh.rest_len, mov)
        pos = clamp_anchor(pos, anc, mesh.limit)

        disp = np.linalg.norm(pos - anc, axis=1)
        hist.append(disp[mesh.hair].copy())

    hist = np.array(hist)
    if verbose:
        print(f"    hair |pos-anchor|: mean={hist[-1].mean():.3f} "
              f"p95={np.percentile(hist[-1], 95):.3f} max={hist[-1].max():.3f}")
    return pos, anc, hist


def main(path):
    mesh = build(path)

    print("\n--- A) standing still, current solver (3s) ---")
    p_old, a_old, h_old = run(mesh, 180)

    print("--- A) standing still, pin consensus (3s) ---")
    p_new, a_new, h_new = run(mesh, 180, pins_new=True)

    # how much do the two differ at rest?
    diff = np.linalg.norm(p_old - p_new, axis=1)
    print(f"    rest shape difference: mean={diff.mean()*1000:.1f}mm "
          f"max={diff.max()*1000:.1f}mm  (n={len(diff)})")

    print("\n--- B) running 6 m/s, current solver (3s) ---")
    p_run_old, a_run_old, h_run_old = run(mesh, 180, speed=6.0)

    print("--- B) running 6 m/s, frame transport (3s) ---")
    p_run_new, a_run_new, h_run_new = run(mesh, 180, speed=6.0, fix=True)

    print("\n--- C) stop after 3s, then 1s settle ---")
    p_stop_old, _, _ = run(mesh, 240, speed=6.0, stop_at=3.0)
    p_stop_new, _, _ = run(mesh, 240, speed=6.0, stop_at=3.0, fix=True)

    for name, arr in (("current", p_stop_old), ("fix", p_stop_new)):
        d = np.linalg.norm(arr - (mesh.pos0 + np.array([18.0, 0, 0])), axis=1)
        print(f"    {name}: after stop, hair-vs-anchor mean={d[mesh.hair].mean():.3f} "
              f"max={d[mesh.hair].max():.3f}")

    # where is the back-of-head bulge at rest?  compare old/new by region
    z = mesh.pos0[:, 2]
    y = mesh.pos0[:, 1]
    hair = mesh.hair
    for label, lo, hi in (("head top", 1.55, 1.80), ("back low", 1.35, 1.55)):
        sel = hair[(z[hair] >= lo) & (z[hair] < hi)]
        if len(sel) == 0:
            continue
        do = np.linalg.norm(p_old[sel] - mesh.pos0[sel], axis=1)
        dn = np.linalg.norm(p_new[sel] - mesh.pos0[sel], axis=1)
        print(f"    rest displacement {label}: old mean={do.mean()*1000:.0f}mm "
              f"max={do.max()*1000:.0f}mm | consensus mean={dn.mean()*1000:.0f}mm "
              f"max={dn.max()*1000:.0f}mm")


if __name__ == "__main__":
    main(sys.argv[1])
