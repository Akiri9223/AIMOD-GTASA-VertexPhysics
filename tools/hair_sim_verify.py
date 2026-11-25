"""Offline reproduction of VertexPhysics' hair solver on the real ped mesh.

Purpose: prove the two reported artefacts and validate the fixes BEFORE
touching the game build.

  static  : character standing still  -> hair is pushed out of the body
                                        capsules that already contain it
  moving  : character running         -> permanent velocity-proportional lag
                                        (world-space Verlet without frame
                                        transport)

Fixes validated here:
  F1 frame transport for hair (pos & prev ride the anchor's own motion) plus
     the anchor's acceleration as a fictitious force
  F2 hair capsule ignore mask built from the rest pose (a capsule that already
     contains the particle at rest must not push it)
  F3 pin consensus (weighted average target instead of "last pin wins")

Usage:
  python tools/hair_sim_verify.py <dff> [--png out.png]
"""

import argparse
import importlib.util
import os
import sys
import types
from collections import defaultdict

import numpy as np

DFF_CORE = (r"C:\Users\Chipachipy\AppData\Roaming\Blender Foundation\Blender"
            r"\5.2\extensions\user_default\inu_tools_gta_sa\core")

G = 9.8
DT = 1.0 / 60.0

SCALE = 1.0
MARGIN = 0.02

BONE_IDS = dict(PELVIS1=1, PELVIS=2, SPINE1=3, UPPERTORSO=4, NECK=5, HEAD=8,
                LEFTSHOULDER=32, LEFTELBOW=33, LEFTWRIST=34,
                RIGHTSHOULDER=22, RIGHTELBOW=23, RIGHTWRIST=24,
                LEFTHIP=41, LEFTKNEE=42, LEFTANKLE=43, LEFTFOOT=44,
                RIGHTHIP=51, RIGHTKNEE=52, RIGHTANKLE=53, RIGHTFOOT=54)

CAPDEFS = [
    ("PELVIS1", "PELVIS1", 0.13), ("PELVIS", "PELVIS", 0.13),
    ("PELVIS", "LEFTHIP", 0.12), ("PELVIS", "RIGHTHIP", 0.12),
    ("LEFTHIP", "LEFTKNEE", 0.11), ("RIGHTHIP", "RIGHTKNEE", 0.11),
    ("LEFTKNEE", "LEFTANKLE", 0.09), ("RIGHTKNEE", "RIGHTANKLE", 0.09),
    ("LEFTHIP", "LEFTHIP", 0.115), ("RIGHTHIP", "RIGHTHIP", 0.115),
    ("LEFTKNEE", "LEFTKNEE", 0.09), ("RIGHTKNEE", "RIGHTKNEE", 0.09),
    ("UPPERTORSO", "UPPERTORSO", 0.14), ("PELVIS", "SPINE1", 0.13),
    ("SPINE1", "UPPERTORSO", 0.13), ("NECK", "HEAD", 0.10),
    ("LEFTSHOULDER", "LEFTELBOW", 0.07), ("RIGHTSHOULDER", "RIGHTELBOW", 0.07),
    ("LEFTELBOW", "LEFTWRIST", 0.065), ("RIGHTELBOW", "RIGHTWRIST", 0.065),
    ("LEFTANKLE", "LEFTFOOT", 0.07), ("RIGHTANKLE", "RIGHTFOOT", 0.07),
]

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
            return (SOFT[i][1] + (SOFT[i + 1][1] - SOFT[i][1]) * t,
                    SOFT[i][2] + (SOFT[i + 1][2] - SOFT[i][2]) * t)
    return SOFT[-1][1], SOFT[-1][2]


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


def seg_closest(p, a, b):
    """closest point on segment a-b for every row of p (N,3)."""
    ab = b - a
    ll = ab.dot(ab)
    if ll < 1e-10:
        return np.repeat(a[None, :], len(p), axis=0)
    t = np.clip(((p - a) @ ab) / ll, 0.0, 1.0)
    return a[None, :] + t[:, None] * ab[None, :]


class PinTable:
    def __init__(self, pins):
        part, target, rest, layer = [], [], [], []
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
        self.w = 1.0 / np.maximum(self.rest ** 2, 1e-6) if len(self.part) else np.zeros(0)

    def _targets(self, off):
        return self.target + off[None, :]

    def solve_seq(self, pos, off=None):
        o = np.zeros(3) if off is None else off
        for k in range(self.maxlayer):
            sel = self.layer == k
            idx = self.part[sel]
            p = pos[idx]
            t = self._targets(o)[sel]
            r = self.rest[sel]
            d = p - t
            L = np.linalg.norm(d, axis=1)
            L[L < 1e-5] = 1e-5
            pos[idx] = t + d * (r / L)[:, None]

    def solve_consensus(self, pos, off=None, alpha=1.0):
        o = np.zeros(3) if off is None else off
        d = pos[self.part] - self._targets(o)
        L = np.linalg.norm(d, axis=1)
        L[L < 1e-5] = 1e-5
        want = self._targets(o) + d * (self.rest / L)[:, None]

        acc = np.zeros_like(pos)
        wsum = np.zeros(len(pos))
        np.add.at(acc, self.part, want * self.w[:, None])
        np.add.at(wsum, self.part, self.w)
        hit = wsum > 1e-9
        if not hit.any():
            return
        pos[hit] += (acc[hit] / wsum[hit][:, None] - pos[hit]) * alpha


class Rig:
    pass


def build(path):
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
    pos0 = verts[p_idx].copy()
    ptype = np.array([types[v] for v in p_idx])
    stiff = np.zeros(m)
    ret = np.zeros(m)
    anch = np.zeros(m)
    for i, v in enumerate(p_idx):
        r, gg, bb = colors[v]
        s, rr = softness(gg)
        stiff[i] = max(s, 0.55) if ptype[i] == "cloth" else s
        ret[i] = rr
        anch[i] = max(0.0, min(1.0, 1.0 - bb / 255.0))

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
                d = float(np.linalg.norm(pos0[pid[a]] - verts[idx[b]]))
                if d > 1e-4:
                    pins[pid[a]].append((verts[idx[b]].copy(), d))

    e = np.array(sorted(edges), dtype=int)
    rest_len = np.linalg.norm(pos0[e[:, 1]] - pos0[e[:, 0]], axis=1)
    keep = rest_len >= 1e-4
    e, rest_len = e[keep], rest_len[keep]
    cstiff = np.clip(0.5 * (stiff[e[:, 0]] + stiff[e[:, 1]]), 0.05, 1.0)

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
        anchor_pts = [t for i in members for t, _d in pins[i]]
        centroid = pos0[members].mean(axis=0)
        radius = np.linalg.norm(pos0[members] - centroid, axis=1).max()
        for i in members:
            if anchor_pts:
                best = np.linalg.norm(np.array(anchor_pts) - pos0[i], axis=1).min()
            else:
                best = radius
            span[i] = best
            lead = 0.10 if ptype[i] == "hair" else 0.20
            cap = 4.0 if ptype[i] == "hair" else 10.0
            limit[i] = min(cap, max(lead, span[i] * 2.0 + lead))

    # ---- bone capsules at bind pose ----
    hanim = None
    for f in getattr(clump, "frames", []) or []:
        if getattr(f, "hanim", None):
            hanim = f.hanim
            break
    bone_id = [b.bone_id for b in hanim.bones]
    bone_pos = {}
    for i, mat in enumerate(g.skin.bone_matrices):
        if i >= len(bone_id):
            break
        M = np.array(mat, dtype=float).reshape(4, 4)
        try:
            bone_pos[bone_id[i]] = np.linalg.inv(M)[3, :3]
        except np.linalg.LinAlgError:
            pass

    caps = []
    for an, bn, r0 in CAPDEFS:
        a = bone_pos.get(BONE_IDS[an])
        b = bone_pos.get(BONE_IDS[bn])
        if a is None or b is None:
            continue
        caps.append((a.copy(), b.copy(), r0 * SCALE + MARGIN, f"{an}->{bn}"))

    rig = Rig()
    rig.pos0 = pos0
    rig.type = ptype
    rig.stiff = stiff
    rig.ret = ret
    rig.anchor = anch
    rig.edges = e
    rig.rest_len = rest_len
    rig.cstiff = cstiff
    rig.pins = pins
    rig.pintab = PinTable(pins)
    rig.span = span
    rig.limit = limit
    rig.hair = np.where(ptype == "hair")[0]
    rig.caps = caps
    rig.tri = [(t.a, t.b, t.c) for t in g.triangles]
    rig.v2p = v2p
    rig.verts = verts
    rig.dff = clump
    return rig


def capsule_ignore_mask(rig, depth_threshold=0.005):
    """A capsule that already contains the rest-pose vertex must not push it."""
    m = len(rig.pos0)
    mask = np.zeros((len(rig.caps), m), dtype=bool)
    for ci, (a, b, r, _name) in enumerate(rig.caps):
        closest = seg_closest(rig.pos0, a, b)
        d = np.linalg.norm(rig.pos0 - closest, axis=1)
        mask[ci] = d < (r - depth_threshold)
    return mask


def resolve_capsules(rig, pos, off, mask, margin_cloth=0.0):
    for ci, (a, b, r, _name) in enumerate(rig.caps):
        skip = mask[ci]
        closest = seg_closest(pos, a + off, b + off)
        d = pos - closest
        dist = np.linalg.norm(d, axis=1)
        rad = r + np.where(rig.type == "cloth", margin_cloth, 0.0)
        bad = (dist < rad) & (~skip)
        if not bad.any():
            continue
        dist[dist < 1e-5] = 1e-5
        pos[bad] = closest[bad] + d[bad] * (rad[bad] / dist[bad])[:, None]


def clamp_anchor(pos, anch, lim):
    d = anch - pos
    d2 = (d * d).sum(axis=1)
    bad = d2 > lim * lim
    if bad.any():
        pos[bad] = anch[bad] - d[bad] * (lim[bad] / np.sqrt(d2[bad]))[:, None]


def limit_stretch(rig, pos, movable, passes=2, immovable=None):
    for _ in range(passes):
        d = pos[rig.edges[:, 1]] - pos[rig.edges[:, 0]]
        L = np.linalg.norm(d, axis=1)
        lim = rig.rest_len * 1.35
        over = (L > lim) & (L > 1e-6)
        if not over.any():
            return
        ex = L[over] - lim[over]
        dirv = d[over] / L[over, None]
        wa = movable[rig.edges[over, 0]].astype(float)
        wb = movable[rig.edges[over, 1]].astype(float)
        if immovable is not None:
            wa[immovable[rig.edges[over, 0]]] = 0.0
            wb[immovable[rig.edges[over, 1]]] = 0.0
        ws = wa + wb
        ok = ws > 0
        if not ok.any():
            return
        i = np.where(over)[0][ok]
        adj = ex[ok] / ws[ok]
        np.add.at(pos, rig.edges[i, 0], dirv[ok] * (adj * wa[ok])[:, None])
        np.add.at(pos, rig.edges[i, 1], -dirv[ok] * (adj * wb[ok])[:, None])


def simulate(rig, frames, speed=0.0, stop_at=None, fix_transport=False,
             pin_consensus=False, use_capsules=False, capsule_fix=False,
             iters=12, damping=0.92, record=False):
    h = DT
    h2 = h * h
    maxstep = 8.0 * h
    movable = (rig.type != "body")

    mask = capsule_ignore_mask(rig) if capsule_fix else np.zeros(
        (len(rig.caps), len(rig.pos0)), dtype=bool)

    # particles whose root is glued to rigid geometry: do not stretch them off
    pinned = np.array([bool(len(p)) for p in rig.pins])

    pos = rig.pos0.copy()
    prev = pos.copy()
    anchor_prev = pos.copy()
    fvel = np.zeros_like(pos)

    track = []

    for f in range(frames):
        t = f * h
        # realistic ramp: reach the target speed over 0.5 s
        t_eff = t if (stop_at is None or t < stop_at) else stop_at
        ramp_eff = min(1.0, t_eff / 0.5) if speed > 0 else 0.0
        dist = speed * (t_eff * ramp_eff - 0.25 * ramp_eff * ramp_eff)
        off = np.array([dist, 0.0, 0.0])
        anc = rig.pos0 + off

        move = anc - anchor_prev
        anchor_prev = anc.copy()

        if fix_transport:
            pos += move
            prev += move
            avel = move / h
            acc = (avel - fvel) / h
            fvel += (avel - fvel) * 0.35
            alen = np.linalg.norm(acc, axis=1)
            bad = alen > 30.0
            if bad.any():
                acc[bad] *= (30.0 / alen[bad])[:, None]
        else:
            acc = np.zeros_like(pos)

        vel = (pos - prev) * damping
        vel[:, 2] -= G * h2
        vel -= acc * h2
        v2 = (vel * vel).sum(axis=1)
        big = v2 > maxstep ** 2
        if big.any():
            vel[big] *= (maxstep / np.sqrt(v2[big]))[:, None]

        prev = pos.copy()
        pos += vel
        clamp_anchor(pos, anc, rig.limit)

        for _ in range(iters):
            d = pos[rig.edges[:, 1]] - pos[rig.edges[:, 0]]
            L = np.linalg.norm(d, axis=1)
            L[L < 1e-6] = 1e-6
            diff = (L - rig.rest_len) / L
            w = rig.cstiff * 0.5
            np.add.at(pos, rig.edges[:, 0], d * (diff * w)[:, None])
            np.add.at(pos, rig.edges[:, 1], -d * (diff * w)[:, None])

            if pin_consensus:
                rig.pintab.solve_consensus(pos, off)
            else:
                rig.pintab.solve_seq(pos, off)

            pull = rig.anchor * rig.ret * rig.anchor
            mm = movable & (pull > 0)
            pos[mm] += (anc[mm] - pos[mm]) * pull[mm, None]

            if use_capsules:
                resolve_capsules(rig, pos, off, mask)

        limit_stretch(rig, pos, movable, 2, pinned if pin_consensus else None)
        if use_capsules:
            resolve_capsules(rig, pos, off, mask)
            resolve_capsules(rig, pos, off, mask)
            limit_stretch(rig, pos, movable, 1, pinned if pin_consensus else None)
            resolve_capsules(rig, pos, off, mask)
        clamp_anchor(pos, anc, rig.limit)

        if record:
            track.append(np.linalg.norm(pos[rig.hair] - anc[rig.hair], axis=1).mean())

    return pos, anc, np.array(track)


def stretch_stats(rig, pos):
    L = np.linalg.norm(pos[rig.edges[:, 1]] - pos[rig.edges[:, 0]], axis=1)
    r = L / np.maximum(rig.rest_len, 1e-6)
    return float(r.max()), float(np.percentile(r, 99)), float(r.mean())


def report(name, rig, pos, anc):
    hair = rig.hair
    d = np.linalg.norm(pos - rig.pos0, axis=1)
    lag = np.linalg.norm((pos - anc)[hair], axis=1)
    smax, s99, smean = stretch_stats(rig, pos)
    print(f"  {name}: |pos-anchor| mean={lag.mean()*1000:6.1f}mm "
          f"max={lag.max()*1000:6.1f}mm | edge stretch "
          f"mean={smean:.3f} p99={s99:.3f} max={smax:.3f} | "
          f"move-from-rest mean={d[hair].mean()*1000:6.1f}mm")
    for label, lo, hi in (("crown", 0.60, 0.80), ("back/side head", 0.50, 0.60),
                          ("shoulders", 0.35, 0.50)):
        sel = hair[(rig.pos0[hair, 2] >= lo) & (rig.pos0[hair, 2] < hi)]
        if len(sel) == 0:
            continue
        ds = d[sel]
        print(f"      {label:14s} n={len(sel):4d} mean={ds.mean()*1000:5.1f}mm "
              f"max={ds.max()*1000:5.1f}mm")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("dff")
    ap.add_argument("--png", default=None)
    ap.add_argument("--frames", type=int, default=180)
    args = ap.parse_args()

    rig = build(args.dff)
    print(f"particles={len(rig.pos0)} hair={len(rig.hair)} "
          f"edges={len(rig.edges)} pinned={sum(1 for p in rig.pins if p)} "
          f"capsules={len(rig.caps)}")
    m = capsule_ignore_mask(rig)
    print("capsules that already contain rest-pose hair (would push it out):")
    for ci, (a, b, r, name) in enumerate(rig.caps):
        cnt = int(m[ci][rig.hair].sum())
        if cnt:
            print(f"   {name:26s} r={r:.3f} hairInside={cnt:4d}")

    print("\n=== A) standing still (3 s) ===")
    p0, a0, _ = simulate(rig, 180)
    report("current solver              ", rig, p0, a0)

    p2, a2, _ = simulate(rig, 180, use_capsules=True)
    report("current (with capsules)     ", rig, p2, a2)

    p3, a3, _ = simulate(rig, 180, use_capsules=True, capsule_fix=True,
                         pin_consensus=True)
    report("fixed (mask + consensus)    ", rig, p3, a3)

    print("\n=== B) running at 6 m/s (3 s) ===")
    pb, ab, _ = simulate(rig, 180, speed=6.0, use_capsules=True)
    report("current (with capsules)     ", rig, pb, ab)

    pbf, abf, _ = simulate(rig, 180, speed=6.0, use_capsules=True,
                           capsule_fix=True, pin_consensus=True)
    report("fixed, no transport         ", rig, pbf, abf)

    pn, an, _ = simulate(rig, 180, speed=6.0, use_capsules=True,
                         capsule_fix=True, pin_consensus=True,
                         fix_transport=True)
    report("fixed + frame transport     ", rig, pn, an)

    print("\n=== C) stop after 3 s, settle 1 s ===")
    frames = 60 * 4
    ps_old, as_old, _ = simulate(rig, frames, speed=6.0, stop_at=3.0,
                                 use_capsules=True, capsule_fix=True,
                                 pin_consensus=True)
    report("fixed, no transport       ", rig, ps_old, as_old)
    ps_new, as_new, _ = simulate(rig, frames, speed=6.0, stop_at=3.0,
                                 use_capsules=True, capsule_fix=True,
                                 pin_consensus=True, fix_transport=True)
    report("fixed + frame transport   ", rig, ps_new, as_new)
    if args.png:
        render(rig, args.png, {"rest pose": rig.pos0,
                               "current solver": p2,
                               "fixed": p3})
        print(f"\nwrote {args.png}")


def render(rig, path, clouds):
    """Side view (Y-Z) of the hair particles + triangle wireframe, before/after."""
    from PIL import Image, ImageDraw

    W, H = 900, 700
    img = Image.new("RGB", (W, H), (18, 18, 22))
    dr = ImageDraw.Draw(img)

    panels = list(clouds.items())
    pw = W // len(panels)

    ymin, ymax = -0.35, 0.35
    zmin, zmax = 0.25, 0.85

    def proj(p, x0):
        u = (p[1] - ymin) / (ymax - ymin)
        v = (p[2] - zmin) / (zmax - zmin)
        return (x0 + u * pw, H - v * H * 0.95 - 0.02 * H)

    for k, (name, cloud) in enumerate(panels):
        x0 = k * pw
        for t in rig.tri:
            a, b, c = rig.v2p[t[0]], rig.v2p[t[1]], rig.v2p[t[2]]
            for i, j in ((a, b), (b, c), (c, a)):
                if i < 0 or j < 0:
                    continue
                dr.line([proj(cloud[i], x0), proj(cloud[j], x0)], fill=(52, 52, 60))
        for i in rig.hair:
            x, y = proj(cloud[i], x0)
            dr.ellipse([x - 1.1, y - 1.1, x + 1.1, y + 1.1], fill=(220, 90, 90))
        dr.text((x0 + 10, 10), name, fill=(230, 230, 230))

    img.save(path)


if __name__ == "__main__":
    main()
