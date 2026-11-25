"""Measure a ped DFF: bind-pose bone positions (from the frame hierarchy),
mesh thickness along each limb, and where the cloth (skirt) sits relative to
the legs.  Used to fit the physics capsule radii to the *real* mesh instead
of guessing numbers like 0.10 for every thigh.
"""

import importlib.util
import os
import sys
import types
from collections import Counter

import numpy as np

DFF_CORE = (r"C:\Users\Chipachipy\AppData\Roaming\Blender Foundation\Blender"
            r"\5.2\extensions\user_default\inu_tools_gta_sa\core")

BONE_NAMES = {
    1: "PELVIS1", 2: "PELVIS", 3: "SPINE1", 4: "UPPERTORSO", 5: "NECK",
    6: "HEAD2", 7: "HEAD1", 8: "HEAD",
    21: "RIGHTUPPERTORSO", 22: "RIGHTSHOULDER", 23: "RIGHTELBOW",
    24: "RIGHTWRIST", 25: "RIGHTHAND",
    31: "LEFTUPPERTORSO", 32: "LEFTSHOULDER", 33: "LEFTELBOW",
    34: "LEFTWRIST", 35: "LEFTHAND",
    41: "LEFTHIP", 42: "LEFTKNEE", 43: "LEFTANKLE", 44: "LEFTFOOT",
    51: "RIGHTHIP", 52: "RIGHTKNEE", 53: "RIGHTANKLE", 54: "RIGHTFOOT",
}


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


def frame_matrix(f):
    """Row-vector convention: p' = p * R + T."""
    r = f.rotation
    m = np.eye(4)
    m[0, :3] = r[0:3]
    m[1, :3] = r[3:6]
    m[2, :3] = r[6:9]
    m[:3, 3] = f.position
    return m


def world_matrices(frames):
    cache = {}

    def get(i):
        if i in cache:
            return cache[i]
        f = frames[i]
        local = frame_matrix(f)
        if f.parent >= 0:
            world = local @ get(f.parent)
        else:
            world = local
        cache[i] = world
        return world

    for i in range(len(frames)):
        get(i)
    return cache


def seg_dist(points, a, b):
    ab = b - a
    ll = float(ab @ ab)
    if ll < 1e-12:
        return np.linalg.norm(points - a, axis=1)
    t = np.clip(((points - a) @ ab) / ll, 0.0, 1.0)
    closest = a[None, :] + t[:, None] * ab[None, :]
    return np.linalg.norm(points - closest, axis=1)


def main(path):
    dff = load_dff_module()
    clump = dff.read_dff_file(path)
    frames = clump.frames
    wm = world_matrices(frames)

    print(f"file={path}  frames={len(frames)} geoms={len(clump.geometries)}")

    print("\n-- bind-pose bone origins (model space) --")
    bone_pos = {}
    for i, f in enumerate(frames):
        bid = f.hanim.bone_id if f.hanim else 0
        if not bid:
            continue
        p = wm[i][:3, 3]
        bone_pos[bid] = p
        print(f"  {BONE_NAMES.get(bid, f.name):16s} id={bid:3d} "
              f"pos=({p[0]:7.3f},{p[1]:7.3f},{p[2]:7.3f})")

    g = clump.geometries[0]
    V = np.array(g.vertices, dtype=np.float64)
    colors = [(c.r, c.g, c.b) for c in g.prelit_colors]
    types = [classify(c) for c in colors]
    is_phys = np.array([t is not None for t in types])
    print(f"\nverts={len(V)} physics={int(is_phys.sum())} rigid={int((~is_phys).sum())}")
    print(f"bbox min={V.min(0).round(3)} max={V.max(0).round(3)}")

    skin = g.skin
    used = list(skin.bones_used)
    idx = np.array(skin.bone_indices, dtype=np.int64)
    w = np.array(skin.bone_weights, dtype=np.float64)
    dom = idx[np.arange(len(V)), np.argmax(w, axis=1)].astype(np.int64)

    # per-bone vertex sets (dominant weight only)
    bone_verts = {}
    for k, frame_idx in enumerate(used):
        bone_verts[k] = np.where(dom == k)[0]

    def bone_index(gta_id):
        for k, frame_idx in enumerate(used):
            f = frames[frame_idx]
            if f.hanim and f.hanim.bone_id == gta_id:
                return k
        return None

    pairs = [
        ("THIGH L", 41, 42, 0.10),
        ("THIGH R", 51, 52, 0.10),
        ("SHIN L", 42, 43, 0.085),
        ("SHIN R", 52, 53, 0.085),
        ("HIPSPAN", 2, 3, 0.13),
    ]

    print("\n-- limb mesh radius (distance of rigid verts to capsule axis) --")
    for name, ia, ib, cur in pairs:
        ka, kb = bone_index(ia), bone_index(ib)
        if ka is None or kb is None:
            print(f"  {name}: bone missing")
            continue
        a, b = bone_pos[ia], bone_pos[ib]
        sel = np.concatenate([bone_verts[ka], bone_verts[kb]])
        pts = V[sel]
        d = seg_dist(pts, a, b)
        seglen = float(np.linalg.norm(b - a))
        # reject verts far outside the limb (other body parts sharing a bone)
        keep = d < 0.35
        d = d[keep]
        q = np.percentile(d, [50, 75, 90, 95, 99]) if d.size else [0] * 5
        print(f"  {name:8s} seg={seglen:5.3f}m verts={d.size:5d} "
              f"r50={q[0]:.3f} r75={q[1]:.3f} r90={q[2]:.3f} r95={q[3]:.3f} "
              f"r99={q[4]:.3f} | current={cur:.3f}")

    # where is the cloth relative to the legs?
    C = V[is_phys]
    print(f"\n-- cloth vs legs (bind pose), cloth verts={len(C)} --")
    for name, ia, ib, cur in pairs[:4]:
        a, b = bone_pos[ia], bone_pos[ib]
        d = seg_dist(C, a, b)
        print(f"  {name:8s} min={d.min():.3f} mean={d.mean():.3f} max={d.max():.3f} "
              f"inside_r({cur:.3f})={int((d < cur).sum())}")

    print(f"\ncloth bbox min={C.min(0).round(3)} max={C.max(0).round(3)}")
    print(f"cloth z range={C[:,2].min():.3f}..{C[:,2].max():.3f}")

    # horizontal cross sections of the legs: radius as function of height
    print("\n-- thigh/shin radius by height (z) --")
    ka, kb = bone_index(51), bone_index(52)
    sel = np.concatenate([bone_verts[ka], bone_verts[kb]])
    pts = V[sel]
    a, b = bone_pos[51], bone_pos[52]
    d = seg_dist(pts, a, b)
    zs = pts[:, 2]
    lo, hi = min(a[2], b[2]), max(a[2], b[2])
    for i in range(10):
        z0 = lo + (hi - lo) * i / 10.0
        z1 = lo + (hi - lo) * (i + 1) / 10.0
        m = (zs >= z0) & (zs < z1) & (d < 0.35)
        if m.sum() < 4:
            continue
        print(f"   z={z0:6.3f}..{z1:6.3f} n={int(m.sum()):4d} "
              f"r50={np.percentile(d[m],50):.3f} r90={np.percentile(d[m],90):.3f} "
              f"rmax={d[m].max():.3f}")


if __name__ == "__main__":
    target = sys.argv[1] if len(sys.argv) > 1 else r"A:\Vanilla GTASA\modloader\RZL-Skin\CuteMaid.dff"
    main(target)
