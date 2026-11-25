"""Check how the 22 bone capsules overlap the hair at the bind pose.

Reproduces AddDefaultColliders' ped capsule list (radii after
bodycolliderscale / bodycollidermargin) and reports, for every capsule, how
many hair particles sit inside it and how far they would be pushed out by
ResolveCapsule.  This tells us whether the "back of the head bulge" can be a
collision artefact.

Usage: python tools/hair_capsule_check.py <dff>
"""

import importlib.util
import os
import sys
import types
from collections import Counter

import numpy as np

DFF_CORE = (r"C:\Users\Chipachipy\AppData\Roaming\Blender Foundation\Blender"
            r"\5.2\extensions\user_default\inu_tools_gta_sa\core")

SCALE = 1.0
MARGIN = 0.02


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


BONES = dict(
    PELVIS1=1, PELVIS=2, SPINE1=3, UPPERTORSO=4,
    NECK=5, HEAD=8,
    LEFTSHOULDER=32, LEFTELBOW=33, LEFTWRIST=34,
    RIGHTSHOULDER=22, RIGHTELBOW=23, RIGHTWRIST=24,
    LEFTHIP=41, LEFTKNEE=42, LEFTANKLE=43, LEFTFOOT=44,
    RIGHTHIP=51, RIGHTKNEE=52, RIGHTANKLE=53, RIGHTFOOT=54,
)

# (a, b, base radius) exactly like AddDefaultColliders
CAPDEFS = [
    ("PELVIS1", "PELVIS1", 0.13),
    ("PELVIS", "PELVIS", 0.13),
    ("PELVIS", "LEFTHIP", 0.12),
    ("PELVIS", "RIGHTHIP", 0.12),
    ("LEFTHIP", "LEFTKNEE", 0.11),
    ("RIGHTHIP", "RIGHTKNEE", 0.11),
    ("LEFTKNEE", "LEFTANKLE", 0.09),
    ("RIGHTKNEE", "RIGHTANKLE", 0.09),
    ("LEFTHIP", "LEFTHIP", 0.115),
    ("RIGHTHIP", "RIGHTHIP", 0.115),
    ("LEFTKNEE", "LEFTKNEE", 0.09),
    ("RIGHTKNEE", "RIGHTKNEE", 0.09),
    ("UPPERTORSO", "UPPERTORSO", 0.14),
    ("PELVIS", "SPINE1", 0.13),
    ("SPINE1", "UPPERTORSO", 0.13),
    ("NECK", "HEAD", 0.10),
    ("LEFTSHOULDER", "LEFTELBOW", 0.07),
    ("RIGHTSHOULDER", "RIGHTELBOW", 0.07),
    ("LEFTELBOW", "LEFTWRIST", 0.065),
    ("RIGHTELBOW", "RIGHTWRIST", 0.065),
    ("LEFTANKLE", "LEFTFOOT", 0.07),
    ("RIGHTANKLE", "RIGHTFOOT", 0.07),
]


def seg_dist(p, a, b):
    ab = b - a
    ll = ab.dot(ab)
    if ll < 1e-10:
        return np.linalg.norm(p - a, axis=-1)
    t = np.clip((p - a).dot(ab) / ll, 0.0, 1.0)
    return np.linalg.norm(p - (a + t[:, None] * ab), axis=-1)


def main(path):
    dff = load_dff_module()
    clump = dff.read_dff_file(path)
    g = max(clump.geometries, key=lambda gg: len(gg.vertices))
    n = len(g.vertices)
    verts = np.array([v[:3] for v in g.vertices], dtype=float)
    colors = [(c.r, c.g, c.b) if hasattr(c, "r") else tuple(c)
              for c in g.prelit_colors]
    types = [classify(c) for c in colors]
    hair = np.array([i for i in range(n) if types[i] == "hair"])

    print(f"verts={n} hair={len(hair)}")

    skin = g.skin
    print(f"skin bones={skin.num_bones} matrices={len(skin.bone_matrices)}")

    # bone order -> GTA bone id
    hanim = None
    for f in getattr(clump, "frames", []) or []:
        if getattr(f, "hanim", None):
            hanim = f.hanim
            break
    if hanim is None:
        print("no hanim data")
        return

    bone_id = [b.bone_id for b in hanim.bones]
    print(f"hanim bones={len(bone_id)} ids={bone_id}")

    pos = {}
    for i, m in enumerate(skin.bone_matrices):
        if i >= len(bone_id):
            break
        M = np.array(m, dtype=float).reshape(4, 4)
        try:
            inv = np.linalg.inv(M)
        except np.linalg.LinAlgError:
            continue
        pos[bone_id[i]] = inv[3, :3]

    print("bone bind positions (z ascending):")
    for bid, p in sorted(pos.items(), key=lambda kv: kv[1][2]):
        print(f"   bone {bid:3d} pos={np.round(p, 3)}")

    def P(name):
        bid = BONES[name]
        return pos.get(bid)

    print("\ncapsule penetration against hair (bind pose):")
    total = 0
    for an, bn, r0 in CAPDEFS:
        a, b = P(an), P(bn)
        if a is None or b is None:
            print(f"   {an}->{bn}: bone missing")
            continue
        r = r0 * SCALE + MARGIN
        d = seg_dist(verts[hair], a, b)
        inside = d < r
        cnt = int(inside.sum())
        if cnt == 0:
            continue
        pen = (r - d[inside])
        total += cnt
        print(f"   {an}->{bn} r={r:.3f}: hairInside={cnt:4d} "
              f"maxPush={pen.max()*1000:5.1f}mm meanPush={pen.mean()*1000:5.1f}mm")
    print(f"   (particles may overlap several capsules; hits={total})")


if __name__ == "__main__":
    main(sys.argv[1])
