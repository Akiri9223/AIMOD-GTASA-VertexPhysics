"""CuteMaid diagnosis: which bones drive the skirt, how thick is the leg mesh,
and how close the skirt sits to the body in bind pose.
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


def main(path):
    dff = load_dff_module()
    clump = dff.read_dff_file(path)
    frames = clump.frames
    g = clump.geometries[0]
    V = np.array(g.vertices, dtype=np.float64)
    colors = [(c.r, c.g, c.b) for c in g.prelit_colors]
    types = [classify(c) for c in colors]
    is_phys = np.array([t is not None for t in types])

    skin = g.skin
    used = list(skin.bones_used)
    idx = np.array(skin.bone_indices, dtype=np.int64)
    w = np.array(skin.bone_weights, dtype=np.float64)

    def frame_bone(k):
        f = frames[used[k]]
        return (f.hanim.bone_id if f.hanim else 0), f.name

    # merge duplicate positions like the plugin does (0.1mm quantisation)
    keys = np.round(V * 10000.0).astype(np.int64)
    _, first = np.unique(keys, axis=0, return_index=True)
    print(f"unique positions={len(first)} (plugin particles are merged this way)")

    phys_idx = np.where(is_phys)[0]
    print(f"physics verts={len(phys_idx)}")

    print("\n-- skull of the skirt: dominant bone of each physics vertex --")
    hist = Counter()
    for v in phys_idx:
        wi = np.argmax(w[v])
        bid, nm = frame_bone(idx[v][wi])
        hist[(bid, BONE_NAMES.get(bid, nm), round(float(w[v][wi]), 2))] += 1
    for (bid, nm, ww), n in hist.most_common():
        print(f"   bone {bid:3d} {nm:14s} w~{ww:.2f}  x{n}")

    noweight = sum(1 for v in phys_idx if w[v].max() <= 0.0001)
    print(f"physics verts without any weight: {noweight}/{len(phys_idx)}")

    # Which vertex positions do the weighted skirt verts sit at (z heights)?
    wz = V[phys_idx]
    print(f"\nskirt z range {wz[:,2].min():.3f} .. {wz[:,2].max():.3f}")

    # Leg mesh thickness via PCA on thigh-dominant verts.
    print("\n-- leg mesh thickness (PCA axis of thigh-dominant verts) --")
    for legname, hipid, kneeid in (("LEFT", 41, 42), ("RIGHT", 51, 52)):
        kthigh = None
        for k in range(len(used)):
            bid, nm = frame_bone(k)
            if bid == hipid:
                kthigh = k
        if kthigh is None:
            print(f"  {legname}: no thigh bone in skin")
            continue
        sel = np.where((~is_phys) & (idx[:, np.argmax(w, axis=1)] == kthigh))[0]
        P = V[sel]
        print(f"  {legname}: thigh-dominant rigid verts={len(P)}")
        if len(P) < 20:
            continue
        c = P.mean(0)
        u, s, vt = np.linalg.svd(P - c, full_matrices=False)
        axis = vt[0]
        t = (P - c) @ axis
        radial = (P - c) - np.outer(t, axis)
        r = np.linalg.norm(radial, axis=1)
        print(f"    axis dir={axis.round(3)} length_span={t.max()-t.min():.3f}m")
        print(f"    radius r50={np.percentile(r,50):.3f} r75={np.percentile(r,75):.3f} "
              f"r90={np.percentile(r,90):.3f} r95={np.percentile(r,95):.3f} "
              f"r99={np.percentile(r,99):.3f} rmax={r.max():.3f}")

    # distance from each skirt vertex to nearest rigid vertex (bind pose gap)
    print("\n-- skirt vertex -> nearest rigid (body) vertex --")
    rigid_idx = np.where(~is_phys)[0]
    R = V[rigid_idx]
    # brute force in chunks
    dmin = np.empty(len(phys_idx))
    for i, v in enumerate(phys_idx):
        d = np.sqrt(((R - V[v]) ** 2).sum(1))
        dmin[i] = d.min()
    print(f"  min={dmin.min():.3f} mean={dmin.mean():.3f} max={dmin.max():.3f}")
    print(f"  verts closer than 0.01m: {(dmin < 0.01).sum()}/{len(dmin)}")
    print(f"  verts closer than 0.02m: {(dmin < 0.02).sum()}/{len(dmin)}")

    # how many skirt verts are below the hip line / near the thighs
    print("\n-- skirt vertices by height percentile --")
    for q in (0, 20, 40, 60, 80, 100):
        print(f"   p{q}: z={np.percentile(wz[:,2], q):.3f}")


if __name__ == "__main__":
    target = sys.argv[1] if len(sys.argv) > 1 else r"A:\Vanilla GTASA\modloader\RZL-Skin\CuteMaid.dff"
    main(target)
