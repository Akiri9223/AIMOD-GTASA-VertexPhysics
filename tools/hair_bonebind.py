"""Which skeleton bones do the HAIR vertices actually use?

!! 注意：这个脚本的结论**不可靠**，不要直接采信 !!
它的做法是：DFF 的逐顶点骨骼索引 -> 'bones_used' 重映射 -> HAnim 节点 ID。
但 tk7Chi2.dff 是 mobile/WDGL 变体的皮肤：
  * 逐顶点索引和 bones_used 的条目数对不上（不同解释会映射到 neck / L-elbow / 越界）；
  * Head(nodeId 8) 与 Neck(5) 的绑定位置只差 4mm，靠坐标也区分不了。
实测结论（2026-11-21）：本脚本曾断言"头发 100% 在 neck"，而 Blender 顶点组里
头发实际是 100% 在 Head —— 作者的数据为准。

保留这个脚本只作参考：它能可靠给出的是"骨骼绑定位置"（skinToBone 的逆矩阵平移）
和"每个顶点占主导权重的索引数值"，不能可靠给出"这个索引对应哪根骨骼"。
真实权重请从 Blender 里看（Weight Paint -> 选顶点组）。

Usage: python tools/hair_bonebind.py <dff>
"""

import importlib.util
import os
import sys
import types
from collections import Counter

import numpy as np

DFF_CORE = (r"C:\Users\Chipachipy\AppData\Roaming\Blender Foundation\Blender"
            r"\5.2\extensions\user_default\inu_tools_gta_sa\core")

BONE_NAMES = {0: "root", 1: "pelvis1", 2: "pelvis", 3: "spine1", 4: "uppertorso",
              5: "neck", 6: "head2", 7: "head1", 8: "head",
              21: "R upper torso", 22: "R shoulder", 23: "R elbow",
              24: "R wrist", 25: "R hand", 26: "R thumb",
              31: "L upper torso", 32: "L shoulder", 33: "L elbow",
              34: "L wrist", 35: "L hand", 36: "L thumb",
              41: "L hip", 42: "L knee", 43: "L ankle", 44: "L foot",
              51: "R hip", 52: "R knee", 53: "R ankle", 54: "R foot",
              201: "201", 301: "301", 302: "302"}


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


def main(path):
    dff = load_dff_module()
    clump = dff.read_dff_file(path)
    g = max(clump.geometries, key=lambda gg: len(gg.vertices))
    n = len(g.vertices)

    colors = [(c.r, c.g, c.b) if hasattr(c, "r") else tuple(c)
              for c in g.prelit_colors]
    types = [classify(c) for c in colors]

    skin = g.skin
    used = list(getattr(skin, "bones_used", []) or [])
    print(f"verts={n} bones={skin.num_bones} used={skin.num_used} "
          f"bones_used={used}")

    hanim = None
    for f in getattr(clump, "frames", []) or []:
        if getattr(f, "hanim", None):
            hanim = f.hanim
            break
    node_ids = [b.bone_id for b in hanim.bones]
    print(f"hanim node ids={node_ids}")

    def global_bone(local_idx):
        if not used:
            return local_idx
        if local_idx < len(used):
            return used[local_idx]
        return None

    for want in ("hair", "cloth"):
        hist = Counter()
        pairs = Counter()
        for v in range(n):
            if types[v] != want:
                continue
            idx = skin.bone_indices[v]
            w = skin.bone_weights[v]
            best = None
            for k in range(4):
                if w[k] <= 1e-6:
                    continue
                gb = global_bone(idx[k])
                if gb is None:
                    continue
                bid = node_ids[gb] if gb < len(node_ids) else -1
                pairs[(bid, round(w[k], 2))] += 1
                if best is None or w[k] > best[1]:
                    best = (bid, w[k])
            if best is not None:
                hist[best[0]] += 1

        print(f"\n{want}: dominant-bone histogram (bone id -> #vertices)")
        for bid, cnt in hist.most_common():
            print(f"   {bid:4d} {BONE_NAMES.get(bid, '?'):14s} {cnt}")

        print(f"   top weight pairs (bone, w):")
        for (bid, wv), cnt in pairs.most_common(10):
            print(f"     bone {bid:4d} w={wv:.2f} x{cnt}")


if __name__ == "__main__":
    main(sys.argv[1])
