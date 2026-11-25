"""最小 RenderWare DFF 解析器：只取 Geometry 的顶点/三角形/预光照色。

用于离线检查 VertexPhysics 的布料拓扑（哪些顶点是布料、哪些是刚体、
连通片有多大、挂点有多少）。
"""

import struct
import sys
from collections import Counter, defaultdict

SECTION_NAMES = {
    0x01: "Struct", 0x02: "String", 0x03: "Extension", 0x06: "Camera",
    0x07: "Texture", 0x08: "GeometryList", 0x09: "Material",
    0x0A: "MaterialList", 0x0B: "Atomic", 0x0C: "TextureNative",
    0x0D: "TextureDictionary", 0x0E: "FrameList", 0x0F: "Geometry",
    0x10: "Clump", 0x11: "AtomicSector", 0x12: "PlaneSector",
    0x13: "World", 0x14: "Sector", 0x15: "Frame",
    0x16: "Light", 0x18: "UnicodeString", 0x1A: "BinMeshPLG",
    0x1F: "CollisionPLG", 0x20: "FrameName", 0x253F2F1: "ExtraVertColour",
}


def sections(buf, start, end, out, depth=0):
    pos = start
    while pos + 12 <= end:
        typ, size, ver = struct.unpack_from("<III", buf, pos)
        if size < 12 or pos + size > end:
            break
        out.append((depth, pos, typ, size, ver))
        # 容器节点
        if typ in (0x10, 0x08, 0x0E, 0x0B, 0x0F, 0x07, 0x0C, 0x0D):
            sections(buf, pos + 12, pos + size, out, depth + 1)
        pos += size


def parse_geometry_struct(buf, off, size):
    """解析 Geometry 的 Struct（RW 3.7）。返回 dict。"""
    end = off + size
    p = off

    def u32():
        nonlocal p
        v = struct.unpack_from("<I", buf, p)[0]
        p += 4
        return v

    def f32():
        nonlocal p
        v = struct.unpack_from("<f", buf, p)[0]
        p += 4
        return v

    flags = u32()
    num_tris = u32()
    num_verts = u32()
    num_morph = u32()

    res = {
        "flags": flags, "num_tris": num_tris, "num_verts": num_verts,
        "num_morph": num_morph,
    }

    morphs = []
    for _ in range(num_morph):
        sphere = [f32() for _ in range(4)]
        has_pos = u32()
        has_nrm = u32()
        pos = [f32() for _ in range(num_verts * 3)] if has_pos else None
        nrm = [f32() for _ in range(num_verts * 3)] if has_nrm else None
        morphs.append((sphere, pos, nrm))

    res["morphs"] = morphs

    # 纹理坐标套数
    num_uv = u32()
    uvs = []
    for _ in range(num_uv):
        uvs.append([f32() for _ in range(num_verts * 2)])
    res["uvs"] = uvs

    # 预光照色：由 flags 的 PRELIT(0x40) 决定，但部分导出器不置位；
    # 这里用"剩余空间 - 三角形大小(sizeof(RwTriangle)=16)"反推
    tri_bytes = num_tris * 16
    remaining = end - p - tri_bytes

    prelit = None
    if remaining >= num_verts * 4:
        prelit = []
        for _ in range(num_verts):
            r, g, b, a = struct.unpack_from("<BBBB", buf, p)
            p += 4
            prelit.append((r, g, b, a))

    tris = []
    for _ in range(num_tris):
        v0 = struct.unpack_from("<H", buf, p)[0]
        v1 = struct.unpack_from("<H", buf, p + 2)[0]
        v2 = struct.unpack_from("<H", buf, p + 4)[0]
        p += 6
        mat = struct.unpack_from("<H", buf, p)[0]
        p += 2
        # 跳过 8 字节（RW 3.7 triangle 为 8 字节：3*uint16 + uint16 material）
        tris.append((v0, v1, v2, mat))

    res["prelit"] = prelit
    res["tris"] = tris
    res["tail"] = end - p
    return res


def parse(path):
    buf = open(path, "rb").read()
    secs = []
    sections(buf, 0, len(buf), secs)

    print(f"file={path} bytes={len(buf)} sections={len(secs)}")
    for depth, pos, typ, size, ver in secs:
        if depth <= 2:
            print(f"{'  ' * depth}{SECTION_NAMES.get(typ, hex(typ))} "
                  f"type=0x{typ:X} size={size} ver=0x{ver:08X} @{pos}")

    # 找 Geometry -> Struct
    for i, (depth, pos, typ, size, ver) in enumerate(secs):
        if typ != 0x0F:
            continue
        for d2, p2, t2, s2, v2 in secs[i + 1:]:
            if d2 <= depth:
                break
            if d2 == depth + 1 and t2 == 0x01:
                print(f"\nGeometry Struct @{p2} size={s2}")
                g = parse_geometry_struct(buf, p2 + 12, s2 - 12)
                print(f"  flags=0x{g['flags']:X} tris={g['num_tris']} "
                      f"verts={g['num_verts']} morphs={g['num_morph']} "
                      f"uvSets={len(g['uvs'])} prelit={'YES' if g['prelit'] else 'NO'} "
                      f"tail={g['tail']}")
                return g, secs
    raise SystemExit("no Geometry found")


if __name__ == "__main__":
    g, secs = parse(sys.argv[1] if len(sys.argv) > 1
                    else r"A:\Vanilla GTASA\modloader\新建文件夹\carlshou1_lae2.dff")

    prelit = g["prelit"]
    verts = g["morphs"][0][1]
    assert prelit is not None and verts is not None
    vs = [(verts[i * 3], verts[i * 3 + 1], verts[i * 3 + 2])
          for i in range(g["num_verts"])]

    cnt = Counter(prelit)
    print("\ncolor histogram (top 12):")
    for c, n in cnt.most_common(12):
        print(f"  {c} x{n}")

    # 物理类型（与 GeometryPhysics.cpp 的 ClassifyVertexColor 近似）
    def classify(c):
        r, gg, b, a = c
        if (r, gg, b) == (0, 0, 0):
            return "rigid"
        if r == 0:
            return "rigid"
        if r == 64:
            return "hair"
        if r == 128:
            return "cloth"
        if r == 192:
            return "body"
        if r == 254:
            return "soft"
        if r == 3:
            return "hair_legacy"
        return "rigid"

    types = [classify(c) for c in prelit]
    print("\nclassified:", Counter(types))
