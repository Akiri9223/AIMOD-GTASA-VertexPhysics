# -*- coding: utf-8 -*-
"""
VertexPhysics 顶点物理数据导出插件（Blender 3.2 ~ 5.x）

把 Blender 里画的“物理顶点色”导出成 .dat，供 GTA SA 的 VertexPhysics ASI 读取。
这样模型自己的顶点色可以保持原样（或不染色），物理信息不再占用顶点色。

顶点色编码（与 mod 一致）：
    R = 类型：0 Rigid / 64 Hair / 128 Cloth / 192 Body / 254 SoftBody
    G = 软度（0 硬 → 255 软）
    B = 锚定强度（0 固定 → 255 自由）

导出格式：
    # VertexPhysics data v1
    # model: test
    # count: 1155
    # index R G B x y z
    0 128 120 32 12.500000 -3.200000 1.800000

装不上时的最快用法：
    Blender → Scripting 工作区 → 新建 → 粘贴本文件全部内容 → Run Script
    然后 3D 视图按 N → 侧栏底部出现 “VertexPhysics” 标签页
"""



bl_info = {
    "name": "VertexPhysics Exporter",
    "author": "HairPhysicsASI",
    "version": (1, 6, 0),
    "blender": (3, 2, 0),
    "location": "View3D > Sidebar(N) > VertexPhysics",
    "description": "Vertex physics data (.dat) exporter / 顶点物理数据(.dat) 导出插件",
    "category": "Object",
}

import bpy
import os
import re

from mathutils import Vector, kdtree

from bpy.props import StringProperty, EnumProperty, BoolProperty


# ============================================================
#  Language / 语言
#  面板顶部的按钮可以在中文和英文之间切换。
#  The button at the top of the panel toggles Chinese / English.
# ============================================================
_LANG = {"value": "zh"}

_EN = {
    "导出顶点物理数据(.dat)，让 GTA SA 的 VertexPhysics 不使用顶点色": "Export vertex physics data (.dat) so GTA SA VertexPhysics does not use vertex colors",
    "模型名": "Model name",
    "与 IDE / CModelInfo 里的模型名一致（例如 test）": "Must match the model name in IDE / CModelInfo (e.g. test)",
    "输出目录": "Output directory",
    ".dat 输出目录（最终放到 <游戏目录>/vertexphysics）": ".dat output folder (finally copy to <game>/vertexphysics)",
    "坐标轴": "Axis",
    "位置坐标的轴转换（索引匹配为主，位置仅做校验）": "Axis conversion for positions (index match is primary, positions are only verified)",
    "原样导出": "As is",
    "不转换": "No conversion",
    "Z 不变，Y 取反": "Z unchanged, Y negated",
    "Y-up，Z 取反": "Y-up, Z negated",
    "应用物体变换": "Apply object transform",
    "导出时把 Object 变换烘进坐标": "Bake the object transform into coordinates on export",
    "输出到模型子目录": "Write into a model subfolder",
    "多部件模型（车辆）用：输出到 <输出目录>/<模型名>/ 下": "For multi-part models (vehicles): write to <output>/<model name>/",
    "目标物体名前缀": "Target object name prefix",
    "名字以此开头的物体一律当作“目标”（导入回来的 DFF）；": "Objects whose name starts with this are treated as targets (re-imported DFF);",
    "留空则按“是否有物理色”自动判断。带顶点色的模型建议用前缀，避免误判": "Leave empty to auto-detect by physics colors. Using a prefix avoids mistakes.",
    "dump 文件夹": "Dump folder",
    "游戏导出的几何体 dump 目录（vertexphysics/dump）": "Folder with geometry dumps exported by the game (vertexphysics/dump)",
    "文件名去掉 .001 后缀": "Strip .001 from file names",
    "把物体名末尾的 .001/.002 去掉再作为 dat 文件名": "Remove trailing .001/.002 from object names when naming .dat files",
    "按 part_01 编号命名": "Name as part_01, part_02 ...",
    "dat 文件名改为 part_01.dat、part_02.dat …（避免重名与奇怪后缀）": "Name .dat files part_01.dat, part_02.dat ... (avoids duplicates and odd suffixes)",
    "导出后清空顶点色": "Clear vertex colors after export",
    "导出成功后把顶点色设为白色。注意：清空后再导出会得到空数据，建议手动点按钮": "Set vertex colors to white after export. Note: exporting again after clearing yields empty data, use the button manually.",
    "导出 .dat": "Export .dat",
    "把当前物体的物理顶点色导出为 .dat": "Export the current object's physics vertex colors as .dat",
    "当前物体不是网格": "Active object is not a mesh",
    "该网格没有顶点色属性": "This mesh has no color attribute",
    "无法创建目录: %s": "Cannot create folder: %s",
    "写入失败: %s": "Write failed: %s",
    "顶点色全是白色，没有物理数据；请先给顶点上色（或用“快捷上色”）再导出": "All vertex colors are white: no physics data. Paint them first (or use Quick paint) and export again.",
    "已导出 %s（顶点 %d，物理顶点 %d）": "Exported %s (%d vertices, %d physics vertices)",
    "顶点色已清空为白色": "Vertex colors cleared to white",
    "导出全部选中物体": "Export all selected objects",
    "对每个选中的网格各导出一个 .dat（文件名取物体名）": "Export one .dat per selected mesh (file name = object name)",
    "已导出 %d 个 .dat 到 %s": "Exported %d .dat files to %s",
    "清空顶点色": "Clear vertex colors",
    "把当前物体的顶点色设为白色": "Set the current object's vertex colors to white",
    "请先选择一个网格物体": "Select a mesh object first",
    "按 DFF 顶点顺序导出（推荐）": "Export in DFF vertex order (recommended)",
    "选中两个网格：先选带物理色的原模型，再选【重新导出的 DFF 导入回来的物体】。": "Select two meshes: first the source model with physics colors, then the re-imported DFF object.",
    "插件会按位置把颜色复制到 DFF 的顶点顺序上，生成与游戏几何体顶点数完全一致的 .dat，": "Colors are copied by position onto the DFF vertex order, producing a .dat with exactly the game's vertex count,",
    "mod 端直接按索引读取，不需要猜坐标轴": "so the mod can read it by index without guessing the axis.",
    "请正好选中两个网格：原模型 + 导入回来的 DFF": "Select exactly two meshes: source model + re-imported DFF",
    "第一个物体没有顶点色": "The first object has no vertex colors",
    "已按 DFF 顺序导出 %s（DFF 顶点 %d，精确匹配 %d）": "Exported %s in DFF order (%d DFF vertices, %d exact matches)",
    "物体包围盒中心（世界空间）。": "Bounding-box center of the object (world space).",
    "是否含有非白色的顶点色（用来区分“源”和“目标”）。": "Whether it has non-white vertex colors (used to tell source from target).",
    "网格自身的局部包围盒（导入的 DFF 局部坐标 == 游戏几何体坐标）。": "Local bounding box of the mesh (imported DFF local coords == game geometry coords).",
    "把 source 的顶点色按 target（导入回来的 DFF）的顶点顺序写出 dat。": "Write a .dat using source colors in the target (re-imported DFF) vertex order.",
    "批量按 DFF 顺序导出": "Batch export in DFF order",
    "适用于车辆等多部件模型：选中所有物体（带物理色的源 + 导入回来的 DFF 目标），": "For multi-part models such as vehicles: select all objects (colored sources + re-imported DFF targets),",
    "插件按“是否有物理色”自动区分两类，再用包围盒中心就近配对，逐个导出到 <输出目录>/<模型名>/ 下": "the add-on splits them by physics colors, pairs them by nearest bounding-box center, and writes each part to <output>/<model name>/",
    "请至少选中两个网格（源 + 目标）": "Select at least two meshes (source + target)",
    "无法区分源/目标：请用“目标物体名前缀”，或确保一部分物体带物理色、另一部分不带": "Cannot tell source from target: set a target name prefix, or make sure only some objects have physics colors",
    "没有找到可配对的物体": "No pairable objects found",
    "批量导出完成：%d 个部件 -> %s（模型名 %s）": "Batch export done: %d parts -> %s (model %s)",
    "给选中物体加此前缀": "Add prefix to selected objects",
    "把「目标物体名前缀」加到所有选中网格的名字前面（已带前缀的跳过）": "Prepend the target prefix to all selected meshes (already-prefixed objects are skipped)",
    "请先在「目标物体名前缀」里填写前缀，例如 dff_": "Fill in the target prefix first, e.g. dff_",
    "已给 %d 个物体加上前缀 %s": "Added prefix %s to %d objects",
    "从游戏 dump 生成 dat（精确）": "Build .dat from game dump (exact)",
    "选择游戏导出的 dump（vertexphysics/dump/*.txt），": "Pick a dump exported by the game (vertexphysics/dump/*.txt),",
    "用当前带物理色的物体按位置取色，生成顶点数与游戏几何体完全一致的 dat": "sample the active object's colors by position and write a .dat with exactly the game's vertex count",
    "请选择有效的 dump 文件（.txt）": "Choose a valid dump file (.txt)",
    "请先选中带物理色的源网格物体": "Select the source mesh with physics colors first",
    "该物体没有顶点色": "This object has no vertex colors",
    "dump 文件里没有顶点数据": "No vertex data in the dump file",
    "已生成 %s（顶点 %d，匹配 %d，轴 %s）": "Generated %s (%d vertices, %d matched, axis %s)",
    "读取 dump，返回 (points, declared_count)。": "Read a dump file and return (points, declared_count).",
    "返回 (名称, 变换函数, 平均距离)。": "Return (name, transform function, average distance).",
    "从 dump 文件夹批量生成": "Batch build from a dump folder",
    "选中多个带物理色的部件物体，指定 dump 文件夹；": "Select several colored parts and pick the dump folder;",
    "每个 dump 自动匹配最合适的源物体，逐对生成 dat": "each dump is matched to the best source object and a .dat is generated per pair",
    "dump 文件夹不存在：%s": "Dump folder does not exist: %s",
    "请先选中带物理色的部件物体": "Select colored part objects first",
    "dump 文件夹里没有 .txt 文件": "No .txt files in the dump folder",
    "[VertexPhysics] 跳过 %s（最近距离 %.4f 太大）": "[VertexPhysics] skip %s (nearest distance %.4f too large)",
    "已生成 %d 个 dat -> %s（用到 %d 个源物体）": "Generated %d .dat files -> %s (used %d source objects)",
    "给选中顶点刷类型色": "Paint type color on selected vertices",
    "把选中顶点刷成指定物理类型": "Paint selected vertices with the chosen physics type",
    "类型": "Type",
    "没有选中顶点（编辑模式里选）": "No vertices selected (select them in Edit Mode)",
    "已给 %d 个顶点上色": "Painted %d vertices",
    "VertexPhysics 顶点物理": "VertexPhysics",
    "导出物理数据": "Export physics data",
    "快捷上色（可选）": "Quick paint (optional)",
    "界面语言已切换": "UI language switched",
}


def TD(s):
    """提示气泡用：始终返回「中文 / English」，中英模式下都能看懂。"""
    en = _EN.get(s)
    if not en:
        return s
    return s + " / " + en

def T(s):
    """中文模式下返回原文；英文模式下查表翻译，查不到就返回原文。"""
    if _LANG["value"] == "en":
        return _EN.get(s, s)
    return s


class VERTEXPHYSICS_OT_toggle_lang(bpy.types.Operator):
    bl_idname = "vertexphysics.toggle_lang"
    bl_label = "Language / 语言"
    bl_description = "Switch UI language (Chinese / English) / 在中文和英文之间切换"

    def execute(self, context):
        _LANG["value"] = "en" if _LANG["value"] == "zh" else "zh"
        self.report({"INFO"}, "UI language: " + _LANG["value"] + " / 界面语言已切换")
        return {"FINISHED"}


PHYS_TYPES = {
    "Rigid": 0,
    "Hair": 64,
    "Cloth": 128,
    "Body": 192,
    "SoftBody": 254,
}


# ------------------------------------------------------------------
# 顶点色读取（兼容 3.x / 4.x / 5.x 的属性系统）
# ------------------------------------------------------------------

def get_color_attribute(mesh):
    attrs = []

    if hasattr(mesh, "color_attributes"):
        for a in mesh.color_attributes:
            attrs.append(a)

    if attrs:
        for a in attrs:
            if getattr(a, "domain", "POINT") == 'POINT':
                return a
        return attrs[0]

    # 老版本（3.2 以前）的回退
    legacy = getattr(mesh, "vertex_colors", None)
    if legacy and len(legacy) > 0:
        return legacy[0]

    return None


def linear_to_srgb_byte(v):
    """Blender 的 color attribute 是线性值；用户画的是 sRGB 字节值。
    导出时必须换算回去，否则 64 会变成 13、128 会变成 55。"""
    v = max(0.0, min(1.0, v))

    if v <= 0.0031308:
        s = v * 12.92
    else:
        s = 1.055 * (v ** (1.0 / 2.4)) - 0.055

    return int(round(max(0.0, min(1.0, s)) * 255.0))


def read_vertex_colors(mesh):
    result = {}
    attr = get_color_attribute(mesh)

    if attr is None:
        return result

    # 老式 loop 颜色层
    if not hasattr(attr, "domain"):
        for loop in mesh.loops:
            vi = loop.vertex_index
            if vi in result:
                continue
            c = attr.data[loop.index].color
            result[vi] = (linear_to_srgb_byte(c[0]),
                          linear_to_srgb_byte(c[1]),
                          linear_to_srgb_byte(c[2]))
        return result

    if attr.domain == 'POINT':
        for i, d in enumerate(attr.data):
            c = d.color
            result[i] = (linear_to_srgb_byte(c[0]),
                         linear_to_srgb_byte(c[1]),
                         linear_to_srgb_byte(c[2]))
    else:
        for loop in mesh.loops:
            vi = loop.vertex_index
            if vi in result:
                continue
            c = attr.data[loop.index].color
            result[vi] = (linear_to_srgb_byte(c[0]),
                          linear_to_srgb_byte(c[1]),
                          linear_to_srgb_byte(c[2]))

    return result


def convert_position(co, mode):
    x, y, z = co

    if mode == 'ZUP_TO_YUP':
        return (x, z, -y)
    if mode == 'ZUP_TO_ZUP_FLIPY':
        return (x, -y, z)
    if mode == 'ZUP_TO_YUP_FLIPZ':
        return (x, z, y)

    return (x, y, z)


def collect_object_data(obj, apply_transform, axis_mode):
    if obj is None or obj.type != 'MESH':
        return None

    mesh = obj.data
    colors = read_vertex_colors(mesh)

    coords = []

    for v in mesh.vertices:
        co = v.co

        if apply_transform:
            w = obj.matrix_world @ co
            co = (w.x, w.y, w.z)
        else:
            co = (co.x, co.y, co.z)

        coords.append(convert_position(co, axis_mode))

    return {
        "mesh": mesh,
        "colors": colors,
        "coords": coords,
        "count": len(mesh.vertices),
    }


def write_dat(path, model_name, data):
    colors = data["colors"]
    coords = data["coords"]
    count = data["count"]

    lines = [
        "# VertexPhysics data v1",
        "# model: %s" % model_name,
        "# count: %d" % count,
        "# index R G B x y z",
    ]

    # 包围盒指纹（用于顶点数相同的多个部件之间区分）
    if count > 0:
        mn = [1e18, 1e18, 1e18]
        mx = [-1e18, -1e18, -1e18]

        for (x, y, z) in coords:
            for i, c in enumerate((x, y, z)):
                if c < mn[i]:
                    mn[i] = c
                if c > mx[i]:
                    mx[i] = c

        lines.insert(3, "# bbox: %.6f %.6f %.6f %.6f %.6f %.6f" % (
            mn[0], mn[1], mn[2], mx[0], mx[1], mx[2]))

    for i in range(count):
        r, g, b = colors.get(i, (255, 255, 255))
        x, y, z = coords[i]
        lines.append("%d %d %d %d %.6f %.6f %.6f" % (i, r, g, b, x, y, z))

    with open(path, "w", encoding="utf-8", newline="\n") as f:
        f.write("\n".join(lines))
        f.write("\n")


def neutralize_colors(obj):
    mesh = obj.data
    attr = get_color_attribute(mesh)

    if attr is None:
        return False

    for d in attr.data:
        try:
            d.color = (1.0, 1.0, 1.0, 1.0)
        except Exception:
            d.color = (1.0, 1.0, 1.0)

    mesh.update()
    return True


# ------------------------------------------------------------------
# 属性
# ------------------------------------------------------------------

class VertexPhysicsProperties(bpy.types.PropertyGroup):
    model_name: StringProperty(
        name=T("模型名"),
        description=TD("与 IDE / CModelInfo 里的模型名一致（例如 test）"),
        default="test",
    )

    output_dir: StringProperty(
        name=T("输出目录"),
        description=TD(".dat 输出目录（最终放到 <游戏目录>/vertexphysics）"),
        default="//vertexphysics",
        subtype='DIR_PATH',
    )

    axis_mode: EnumProperty(
        name=T("坐标轴"),
        description=TD("位置坐标的轴转换（索引匹配为主，位置仅做校验）"),
        items=[
            ('NONE', T("原样导出"), T("不转换")),
            ('ZUP_TO_YUP', "Z-up → Y-up", "(x, z, -y)"),
            ('ZUP_TO_ZUP_FLIPY', T("Z 不变，Y 取反"), "(x, -y, z)"),
            ('ZUP_TO_YUP_FLIPZ', T("Y-up，Z 取反"), "(x, z, y)"),
        ],
        default='NONE',
    )

    apply_transform: BoolProperty(
        name=T("应用物体变换"),
        description=TD("导出时把 Object 变换烘进坐标"),
        default=False,
    )

    use_subdir: BoolProperty(
        name=T("输出到模型子目录"),
        description=TD("多部件模型（车辆）用：输出到 <输出目录>/<模型名>/ 下"),
        default=False,
    )

    target_prefix: StringProperty(
        name=T("目标物体名前缀"),
        description=(
            TD("名字以此开头的物体一律当作“目标”（导入回来的 DFF）；"
            "留空则按“是否有物理色”自动判断。带顶点色的模型建议用前缀，避免误判")),
        default="",
    )

    dump_dir: StringProperty(
        name=T("dump 文件夹"),
        description=TD("游戏导出的几何体 dump 目录（vertexphysics/dump）"),
        default="//vertexphysics/dump",
        subtype='DIR_PATH',
    )

    strip_name_suffix: BoolProperty(
        name=T("文件名去掉 .001 后缀"),
        description=TD("把物体名末尾的 .001/.002 去掉再作为 dat 文件名"),
        default=True,
    )

    numbered_names: BoolProperty(
        name=T("按 part_01 编号命名"),
        description=TD("dat 文件名改为 part_01.dat、part_02.dat …（避免重名与奇怪后缀）"),
        default=False,
    )

    neutralize: BoolProperty(
        name=T("导出后清空顶点色"),
        description=TD("导出成功后把顶点色设为白色。注意：清空后再导出会得到空数据，建议手动点按钮"),
        default=False,
    )


# ------------------------------------------------------------------
# 操作
# ------------------------------------------------------------------

class VERTEXPHYSICS_OT_export(bpy.types.Operator):
    bl_idname = "vertexphysics.export_dat"
    bl_label = T("导出 .dat")
    bl_description = TD("把当前物体的物理顶点色导出为 .dat")

    def execute(self, context):
        props = context.scene.vertexphysics
        obj = context.object

        data = collect_object_data(obj, props.apply_transform, props.axis_mode)

        if data is None:
            self.report({'ERROR'}, T("当前物体不是网格"))
            return {'CANCELLED'}

        if not data["colors"]:
            self.report({'ERROR'}, T("该网格没有顶点色属性"))
            return {'CANCELLED'}

        out_dir = bpy.path.abspath(props.output_dir) or os.path.dirname(bpy.data.filepath)

        try:
            os.makedirs(out_dir, exist_ok=True)
        except Exception as e:
            self.report({'ERROR'}, T("无法创建目录: %s") % e)
            return {'CANCELLED'}

        name = (props.model_name or "").strip() or obj.name
        path = os.path.join(out_dir, name + ".dat")

        try:
            write_dat(path, name, data)
        except Exception as e:
            self.report({'ERROR'}, T("写入失败: %s") % e)
            return {'CANCELLED'}

        marked = 0
        for rgb in data["colors"].values():
            if not (rgb[0] == rgb[1] == rgb[2]):
                marked += 1

        if marked == 0:
            try:
                os.remove(path)
            except Exception:
                pass

            self.report(
                {'ERROR'},
                T("顶点色全是白色，没有物理数据；请先给顶点上色（或用“快捷上色”）再导出"))
            return {'CANCELLED'}

        self.report({'INFO'}, T("已导出 %s（顶点 %d，物理顶点 %d）") % (path, data["count"], marked))

        if props.neutralize:
            neutralize_colors(obj)
            self.report({'INFO'}, T("顶点色已清空为白色"))

        return {'FINISHED'}


class VERTEXPHYSICS_OT_export_all(bpy.types.Operator):
    bl_idname = "vertexphysics.export_dat_all"
    bl_label = T("导出全部选中物体")
    bl_description = TD("对每个选中的网格各导出一个 .dat（文件名取物体名）")

    def execute(self, context):
        props = context.scene.vertexphysics
        out_dir = bpy.path.abspath(props.output_dir) or os.path.dirname(bpy.data.filepath)

        os.makedirs(out_dir, exist_ok=True)

        exported = 0

        for obj in context.selected_objects:
            if obj.type != 'MESH':
                continue

            data = collect_object_data(obj, props.apply_transform, props.axis_mode)

            if data is None or not data["colors"]:
                continue

            write_dat(os.path.join(out_dir, obj.name + ".dat"), obj.name, data)
            exported += 1

            if props.neutralize:
                neutralize_colors(obj)

        self.report({'INFO'}, T("已导出 %d 个 .dat 到 %s") % (exported, out_dir))
        return {'FINISHED'}


class VERTEXPHYSICS_OT_neutralize(bpy.types.Operator):
    bl_idname = "vertexphysics.neutralize_colors"
    bl_label = T("清空顶点色")
    bl_description = TD("把当前物体的顶点色设为白色")

    def execute(self, context):
        obj = context.object

        if obj is None or obj.type != 'MESH':
            self.report({'ERROR'}, T("请先选择一个网格物体"))
            return {'CANCELLED'}

        neutralize_colors(obj)
        return {'FINISHED'}


class VERTEXPHYSICS_OT_bake_dff_order(bpy.types.Operator):
    bl_idname = "vertexphysics.bake_dff_order"
    bl_label = T("按 DFF 顶点顺序导出（推荐）")
    bl_description = (
        TD("选中两个网格：先选带物理色的原模型，再选【重新导出的 DFF 导入回来的物体】。"
          "插件会按位置把颜色复制到 DFF 的顶点顺序上，生成与游戏几何体顶点数完全一致的 .dat，"
          "mod 端直接按索引读取，不需要猜坐标轴"))
    def execute(self, context):
        props = context.scene.vertexphysics
        selected = [o for o in context.selected_objects if o.type == 'MESH']

        if len(selected) != 2:
            self.report({'ERROR'}, T("请正好选中两个网格：原模型 + 导入回来的 DFF"))
            return {'CANCELLED'}

        source, target = selected[0], selected[1]

        src_mesh = source.data
        tgt_mesh = target.data

        src_colors = read_vertex_colors(src_mesh)

        if not src_colors:
            self.report({'ERROR'}, T("第一个物体没有顶点色"))
            return {'CANCELLED'}

        # 两边都换算到世界空间，位置才对得上
        src_co = [source.matrix_world @ v.co for v in src_mesh.vertices]

        lines = [
            "# VertexPhysics data v1",
            "# model: %s" % (props.model_name or target.name),
            "# count: %d" % len(tgt_mesh.vertices),
            "# index R G B x y z",
        ]

        matched = 0

        for tv in tgt_mesh.vertices:
            pos = target.matrix_world @ tv.co

            best = 1e30
            best_i = -1

            for i, sc in enumerate(src_co):
                d = (sc - pos).length_squared
                if d < best:
                    best = d
                    best_i = i

            if best_i >= 0 and best <= 1e-6:
                rgb = src_colors.get(best_i, (255, 255, 255))
                matched += 1
            elif best_i >= 0:
                # 位置有微小偏差时仍取最近点（同一拓扑，最近点就是对应点）
                rgb = src_colors.get(best_i, (255, 255, 255))
            else:
                rgb = (255, 255, 255)

            lines.append("%d %d %d %d %.6f %.6f %.6f" % (
                tv.index, rgb[0], rgb[1], rgb[2],
                tv.co.x, tv.co.y, tv.co.z))

        out_dir = bpy.path.abspath(props.output_dir) or os.path.dirname(bpy.data.filepath)
        os.makedirs(out_dir, exist_ok=True)

        name = (props.model_name or "").strip() or target.name
        path = os.path.join(out_dir, name + ".dat")

        with open(path, "w", encoding="utf-8", newline="\n") as f:
            f.write("\n".join(lines))
            f.write("\n")

        self.report({'INFO'},
            T("已按 DFF 顺序导出 %s（DFF 顶点 %d，精确匹配 %d）")
            % (path, len(tgt_mesh.vertices), matched))

        return {'FINISHED'}


def object_center_world(obj):
    """物体包围盒中心（世界空间）。"""
    corners = [obj.matrix_world @ v.co for v in obj.data.vertices]

    if not corners:
        return obj.matrix_world.translation

    mn = Vector((1e18, 1e18, 1e18))
    mx = Vector((-1e18, -1e18, -1e18))

    for c in corners:
        mn.x = min(mn.x, c.x); mn.y = min(mn.y, c.y); mn.z = min(mn.z, c.z)
        mx.x = max(mx.x, c.x); mx.y = max(mx.y, c.y); mx.z = max(mx.z, c.z)

    return (mn + mx) * 0.5


def has_physics_colors(obj):
    """是否含有非白色的顶点色（用来区分“源”和“目标”）。"""
    colors = read_vertex_colors(obj.data)

    for rgb in colors.values():
        if not (rgb[0] == rgb[1] == rgb[2]):
            return True

    return False


def mesh_bbox_local(mesh):
    """网格自身的局部包围盒（导入的 DFF 局部坐标 == 游戏几何体坐标）。"""
    if not mesh.vertices:
        return None

    mn = [1e18, 1e18, 1e18]
    mx = [-1e18, -1e18, -1e18]

    for v in mesh.vertices:
        co = v.co
        for i, c in enumerate((co.x, co.y, co.z)):
            if c < mn[i]:
                mn[i] = c
            if c > mx[i]:
                mx[i] = c

    return mn, mx


def bbox_header(mesh):
    box = mesh_bbox_local(mesh)

    if not box:
        return None

    mn, mx = box

    return "# bbox: %.6f %.6f %.6f %.6f %.6f %.6f" % (
        mn[0], mn[1], mn[2], mx[0], mx[1], mx[2])


def write_dff_order_dat(source, target, path, model_name):
    """把 source 的顶点色按 target（导入回来的 DFF）的顶点顺序写出 dat。"""
    src_mesh = source.data
    tgt_mesh = target.data

    src_colors = read_vertex_colors(src_mesh)
    src_co = [source.matrix_world @ v.co for v in src_mesh.vertices]

    lines = [
        "# VertexPhysics data v1",
        "# model: %s" % model_name,
        "# count: %d" % len(tgt_mesh.vertices),
    ]

    box = bbox_header(tgt_mesh)

    if box:
        lines.append(box)

    lines.append("# index R G B x y z")

    matched = 0

    for tv in tgt_mesh.vertices:
        pos = target.matrix_world @ tv.co

        best = 1e30
        best_i = -1

        for i, sc in enumerate(src_co):
            d = (sc - pos).length_squared
            if d < best:
                best = d
                best_i = i

        if best_i >= 0:
            rgb = src_colors.get(best_i, (255, 255, 255))
            if best <= 1e-6:
                matched += 1
        else:
            rgb = (255, 255, 255)

        lines.append("%d %d %d %d %.6f %.6f %.6f" % (
            tv.index, rgb[0], rgb[1], rgb[2],
            tv.co.x, tv.co.y, tv.co.z))

    with open(path, "w", encoding="utf-8", newline="\n") as f:
        f.write("\n".join(lines))
        f.write("\n")

    return matched, len(tgt_mesh.vertices)


class VERTEXPHYSICS_OT_bake_batch(bpy.types.Operator):
    bl_idname = "vertexphysics.bake_dff_order_batch"
    bl_label = T("批量按 DFF 顺序导出")
    bl_description = (
        TD("适用于车辆等多部件模型：选中所有物体（带物理色的源 + 导入回来的 DFF 目标），"
        "插件按“是否有物理色”自动区分两类，再用包围盒中心就近配对，逐个导出到 <输出目录>/<模型名>/ 下"))

    def execute(self, context):
        props = context.scene.vertexphysics
        meshes = [o for o in context.selected_objects if o.type == 'MESH']

        if len(meshes) < 2:
            self.report({'ERROR'}, T("请至少选中两个网格（源 + 目标）"))
            return {'CANCELLED'}

        prefix = (props.target_prefix or "").strip()

        if prefix:
            targets = [o for o in meshes if o.name.startswith(prefix)]
            sources = [o for o in meshes if not o.name.startswith(prefix)]
        else:
            sources = [o for o in meshes if has_physics_colors(o)]
            targets = [o for o in meshes if o not in sources]

        if not sources or not targets:
            self.report({'ERROR'},
                T("无法区分源/目标：请用“目标物体名前缀”，或确保一部分物体带物理色、另一部分不带"))
            return {'CANCELLED'}

        centers = {o.name: object_center_world(o) for o in sources}

        used = set()
        pairs = []

        for t in targets:
            tc = object_center_world(t)

            best_name = None
            best_dist = 1e30

            for s in sources:
                if s.name in used:
                    continue

                d = (centers[s.name] - tc).length_squared

                if d < best_dist:
                    best_dist = d
                    best_name = s.name

            if best_name:
                used.add(best_name)
                pairs.append((next(s for s in sources if s.name == best_name), t))

        if not pairs:
            self.report({'ERROR'}, T("没有找到可配对的物体"))
            return {'CANCELLED'}

        model = (props.model_name or "").strip() or targets[0].name
        base = bpy.path.abspath(props.output_dir) or os.path.dirname(bpy.data.filepath)
        out_dir = os.path.join(base, model) if props.use_subdir else base

        os.makedirs(out_dir, exist_ok=True)

        done = 0

        for idx, (src, tgt) in enumerate(pairs):
            if props.numbered_names:
                fname = "part_%02d.dat" % (idx + 1)
            else:
                base = tgt.name

                if props.strip_name_suffix:
                    base = re.sub(r"\.\d{3,}$", "", base)

                fname = base + ".dat"

            path = os.path.join(out_dir, fname)

            matched, total = write_dff_order_dat(src, tgt, path, model)
            done += 1

            print("[VertexPhysics] %s <- %s  (%d/%d)" % (fname, src.name, matched, total))

        self.report({'INFO'},
            T("批量导出完成：%d 个部件 -> %s（模型名 %s）") % (done, out_dir, model))

        return {'FINISHED'}


class VERTEXPHYSICS_OT_add_prefix(bpy.types.Operator):
    bl_idname = "vertexphysics.add_target_prefix"
    bl_label = T("给选中物体加此前缀")
    bl_description = TD("把「目标物体名前缀」加到所有选中网格的名字前面（已带前缀的跳过）")

    def execute(self, context):
        props = context.scene.vertexphysics
        prefix = (props.target_prefix or "").strip()

        if not prefix:
            self.report({'ERROR'}, T("请先在「目标物体名前缀」里填写前缀，例如 dff_"))
            return {'CANCELLED'}

        count = 0

        for o in context.selected_objects:
            if o.type != 'MESH':
                continue

            if o.name.startswith(prefix):
                continue

            o.name = prefix + o.name
            count += 1

        self.report({'INFO'}, T("已给 %d 个物体加上前缀 %s") % (count, prefix))
        return {'FINISHED'}


class VERTEXPHYSICS_OT_dat_from_dump(bpy.types.Operator):
    bl_idname = "vertexphysics.dat_from_dump"
    bl_label = T("从游戏 dump 生成 dat（精确）")
    bl_description = (
        TD("选择游戏导出的 dump（vertexphysics/dump/*.txt），"
        "用当前带物理色的物体按位置取色，生成顶点数与游戏几何体完全一致的 dat"))

    filepath: StringProperty(subtype='FILE_PATH')
    filter_glob: StringProperty(default="*.txt", options={'HIDDEN'}, maxlen=255)

    def invoke(self, context, event):
        context.window_manager.fileselect_add(self)
        return {'RUNNING_MODAL'}

    def execute(self, context):
        props = context.scene.vertexphysics
        path = bpy.path.abspath(self.filepath)

        if not path or not os.path.isfile(path):
            self.report({'ERROR'}, T("请选择有效的 dump 文件（.txt）"))
            return {'CANCELLED'}

        obj = context.object

        if obj is None or obj.type != 'MESH':
            self.report({'ERROR'}, T("请先选中带物理色的源网格物体"))
            return {'CANCELLED'}

        colors = read_vertex_colors(obj.data)

        if not colors:
            self.report({'ERROR'}, T("该物体没有顶点色"))
            return {'CANCELLED'}

        # ---- 读取 dump ----
        dump_points = []
        declared_count = 0

        with open(path, "r", encoding="utf-8", errors="ignore") as f:
            for line in f:
                s = line.strip()

                if not s:
                    continue

                if s.startswith("#"):
                    if "verts:" in s:
                        try:
                            declared_count = int(s.split(":")[1].strip())
                        except Exception:
                            pass
                    continue

                parts = s.split()
                if len(parts) < 4:
                    continue

                try:
                    dump_points.append((float(parts[1]), float(parts[2]), float(parts[3])))
                except Exception:
                    continue

        if not dump_points:
            self.report({'ERROR'}, T("dump 文件里没有顶点数据"))
            return {'CANCELLED'}

        # ---- 源网格坐标（世界空间）----
        src_co = [obj.matrix_world @ v.co for v in obj.data.vertices]

        # ---- 自动选一个轴变换（dump 坐标 → Blender 坐标）----
        transforms = [
            ("xyz", lambda p: (p[0], p[1], p[2])),
            ("x,z,-y", lambda p: (p[0], p[2], -p[1])),
            ("x,-y,z", lambda p: (p[0], -p[1], p[2])),
            ("x,z,y", lambda p: (p[0], p[2], p[1])),
            ("x,-z,y", lambda p: (p[0], -p[2], p[1])),
            ("x,y,-z", lambda p: (p[0], p[1], -p[2])),
            ("-x,y,z", lambda p: (-p[0], p[1], p[2])),
            ("z,y,x", lambda p: (p[2], p[1], p[0])),
            ("-y,x,z", lambda p: (-p[1], p[0], p[2])),
            ("y,x,z", lambda p: (p[1], p[0], p[2])),
        ]

        kd = kdtree.KDTree(len(src_co))

        for i, c in enumerate(src_co):
            kd.insert(c, i)

        kd.balance()

        sample = dump_points[::max(1, len(dump_points) // 64)][:64]

        best_name = None
        best_tf = None
        best_score = 1e30

        for name, tf in transforms:
            total = 0.0

            for p in sample:
                co = tf(p)
                _, _, d = kd.find(co)
                total += d

            if total < best_score:
                best_score = total
                best_name = name
                best_tf = tf

        # ---- 生成 dat ----
        lines = [
            "# VertexPhysics data v1",
            "# model: %s" % ((props.model_name or "").strip() or obj.name),
            "# count: %d" % len(dump_points),
        ]

        # dump 坐标原样写入（= 游戏坐标），mod 端 bbox 指纹才准
        mn = [1e18, 1e18, 1e18]
        mx = [-1e18, -1e18, -1e18]

        for p in dump_points:
            for i, c in enumerate(p):
                if c < mn[i]:
                    mn[i] = c
                if c > mx[i]:
                    mx[i] = c

        lines.append("# bbox: %.6f %.6f %.6f %.6f %.6f %.6f" % (
            mn[0], mn[1], mn[2], mx[0], mx[1], mx[2]))
        lines.append("# index R G B x y z")

        matched = 0

        for idx, p in enumerate(dump_points):
            co = best_tf(p)
            _, vi, dist = kd.find(co)

            rgb = colors.get(vi, (255, 255, 255)) if vi is not None else (255, 255, 255)

            if vi is not None:
                matched += 1

            lines.append("%d %d %d %d %.6f %.6f %.6f" % (
                idx, rgb[0], rgb[1], rgb[2], p[0], p[1], p[2]))

        out_dir = bpy.path.abspath(props.output_dir) or os.path.dirname(path)
        os.makedirs(out_dir, exist_ok=True)

        model = (props.model_name or "").strip() or obj.name
        fname = os.path.splitext(os.path.basename(path))[0] + ".dat"
        out_path = os.path.join(out_dir, fname)

        with open(out_path, "w", encoding="utf-8", newline="\n") as f:
            f.write("\n".join(lines))
            f.write("\n")

        self.report({'INFO'},
            T("已生成 %s（顶点 %d，匹配 %d，轴 %s）")
            % (out_path, len(dump_points), matched, best_name))

        return {'FINISHED'}


DUMP_TRANSFORMS = [
    ("xyz", lambda p: (p[0], p[1], p[2])),
    ("x,z,-y", lambda p: (p[0], p[2], -p[1])),
    ("x,-y,z", lambda p: (p[0], -p[1], p[2])),
    ("x,z,y", lambda p: (p[0], p[2], p[1])),
    ("x,-z,y", lambda p: (p[0], -p[2], p[1])),
    ("x,y,-z", lambda p: (p[0], p[1], -p[2])),
    ("-x,y,z", lambda p: (-p[0], p[1], p[2])),
    ("z,y,x", lambda p: (p[2], p[1], p[0])),
    ("-y,x,z", lambda p: (-p[1], p[0], p[2])),
    ("y,x,z", lambda p: (p[1], p[0], p[2])),
]


def read_dump_file(path):
    """读取 dump，返回 (points, declared_count)。"""
    points = []
    declared = 0

    with open(path, "r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            s = line.strip()

            if not s:
                continue

            if s.startswith("#"):
                if "verts:" in s:
                    try:
                        declared = int(s.split(":")[1].strip())
                    except Exception:
                        pass
                continue

            parts = s.split()

            if len(parts) < 4:
                continue

            try:
                points.append((float(parts[1]), float(parts[2]), float(parts[3])))
            except Exception:
                continue

    return points, declared


def make_kdtree(coords):
    kd = kdtree.KDTree(len(coords))

    for i, c in enumerate(coords):
        kd.insert(c, i)

    kd.balance()
    return kd


def best_transform(dump_points, kd, sample_count=32):
    """返回 (名称, 变换函数, 平均距离)。"""
    if not dump_points:
        return None, None, 1e30

    step = max(1, len(dump_points) // sample_count)
    sample = dump_points[::step][:sample_count]

    best = (None, None, 1e30)

    for name, tf in DUMP_TRANSFORMS:
        total = 0.0

        for p in sample:
            _, _, d = kd.find(tf(p))
            total += d

        avg = total / max(1, len(sample))

        if avg < best[2]:
            best = (name, tf, avg)

    return best


class VERTEXPHYSICS_OT_dat_from_dump_folder(bpy.types.Operator):
    bl_idname = "vertexphysics.dat_from_dump_folder"
    bl_label = T("从 dump 文件夹批量生成")
    bl_description = (
        TD("选中多个带物理色的部件物体，指定 dump 文件夹；"
        "每个 dump 自动匹配最合适的源物体，逐对生成 dat"))

    def execute(self, context):
        props = context.scene.vertexphysics

        dump_dir = bpy.path.abspath(props.dump_dir)

        if not dump_dir or not os.path.isdir(dump_dir):
            self.report({'ERROR'}, T("dump 文件夹不存在：%s") % dump_dir)
            return {'CANCELLED'}

        sources = [o for o in context.selected_objects
                   if o.type == 'MESH' and has_physics_colors(o)]

        if not sources:
            self.report({'ERROR'}, T("请先选中带物理色的部件物体"))
            return {'CANCELLED'}

        # 预计算每个源物体的坐标与 KDTree
        src_info = []

        for o in sources:
            co = [o.matrix_world @ v.co for v in o.data.vertices]
            src_info.append({
                "obj": o,
                "co": co,
                "kd": make_kdtree(co),
                "colors": read_vertex_colors(o.data),
            })

        dumps = sorted(
            os.path.join(dump_dir, f)
            for f in os.listdir(dump_dir)
            if f.lower().endswith(".txt"))

        if not dumps:
            self.report({'ERROR'}, T("dump 文件夹里没有 .txt 文件"))
            return {'CANCELLED'}

        model = (props.model_name or "").strip() or "model"
        base = bpy.path.abspath(props.output_dir) or dump_dir
        out_dir = os.path.join(base, model) if props.use_subdir else base
        os.makedirs(out_dir, exist_ok=True)

        done = 0
        used = set()

        for path in dumps:
            points, declared = read_dump_file(path)

            if not points:
                continue

            # 选最匹配的源物体 + 轴变换
            best = (None, None, None, 1e30)   # info, name, tf, dist

            for info in src_info:
                name, tf, avg = best_transform(points, info["kd"])

                if tf is None:
                    continue

                if avg < best[3]:
                    best = (info, name, tf, avg)

            info, tf_name, tf, avg = best

            if info is None:
                continue

            # 距离太大说明不匹配（避免把车身 dump 配到车灯上）
            if avg > 0.05:
                print(T("[VertexPhysics] 跳过 %s（最近距离 %.4f 太大）")
                      % (os.path.basename(path), avg))
                continue

            colors = info["colors"]

            lines = [
                "# VertexPhysics data v1",
                "# model: %s" % model,
                "# count: %d" % len(points),
            ]

            mn = [1e18, 1e18, 1e18]
            mx = [-1e18, -1e18, -1e18]

            for p in points:
                for i, c in enumerate(p):
                    if c < mn[i]:
                        mn[i] = c
                    if c > mx[i]:
                        mx[i] = c

            lines.append("# bbox: %.6f %.6f %.6f %.6f %.6f %.6f" % (
                mn[0], mn[1], mn[2], mx[0], mx[1], mx[2]))
            lines.append("# index R G B x y z")

            matched = 0

            for idx, p in enumerate(points):
                _, vi, dist = info["kd"].find(tf(p))

                if vi is None:
                    rgb = (255, 255, 255)
                else:
                    rgb = colors.get(vi, (255, 255, 255))
                    matched += 1

                lines.append("%d %d %d %d %.6f %.6f %.6f" % (
                    idx, rgb[0], rgb[1], rgb[2], p[0], p[1], p[2]))

            fname = os.path.splitext(os.path.basename(path))[0] + ".dat"
            out_path = os.path.join(out_dir, fname)

            with open(out_path, "w", encoding="utf-8", newline="\n") as f:
                f.write("\n".join(lines))
                f.write("\n")

            print("[VertexPhysics] %s -> %s（%d/%d）"
                  % (fname, info["obj"].name, matched, len(points)))

            done += 1
            used.add(info["obj"].name)

        self.report({'INFO'},
            T("已生成 %d 个 dat -> %s（用到 %d 个源物体）")
            % (done, out_dir, len(used)))

        return {'FINISHED'}


class VERTEXPHYSICS_OT_paint_type(bpy.types.Operator):
    bl_idname = "vertexphysics.paint_type"
    bl_label = T("给选中顶点刷类型色")
    bl_description = TD("把选中顶点刷成指定物理类型")

    phys_type: EnumProperty(
        name=T("类型"),
        items=[(k, k, "") for k in PHYS_TYPES.keys()],
        default='Cloth',
    )

    def execute(self, context):
        obj = context.object

        if obj is None or obj.type != 'MESH':
            self.report({'ERROR'}, T("请先选择一个网格物体"))
            return {'CANCELLED'}

        mesh = obj.data
        attr = get_color_attribute(mesh)

        if attr is None:
            self.report({'ERROR'}, T("该网格没有顶点色属性"))
            return {'CANCELLED'}

        target = set(v.index for v in mesh.vertices if v.select)

        if not target:
            self.report({'ERROR'}, T("没有选中顶点（编辑模式里选）"))
            return {'CANCELLED'}

        value = PHYS_TYPES[self.phys_type] / 255.0

        if getattr(attr, "domain", "POINT") == 'POINT':
            for i in target:
                if i < len(attr.data):
                    attr.data[i].color = (value, 0.5, 0.2, 1.0)
        else:
            for loop in mesh.loops:
                if loop.vertex_index in target:
                    attr.data[loop.index].color = (value, 0.5, 0.2, 1.0)

        mesh.update()
        self.report({'INFO'}, T("已给 %d 个顶点上色") % len(target))
        return {'FINISHED'}


# ------------------------------------------------------------------
# 面板（同时挂到 VertexPhysics 和 Item 两个标签，方便找到）
# ------------------------------------------------------------------

class VERTEXPHYSICS_PT_panel(bpy.types.Panel):
    bl_label = "VertexPhysics"
    bl_idname = "VERTEXPHYSICS_PT_panel"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = "VertexPhysics"

    def draw(self, context):
        layout = self.layout

        # 语言切换按钮 / Language toggle button
        lang_row = layout.row()
        lang_row.alignment = 'RIGHT'
        lang_row.operator(
            "vertexphysics.toggle_lang",
            text=("中文" if _LANG["value"] == "en" else "English"))

        props = context.scene.vertexphysics

        def op(layout_, idname, icon=None, label=None):
            """安全调用 operator，并按当前语言传入按钮文本。"""
            kw = {}
            if label:
                kw["text"] = T(label)
            try:
                if icon:
                    layout_.operator(idname, icon=icon, **kw)
                else:
                    layout_.operator(idname, **kw)
            except Exception:
                try:
                    layout_.operator(idname, **kw)
                except Exception:
                    pass

        box = layout.box()
        box.label(text=T("导出物理数据"))
        box.prop(props, "model_name", text=T("模型名"))
        box.prop(props, "output_dir", text=T("输出目录"))
        box.prop(props, "axis_mode", text=T("坐标轴"))
        box.prop(props, "apply_transform", text=T("应用物体变换"))
        box.prop(props, "neutralize", text=T("导出后清空顶点色"))

        layout.prop(props, "use_subdir", text=T("输出到模型子目录"))
        layout.prop(props, "target_prefix", text=T("目标物体名前缀"))
        layout.prop(props, "strip_name_suffix", text=T("文件名去掉 .001 后缀"))
        layout.prop(props, "numbered_names", text=T("按 part_01 编号命名"))

        op(layout, "vertexphysics.add_target_prefix", 'FONT_DATA',
           T("给选中物体加此前缀"))
        op(layout, "vertexphysics.bake_dff_order_batch", 'SELECT_EXTEND',
           T("批量按 DFF 顺序导出"))
        layout.prop(props, "dump_dir", text=T("dump 文件夹"))
        op(layout, "vertexphysics.dat_from_dump_folder", 'FILE_FOLDER',
           T("从 dump 文件夹批量生成"))
        op(layout, "vertexphysics.dat_from_dump", 'FILE_TICK',
           T("从游戏 dump 生成 dat（精确）"))
        op(layout, "vertexphysics.bake_dff_order", 'SNAP_ON',
           T("按 DFF 顶点顺序导出（推荐）"))
        op(layout, "vertexphysics.export_dat", 'EXPORT', T("导出 .dat"))
        op(layout, "vertexphysics.export_dat_all", 'DUPLICATE',
           T("导出全部选中物体"))
        op(layout, "vertexphysics.neutralize_colors", 'X', T("清空顶点色"))

        box2 = layout.box()
        box2.label(text=T("快捷上色（可选）"))
        op(box2, "vertexphysics.paint_type", 'BRUSH_DATA',
           T("给选中顶点刷类型色"))

classes = (
    VertexPhysicsProperties,
    VERTEXPHYSICS_OT_export,
    VERTEXPHYSICS_OT_export_all,
    VERTEXPHYSICS_OT_neutralize,
    VERTEXPHYSICS_OT_add_prefix,
    VERTEXPHYSICS_OT_dat_from_dump_folder,
    VERTEXPHYSICS_OT_dat_from_dump,
    VERTEXPHYSICS_OT_bake_batch,
    VERTEXPHYSICS_OT_bake_dff_order,
    VERTEXPHYSICS_OT_paint_type,
    VERTEXPHYSICS_PT_panel,
    VERTEXPHYSICS_OT_toggle_lang,
)


def register():
    for cls in classes:
        try:
            bpy.utils.register_class(cls)
        except Exception as e:
            print("[VertexPhysics] register %s failed: %s" % (cls.__name__, e))

    bpy.types.Scene.vertexphysics = bpy.props.PointerProperty(
        type=VertexPhysicsProperties)

    print("[VertexPhysics] addon registered (v1.6.0, i18n zh/en)")


def unregister():
    if hasattr(bpy.types.Scene, "vertexphysics"):
        del bpy.types.Scene.vertexphysics

    for cls in reversed(classes):
        try:
            bpy.utils.unregister_class(cls)
        except Exception:
            pass


if __name__ == "__main__":
    register()
