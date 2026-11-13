# VertexPhysics for GTA SA — 使用文档与教程

> 给人物、车辆、地图物件加上 **头发 / 布料 / 身体软组织** 的顶点物理。
> 物理数据可以来自模型顶点色，也可以来自外部 `.dat` 文件（推荐，模型不会被染色）。

版本：ASI v1.0（本文档对应源码状态） / Blender 插件 v1.6.0
适用：GTA San Andreas 1.0 US（MSVC x86 编译，输出 `.asi`）

---

## 目录

1. [它能做什么 / 不能做什么](#1-它能做什么--不能做什么)
2. [安装](#2-安装)
3. [快速开始（5 分钟）](#3-快速开始5-分钟)
4. [顶点色编码](#4-顶点色编码)
5. [ini 配置详解](#5-ini-配置详解)
6. [三种数据来源](#6-三种数据来源)
7. [dump 精确流程（顶点顺序 100% 对齐）](#7-dump-精确流程)
8. [Blender 插件教程](#8-blender-插件教程)
9. [多部件模型（车辆）](#9-多部件模型车辆)
10. [支持的模型类型 × 物理类型](#10-支持的模型类型--物理类型)
11. [日志与排错](#11-日志与排错)
12. [已知限制（引擎层面）](#12-已知限制引擎层面)
13. [性能建议](#13-性能建议)

---

## 1. 它能做什么 / 不能做什么

### 支持

| 类型 | R 值 | 表现 |
|---|---|---|
| **Hair 头发** | `64` | 挂在头皮/帽子上，走路转身有滞后甩动，会受风 |
| **Cloth 布料** | `128` | 披风、裙摆、旗帜：重力下垂 + 摆动 + 滞后 |
| **Body 身体软组织** | `192` | 胸部/臀部等：跟随骨骼运动时的弹簧抖动，带边界渐隐 |
| **SoftBody 软体** | `254` | 类似布料但更硬，适合软装饰 |

### 支持在哪些模型上工作

- ✅ **人物角色**（ped）：三类物理都能用
- ✅ **车辆**（vehicle）：三类物理都能用（每个部件独立 rig）
- ⚠️ **地图静态物件**：只有风摆（头发/布料/软体都按风摆处理），
  地图物件的 `192`（Body）**不工作**（没有骨骼可驱动）

### 不能做

- ❌ 给模型加新骨骼（GTA SA 骨架固定 32 根，加骨骼会崩）
- ❌ 让顶点脱离蒙皮（人物原子必须带 skin，去掉就崩）
- ❌ UE/Blender 那种“一缕缕发丝飘动”（需要发丝结构或骨骼链）
- ❌ 运行时导入 FBX/OBJ（引擎只认 DFF）

> 详细原因见 [12. 已知限制](#12-已知限制引擎层面)。

---

## 2. 安装

### 2.1 ASI

1. 确认已安装 **CLEO**（提供 `.asi` 加载）或任何 ASI Loader；
2. 把 `Vertex Physics mod` 放到：

   ```
   <游戏目录>\modloader\
   ```
   例如 `A:\Vanilla GTASA\modloader\Vertex Physics mod`
3. 启动游戏 → 首次运行会在游戏目录生成配置与日志。

### 2.2 目录结构

```
<游戏目录>\
├─ VertexPhysics.asi            (或 modloader 下)
├─ vertexphysics.ini             ← 配置文件
├─ vertexphysics\                ← 物理数据目录（自动创建）
│   ├─ wfyro.dat                 ← 单部件模型
│   ├─ admiral\                  ← 多部件模型（车辆）
│   │   ├─ chassis.dat
│   │   └─ wheel.dat
│   └─ dump\                     ← dumpGeometry=1 时导出的几何体
│       └─ wfyro_1094.txt
```

### 2.3 Blender 插件

1. Blender → **编辑 → 偏好设置 → 插件 → 安装…**；
2. 选择 `blender vertexphysics addon.py`；
3. 勾选启用 **VertexPhysics Exporter**；
4. 3D 视图按 `N` → 侧栏 **VertexPhysics** 标签页。

---

## 3. 快速开始（5 分钟）

**目标**：让一个人物的披风有物理。

1. Blender 里给披风顶点刷色：`128, 200, 255`
   （= Cloth / 软度 200 / 锚定全关）
2. 导出 DFF，放进游戏（modloader）；
3. 打开 `vertexphysics.ini`：

   ```ini
   datonly=0          ; 先用顶点色测试
   wfyro              ; 追加模型名（与 gta3.img 内模型名一致）
   ```

4. 启动游戏 → 看 `vertexphysics.log`：

   ```
   [Config] resolved wfyro -> modelId=92
   [Entity] rig created modelId=92 ... cloth=221 ...
   ```

5. 进游戏看披风是否摆动。

> 想避免顶点色被“染”到模型上？用第 6 节的 `.dat` 方案。

---

## 4. 顶点色编码

三个通道各自含义：

| 通道 | 含义 | 取值 |
|---|---|---|
| **R** | 物理类型 | `0/64/128/192/254` |
| **G** | 软度（越大越软） | `0~255` |
| **B** | 锚定强度（越大越自由） | `0~255` |

### 软度 G 的实际映射

| G | 刚度 | 回位强度 | 手感 |
|---|---|---|---|
| 0 | 1.00 | 0.50 | 很硬 |
| 64 | 0.77 | 0.38 | 偏硬 |
| 128 | 0.55 | 0.26 | 中等 |
| 192 | 0.32 | 0.14 | 软 |
| 255 | 0.10 | 0.02 | 很软 |

### 锚定 B 的行为

| B | 效果 |
|---|---|
| `0` | **硬锚定**：完全固定在骨骼上（适合发根、裙腰） |
| `30~80` | 强锚定：跟着身体，摆动小 |
| `150` | 中等 |
| `255` | **完全不锚定**：只靠挂点/约束/重力/风 |

### 推荐配色

| 部位 | 颜色 | 说明 |
|---|---|---|
| 披风/裙摆上沿 | `128,120,60` | 固定在腰部 |
| 披风/裙摆中段 | `128,180,150` | 过渡 |
| 披风/裙摆下摆 | `128,200,255` | 自由摆动 |
| 头发根部 | `64,120,40` | 固定在头皮 |
| 头发中段 | `64,160,150` | 过渡 |
| 发梢 | `64,200,255` | 自由甩动 |
| 胸部软组织 | `192,40,40` | 轻微抖动（G 小=幅度小） |
| 臀部软组织 | `192,90,60` | 明显一些 |

> **纯白 `255,255,255` 和纯灰（三通道相等）不参与物理**，
> 所以“不刷色”或“刷白”的顶点就是完全跟随骨骼。

---

## 5. ini 配置详解

文件：`<游戏目录>\vertexphysics.ini`（修改后需重启游戏）

```ini
datonly=0
worldRadius=2.5
interactRadius=60
maxInstances=24
wind=1.0
mapphysics=1
mapconvert=2
dumpGeometry=0
```

| 键 | 默认 | 范围 | 说明 |
|---|---|---|---|
| `datonly` | 1 | 0/1 | 1=只读 `.dat`；0=读模型顶点色 |
| `worldRadius` | 2.5 | 0.5~20 | 跨模型碰撞收集半径（米）。披风很长时调到 6~10 |
| `interactRadius` | 8 | 1~50 | 地图物件物理扫描半径（米） |
| `maxInstances` | 8 | 0~64 | 地图物件物理实例上限，0=关闭 |
| `wind` | 1.0 | 0~5 | 全局风力倍率（乘到天气强度上） |
| `mapphysics` | 0 | 0/1 | 地图物件物理开关 |
| `mapconvert` | 0 | 0/1/2 | 0=关 / 1=换入换回 / 2=常驻换入（推荐） |
| `dumpGeometry` | 0 | 0/1 | 1=把白名单模型几何体导出成 dump |

### 白名单

```
# 每行一个模型名（小写，与 gta3.img 里一致）
wfyro
admiral
test
```

> 没在白名单里的模型**完全不处理**（日志会提示 `[MarkerModel]`）。

### 天气风力

```
wind<天气ID>=角度,强度,湍流频率
winddefault=0,0.6,0.15
```

| 字段 | 含义 |
|---|---|
| 角度 | 0~360°，0=+X（东），90=+Y（北） |
| 强度 | 0 无风 / 1 微风 / 3 大风 / 6 强风 |
| 湍流 | 0 平稳 / 0.3 缓慢 / 1.5 快速（Hz） |

常用天气 ID：

| ID | 天气 | ID | 天气 |
|---|---|---|---|
| 0~3 | 洛杉矶晴 | 12 | LV 多云 |
| 4 | LA 多云 | 15 | 郊外多云 |
| 8 | SF 雨 | 16 | 郊外雨 |
| 9 | SF 雾 | 19 | 沙漠沙尘暴 |
| | | 20 | 水下 |

---

## 6. 三种数据来源

### 方案 A：直接用顶点色（最快）

- 优点：不用生成额外文件，改完刷色导出 DFF 就能测；
- 缺点：**顶点色同时是模型预光照色**，刷物理颜色会让模型整体变色；
- 用法：`datonly=0`，白名单加模型名。

### 方案 B：单个 `.dat`（推荐，模型不变色）

1. 刷完顶点色；
2. Blender 插件点 **「导出 .dat」**；
3. 把 `<模型名>.dat` 放到 `vertexphysics\`；
4. `datonly=1`。

mod 会临时把 dat 颜色写进克隆体做物理计算，**用完立刻恢复原色**，所以模型不会被染色。

### 方案 C：多部件 `.dat` 文件夹（车辆）

```
vertexphysics\admiral\
    chassis.dat
    wheel.dat
    ...
```

mod 会按**顶点数**把每个 dat 匹配给车辆的每个 atomic，顶点数相同时用 `# bbox:` 判定。

---

## 7. dump 精确流程

**什么时候需要**：Blender 的顶点和游戏里的顶点顺序/数量对不上（常见于车辆、多材质模型）。

### 步骤

1. **ini 打开 dump**

   ```ini
   datonly=1
   dumpGeometry=1
   wfyro
   ```

2. **进游戏让模型出现一次**（走到它旁边 / 刷出来），然后退出。

   生成：`vertexphysics\dump\wfyro_1094.txt`

3. **回到 Blender**：刷好顶点色 → 点 **「从游戏 dump 生成 dat（精确）」** → 选择那个 `.txt`。

   插件会用 KDTree 按坐标把颜色映射到 dump 的顶点顺序上，生成顶点数**完全一致**的 `.dat`。

4. **把 dat 放到** `vertexphysics\wfyro.dat`（或 `wfyro_1094.dat`，两种都认），重启游戏。

### 验证

```
[Dat] loaded A:\Vanilla GTASA\vertexphysics\wfyro.dat vertices=1094
[Entity] rig created modelId=92 ... hair=... cloth=...
```

---

## 8. Blender 插件教程

面板：3D 视图 `N` → **VertexPhysics** 标签页
右上角按钮：`English` / `中文` 切换界面语言（提示气泡固定双语显示）

### 8.1 属性

| 项 | 说明 |
|---|---|
| **模型名** | 与 IDE/CModelInfo 一致（例如 test），也用于 dat 文件名 |
| **输出目录** | dat 输出位置，默认 `//vertexphysics`（相对 blend 文件） |
| **坐标轴** | 位置坐标转换（索引匹配为主，位置仅校验） |
| **应用物体变换** | 导出时把 Object 变换烘进坐标 |
| **导出后清空顶点色** | 导出成功后把顶点色刷白（默认关，**别开**，否则再导出会得到空数据） |
| **输出到模型子目录** | 多部件模型用：输出到 `<输出目录>/<模型名>/` |
| **目标物体名前缀** | 名字以此开头的物体当作“目标”（导入回来的 DFF） |
| **文件名去掉 .001 后缀** | 物体名末尾 `.001` 去掉再命名 |
| **按 part_01 编号命名** | 生成 `part_01.dat`、`part_02.dat`… |
| **dump 文件夹** | 游戏导出的几何体 dump 目录 |

### 8.2 按钮

| 按钮 | 用途 |
|---|---|
| **导出 .dat** | 当前物体 → 一个 dat |
| **导出全部选中物体** | 每个选中的网格各导出一个 dat |
| **清空顶点色** | 顶点色刷白 |
| **给选中顶点刷类型色** | 编辑模式下给选中顶点刷指定类型色 |
| **按 DFF 顶点顺序导出（推荐）** | 选两个物体：源（带色）+ 重新导入的 DFF，按 DFF 顺序生成 dat |
| **批量按 DFF 顺序导出** | 多部件批量版本 |
| **给选中物体加此前缀** | 给选中网格加“目标前缀” |
| **从游戏 dump 生成 dat（精确）** | 用 dump 文件生成顶点顺序完全一致的 dat |
| **从 dump 文件夹批量生成** | 一次处理整个 dump 文件夹 |

### 8.3 快捷上色

**给选中顶点刷类型色**：编辑模式选中顶点 → 选择类型（Hair/Cloth/Body/SoftBody）+ 软度 + 锚定 → 点按钮。

---

## 9. 多部件模型（车辆）

### 导出

1. 选中所有部件物体；
2. 勾选 **输出到模型子目录**；
3. 点 **导出全部选中物体**（或批量按 DFF 顺序导出）；
4. 得到 `<输出目录>\admiral\*.dat`；
5. 整个文件夹复制到 `vertexphysics\admiral\`。

### 重要：只给需要的部件建 dat

**没有物理颜色的部件不要放 dat**，否则 mod 会为它创建 rig（克隆几何体、每帧碰撞收集），
车辆部件一多就会明显掉帧。

> 新版本已经会自动跳过“dat 全白”的部件，但最省事的做法仍是只给需要的部件导出。

### 验证

```
[Entity] skip modelId=445 verts=105 (dat has no physics colors)
[Entity] rig created modelId=445 verts=1802 cloth=63 ...
```

只有一条 `rig created` 说明配置正确。

---

## 10. 支持的模型类型 × 物理类型

| 模型 | Hair(64) | Cloth(128) | Body(192) | SoftBody(254) |
|---|---|---|---|---|
| 人物 ped | ✅ | ✅ | ✅ | ✅ |
| 车辆 vehicle | ✅ | ✅ | ✅ | ✅ |
| 地图物件 | ⚠️ 仅风摆 | ⚠️ 仅风摆 | ❌ 无效 | ⚠️ 仅风摆 |

说明：

- 人物/车辆走 **世界空间**：有骨骼锚点、挂点约束、风力、Body 弹簧；
- 地图物件走 **模型空间**：没有骨骼，只有风 + 湍流 + 重力 + 形状记忆，
  且 `invMass=0` 的 Body 粒子在该分支会被跳过。

---

## 11. 日志与排错

### 日志文件

| 文件 | 内容 |
|---|---|
| `wfyro_hair.log` | 配置、模型扫描、dat 加载、rig 创建、渲染钩子 |
| `geophys_debug.log` | 颜色分类 `[ColorHist]`、步进 `[Step]`、身体分组 `[BodyGroups]` |

### 常用日志行

| 日志 | 含义 |
|---|---|
| `[Config] resolved wfyro -> modelId=92` | 白名单解析成功 |
| `[Dat] loaded ... vertices=1094` | dat 已加载 |
| `[Dat] not found: ...` | dat 路径/文件名不对 |
| `[ColorHist] unique=3 | (128,200,255)=990` | 颜色分类结果（这一行最重要） |
| `[Entity] rig created ... hair=/cloth=/body=` | rig 创建成功 |
| `[Entity] init FAILED ...` | 初始化失败（通常是没物理颜色） |
| `[Entity] skip modelId=... (dat has no physics colors)` | 该部件没有物理，已跳过 |
| `[VER_82][Step] ... hair(n,max=x m)` | 每类粒子的最大偏移（判断摆幅） |

### 常见问题

| 现象 | 原因 | 解决 |
|---|---|---|
| 完全没有物理 | 模型不在白名单 / 没有 dat / 顶点全白 | 看 `[ColorHist]` 与 `[Dat]` 日志 |
| 物理只在几个点上 | 只刷了几个顶点 | 重刷，确认 `(128,200,255)` 数量足够 |
| 模型整体变色 | 用顶点色方案 | 改用 `.dat`（`datonly=1`） |
| 布料/头发整体跟着转 | 锚定 B 太大 | 提高 B 到 255，让锚定关闭 |
| 披风/头发被拉长 | 约束链太长 | 优先用 B=255 + 挂点；必要时降低软度 |
| 车辆一出现就卡 | 每个部件都建了 rig | 只给需要的部件导 dat（新版本会自动跳过全白 dat） |
| 地图物件不动 | `mapphysics=0` / 不在扫描半径 | 打开 `mapphysics=1`，调大 `interactRadius` |
| 头发/布料穿过身体 | 碰撞半径太小 | 调大 `worldRadius` |

---

## 12. 已知限制（引擎层面）

GTA SA 的 RenderWare 3.7 是 2004 年为 PS2 内存预算设计的**预编译**架构：

| 限制 | 原因 |
|---|---|
| 不能加新骨骼 | 人物骨架在 clump 创建时按固定 32 根分配，动画（IFP）按索引存关键帧 |
| 不能去掉蒙皮 | 人物渲染分支假定原子带 `RpSkin`，去掉直接崩 |
| 不能运行时导入 FBX/OBJ | 只有 DFF/TXD/IFP/COL 解析器 |
| 改蒙皮权重会撕裂 | `RpSkin` 在创建时预编译骨骼索引重映射，改数据不改缓存会不一致 |
| 头发无法做到发丝级飘动 | 顶点物理 + 蒙皮回写的组合限制 |

要做“真正的飘逸头发”，需要配套的**骨骼扩展器**（独立的引擎级 mod），
这不是本文档这个 ASI 的范畴。

---

## 13. 性能建议

| 建议 | 原因 |
|---|---|
| 车辆只给需要的部件导出 dat | 每个部件一个 rig，开销成倍增加 |
| 地图物件 `maxInstances` 控制在 16~32 | 每个实例一份克隆几何体 |
| `interactRadius` 不要设太大 | 扫描范围内所有建筑都会建实例 |
| 布料粒子数控制在 1000 以内 | 每帧约束迭代 12 次 × 边数 |
| 披风很长时只给边缘做物理 | 中间顶点固定可省大量计算 |
| 不要用 `datonly=0` + 大面积刷色做最终版本 | 顶点色会染色，且物理颜色占用顶点数据 |

---

## 附录：日志里的关键标识含义

| 标识 | 意思 |
|---|---|
| `[VER_82]` | 物理模块通用前缀 |
| `[Entity]` | 实体（人物/车辆/道具）处理 |
| `[MapPhysics]` / `[MapInst]` | 地图物件物理 |
| `[RenderHook]` | 逐实例渲染钩子统计 |
| `[Dat]` | 外部 dat 加载 |
| `[ColorHist]` | 顶点色分类统计 |
| `[WeightInherit]` | 无权重顶点继承邻居权重 |
| `[Pinned]` | 挂点统计 |
| `[BodyGroups]` | 身体软组织分组 |

---
---

# VertexPhysics for GTA SA — Documentation & Tutorial (English)

> Add **Hair / Cloth / Body Soft-Tissue** vertex physics to peds, vehicles, and map objects.
> Physics data can be read from model vertex colors or external `.dat` files (recommended, as it prevents tinting the model).

Version: ASI v1.0 (matches current source code) / Blender Add-on v1.6.0
Compatibility: GTA San Andreas 1.0 US (MSVC x86 build, outputs `.asi`)

---

## Table of Contents

1. [What It Can / Cannot Do](#1-what-it-can--cannot-do)
2. [Installation](#2-installation)
3. [Quick Start (5 Minutes)](#3-quick-start-5-minutes)
4. [Vertex Color Encoding](#4-vertex-color-encoding)
5. [INI Configuration Details](#5-ini-configuration-details)
6. [Three Data Sources](#6-three-data-sources)
7. [Exact Dump Workflow (100% Vertex Order Alignment)](#7-exact-dump-workflow-100-vertex-order-alignment)
8. [Blender Add-on Tutorial](#8-blender-add-on-tutorial)
9. [Multi-Part Models (Vehicles)](#9-multi-part-models-vehicles)
10. [Supported Model Types × Physics Types](#10-supported-model-types--physics-types)
11. [Logs & Troubleshooting](#11-logs--troubleshooting)
12. [Known Limitations (Engine-Level)](#12-known-limitations-engine-level)
13. [Performance Tips](#13-performance-tips)

---

## 1. What It Can / Cannot Do

### Supported Physics Types

| Type | R Value | Behavior |
|---|---|---|
| **Hair** | `64` | Anchored to the scalp/hat; exhibits lag and sway when walking or turning, and reacts to wind |
| **Cloth** | `128` | Capes, skirts, flags: gravity droop + swaying + inertia lag |
| **Body** | `192` | Chest/glutes, etc.: spring jiggle driven by bone movement with boundary falloff |
| **SoftBody** | `254` | Similar to Cloth but stiffer; suitable for soft accessories |

### Supported Model Types

- ✅ **Peds (Characters)**: Supports all physics types
- ✅ **Vehicles**: Supports all physics types (each component gets an independent rig)
- ⚠️ **Static Map Objects**: Wind sway only (Hair/Cloth/SoftBody are all processed as wind sway).
  `192` (Body) **does not work** on map objects (no skeleton bones to drive motion).

### Not Supported

- ❌ Adding new bones to models (GTA SA skeletons are fixed at 32 bones; adding bones causes crashes)
- ❌ Detaching vertices from skinning (ped atomics require `RpSkin`; removing it crashes the game)
- ❌ Strand-by-strand hair simulation like UE/Blender (requires hair card/strand structures or bone chains)
- ❌ Importing FBX/OBJ at runtime (the engine only supports DFF)

> See [12. Known Limitations (Engine-Level)](#12-known-limitations-engine-level) for technical details.

---

## 2. Installation

### 2.1 ASI Plugin

1. Ensure **CLEO** (which loads `.asi` files) or any standard ASI Loader is installed.
2. Place the `Vertex Physics mod` folder into:

   ```
   <Game Directory>\modloader\
   ```
   For example: `A:\Vanilla GTASA\modloader\Vertex Physics mod`
3. Launch the game → configuration and log files will be automatically generated in the game directory on first run.

### 2.2 Directory Structure

```
<Game Directory>\
├─ VertexPhysics.asi            (or inside modloader)
├─ vertexphysics.ini             ← Configuration file
├─ vertexphysics\                ← Physics data directory (auto-created)
│   ├─ wfyro.dat                 ← Single-part model
│   ├─ admiral\                  ← Multi-part model (vehicle)
│   │   ├─ chassis.dat
│   │   └─ wheel.dat
│   └─ dump\                     ← Exported geometry when dumpGeometry=1
│       └─ wfyro_1094.txt
```

### 2.3 Blender Add-on

1. In Blender, go to **Edit → Preferences → Add-ons → Install…**
2. Select `blender vertexphysics addon.py`
3. Check the box to enable **VertexPhysics Exporter**
4. Press `N` in the 3D Viewport → open the **VertexPhysics** tab in the sidebar.

---

## 3. Quick Start (5 Minutes)

**Goal**: Add physics to a character's cape.

1. In Blender, paint the cape vertices with color: `128, 200, 255`
   (= Cloth / Softness 200 / Anchoring completely off)
2. Export the DFF and place it into your game (`modloader`).
3. Open `vertexphysics.ini`:

   ```ini
   datonly=0          ; Test with vertex colors first
   wfyro              ; Append model name (must match the model name in gta3.img)
   ```

4. Launch the game → check `vertexphysics.log`:

   ```
   [Config] resolved wfyro -> modelId=92
   [Entity] rig created modelId=92 ... cloth=221 ...
   ```

5. Check in-game to see if the cape sways.

> Want to avoid tinting your model with vertex colors? Use the `.dat` workflow in Section 6.

---

## 4. Vertex Color Encoding

Meaning of the three RGB channels:

| Channel | Meaning | Value Range |
|---|---|---|
| **R** | Physics Type | `0 / 64 / 128 / 192 / 254` |
| **G** | Softness (higher = softer) | `0~255` |
| **B** | Anchor Strength (higher = freer movement) | `0~255` |

### Softness (G) Mapping

| G | Stiffness | Return Strength | Feel |
|---|---|---|---|
| 0 | 1.00 | 0.50 | Very stiff |
| 64 | 0.77 | 0.38 | Moderately stiff |
| 128 | 0.55 | 0.26 | Medium |
| 192 | 0.32 | 0.14 | Soft |
| 255 | 0.10 | 0.02 | Very soft |

### Anchoring (B) Behavior

| B | Effect |
|---|---|
| `0` | **Hard Anchor**: Completely locked to the bone (ideal for hair roots, waistbands) |
| `30~80` | Strong Anchor: Follows the body closely with minimal sway |
| `150` | Medium |
| `255` | **Unanchored**: Driven solely by pinned vertices, constraints, gravity, and wind |

### Recommended Color Presets

| Region | Color (R, G, B) | Description |
|---|---|---|
| Upper cape / skirt waist | `128,120,60` | Anchored near the waist |
| Mid cape / skirt | `128,180,150` | Transition zone |
| Lower cape / hem | `128,200,255` | Free flowing |
| Hair roots | `64,120,40` | Anchored to the scalp |
| Mid hair | `64,160,150` | Transition zone |
| Hair tips | `64,200,255` | Free swinging |
| Chest soft tissue | `192,40,40` | Subtle jiggle (lower G = smaller amplitude) |
| Glute soft tissue | `192,90,60` | Slightly more pronounced jiggle |

> **Pure white `255,255,255` and pure grayscale (where R = G = B) are excluded from physics**,
> meaning unpainted or white-painted vertices will strictly follow standard bone skinning.

---

## 5. INI Configuration Details

File: `<Game Directory>\vertexphysics.ini` (requires game restart after editing)

```ini
datonly=0
worldRadius=2.5
interactRadius=60
maxInstances=24
wind=1.0
mapphysics=1
mapconvert=2
dumpGeometry=0
```

| Key | Default | Range | Description |
|---|---|---|---|
| `datonly` | 1 | 0/1 | `1` = read `.dat` files only; `0` = read model vertex colors |
| `worldRadius` | 2.5 | 0.5~20 | Cross-model collision collection radius (meters). Increase to `6~10` for long capes |
| `interactRadius` | 8 | 1~50 | Map object physics scan radius (meters) |
| `maxInstances` | 8 | 0~64 | Max active map object physics instances (`0` = disabled) |
| `wind` | 1.0 | 0~5 | Global wind multiplier (multiplied onto weather wind strength) |
| `mapphysics` | 0 | 0/1 | Toggle physics for map objects |
| `mapconvert` | 0 | 0/1/2 | `0` = off / `1` = swap in & out / `2` = persistent swap-in (recommended) |
| `dumpGeometry` | 0 | 0/1 | `1` = dump geometry of whitelisted models to text files |

### Whitelist

```
# One model name per line (lowercase, matching gta3.img)
wfyro
admiral
test
```

> Models not listed in the whitelist are **completely ignored** (logged as `[MarkerModel]`).

### Weather Wind Settings

```
wind<WeatherID>=Angle,Strength,TurbulenceFreq
winddefault=0,0.6,0.15
```

| Field | Meaning |
|---|---|
| Angle | `0~360°`, `0` = +X (East), `90` = +Y (North) |
| Strength | `0` calm / `1` breeze / `3` strong wind / `6` gale |
| Turbulence | `0` steady / `0.3` slow / `1.5` rapid (Hz) |

Common Weather IDs:

| ID | Weather | ID | Weather |
|---|---|---|---|
| 0~3 | Los Santos Sunny | 12 | Las Venturas Cloudy |
| 4 | LS Cloudy | 15 | Countryside Cloudy |
| 8 | San Fierro Rain | 16 | Countryside Rain |
| 9 | SF Fog | 19 | Desert Sandstorm |
| | | 20 | Underwater |

---

## 6. Three Data Sources

### Method A: Direct Vertex Colors (Fastest)

- **Pros**: No extra files needed; paint, export DFF, and test immediately.
- **Cons**: **Vertex colors also act as pre-lighting colors in GTA SA**, so painting physics colors will tint the model in-game.
- **Usage**: Set `datonly=0` and add the model name to the whitelist.

### Method B: Single `.dat` File (Recommended — No Model Tinting)

1. Paint vertex colors in Blender.
2. Click **Export .dat** in the Blender add-on.
3. Place `<model_name>.dat` inside `vertexphysics\`.
4. Set `datonly=1`.

The mod temporarily writes the `.dat` colors into a cloned mesh for physics simulation and **immediately restores original vertex colors afterward**, leaving your model's appearance unaffected.

### Method C: Multi-Part `.dat` Folder (Vehicles)

```
vertexphysics\admiral\
    chassis.dat
    wheel.dat
    ...
```

The mod matches each `.dat` file to the vehicle's atomics by **vertex count**, using `# bbox:` bounding box checks as a tie-breaker when vertex counts match.

---

## 7. Exact Dump Workflow (100% Vertex Order Alignment)

**When to use**: When Blender's vertex order or count does not match the game's runtime mesh (common with vehicles and multi-material meshes).

### Steps

1. **Enable dump in `vertexphysics.ini`**:

   ```ini
   datonly=1
   dumpGeometry=1
   wfyro
   ```

2. **Launch the game and spawn/encounter the model once**, then exit the game.

   This generates: `vertexphysics\dump\wfyro_1094.txt`

3. **Back in Blender**: Paint your vertex colors → click **Generate dat from game dump (Exact)** → select the generated `.txt` file.

   The add-on uses a KDTree to spatially map your vertex colors onto the exact vertex order of the dump, producing a `.dat` with a **100% matching vertex count and order**.

4. **Place the `.dat` file** at `vertexphysics\wfyro.dat` (or `wfyro_1094.dat` — both are recognized) and restart the game.

### Verification

```
[Dat] loaded A:\Vanilla GTASA\vertexphysics\wfyro.dat vertices=1094
[Entity] rig created modelId=92 ... hair=... cloth=...
```

---

## 8. Blender Add-on Tutorial

Panel location: 3D Viewport `N` → **VertexPhysics** tab
Top-right button: `English` / `中文` toggles UI language (tooltips are always bilingual).

### 8.1 Properties

| Property | Description |
|---|---|
| **Model Name** | Matches IDE/CModelInfo name (e.g., `test`); also used as the `.dat` filename |
| **Output Directory** | Destination folder for `.dat` files; defaults to `//vertexphysics` (relative to `.blend` file) |
| **Coordinate Axes** | Axis conversion for positions (matching relies primarily on vertex index; positions are used for validation) |
| **Apply Object Transform** | Bakes object transforms into vertex coordinates upon export |
| **Clear Vertex Colors After Export** | Resets vertex colors to white after exporting (off by default — **keep off**, or subsequent exports will be blank) |
| **Export to Model Subdirectory** | For multi-part models: outputs to `<Output Directory>/<Model Name>/` |
| **Target Object Prefix** | Objects starting with this prefix are treated as "targets" (re-imported DFF meshes) |
| **Strip .001 Suffix** | Removes trailing `.001` from object names when naming `.dat` files |
| **Name as part_01, part_02…** | Sequentially names exported files as `part_01.dat`, `part_02.dat`, etc. |
| **Dump Folder** | Directory containing geometry dumps exported by the game |

### 8.2 Operators / Buttons

| Button | Function |
|---|---|
| **Export .dat** | Exports the active object to a single `.dat` file |
| **Export All Selected Objects** | Exports each selected mesh as its own `.dat` file |
| **Clear Vertex Colors** | Resets vertex colors to pure white |
| **Paint Selected Vertices** | Assigns the chosen physics type, softness, and anchor color to selected vertices in Edit Mode |
| **Export by DFF Vertex Order (Recommended)** | Select two objects: Source (painted) + Re-imported DFF; generates a `.dat` matching the DFF's vertex order |
| **Batch Export by DFF Order** | Multi-part batch version of the DFF order exporter |
| **Add Prefix to Selected** | Prepends the "Target Object Prefix" to selected meshes |
| **Generate dat from Game Dump (Exact)** | Uses a game dump file to generate a `.dat` with identical vertex ordering |
| **Batch Generate from Dump Folder** | Processes an entire folder of dump files at once |

### 8.3 Quick Painting

**Paint Selected Vertices**: Enter Edit Mode → select vertices → choose Type (`Hair` / `Cloth` / `Body` / `SoftBody`) + Softness + Anchoring → click the button.

---

## 9. Multi-Part Models (Vehicles)

### Exporting

1. Select all vehicle component objects.
2. Enable **Export to Model Subdirectory**.
3. Click **Export All Selected Objects** (or **Batch Export by DFF Order**).
4. You will get `<Output Directory>\admiral\*.dat`.
5. Copy the entire folder to `vertexphysics\admiral\`.

### Important: Only Create `.dat` Files for Parts That Need Physics

**Do not include `.dat` files for parts with no physics colors.** Otherwise, the mod will still create a physics rig for them (cloning geometry and running per-frame collision checks), which can noticeably drop FPS on complex vehicles.

> Newer versions automatically skip `.dat` files that are all-white, but exporting only the necessary parts remains best practice.

### Verification

```
[Entity] skip modelId=445 verts=105 (dat has no physics colors)
[Entity] rig created modelId=445 verts=1802 cloth=63 ...
```

Seeing `rig created` only for the intended part confirms your setup is optimal.

---

## 10. Supported Model Types × Physics Types

| Model Type | Hair (64) | Cloth (128) | Body (192) | SoftBody (254) |
|---|---|---|---|---|
| Ped (Character) | ✅ | ✅ | ✅ | ✅ |
| Vehicle | ✅ | ✅ | ✅ | ✅ |
| Map Object | ⚠️ Wind sway only | ⚠️ Wind sway only | ❌ Inactive | ⚠️ Wind sway only |

Notes:

- **Peds & Vehicles** simulate in **World Space**: supports bone anchoring, pinned vertices, wind forces, and Body spring dynamics.
- **Map Objects** simulate in **Model Space**: no bones are present, so simulation uses wind + turbulence + gravity + shape memory; Body particles (`invMass=0`) are skipped in this branch.

---

## 11. Logs & Troubleshooting

### Log Files

| File | Contents |
|---|---|
| `wfyro_hair.log` | Configuration loading, model scanning, `.dat` loading, rig creation, render hooks |
| `geophys_debug.log` | Color classification (`[ColorHist]`), simulation steps (`[Step]`), body soft-tissue grouping (`[BodyGroups]`) |

### Key Log Messages

| Log Entry | Meaning |
|---|---|
| `[Config] resolved wfyro -> modelId=92` | Whitelist model name resolved successfully |
| `[Dat] loaded ... vertices=1094` | `.dat` file loaded |
| `[Dat] not found: ...` | Incorrect `.dat` path or filename |
| `[ColorHist] unique=3 | (128,200,255)=990` | Vertex color classification breakdown (most important diagnostic line) |
| `[Entity] rig created ... hair=/cloth=/body=` | Physics rig initialized successfully |
| `[Entity] init FAILED ...` | Initialization failed (usually due to zero physics-colored vertices) |
| `[Entity] skip modelId=... (dat has no physics colors)` | Part skipped because it has no physics colors |
| `[VER_82][Step] ... hair(n,max=x m)` | Max particle displacement per category (useful for judging sway amplitude) |

### Common Issues

| Symptom | Cause | Solution |
|---|---|---|
| No physics at all | Model not in whitelist / missing `.dat` / vertices are all white | Check `[ColorHist]` and `[Dat]` entries in the log |
| Physics only affects a few points | Only a few vertices were painted | Repaint and verify sufficient `(128,200,255)` vertex counts |
| Entire model changes color | Using vertex color mode (`datonly=0`) | Switch to `.dat` files (`datonly=1`) |
| Cloth/hair rotates rigidly with body | Anchor `B` value is too low | Increase `B` up to `255` to disable bone anchoring |
| Cape/hair stretches excessively | Constraint chain is too long | Use `B=255` + pinned root vertices; reduce Softness (`G`) if needed |
| Game stutters when vehicle spawns | Every vehicle component created a rig | Only export `.dat` files for parts that actually need physics |
| Map objects don't move | `mapphysics=0` or object is out of scan range | Set `mapphysics=1` and increase `interactRadius` |
| Hair/cloth clips through body | Collision radius is too small | Increase `worldRadius` in `vertexphysics.ini` |

---

## 12. Known Limitations (Engine-Level)

GTA SA runs on RenderWare 3.7, a **pre-compiled** engine architecture designed in 2004 for PS2 memory budgets:

| Limitation | Reason |
|---|---|
| Cannot add new bones | Ped skeletons are allocated at a fixed 32 bones during clump creation; IFP animations store keyframes by fixed bone index |
| Cannot remove skinning | The ped rendering pipeline assumes `RpSkin` is present on atomics; removing it causes an immediate crash |
| Cannot import FBX/OBJ at runtime | The engine only includes parsers for DFF, TXD, IFP, and COL |
| Modifying skin weights causes tearing | `RpSkin` pre-compiles bone index remapping tables at creation; altering weights without rebuilding caches causes desync |
| Cannot achieve strand-level hair flow | Inherent limitation of vertex-level simulation written back onto rigid bone skinning |

Achieving true multi-strand flowing hair requires a dedicated **bone extender** (a separate engine-level mod), which is outside the scope of this ASI plugin.

---

## 13. Performance Tips

| Tip | Reason |
|---|---|
| Only export `.dat` files for vehicle parts that need physics | Each `.dat` creates an independent rig, multiplying CPU/memory overhead |
| Keep map object `maxInstances` between `16~32` | Every active map instance allocates a cloned geometry buffer |
| Avoid setting `interactRadius` too high | Every building/prop within range will instantiate a physics rig |
| Keep cloth particle counts under `1,000` | Constraints iterate 12 times per edge every single frame |
| For very long capes, apply physics only to the outer edges | Keeping inner vertices anchored saves substantial computation |
| Do not ship final mods using `datonly=0` with large painted areas | Tints model pre-lighting and consumes vertex color data |

---

## Appendix: Key Log Prefixes

| Prefix | Meaning |
|---|---|
| `[VER_82]` | General physics module prefix |
| `[Entity]` | Entity processing (peds / vehicles / props) |
| `[MapPhysics]` / `[MapInst]` | Static map object physics |
| `[RenderHook]` | Per-instance render hook statistics |
| `[Dat]` | External `.dat` file loading |
| `[ColorHist]` | Vertex color classification histogram |
| `[WeightInherit]` | Unweighted vertices inheriting neighbor bone weights |
| `[Pinned]` | Pinned anchor vertex statistics |
| `[BodyGroups]` | Body soft-tissue island grouping |