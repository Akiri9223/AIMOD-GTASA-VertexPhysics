# VertexPhysics（GTA SA 顶点色物理插件）—— 交接简报

> **本文对应代码状态：2026-11-26**（源码 `src/`，6456 + 4394 + 2567 行）
> 上一版简报（2026-11-23）已过期，归档在 `docs/BRIEF_2026-11-23_archive.md`。
> 若本文与源码冲突，**以源码为准**，并顺手修正本文。

---

## 0. 先读这一段

1. **这不是一个"按模型名白名单挂布料"的 mod**：它扫描**所有实体**的几何体顶点色，
   只要顶点带物理标记（R = 0x40 / 0x80 / 0xC0 / 0xFE），就为那个模型建立一个
   "克隆几何体 + 物理 rig"，每帧算完再写回顶点。
2. **三套坐标系**（第 2.3 节）是绝大多数 bug 的来源，改任何东西前先确认自己在哪套坐标里。
3. **只有 Hair / Cloth 两种粒子参与"世界碰撞"**；Body 是纯弹簧（`invMass = 0`），
   SoftBody 只和自身胶囊碰。
4. 编译产物 `build/VertexPhysics.asi`，安装到游戏根目录，配置 `vertexphysics.ini`。

---

## 1. 工程与构建

| 项 | 值 |
|---|---|
| 工程根 | `A:\GTASA Modding\GTASA C++ Dev\HairPhysicsASI` |
| 源码 | `src/main.cpp`、`src/physics/*.cpp/.h` |
| 工具链 | MSVC **x86**（`B:\Visual Studio\cache\VC\Tools\MSVC\14.51.36231\bin\Hostx86\x86`） |
| 依赖 | `plugin-sdk`（`B:\plugin-sdk-master`）、`safetyhook`（随 plugin-sdk） |
| 构建 | `build\build_all.bat`（**路径无关**，用 `%~dp0` 推算根目录，全引号，支持带空格路径） |
| 产物 | `build\VertexPhysics.asi`（约 1.55 MB） |
| 安装 | 复制到 `A:\Vanilla GTASA\VertexPhysics.asi` |

> `build_local.bat` 现在只是 `call build_all.bat` 的包装，**不要**再复制一份命令行进去。

编译命令：

```bat
cmd /c "A:\GTASA Modding\GTASA C++ Dev\HairPhysicsASI\build\build_all.bat"
```

---

## 2. 架构与数据流

### 2.1 顶点色协议（`ClassifyVertexColor`，`GeometryPhysics.cpp:570`）

| R | 类型 | invMass | 行为 |
|---|---|---|---|
| `0x40` (64) | Hair | 1.0 | 粒子物理 + 自身胶囊 + 世界碰撞 |
| `0x80` (128) | Cloth | 1.1 | 粒子物理 + 自身胶囊 + 世界碰撞 |
| `0xC0` (192) | Body | **0** | 不参与粒子/碰撞，纯弹簧偏移 |
| `0xFE` (254) | SoftBody | 1.4 | 粒子物理 + 自身胶囊（**不参与世界碰撞**） |
| `0x03` (3) | legacy Hair | 1.0 | 兼容旧标记，按 Hair 处理 |
| 其它/未刷色 | Rigid | — | 不是粒子，只作为锚点与约束边界 |

- **G** → `LookupSoftness`（刚度 / 回位强度）
- **B** → `anchorFactor = 1 - B/255`；**`B == 0` 时 `hardAnchor = true`**（对 Hair/Cloth/SoftBody 生效，
  **Body 除外**，见 `:619`）
- `hardAnchor` 粒子每帧被贴回锚点（`:4890`），且**跳过自身胶囊碰撞**（`:5335/5373/5402`）

### 2.2 三条 rig 路径

| 路径 | 触发 | 代码 | 说明 |
|---|---|---|---|
| **实体 rig** | 池里的 Ped / Vehicle / Object（含手持武器 clump） | `EntityPhysics.cpp:3733` 起 | 每个 atomic 一个克隆 + rig |
| **地图实例 rig** | `mapconvert >= 1` | `UpdateMapInstances` | 给 CBuilding 建"私有克隆"，渲染钩子逐实例换几何体 |
| **地图物件模型空间** | `mapphysics=1` 且 `mapconvert=0` | `GeometryPhysics.cpp:2966` 的 modelSpace 分支 | **简化求解，不做世界碰撞** |

### 2.3 三套坐标

1. **模型空间**：几何体原始顶点（`restLocal`）；
2. **世界空间**：粒子状态 `posWorld / prevWorld / anchorWorld`（Verlet 全在这里）；
3. **骨骼空间**：`boneMatrices[i]`（HAnim 世界矩阵）+ `inverseBinds`。

锚点公式（蒙皮模型）：

```cpp
blended = Σ_k w_k * (inverseBind[b_k] * boneMatrix[b_k]);
anchorWorld = blended * restLocal;
```

### 2.4 每帧流程

```
drawingEvent.before (main.cpp)
  └─ UpdateEntityPhysics(deltaTime)               EntityPhysics.cpp:3733
       ├─ [LoadGuard] 玩家 ped 变了？→ ResetAllRigs()，静默 60 帧
       ├─ RegisterAllPedCapsules()                30m 内所有 ped 的骨胶囊 → 世界注册表
       ├─ UpdateMapInstances()                    地图实例：Collect → 自身 col/顶面 → Step → Apply
       └─ 实体 rig 循环（每个候选 atomic）
            ├─ WorldCollision_Collect(...)       清空并重填障碍表（球/盒/三角面/胶囊/地面）
            ├─ [objselfcol] 物件自身 col          只加 col 球/盒/三角面，不加渲染网格
            ├─ WorldCollision_BuildTriangleGrid()
            ├─ StepGeometryPhysics(rig, atomic)   Verlet + 约束 + 碰撞 + Body 弹簧
            ├─ ApplyGeometryPhysics(rig, atomic)  写回克隆顶点 + 重算法线
            └─ 把 rig.colliders 注册成世界胶囊（给别的 rig 用）
```

---

## 3. 碰撞体系（当前真实状态）

### 3.1 自身骨胶囊（`rig.colliders`）

生成：`StepGeometryPhysics` 内按 GTA 骨骼 ID 建 22 根（`pedBoneCapsules`），
无蒙皮模型走 `AddDefaultColliders`（`:6411`，5 个原点占位胶囊）。

部位分组（`PhysicsCapsule::group`，用于 `capskip` / `capsgroupscale*`）：

| bit | 值 | 部位 |
|---|---|---|
| 0 | 1 | 骨盆球 |
| 1 | 2 | 髋连接 + 髋关节球 |
| 2 | 4 | 大腿 |
| 3 | 8 | 膝球 |
| 4 | 16 | 小腿 |
| 5 | 32 | 躯干（骨盆→脊柱→胸 + 胸球） |
| 6 | 64 | 颈/头 |
| 7 | 128 | 手臂/前臂 |
| 8 | 256 | 脚 |

半径公式：

```
最终半径 = 基础半径 × bodycolliderscale × 部位倍率(capsgroupscale*) × 索引倍率(capsulescale)
           + bodycollidermargin
```

布料 vs 自身胶囊的两道保护：

1. **静息深度折扣**（`clothCapDepth`）：静止时就埋进胶囊的深度会被扣掉，
   静止时推力≈0，只有"比静息更深"才推；
2. **多胶囊平均推出**（`pushSum`）：同一粒子被多根重叠胶囊推时取平均，
   一帧只应用一次，消除"骨盆↔大腿"互相推来推去的抖动。

头发 vs 自身胶囊：静息已重叠的胶囊直接屏蔽（`hairs kip` → `capsuleIgnore` 位掩码）。

### 3.2 世界障碍（`WorldCollision_Collect`，`WorldCollision.cpp:1209`）

| 来源 | 形态 | 余量 |
|---|---|---|
| 地面 | 高度查询（单帧抬升 ≤5cm，>30cm 拒绝） | `groundmargin` |
| Ped 池 | col 球/盒（+ 骨胶囊注册表） | — / `bodycollidermargin` |
| Vehicle 池 | col 球/盒；`carmesh=1` 时另加渲染网格（600 面/车） | `carmargin` |
| Object 池 | col 球/盒/三角面 | `objmargin` |
| 静态建筑 | 扇区枚举 CBuilding 的 col 球/盒/三角面 + 薄墙实心板 | `buildingmargin` |
| 地图实例自身 | `mapselfcol`：自身 col（网格读不到时退回可见网格）+ `mapselfbox` 顶面托盘 | `mapselfmargin` |
| CObject 自身 | `objselfcol`：自身 col 球/盒/三角面 | `objselfmargin` |

容量上限（超过就静默丢弃，表现为"远处没碰撞"）：

| 项 | 上限 |
|---|---|
| 障碍球/盒 | 400 |
| 三角面 | 默认 900；每车 +600；建筑合计 1600、单栋 500 |
| 世界胶囊 | 每次收集 256 根（注册表存活 90 次收集） |

**只有 Hair / Cloth 参与世界碰撞**（`GeometryPhysics.cpp:5112`）。

### 3.3 自碰撞（owner 是否包含自己）

| rig | 自身 col | 自身渲染网格 | 开关 |
|---|---|---|---|
| Ped | ❌ | ❌ | —（身体由骨胶囊代表） |
| Vehicle | ✅ 球/盒/三角面 | ❌（`carmesh` 只管别的车） | `selfcollide` |
| CObject | ✅ 球/盒/三角面 | ❌ | `objselfcol` |
| 地图实例 | ✅ col（网格仅非 native 几何体有效） | ⚠️ 同上 | `mapselfcol` / `mapselfbox` |

---

## 4. 头发 vs 布料：手感参数差异

布料有 `clothgravity / clothfollow / clothdamping / clothinertia / clothwind`，
头发原来**没有对应项**，表现是"跑动时向前拉伸"。后来补齐：

| ini | 默认 | 对应布料参数 | 作用 |
|---|---|---|---|
| `hairride` | **0.6** | `clothinertia` 0.6 | 每帧跟随身体位移的比例，剩余交给 Verlet 滞后 ⇒ 跑动自然后拖 |
| `hairdamping` | **0.98** | `clothdamping` 0.98 | 每帧阻尼（60fps 基准），旧值 0.92 更"僵" |
| `hairpull` | 1.0 | — | 锚点回拉总倍率（<1 更自由） |
| `hairwind` | 1.0 | `clothwind` 1.8 | 受风倍率（细长发片想更飘就调大） |
| `hairfollow` | 1.0 | `clothfollow` 0.0 | 有挂点时的回拉倍率（0 = 像布料一样自由垂坠） |

`hairride` 是核心：旧实现是 **100% 全额跟随** anchorMove，匀速时完全不落后，
只有加速度项在推 —— 视觉上就是"向前拉伸"。

---

## 5. Body 弹簧（R = 0xC0）

`invMass = 0`，位置 = `anchorWorld + jiggleOffset * boundaryFade`。
按 `bodygroup` 分组（`piece` / `bone` / `cluster`），每组一个弹簧 + 一个
**确定性随机轴**（绕它旋转驱动方向，最多 ≈40°）⇒ 左胸/右胸/腹能往不同方向抖。

完整推导见 `docs/VertexPhysics_README.md` 第 **10.5** 节（含行号与调参表）。
关键 ini：`bodygroup / bodygroupsplit / bodyrandom / bodysmooth / bodyk / bodyc / bodydrive / bodyamp`。

---

## 6. ini 全表（当前解析的所有键）

### 6.1 扫描与范围
`worldradius`(2.5) · `interactradius`(8) · `maxinstances`(8) · `collideradius`(0) · `wind`(1.0)

### 6.2 地图
`mapconvert`(0) · `mapphysics`(1) · `mapcollision` · `mapselfcol`(1) · `mapselfbox` · `mapselfmargin`(0.02，旧名 `mapobjmargin`)

### 6.3 布料
`clothgravity`(1.0) · `clothfollow`(0.0) · `clothdamping`(0.98) · `clothpinstiffness`(0.15) ·
`clothwind`(1.8) · `clothinertia`(0.6) · `clothsweep` · `pedclothwindflip` · `collsteps`(2，1~4)

### 6.4 头发
`hairpins` · `haircapsulemask` · `hairrootexempt` · `hairdiag` · `hairpiecefix` · `hairrotrans` ·
`hairinertia` · `hairworldcollide` · `hairride`(0.6) · `hairdamping`(0.98) · `hairpull`(1.0) ·
`hairwind`(1.0) · `hairfollow`(1.0)

### 6.5 身体
`bodycolliderscale`(1.0) · `bodycollidermargin`(0.01) · `bodyk`(0) · `bodyc`(0) · `bodydrive`(1.30) ·
`bodyamp`(1.0) · `bodygroup`(piece) · `bodygroupsplit`(3) · `bodyrandom`(0.25) · `bodysmooth`(3)

### 6.6 胶囊（自身）
`capskip`(0) · `capsgroupscale<part>`(1.0，part = pelvis/hip/thigh/knee/calf/torso/neck/arms/feet) ·
`capsulescale`（逗号分隔逐索引） · `clothcapsulemask`(1) · `clothcapsulemaskdepth`(0.008) ·
`characterclothselfmargin`(0.015，旧名 `clothmargin`)

### 6.7 世界余量
`carmargin`(0.02) · `groundmargin`(0.02) · `objmargin`(0.02) · `objselfcol`(1) · `objselfmargin`(0.02) ·
`buildingcol`(1) · `buildingmargin`(0.02) · `selfcollide`(1) · `carmesh`(1)

### 6.8 诊断
`colordiag`(0) · `cjparts`(0) · `dumpgeometry` · `datonly`

---

## 7. 诊断日志速查

| 日志 | 文件 | 含义 |
|---|---|---|
| `[PERF]` / `[EntityScan]` | `vertexphysics.log` | 每 300 帧：rigs / instances / particles |
| `[Perf] update= map= ent= collect= step= apply=` | 同上 | 每段耗时（ms/帧） |
| `[Entity] rig created modelId=… hair= cloth= body= anchor=` | 同上 | rig 建立 |
| `[SkinDiag] geomVerts= bones= split= idxFmt=u8\|u32 weights= indices=` | 同上 | 蒙皮格式（关键：u8 vs u32） |
| `[NoSkinWeight]` | 同上 | 权重读不出（u8 修复后应大幅减少） |
| `[VER_82][Colliders] capsules= … radii=` | 同上 | 最终胶囊半径（乘过所有倍率） |
| `[ClothPiece] n= pins= soft= hard= size=` | 同上 | 每个连通片 |
| `[MapPhysics] rig created modelId=` / `[MapInst] init took=` | 同上 | 地图实例路径 |
| `[SelfCol] modelId= hasData= hasVerts= hasTris=` | 同上 | mapselfcol 是否加到东西 |
| `[ObjSelfCol] modelId= added= spheres= boxes= tris=` | 同上 | objselfcol 结果 |
| `[LoadGuard] reset (…) entries= mapInst=` | 同上 | 读档/换人清空 rig |
| `[BodyDiag] modelId= body= groups= g0(…)` | 同上 | Body 分组与各组偏移 |
| `[ClothKick] modelId= hits= kick= corr= clamp= kind=:id obs= caps= tris=` | `vertexphysics_physics.log` | 单帧碰撞来源与修正量（`kind`：1 地面 / 2 三角面 / 3 球 / 4 盒 / 5 胶囊 / 6 顶面） |

---

## 8. 已知问题（诚实清单）

1. **高速穿过布料仍会穿**（用户实测，2026-11-25）。已尝试并**回滚**的方案：
   胶囊运动采样、身体网格（P2）、玩家身体网格注册（P3）、胶囊/球的相对运动 CCD（保守推进）。
   当前代码里这些都**不存在**，碰撞仍是"帧末位置 + 粒子自身位移子步"。
2. **盒型障碍没有扫掠**：车辆/物件的 col 盒只在帧末做点测试。
3. **三角面扫掠只针对自身网格**（`SegmentHitsTriangle` + `ownCol`），且用的是**粒子自身**的
   上一帧位置，不包含碰撞体的运动。
4. **地图实例的自身可见网格**：SA 地图几何体常是 `rpGEOMETRYNATIVE`（无 CPU 顶点），
   退回读网格会得到 0 个面，此时只有 col 与顶面托盘有效。
5. **胯部胶囊可能被人为关掉**：`capsgroupscalepelvis=0` / `capsgroupscalehip=0`
   会让该部位半径变 0（等于没有碰撞体），日志 `radii=0.000,...` 可直接确认。
6. **SaveLoader 兼容性 —— 已解决（2026-11-26 用户确认）**。
   原因不在 SaveLoader 本身，而在我们这边：物理更新挂在 `drawingEvent` 上，
   **读档过程也会触发**，而那时游戏正在销毁/重建玩家 ped、重载模型、清空实体池，
   我们持有的 `atomic` / 克隆几何体 / `CBuilding*` 变成悬空指针 ⇒ 崩溃或卡在载入。
   修复：`[LoadGuard]`（`UpdateEntityPhysics` 开头）检测玩家 ped 指针变化 →
   `ResetAllRigs()` 只清空全部容器（**不销毁克隆体**，避免二次释放）→ 静默 60 帧。
   现在与 `modloader\SaveLoader\III.VC.SA.SaveLoader.asi` 可共存，读档正常。
7. **`hardAnchor` 粒子不参与自身胶囊碰撞**（`B=0` 的布料区域永远不碰身体）。
8. 静态建筑三角面有预算上限（1600/500），密集场景会截断。

---

## 9. 不要做的事

- 不要用"加大胶囊半径"掩盖穿模（会造成布料悬空）；
- 不要为了修碰撞而关掉约束/锚点；
- 不要在 `.bat` 里写中文注释（cmd 按 GBK 解析 UTF-8 会执行乱码行）；
- 不要用 `set CL=…` 命名变量（MSVC 的 vcvarsall 会占用 `CL`）；
- 不要在源码里写 `near` / `far` 作变量名（Windows 宏）；
- 不要用 PowerShell `Get-Content`/`Set-Content` 改带中文注释的源码（会破坏编码），
  用编辑工具；
- 不要把 `output/`、`build/`、`*.pdb`、`*.asi` 提交进 git。

---

## 10. 代码地图（2026-11-26 实测行号）

### GeometryPhysics.cpp（6456 行）
| 行 | 内容 |
|---|---|
| 570 | `ClassifyVertexColor`（顶点色 → 类型/软度/锚定） |
| 732 | `InitGeometryPhysics`（分类、焊接、约束、挂点、Body 分组、胶囊表） |
| 2084–2228 | Body `bodyGroup` 分配（piece / bone / cluster k-means） |
| 2966 | `StepGeometryPhysics`（modelSpace 分支 / 主求解） |
| 4890 | 硬锚定贴回锚点 |
| 5105 | 世界碰撞循环（子步 + 修正上限） |
| 5335/5373/5402 | 自身胶囊三遍（静态 / CCD / 收尾） |
| 4399–4500 | Body 组位移 → 顶点位移（同组平滑、4mm 限制、boundaryFade） |
| 5904 | `GeometryPhysics_SetCollisionSubsteps`（collsteps） |
| 5911/5921 | Body 手感 / 分组 setter |
| 5932/5940/5946/5952 | 胶囊屏蔽 / 跳过 / 部位倍率 / 索引倍率 |
| 5960 | `ApplyGeometryPhysics`（写回顶点 + 法线） |
| 6411 | `AddDefaultColliders`（非蒙皮模型的 5 个占位胶囊） |
| 6444/6450 | 头发风/挂点、跟随比例 setter |

### EntityPhysics.cpp（4394 行）
| 行 | 内容 |
|---|---|
| 3733 | `UpdateEntityPhysics`（LoadGuard / 胶囊注册 / 地图实例 / 实体循环） |
| 1306 | 地图实例创建（`CreateMapInstance` 附近） |
| 2146 | （WorldCollision 侧）`WorldCollision_AddEntityOwnColModel` |
| 2253–2312 | 配置默认值块 |
| 2400–2600 | `vertexphysics.ini` 解析（全部 key） |
| 3975 | `objselfcol` 自身 col 注入 |
| 4030–4130 | 每帧收集、Step、Apply、世界胶囊注册、Body 诊断 |

### WorldCollision.cpp（2567 行）
| 行 | 内容 |
|---|---|
| 1209 | `WorldCollision_Collect`（地面 + 池 + 建筑 + 胶囊注册表） |
| 1306 | `SetEntityCapsules`（世界胶囊注册，含存活戳） |
| 1398 | `WorldCollision_CollidePoint`（地面/顶面/三角面/球/盒/胶囊） |
| 1846 | `SegmentHitsTriangle`（粒子自身运动扫掠） |
| 1951 | `BuildTriangleGrid`（空间网格） |
| 2146 | `AddEntityOwnColModel`（自身 col：球/盒/三角面，支持 margin 覆盖） |
| 1956/1967 | 顶面托盘（mapselfbox） |
| 2492 | `AddEntityOwnMesh`（网格回退，含 `dynamicVertexMap` 排除布料三角形） |
