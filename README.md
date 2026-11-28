# AIMOD-GTASA-VertexPhysics
这是什么：GTA SA 1.0 US 的顶点色驱动布料/头发/身体/软体物理 ASI；

依赖：plugin-sdk（默认放在 B:\plugin-sdk-master，需自行修改 build_all.bat 里的 PLUGIN_SDK）、Visual Studio x86 工具链；

编译：build\build_all.bat → build\VertexPhysics.asi；

安装：ASI 放到游戏根目录 + vertexphysics.ini；

顶点色规则：R=64 头发 / R=128 布料 / R=192 身体 / R=254 软体/ 未着色=刚性，G=软度，B=锚定强度（B=0 为硬锚定）；

# Cloth Physics
<img width="480" height="270" alt="2026-11-2605-34-15-ezgif com-optimize" src="https://github.com/user-attachments/assets/3b675547-c9da-477b-9369-6f86cad04125" />
<img width="480" height="270" alt="2026-11-2605-30-50-ezgif com-optimize" src="https://github.com/user-attachments/assets/3865526a-39e7-4e47-9961-2b28aab0b620" />
<img width="480" height="270" alt="2026-11-2605-30-29-ezgif com-optimize" src="https://github.com/user-attachments/assets/6217cafd-5761-4338-9c28-dc30cc9ad91b" />
<img width="480" height="270" alt="2026-11-26 05-33-51" src="https://github.com/user-attachments/assets/b73585f4-0e76-486a-89dc-9fb0ac196044" />
<img width="480" height="270" alt="2026-11-26 05-32-29" src="https://github.com/user-attachments/assets/d929a961-6fe1-47a9-90c8-b6d7cb11a361" />
<img width="480" height="270" alt="2026-11-26 05-32-06" src="https://github.com/user-attachments/assets/f5b9a6ac-c548-4f82-b4e0-4ca4d13c844a" />
<img width="480" height="270" alt="2026-11-26 05-30-01" src="https://github.com/user-attachments/assets/0d82089f-7081-48cb-ba13-5ea899d2c15f" />
<img width="480" height="270" alt="2026-11-26 05-35-06" src="https://github.com/user-attachments/assets/1d293d33-1775-4332-91bf-ab8c912d022e" />

# Hair Physics
<img width="480" height="270" alt="2026-11-2812-41-34-ezgif com-optimize" src="https://github.com/user-attachments/assets/726adcd3-73f9-4e0b-885a-762561157e91" />
<img width="480" height="270" alt="2026-11-28 12-40-44" src="https://github.com/user-attachments/assets/f9c243a3-bc80-4ea9-895c-d897846a9980" />
<img width="480" height="270" alt="2026-11-28 12-40-00" src="https://github.com/user-attachments/assets/9297a83d-59a6-4d9f-b206-4d592459211d" />

# Body Physics
<img width="480" height="270" alt="2026-11-28 12-53-22" src="https://github.com/user-attachments/assets/0e72760b-4502-46b2-ab4d-b6577c804ff9" />
<img width="480" height="270" alt="2026-11-28 12-53-00" src="https://github.com/user-attachments/assets/9e42036f-1f79-4380-b8a0-ff4e76562998" />

# Softbody Physics
<img width="480" height="270" alt="2026-11-28 13-05-24" src="https://github.com/user-attachments/assets/9164d1d5-1d5e-4397-a6bc-3fcebe830623" />

# 物理效果和各类模型之间碰撞关系
## 碰撞关系总表

### 1. 物理类型

| 类型 | 顶点色 R | invMass | 说明 |
|---|---|---|---|
| **Hair** | `0x40` (64) | 1.0 | 头发 / 发束 / 尾巴 |
| **Cloth** | `0x80` (128) | 1.1 | 布料 / 披风 / 裙子 / 旗帜 |
| **Body** | `0xC0` (192) | **0** | 身体软组织：**不做粒子模拟**，纯弹簧偏移 |
| **SoftBody** | `0xFE` (254) | 1.4 | 柔性体（**必须精确 254**） |
| **Rigid** | 其它 / 未刷色 | — | 不是粒子，只作为锚点与约束边界 |

---

### 2. 物理类型 × 自身碰撞体

| 自身碰撞体 | Hair | Cloth | Body | SoftBody |
|---|---|---|---|---|
| **自身骨胶囊**（22 根人体胶囊） | ✅（静息已重叠的胶囊会屏蔽） | ✅（静息深度折扣 + 多胶囊平均推出） | ❌ | ✅ |
| **自身 col 球/盒/三角面**（载具 `selfcollide`、物件 `objselfcol`、地图实例 `mapselfcol`） | ✅ | ✅ | ❌ | ❌ |
| **自身渲染网格** | ❌ | ❌ | ❌ | ❌ |
| **顶面托盘**（地图实例 `mapselfbox`） | ✅ | ✅ | ❌ | ❌ |

> Body 的 `invMass = 0`，**不参与任何碰撞求解**，只做“每组一个弹簧位移”。
> Rigid 顶点不是粒子，不参与碰撞。

---

### 3. 物理类型 × 世界碰撞体

只有 **Hair / Cloth** 会参与世界碰撞（代码里唯一的放行条件）。

| 世界碰撞体 | Hair | Cloth | Body | SoftBody |
|---|---|---|---|---|
| 地面 | ✅ | ✅ | ❌ | ❌ |
| 行人 col 球/盒 | ✅ | ✅ | ❌ | ❌ |
| **行人骨胶囊**（30m 内注册） | ✅ | ✅ | ❌ | ❌ |
| 载具 col 球/盒（`carmargin`） | ✅ | ✅ | ❌ | ❌ |
| 载具渲染网格（`carmesh=1`，600 面/车） | ✅ | ✅ | ❌ | ❌ |
| CObject col 球/盒（`objmargin`） | ✅ | ✅ | ❌ | ❌ |
| CObject col 三角面 | ✅ | ✅ | ❌ | ❌ |
| **静态建筑 col 球/盒/三角面**（`buildingcol=1`） | ✅ | ✅ | ❌ | ❌ |
| 地图实例自身 col / 顶面托盘 | ✅ | ✅ | ❌ | ❌ |
| 其它地图实例的 col（同 `buildingcol` 路径） | ✅ | ✅ | ❌ | ❌ |
| 其它地图实例的**渲染网格** | ❌ | ❌ | ❌ | ❌ |
| 其它布料 / 头发（布-布自碰撞） | ❌ | ❌ | ❌ | ❌ |
| 过场物件（cutscene dummy） | ❌ 不收集 | ❌ | ❌ | ❌ |

---

### 4. 模型类型 → 它向其他物理对象提供什么碰撞体

| 模型类型 | col 球 | col 盒 | col 三角面 | 渲染网格 | 骨胶囊 | 备注 |
|---|---|---|---|---|---|---|
| **Ped（人物）** | ✅ | ✅ | 一般无 | ❌ | ✅（30m 内全部注册） | 身体 = 粗盒 + 骨胶囊两套 |
| **Vehicle（载具）** | ✅ | ✅ | 有则用 | ✅（`carmesh=1`，600 面/车） | — | 球/盒带 `carmargin` |
| **CObject（小物件）** | ✅ | ✅ | 有则用 | ❌ | — | 球/盒带 `objmargin` |
| **CBuilding（静态建筑）** | ✅ | ✅ | ✅ | — | — | 扇区枚举，`buildingcol=1`，带 `buildingmargin` |
| **地图实例（本插件克隆）** | ✅（`mapselfcol`） | ✅ | ✅ | ⚠️ 仅在非 native 几何体时可用 | — | 顶面托盘由 `mapselfbox` 控制 |
| **手持武器（clump）** | — | — | — | — | — | 作为布料/头发 rig 存在，不作为障碍 |
| **Cutscene / 过场** | ❌ | ❌ | ❌ | ❌ | — | 完全不收集 |

---

### 5. 容量上限（超出后静默丢弃，表现为“远处没碰撞”）

| 项 | 默认 | 可调 ini |
|---|---|---|
| 障碍球/盒总数 | 400 | `maxobstacles`（100~2000） |
| 三角面总数 | 900 | `buildingtris`（0~8000） |
| 单栋建筑三角面 | 500 | `buildingtrisper`（0~2000） |
| 载具渲染网格 | 600 面/车 | 固定 |
| 世界胶囊（每次收集） | 256 根 | 固定 |
| 收集半径加成 | 0 | `collideradius`（米） |

---

### 6. 关键 ini 一览

| ini | 作用 |
|---|---|
| `collideradius` | 所有 rig 的碰撞收集半径加成（米） |
| `maxobstacles` / `buildingtris` / `buildingtrisper` | 碰撞预算 |
| `buildingboxshallow` | 建筑 col 球/盒只修浅穿透（防被大盒子弹飞） |
| `characterclothselfmargin` | 布料 ↔ 自身骨胶囊的安全距离（旧名 `clothmargin`） |
| `clothcapsulemask` / `clothcapsulemaskdepth` | 布料 vs 自身胶囊的静息重叠屏蔽 |
| `objselfcol` / `objselfmargin` | CObject 与自身 col 的碰撞及余量 |
| `mapselfcol` / `mapselfbox` / `mapselfmargin` | 地图实例自身 col / 顶面托盘 / 余量 |
| `carmesh` / `carmargin` | 载具渲染网格碰撞 / 余量 |
| `buildingcol` / `buildingmargin` | 静态建筑碰撞 / 余量 |
| `selfcollide` | 载具是否与自身 col 碰撞 |
| `collsteps` | 世界碰撞的每帧采样次数（1~4） |
| `hairworldcollide` | 头发是否参与世界碰撞（0 = 关闭） |

---

### 7. 已知的空白（不是 bug，是尚未实现）

1. **布料 ↔ 布料** 无自碰撞（裙片之间、内外层之间会互穿）；
2. **SoftBody** 不参与世界碰撞（只与自身骨胶囊碰）；
3. **Body** 完全无碰撞；
4. 其他地图实例的**渲染网格**不作为碰撞体（只有 col）；
5. 三角面/盒/球的碰撞以**帧末位置**为主，高速穿透仍有残余（胶囊有扫掠，其余没有）。
