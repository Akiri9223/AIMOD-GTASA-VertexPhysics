# 修复记录：Body 软组织移动时乱抖（2026-11-23 04:00）—— ✅ 已修好

> **状态：用户进游戏确认「抖动问题修好了」（2026-11-23 04:02）。**
> 基线：03:10 版（头发全部修复、布料保持原样）。本文档只讲 Body 弹簧这一项。
>
> 改动文件：`src/physics/GeometryPhysics.cpp`（Body 分组弹簧段）、`GeometryPhysics.h`（`BodyGroupState::accSmooth` + 诊断字段）
> 构建：`VertexPhysics.asi` 1,375,744 B / 2026-11-23 04:03:33（含每 rig 诊断修正）
> ini：**未改动**（仍是 03:10 那份，8 个 `hair*` 键）

## 1. `Body` 与 `SoftBody` 是两套东西（先分清）

* `PhysType::Body`：**分组弹簧**。同组顶点共用一个位移 `state.offset`，组间靠 `jiggleWeight` 过渡。
  系统状态在 `bodyGroups`（`BodyGroupState`：`prev / anchorVel / offset / vel`）。
* `PhysType::SoftBody`：走普通粒子管线（世界空间 Verlet + 约束），完全不经过分组弹簧。
* 本模型的日志是 `body(167) soft(0)` ⇒ **没有 SoftBody**，抖动只可能来自分组弹簧。
  （`[SUCCESS] ... body=167 soft=0`）

## 2. 三个真因（按贡献排序）

### 2.1 驱动加速度是骨骼矩阵的二阶差分，没滤波也没限幅

```cpp
const RwV3d anchorVel = (anchorAvg - state.prev) / dt;
const RwV3d anchorAcc = (anchorVel - state.anchorVel) / dt;   // ← 原始差分
...
const RwV3d drive = V3Add(anchorAcc, V3Scale(rootAcc, 0.8f));
acc -= drive * 1.30f;
```

骨骼矩阵每帧都在变（30fps 时 dt≈33ms），二阶差分里混着大量高频噪声——
头发那边实测同一个量能到 **数百 m/s²**。这里没有低通、没有限幅，
噪声被 ×1.3 直接灌进弹簧 ⇒ **高频乱抖**。

### 2.2 `rootAcc` 与 `anchorAcc` 重复计

`anchorAcc` 来自**世界空间**的骨骼矩阵，本身已经包含"整机平移/旋转"（根节点运动）；
再加 `0.8 * rootAcc` 等于把同一次冲击算了 **1.8 次**，起步/落脚时更容易激发振荡。

### 2.3 显式欧拉 + 每帧可变 dt，接近稳定边界

`k = 30 + 120*soft`（最大 150 ⇒ ω≈12.2 rad/s），`c = 1.5 + 5*soft`（ζ≈0.19，很弹）。
dt=45ms（22fps）时 `dt·ω ≈ 0.55`，显式欧拉已经贴近稳定极限；帧率一抖，
弹簧响应就跟着变，和噪声叠加就是"嗡嗡"感。

### 2.4 顺带：幅度钳制的实现方式会制造锯齿

```cpp
if (offSq > amp*amp) { offset = clamp到球面; vel *= 0.5f; }   // ← 钳制量与速度不同步
```
每帧"推到边界 → 速度砍半 → 再被驱动推出去"，形成锯齿（同一个"几何修正变成速度"的家族问题）。

## 3. 修法（改的都是 `StepGeometryPhysics` 的 Body 分组段）

1. **驱动滤波 + 限幅**：`accSmooth = lerp(accSmooth, anchorAcc, 0.35)`，再限幅 **25 m/s²**。
   保留真实起步/急停/落脚的冲击，滤掉动画噪声。
2. **去掉重复计**：`drive = accSmooth`，不再叠加 `rootAcc`。
3. **固定子步积分**：弹簧按 **1/120s** 子步推进（每帧 1~4 步，内部 dt 恒定），
   刚度/阻尼响应不再随帧率变化。只影响分组弹簧，不动 `prevWorld`、不动粒子管线。
4. **传送/瞬移保护**：单帧整组锚点位移 > 0.5m ⇒ 清零 `offset/vel/accSmooth`，不当加速度用
   （重生、换模型、镜头切换都会触发）。
5. **幅度钳制改成切向投影**：钳到球面后只去掉**向外**的速度分量，不再砍半。

参数一个没动（`k`、`c`、`driveScale`、`amp` 系数全部保持原值）。

## 4. 新增诊断

```
[VER_82][BodySpring] dt=..ms groups=.. drive=..m/s2 offset=..m vel=..m/s resets=..
```

* `drive`：限幅后的锚点加速度。正常走动 < 10；急停/落脚 15~25。
* `offset`：当前最大位移（正常 ≤ 0.07 = amp 上限）。
* `vel`：最大速度。**正常 < 1.0；抖动时会冒到 2~5**。
* `resets`：本帧被判定为瞬移而清零的组数（正常应为 0；频繁 >0 说明锚点在某些帧跳变）。

## 5. 验收标准

| 场景 | 期望 |
|---|---|
| 静止 | 不抖；`drive≈0`、`vel<0.05` |
| 走动 | 轻微跟随；`drive` 个位数 |
| 跑步 | 有弹性延迟；`drive` 10~20 |
| 急停 | 一次自然回弹，不持续振 |
| 全程 | `vel` 不超过 ~1.5；`[BodySpring]` 不出现 3~5 的尖峰 |

## 6. 下一步（若仍有残留抖动）

1. 看 `[BodySpring]` 的 `vel` 峰值：仍 > 2 ⇒ 驱动还是太猛，把 `kMaxBodyDrive` 降到 15；
2. `resets` 频繁 ⇒ 锚点在跳，问题在 `SyncAnimPose`/骨骼矩阵更新时机，不在弹簧；
3. 若只有某个部位抖 ⇒ 看 `[BodyGroups]` 的分组数，可能是不同骨骼区域被并到了同一组。
4. 真正的**全局固定时间步长**（accumulator + 1/60 子步，同步 anchorWorld/prevWorld/风时间）
   是更彻底的做法，但会影响所有子系统，建议单独一轮做、并保留 A/B 开关。

---

## 7. 实测确认（2026-11-23 04:02）

用户进游戏测试后确认：**移动时身体不再乱抖**。

### 7.1 诊断口径的一个坑（已修）

`[BodySpring]` 最初用**全局** `g_stepFrame % 120` 触发。场景里同时存在多个 rig
（人物 + 旗帜）、`g_stepFrame` 每帧被多个 rig 各自 +1，于是 `%120==0` 只会命中**其中一个** rig
——日志里出现的全是那个静止 rig（`drive=0`），人物的弹簧状态根本没被采样。
现已改成**每 rig 自己的 `bodyDiagFrame`**，并在行首加 `n=<body粒子数>` 便于区分：

```
[VER_82][BodySpring] dt=..ms n=167 groups=.. drive=..m/s2 offset=..m vel=..m/s resets=..
```

### 7.2 仍需确认的一点（手感，不是 bug）

抖动压住之后，要区分两种"不抖"：

1. **弹簧仍在动**（正确）：走动轻微跟随、跑步有弹性延迟、急停一次回弹；
2. **弹簧被压死**（不对）：身体完全没有任何跟随感。

判据看日志 `n=167` 那一行的 `offset` / `vel`：
* 走动时 `offset` 应该有几个 mm ~ 2cm、`vel` 0.05~0.5；
* 若 `offset` 恒为 `0.000`、`vel` 恒为 `0.000`（人物在动的前提下）⇒ 驱动力被滤得太狠，
  把 `kMaxBodyDrive` 从 25 调回 40，或把低通系数 0.35 提到 0.5。

参数位置都在 `StepGeometryPhysics` 的 Body 分组段：
`0.35f`（低通）、`kMaxBodyDrive = 25.0f`（限幅）、`kSpringStep = 1/120`（子步）。

---

## 8. 顺带记录：为什么"固定时间步长"这次只做了弹簧局部版

另一个 AI 的方案里 §6 建议整体改固定时间步长（accumulator + 1/60 子步，同步
`anchorWorld` / `anchorPrevWorld` / `prevWorld` / `BodyGroupState` / 风时间）。

这次只对 **Body 分组弹簧**做了 1/120s 固定子步，原因：

* 分组弹簧是**自包含**的常微分方程（4 个组、每组 2 个状态），子步化不影响任何其它子系统；
* 全局固定步长会改变头发/布料的 transport、`prevWorld`、风时间推进，风险面大很多；
* 抖动的主因是**驱动噪声**（已用低通+限幅解决），帧率依赖只是次因。

全局固定步长仍然值得做（它会让所有子系统的行为与帧率脱钩），但应当**单独一轮**，
并且带 A/B 开关（`fixedstep=0/1`），与手感改动分开验证。
