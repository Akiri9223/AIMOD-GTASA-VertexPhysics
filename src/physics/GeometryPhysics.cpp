#include "GeometryPhysics.h"
#include "common.h"
#include "WorldCollision.h"

#include "rpskin.h"
#include "rphanim.h"
#include "ePedBones.h"
#include "CWeather.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <set>
#include <sstream>
#include <string>
#include <tuple>
#include <vector>

namespace
{
    constexpr int kMaxBones = 64;

    // 挂点约束开关。
    //   true  = 物理片与不动网格之间建连杆（披风/头发因此被吊住）
    //   false = 完全不建连杆，所有物理顶点只受彼此约束 + 重力（用于对照测试）
    constexpr bool kEnablePins = true;

    void GeoPhysLog(const std::string& text)
    {
        // 2026-11-24 12:5x：以前每条日志都 GetModuleFileNameA + 开/关文件，
        // 换装/建 rig 时几万条日志 → 几万次文件开关，游戏会卡死。
        // 现在只开一次，常驻句柄写入。
        static std::ofstream s_log;
        static bool s_init = false;
        static int s_pendingFlush = 0;

        if (!s_init)
        {
            s_init = true;

            char exePath[MAX_PATH] = {};
            DWORD length = GetModuleFileNameA(nullptr, exePath, MAX_PATH);

            std::string path;

            if (length > 0 && length < MAX_PATH)
            {
                std::string tmp(exePath, length);
                size_t pos = tmp.find_last_of("\\/");
                if (pos != std::string::npos)
                {
                    path = tmp.substr(0, pos + 1);
                }
            }

            path += "vertexphysics_physics.log";

            s_log.open(path.c_str(), std::ios::app);
        }

        if (!s_log.is_open()) return;

        s_log << text << "\n";

        // 攒几行再 flush，兼顾性能和崩溃后还能看到日志
        if (++s_pendingFlush >= 16)
        {
            s_pendingFlush = 0;
            s_log.flush();
        }
    }

    // 逐顶点颜色诊断（[ColorScan] Vertex / [ColorHist]）。
    // 默认关闭：一个 rig 就可能刷出几千行，写文件会把游戏拖卡。
    bool g_colorDiag = false;

    // ---- Body（R=0xC0 身体抖动）独立手感旋钮（ini: bodyk/bodyc/bodydrive/bodyamp）----
    // 0 = 走原公式（由顶点色 G 推导）；>0 = 直接覆盖
    float g_bodyK = 0.0f;        // 弹簧刚度 k（越大越快）
    float g_bodyC = 0.0f;        // 阻尼 c（越小回弹次数越多）
    float g_bodyDrive = 1.30f;   // 骨骼加速度驱动倍率（越大抖动越明显）
    float g_bodyAmp = 1.0f;      // 幅度倍率（在 B 通道基础上再乘）

    // ---- Body 分组（ini: bodygroup / bodygroupsplit / bodyrandom / bodysmooth）----
    //
    // 2026-11-24 15:4x：Body 的位移是"每组一个刚性 offset"，原来组键是
    // dominantBone —— 胸/腹都挂同一根脊椎骨 ⇒ 必定同向。
    //   0 = piece  ：每个连通片一组（左胸/右胸/腹各自一份弹簧）
    //   1 = bone   ：按 dominantBone（旧行为）
    //   2 = cluster：片内按锚点位置 k-means 分簇（左右胸+腹 = 3）
    int   g_bodyGroupMode = 0;
    int   g_bodyGroupSplit = 3;
    float g_bodyRandom = 0.25f;  // 每组驱动方向随机旋转强度（0=完全同步）
    int   g_bodySmooth = 3;      // 邻域平滑次数（原 12，越小越"各自为政"）

    // 每粒子每帧的扫掠碰撞子步上限（ini: collsteps，默认 2，范围 1~4）。
    // 只影响"单帧位移很大"时的防穿墙采样精度，不影响重力/约束/响应。
    int g_collisionSubsteps = 2;

    RwV3d V3Add(const RwV3d& a, const RwV3d& b) { return { a.x + b.x, a.y + b.y, a.z + b.z }; }
    RwV3d V3Sub(const RwV3d& a, const RwV3d& b) { return { a.x - b.x, a.y - b.y, a.z - b.z }; }
    RwV3d V3Scale(const RwV3d& v, float s) { return { v.x * s, v.y * s, v.z * s }; }
    float V3Dot(const RwV3d& a, const RwV3d& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
    float V3LenSq(const RwV3d& v) { return v.x * v.x + v.y * v.y + v.z * v.z; }
    float V3Len(const RwV3d& v) { return std::sqrt(V3LenSq(v)); }

    RwV3d TransformPoint(const RwMatrix& m, const RwV3d& p)
    {
        return
        {
            p.x * m.right.x + p.y * m.up.x + p.z * m.at.x + m.pos.x,
            p.x * m.right.y + p.y * m.up.y + p.z * m.at.y + m.pos.y,
            p.x * m.right.z + p.y * m.up.z + p.z * m.at.z + m.pos.z
        };
    }

    RwV3d TransformDir(const RwMatrix& m, const RwV3d& v)
    {
        return
        {
            v.x * m.right.x + v.y * m.up.x + v.z * m.at.x,
            v.x * m.right.y + v.y * m.up.y + v.z * m.at.y,
            v.x * m.right.z + v.y * m.up.z + v.z * m.at.z
        };
    }

    // RenderWare 行向量约定：p' = p * M，M 的行分别是 right/up/at。
    //
    // 行向量矩阵的逆：
    //   M^-1 的行 = ( (b x c).x, (c x a).x, (a x b).x )
    //               ( (b x c).y, (c x a).y, (a x b).y )
    //               ( (b x c).z, (c x a).z, (a x b).z )
    // 也就是把三个叉积按"列"排，而不是按行排。
    // （之前错写成按行排 = 转置，求逆结果不是逆矩阵，
    //   导致写回位置整体偏 ~29cm，且偏差在模型空间里，随头转动。）
    bool InvertMatrix(const RwMatrix& m, RwMatrix& out)
    {
        const RwV3d& a = m.right;
        const RwV3d& b = m.up;
        const RwV3d& c = m.at;

        RwV3d bc = { b.y * c.z - b.z * c.y, b.z * c.x - b.x * c.z, b.x * c.y - b.y * c.x };
        RwV3d ca = { c.y * a.z - c.z * a.y, c.z * a.x - c.x * a.z, c.x * a.y - c.y * a.x };
        RwV3d ab = { a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x };

        float det = a.x * bc.x + a.y * bc.y + a.z * bc.z;
        if (std::fabs(det) < 1e-8f) return false;

        float inv = 1.0f / det;

        out.right = { bc.x * inv, ca.x * inv, ab.x * inv };
        out.up    = { bc.y * inv, ca.y * inv, ab.y * inv };
        out.at    = { bc.z * inv, ca.z * inv, ab.z * inv };

        out.pos.x = -(out.right.x * m.pos.x + out.up.x * m.pos.y + out.at.x * m.pos.z);
        out.pos.y = -(out.right.y * m.pos.x + out.up.y * m.pos.y + out.at.y * m.pos.z);
        out.pos.z = -(out.right.z * m.pos.x + out.up.z * m.pos.y + out.at.z * m.pos.z);

        out.flags = rwMATRIXTYPEORTHONORMAL;
        return true;
    }

    // 自检：M 作用到 p 再被 M^-1 作用回来，应该回到 p
    float InverseResidual(const RwMatrix& m, const RwMatrix& inverse, const RwV3d& p)
    {
        const RwV3d forward = TransformPoint(m, p);
        const RwV3d back    = TransformPoint(inverse, forward);
        return V3Len(V3Sub(back, p));
    }

    // 每种类型的重力比例与"离锚点最大允许距离"
    float GravityScale(PhysType t)
    {
        switch (t)
        {
        case PhysType::Hair:     return 1.00f;
        case PhysType::Cloth:    return 1.00f;   // 布料实际倍率由 rig.clothGravity 决定
        case PhysType::Body:     return 0.08f;   // 身体组织：几乎不受重力，否则会下坠拉成尖
        case PhysType::SoftBody: return 0.15f;
        default:                 return 1.00f;
        }
    }

    float MaxAnchorDistance(PhysType t)
    {
        switch (t)
        {
        case PhysType::Hair:     return 2.00f;
        case PhysType::Cloth:    return 1.50f;
        case PhysType::Body:     return 0.10f;
        case PhysType::SoftBody: return 0.25f;
        default:                 return 2.00f;
        }
    }

    float ClosestSegSq(const RwV3d& p, const RwV3d& a, const RwV3d& b, RwV3d& closest)
    {
        RwV3d ab = V3Sub(b, a);
        RwV3d ap = V3Sub(p, a);
        float ll = V3LenSq(ab);

        if (ll < 1e-10f)
        {
            closest = a;
            return V3LenSq(ap);
        }

        float t = V3Dot(ap, ab) / ll;
        t = std::clamp(t, 0.0f, 1.0f);
        closest = V3Add(a, V3Scale(ab, t));
        return V3LenSq(V3Sub(p, closest));
    }

    // 点正好落在胶囊轴线上时没有"最近表面方向"，随便挑一个会沿固定 +X 推，
    // 竖直的大腿就会把布料横着弹开。这里取一个与轴垂直的稳定方向。
    RwV3d PerpendicularTo(const RwV3d& a, const RwV3d& b)
    {
        RwV3d axis = V3Sub(b, a);
        const float len = V3Len(axis);

        if (len > 1e-6f)
            axis = V3Scale(axis, 1.0f / len);
        else
            axis = { 0.0f, 0.0f, 1.0f };

        const RwV3d helper = (std::fabs(axis.z) < 0.9f)
            ? RwV3d{ 0.0f, 0.0f, 1.0f }
            : RwV3d{ 1.0f, 0.0f, 0.0f };

        RwV3d n = V3Sub(helper, V3Scale(axis, V3Dot(helper, axis)));
        const float nl = V3Len(n);

        if (nl > 1e-6f)
            return V3Scale(n, 1.0f / nl);

        return { 1.0f, 0.0f, 0.0f };
    }

    // 把点推出胶囊表面。返回推出去的距离（0 = 本来就在外面）。
    // outNormal 可选：输出推出方向（从胶囊轴指向点），供最终速度修正使用。
    float ResolveCapsule(RwV3d& p, const RwV3d& a, const RwV3d& b, float r,
                         RwV3d* outNormal = nullptr)
    {
        if (outNormal) *outNormal = { 0.0f, 0.0f, 0.0f };

        if (r <= 0.0f) return 0.0f;

        RwV3d closest;
        const float d2 = ClosestSegSq(p, a, b, closest);

        if (d2 >= r * r) return 0.0f;

        const float d = std::sqrt(std::max(d2, 0.0f));

        const RwV3d n = (d > 1e-5f)
            ? V3Scale(V3Sub(p, closest), 1.0f / d)
            : PerpendicularTo(a, b);

        p = V3Add(closest, V3Scale(n, r));

        if (outNormal) *outNormal = n;

        return r - d;
    }

    void CollideCapsule(RwV3d& p, const RwV3d& a, const RwV3d& b, float r)
    {
        ResolveCapsule(p, a, b, r);
    }

    // 胶囊在 k∈[0,1] 时刻的端点。
    // 这一帧胶囊**自己也在运动**（大腿绕髋旋转时是转，不是平移），
    // 扫掠采样必须在每个时刻用当时的胶囊，而不是全程用本帧末端的胶囊。
    void CapsuleAt(const PhysicsCapsule& cap, float k, RwV3d& a, RwV3d& b)
    {
        a = V3Add(cap.aPrevWorld, V3Scale(V3Sub(cap.aWorld, cap.aPrevWorld), k));
        b = V3Add(cap.bPrevWorld, V3Scale(V3Sub(cap.bWorld, cap.bPrevWorld), k));
    }

    // ============================================================
    // 扫掠碰撞（CCD）：粒子这一帧的轨迹 vs 胶囊这一帧扫过的体积
    //
    // 旧版有两个会直接导致"跑步穿模"的问题：
    //   1) 全程用本帧末端的胶囊做距离测试。大腿快摆时胶囊是**转**的，
    //      末端姿态和 1/3 帧前的姿态相差很大，采样点全落在错误位置。
    //   2) 调用方先算 `relative = (pos-prev) - (cap.aWorld-aPrevWorld)`，
    //      布料跟着身体走时这一项≈0，于是几乎永远走不到扫掠
    //      （日志里 [CCD] hits 长期为 0，等于 CCD 根本没生效）。
    //
    // 现在的做法：
    //   * 子步数按 max(胶囊端点位移, 粒子位移) 决定，不再只看粒子；
    //   * 每个子步用插值后的胶囊端点做"推出"；
    //   * 粒子在一个"跟着无碰撞路径走"的累加位置上被逐步推出，
    //     所以它是沿身体表面滑过去的，不会被弹回去，也不会停在半路。
    // ============================================================
    int CollideCapsuleSwept(RwV3d& pos, const RwV3d& prev,
                            const PhysicsCapsule& cap,
                            RwV3d& outNormal,
                            float margin = 0.0f)
    {
        const float r = cap.radius + margin;
        if (r <= 0.0f) return 0;

        const RwV3d a0 = cap.aPrevWorld;
        const RwV3d b0 = cap.bPrevWorld;

        const float capMove = std::max(
            V3Len(V3Sub(cap.aWorld, a0)),
            V3Len(V3Sub(cap.bWorld, b0)));

        // 传送 / 重生：胶囊一帧飞了一米以上，当成瞬移处理（否则布料会被甩出去）
        if (capMove > 1.2f) return 0;

        const RwV3d pMove = V3Sub(pos, prev);
        const float travel = V3Len(pMove);

        // 粗剔除：起点和终点都够不着，而且这一帧也走不到
        RwV3d tmp;
        const float d0 = ClosestSegSq(prev, a0, b0, tmp);
        const float d1 = ClosestSegSq(pos, cap.aWorld, cap.bWorld, tmp);
        const float reach = capMove + travel + 0.01f;

        if (std::min(d0, d1) > r + reach) return 0;

        const float stepLen = std::max(r * 0.4f, 0.008f);
        const int steps = std::clamp(
            1 + static_cast<int>(std::max(capMove, travel) / stepLen), 1, 16);

        const float r2 = r * r;

        // 起点先推出：上一帧结束时腿可能已经压上来了
        RwV3d q = prev;
        ResolveCapsule(q, a0, b0, r);

        RwV3d prevTarget = prev;
        bool haveNormal = false;
        int hits = 0;

        for (int i = 1; i <= steps; ++i)
        {
            const float k = static_cast<float>(i) / static_cast<float>(steps);

            // 无碰撞路径上的位置：这里只累加"该往前走多少"
            const RwV3d target = V3Add(prev, V3Scale(pMove, k));
            q = V3Add(q, V3Sub(target, prevTarget));
            prevTarget = target;

            RwV3d A, B;
            CapsuleAt(cap, k, A, B);

            RwV3d closest;
            const float d2 = ClosestSegSq(q, A, B, closest);

            if (d2 >= r2) continue;

            const float d = std::sqrt(std::max(d2, 0.0f));

            const RwV3d n = (d > 1e-5f)
                ? V3Scale(V3Sub(q, closest), 1.0f / d)
                : PerpendicularTo(A, B);

            q = V3Add(closest, V3Scale(n, r));

            if (!haveNormal)
            {
                outNormal = n;
                haveNormal = true;
            }

            ++hits;
        }

        pos = q;
        return hits;
    }

    // ============================================================
    // G 通道标定表：0 = 最硬 -> 255 = 最软
    // 线性插值，包含两端共 5 个采样点
    // ============================================================

    struct SoftnessSample
    {
        float g;
        float stiffness;
        float returnStrength;
    };

    constexpr SoftnessSample kSoftness[5] =
    {
        {   0.0f, 1.00f, 0.50f },
        {  64.0f, 0.77f, 0.38f },
        { 128.0f, 0.55f, 0.26f },
        { 192.0f, 0.32f, 0.14f },
        { 255.0f, 0.10f, 0.02f },
    };

    void LookupSoftness(float g, float& stiffness, float& returnStrength)
    {
        if (g <= kSoftness[0].g)
        {
            stiffness = kSoftness[0].stiffness;
            returnStrength = kSoftness[0].returnStrength;
            return;
        }

        if (g >= kSoftness[4].g)
        {
            stiffness = kSoftness[4].stiffness;
            returnStrength = kSoftness[4].returnStrength;
            return;
        }

        for (int i = 0; i < 4; ++i)
        {
            if (g <= kSoftness[i + 1].g)
            {
                const float span = kSoftness[i + 1].g - kSoftness[i].g;
                const float t = (span > 1e-6f) ? (g - kSoftness[i].g) / span : 0.0f;

                stiffness = kSoftness[i].stiffness +
                    (kSoftness[i + 1].stiffness - kSoftness[i].stiffness) * t;

                returnStrength = kSoftness[i].returnStrength +
                    (kSoftness[i + 1].returnStrength - kSoftness[i].returnStrength) * t;
                return;
            }
        }
    }

    // ============================================================
    // 每顶点骨骼混合矩阵
    //
    //   M_v = sum_i ( w_i * (inverseBind_i * bone_i) )
    //
    // 引擎蒙皮算的是 sum_i ( w_i * (p * C_i) )，而仿射组合对 p 是线性的，
    // 所以 sum_i (w_i * (p * C_i)) == p * M_v 完全成立。
    // 用"主导骨骼"代替全部权重、再乘世界矩阵，会让写回点和渲染点不一致，
    // 顶点权重越平均（肩、发根、布料接缝）裂得越明显——这就是头发和发根断开的根因。
    // ============================================================

    bool BlendVertexMatrix(
        const RwMatrix* boneMatrices,
        const RwMatrix* inverseBinds,
        const VertexParticle& p,
        RwUInt32 maxBone,
        RwMatrix& out)
    {
        float weights[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
        float sum = 0.0f;

        for (int k = 0; k < 4; ++k)
        {
            const int bone = p.bones[k];
            if (bone < 0 || bone >= static_cast<int>(maxBone)) continue;
            if (p.weights[k] <= 0.0f) continue;

            weights[k] = p.weights[k];
            sum += weights[k];
        }

        if (sum < 1e-6f) return false;

        RwMatrix acc;
        std::memset(&acc, 0, sizeof(acc));

        for (int k = 0; k < 4; ++k)
        {
            if (weights[k] <= 0.0f) continue;

            const int bone = p.bones[k];

            RwMatrix combined;
            RwMatrixMultiply(&combined, &inverseBinds[bone], &boneMatrices[bone]);

            const float w = weights[k] / sum;

            acc.right.x += combined.right.x * w;
            acc.right.y += combined.right.y * w;
            acc.right.z += combined.right.z * w;

            acc.up.x += combined.up.x * w;
            acc.up.y += combined.up.y * w;
            acc.up.z += combined.up.z * w;

            acc.at.x += combined.at.x * w;
            acc.at.y += combined.at.y * w;
            acc.at.z += combined.at.z * w;

            acc.pos.x += combined.pos.x * w;
            acc.pos.y += combined.pos.y * w;
            acc.pos.z += combined.pos.z * w;
        }

        acc.flags = rwMATRIXTYPEORTHONORMAL;
        out = acc;
        return true;
    }

    // 混合矩阵的线性部分正常应该是"平均后的旋转"，长度约等于 1。
    // 如果权重分布极端（或某些骨骼矩阵异常），平均后长度会明显变小，
    // 这时求逆会把厘米级位移放大成几十厘米的尖刺——必须退回主导骨骼。
    bool BlendIsSane(const RwMatrix& m)
    {
        const float scale =
            (V3Len(m.right) + V3Len(m.up) + V3Len(m.at)) / 3.0f;

        const float det =
            m.right.x * (m.up.y * m.at.z - m.up.z * m.at.y) +
            m.right.y * (m.up.z * m.at.x - m.up.x * m.at.z) +
            m.right.z * (m.up.x * m.at.y - m.up.y * m.at.x);

        return scale > 0.60f && scale < 1.60f && det > 0.05f;
    }

    bool BuildVertexMatrix(
        const RwMatrix* boneMatrices,
        const RwMatrix* inverseBinds,
        const VertexParticle& p,
        RwUInt32 maxBone,
        RwMatrix& out,
        bool* usedFallback)
    {
        if (usedFallback) *usedFallback = false;

        // 4 骨骼混合理论上和引擎蒙皮完全等价，但它对权重的数值非常敏感：
        // 一旦某根骨骼权重偏大、或者骨骼索引对不上，平均矩阵会严重失真，
        // 用它求逆就会把厘米级位移放大成几十厘米的拉伸（表现为"头发围着
        // 头转着被拉长"）。这里默认只用主导骨骼：正交矩阵，长度守恒，
        // 位移绝不会被放大。
        constexpr bool kUseBlendedSkinning = true;

        if (kUseBlendedSkinning)
        {
            RwMatrix blended;
            if (BlendVertexMatrix(boneMatrices, inverseBinds, p, maxBone, blended) &&
                BlendIsSane(blended))
            {
                out = blended;
                return true;
            }
        }

        const int dominant = p.dominantBone;

        if (dominant >= 0 && dominant < static_cast<int>(maxBone))
        {
            RwMatrixMultiply(&out, &inverseBinds[dominant], &boneMatrices[dominant]);
            if (usedFallback) *usedFallback = true;
            return true;
        }

        return false;
    }

    // ============================================================
    // 从顶点色读取物理参数
    //
    //   R = 类型（0x00/0x40/0x80/0xC0/0xFF，0x03 为旧版兼容）
    //   G = 软度（0 硬 -> 255 软）
    //   B = 锚定强度（0 固定 -> 255 自由）
    // ============================================================

} // namespace


bool ClassifyVertexColor(const RwRGBA& color, VertexPhysParams& out)
{
    PhysType type = PhysType::Rigid;
    bool legacy = false;

    // 模型预光照色里大量是纯白 / 纯灰 (255,255,255)、(128,128,128)…
    // 这类颜色一律不当作物理标记，否则整个游戏世界都会被误判
    if (color.red == color.green && color.green == color.blue)
        return false;

    // R 通道带 ±4 容差：DTF 导出/压缩有时会把 128 写成 127 或 129，
    // 严格相等会让个别顶点"漏掉"，表现为披风/头发上零星几个顶点没有物理。
    const int r = static_cast<int>(color.red);

    auto nearValue = [r](int target) { return std::abs(r - target) <= 4; };

    if (r == 0) return false;                       // Rigid / 未标记

    if (nearValue(0x40))      type = PhysType::Hair;
    else if (nearValue(0x80)) type = PhysType::Cloth;
    else if (nearValue(0xC0)) type = PhysType::Body;
    // SoftBody 用 0xFE 而不是 0xFF：模型里大量"暖白"预光照色是 (255,220,180)，
    // R=255 且各通道不相等，会被误判成 SoftBody（日志里真的出现过 66 个假 SoftBody）。
    else if (r == 0xFE)       type = PhysType::SoftBody;
    else if (nearValue(0x03)) { type = PhysType::Hair; legacy = true; }
    else return false;                              // 不是物理标记

    out.type = type;

    if (legacy)
    {
        // 旧方案：G 是软度，B 没有含义。
        // 现在统一按"自由粒子"处理，由发根自动锚定（见 AutoAnchor）来提供固定点，
        // 否则旧标记会带着 3%/帧 的弱拉回，跑动时被拖成一片。
        out.stiffness = 0.95f;
        out.returnStrength = 0.005f + 0.025f * (1.0f - static_cast<float>(color.green) / 255.0f);
        out.anchorFactor = 0.0f;
        out.hardAnchor = false;
    }
    else
    {
        LookupSoftness(static_cast<float>(color.green), out.stiffness, out.returnStrength);
        out.anchorFactor = 1.0f - static_cast<float>(color.blue) / 255.0f;
        out.hardAnchor = (color.blue == 0);
    }

    // Body 的 B 只表示幅度（0 = 不抖），不能当作硬锚定：
    // 硬锚定会让整块完全不参与抖动，某半边被误标成 B=0 时就会左右不对称。
    // 要完全不动请用 R=0 (Rigid)。
    if (type == PhysType::Body)
        out.hardAnchor = false;

    return true;
}


// ============================================================
//  布料手感：全局默认值（ini 加载时写入，InitGeometryPhysics 时复制进 rig）
// ============================================================
namespace
{
    float g_clothGravityDefault = 1.0f;
    float g_clothFollowDefault  = 0.0f;
    float g_clothDampingDefault = 0.95f;
    float g_clothPinDefault     = 0.15f;
    float g_clothWindDefault    = 1.8f;
    float g_clothInertiaDefault = 0.60f;
    // 布料 vs 自身骨胶囊的安全距离（ini: characterclothselfmargin / 旧名 clothmargin）
    float g_clothMarginDefault  = 0.015f;

    // 人体胶囊的全局调参（ini 的 bodycolliderscale / bodycollidermargin）
    float g_bodyColliderScale  = 1.0f;
    float g_bodyColliderMargin = 0.01f;
}

void SetClothTuning(float gravityScale, float followStrength,
                    float dampingPer60, float pinStiffness, float windScale,
                    float inertia, float collisionMargin)
{
    g_clothGravityDefault = std::clamp(gravityScale, 0.0f, 3.0f);
    g_clothFollowDefault  = std::clamp(followStrength, 0.0f, 1.0f);
    g_clothDampingDefault = std::clamp(dampingPer60, 0.50f, 0.999f);
    g_clothPinDefault     = std::clamp(pinStiffness, 0.02f, 1.0f);
    g_clothWindDefault    = std::clamp(windScale, -10.0f, 10.0f);   // 允许负值 = 反向
    g_clothInertiaDefault = std::clamp(inertia, 0.0f, 0.95f);
    g_clothMarginDefault  = std::clamp(collisionMargin, 0.0f, 0.08f);
}

void SetBodyColliderTuning(float radiusScale, float radiusMargin)
{
    g_bodyColliderScale  = std::clamp(radiusScale, 0.5f, 2.0f);
    g_bodyColliderMargin = std::clamp(radiusMargin, 0.0f, 0.08f);
}

// 人物（蒙皮）布料风向翻转（2026-11-23 11:2x，ini: pedclothwindflip）
namespace
{
    bool g_pedClothWindFlip = true;
}

void SetPedClothWindFlip(int enabled)
{
    g_pedClothWindFlip = (enabled != 0);
}

// ============================================================
//  头发 A/B 诊断开关
// ============================================================
namespace
{
    bool g_hairPinsEnabled        = true;
    bool g_hairCapsuleMaskEnabled = true;

    // 布料版本（2026-11-24 17:3x）
    // 布料贴在臀部/大腿上时，静止姿态就已经在胶囊内部 ⇒ 每帧被硬推出去、
    // 又被锚点/约束拉回来，表现就是"臀部位置猛烈抖动"。
    // 这里对布料也做同样的"静息重叠屏蔽"，但阈值单独可调（布料更大些）。
    bool  g_clothCapsuleMaskEnabled = true;
    float g_clothCapsuleMaskDepth = 0.008f;   // 8mm 以上算建模重叠

    // 按部位跳过胶囊（ini: capskip，位掩码）
    //   1=骨盆球 2=髋连接/髋球 4=大腿 8=膝 16=小腿 32=躯干 64=颈头 128=手臂 256=脚
    int g_capsuleSkipMask = 0;

    // 胶囊半径的"按部位倍率"（ini: capsgroupscale<part>，默认 1.0）
    // 与 bodycolliderscale（整体倍率）叠加：最终 = 基础半径 × 整体 × 部位 × 索引
    std::map<int, float> g_capsuleGroupScale;

    // 胶囊半径的"按索引倍率"（ini: capsulescale=1.0,1.2,...）
    // 索引对应 [Colliders] radii= 的顺序（0-based）
    std::vector<float> g_capsuleIndexScale;
    bool g_hairRootStretchExempt  = true;
    bool g_hairPieceFixEnabled    = true;
    bool g_hairRotTransport       = true;
    bool g_hairInertiaEnabled     = true;

    // 头发跟随比例 / 阻尼 / 回拉（ini: hairride / hairdamping / hairpull）
    float g_hairRide    = 0.6f;
    float g_hairDamping = 0.98f;
    float g_hairPull    = 1.0f;
    float g_hairWind    = 1.0f;   // ini: hairwind
    float g_hairFollow  = 1.0f;   // ini: hairfollow

    // 拉伸上限（ini: hairstretch / hairstretchstep / clothstretch / clothstretchstep）
    bool  g_hairBend        = true;    // ini: hairbend
    float g_hairAcc         = 12.0f;   // ini: hairacc（伪惯性上限 m/s²）
    float g_hairDecay       = 0.6f;    // ini: hairdecay（冲量衰减）
    float g_hairRot         = 1.0f;    // ini: hairrot（旋转跟随 0~1）
    float g_hairStretch     = 1.35f;
    float g_hairStretchStep = 0.015f;
    float g_clothStretch     = 1.35f;
    float g_clothStretchStep = 0.015f;
    // 头发统一骨骼（ini: hairrigidskin，默认 1）
    //   1 = 整片头发 + 挂点都用同一根骨骼的矩阵（刚性，绝不拉伸）
    //   0 = 逐顶点混合（跟引擎蒙皮一致，但权重不一致时会拉扯）
    int   g_hairRigidSkin = 1;
    bool g_hairWorldCollide       = true;
    bool g_hairDiagnostics        = true;
}

void SetHairDebugOptions(int pinsEnabled, int capsuleMaskEnabled,
                         int rootStretchExempt, int pieceFixEnabled,
                         int rotTransport, int inertiaEnabled,
                         int worldCollide, int diagnostics)
{
    g_hairPinsEnabled        = (pinsEnabled != 0);
    g_hairCapsuleMaskEnabled = (capsuleMaskEnabled != 0);
    g_hairRootStretchExempt  = (rootStretchExempt != 0);
    g_hairPieceFixEnabled    = (pieceFixEnabled != 0);
    g_hairRotTransport       = (rotTransport != 0);
    g_hairInertiaEnabled     = (inertiaEnabled != 0);
    g_hairWorldCollide       = (worldCollide != 0);
    g_hairDiagnostics        = (diagnostics != 0);
}


// 抗弯约束：给"同一顶点两侧的邻居"之间补一条刚性距离约束。
// 只在初始化时调用一次（由 InitGeometryPhysics 末尾调用），
// 带安全上限，避免约束数量失控拖垮求解。
static void AddHairBendConstraints(GeometryPhysics& rig)
{
    if (!g_hairBend) return;
    if (rig.particles.empty()) return;

    constexpr size_t kMaxBend = 20000;
    constexpr size_t kMaxTotal = 60000;

    if (rig.constraints.size() >= kMaxTotal) return;

    std::vector<std::vector<int>> nb(rig.particles.size());

    for (const auto& c : rig.constraints)
    {
        if (c.a >= 0 && c.a < static_cast<int>(nb.size()) &&
            c.b >= 0 && c.b < static_cast<int>(nb.size()))
        {
            nb[c.a].push_back(c.b);
            nb[c.b].push_back(c.a);
        }
    }

    size_t added = 0;

    for (size_t i = 0; i < nb.size() && added < kMaxBend; ++i)
    {
        if (static_cast<PhysType>(rig.particles[i].typeU32) != PhysType::Hair) continue;
        if (nb[i].size() < 2) continue;

        for (size_t x = 0; x + 1 < nb[i].size() && added < kMaxBend; ++x)
        {
            for (size_t y = x + 1; y < nb[i].size() && added < kMaxBend; ++y)
            {
                const int a = nb[i][x];
                const int b = nb[i][y];

                if (a == b) continue;
                if (static_cast<PhysType>(rig.particles[a].typeU32) != PhysType::Hair) continue;
                if (static_cast<PhysType>(rig.particles[b].typeU32) != PhysType::Hair) continue;

                const float rl = V3Len(V3Sub(rig.particles[a].restLocal,
                                             rig.particles[b].restLocal));

                if (rl < 1e-4f || rl > 0.5f) continue;

                VertexConstraint c;
                c.a = a;
                c.b = b;
                c.restLength = rl;
                c.stiffness = 0.25f;

                rig.constraints.push_back(c);
                ++added;
            }
        }
    }

    std::ostringstream bb;
    bb << "[HairBend] added=" << added << " total=" << rig.constraints.size();
    GeoPhysLog(bb.str());
}

bool InitGeometryPhysics(
    GeometryPhysics& rig,
    RpGeometry* geometry,
    const RwMatrix* anchorLTM,
    std::string& report)
{
    GeoPhysLog("[VER_82] ==== InitGeometryPhysics ENTER ====");

    rig.initialized = false;
    rig.particles.clear();
    rig.constraints.clear();
    rig.colliders.clear();
    rig.colliderPrevValid = false;
    rig.capsuleMaskBuilt = false;
    rig.clothCapBuilt = false;
    rig.clothCapDepth.clear();
    rig.hairRebindDone = false;
    rig.hairTestLogged = false;
    rig.hairParticleSet.clear();
    rig.clothParticleSet.clear();
    rig.softParticleSet.clear();
    rig.bodyParticleSet.clear();
    rig.vertexToParticle.clear();
    rig.rigidCount = 0;
    rig.hairCount = 0;
    rig.clothCount = 0;
    rig.softCount = 0;
    rig.bodyCount = 0;
    rig.hasSkinAnchor = false;
    rig.skinBoneCount = 0;
    rig.boundingRadius = 0.0f;
    rig.bodyGroups.clear();

    rig.clothGravity = g_clothGravityDefault;
    rig.clothFollow  = g_clothFollowDefault;
    rig.clothDamping = g_clothDampingDefault;
    rig.clothPinStiffness = g_clothPinDefault;
    rig.clothWind = g_clothWindDefault;
    rig.clothInertia = g_clothInertiaDefault;
    rig.clothCollisionMargin = g_clothMarginDefault;

    // 头发 A/B 诊断开关（ini）
    rig.hairPinsEnabled        = g_hairPinsEnabled;
    rig.hairCapsuleMaskEnabled = g_hairCapsuleMaskEnabled;
    rig.hairRootStretchExempt  = g_hairRootStretchExempt;
    rig.hairPieceFixEnabled    = g_hairPieceFixEnabled;
    rig.hairRotTransport       = g_hairRotTransport;
    rig.hairInertiaEnabled     = g_hairInertiaEnabled;
    rig.hairWorldCollideEnabled= g_hairWorldCollide;
    rig.hairDiagnostics        = g_hairDiagnostics;

    // 头发跟随比例 / 阻尼 / 回拉（ini）
    rig.hairRide    = std::clamp(g_hairRide, 0.0f, 1.0f);
    rig.hairDamping = std::clamp(g_hairDamping, 0.50f, 0.999f);
    rig.hairPull    = std::clamp(g_hairPull, 0.0f, 3.0f);
    rig.hairWind    = std::clamp(g_hairWind, 0.0f, 5.0f);
    rig.hairFollow  = std::clamp(g_hairFollow, 0.0f, 1.0f);
    rig.hairStretch     = std::clamp(g_hairStretch, 1.0f, 3.0f);
    rig.hairStretchStep = std::clamp(g_hairStretchStep, 0.001f, 0.10f);
    rig.clothStretch     = std::clamp(g_clothStretch, 1.0f, 3.0f);
    rig.clothStretchStep = std::clamp(g_clothStretchStep, 0.001f, 0.10f);

    // 头发统一骨骼（P：修"靠近发根的头发被向后拉伸"）
    rig.hairRigidSkin = (g_hairRigidSkin != 0);

    if (!geometry || !geometry->morphTarget || geometry->numMorphTargets <= 0)
    {
        GeoPhysLog("[VER_82] ERROR: Invalid geometry");
        return false;
    }

    // 蒙皮是可选的：人物有骨骼，车辆 / 物体 / 建筑没有，
    // 后者用原子的世界矩阵当锚点（刚体跟随）
    RpSkin* skin = RpSkinGeometryGetSkin(geometry);

    RwV3d* verts = RpMorphTargetGetVertices(&geometry->morphTarget[0]);
    if (!verts)
    {
        GeoPhysLog("[VER_82] ERROR: Vertices are NULL");
        return false;
    }

    const int numVerts = RpGeometryGetNumVertices(geometry);
    const int numTris = RpGeometryGetNumTriangles(geometry);
    RpTriangle* tris = RpGeometryGetTriangles(geometry);

    if (numVerts <= 0 || numTris <= 0 || !tris)
    {
        GeoPhysLog("[VER_82] ERROR: Invalid geometry topology");
        return false;
    }

    RwRGBA* colors = geometry->preLitLum;

    // GTA 的 HAnim 骨骼矩阵是世界空间（日志里 BoneMatrix.pos.z≈14.1，而模型空间只有 0.7），
    // 因此锚点 = 世界空间，粒子初始化也必须用世界空间，两者才对得上。
    if (!anchorLTM)
    {
        GeoPhysLog("[VER_82] ERROR: anchor LTM is NULL");
        return false;
    }

    rig.boundingRadius = geometry->morphTarget[0].boundingSphere.radius;

    const RwMatrixWeights* skinWeights = skin ? RpSkinGetVertexBoneWeights(skin) : nullptr;
    const RwUInt32*        skinIndices = skin ? RpSkinGetVertexBoneIndices(skin) : nullptr;
    const RwUInt32         boneCount   = skin ? RpSkinGetNumBones(skin) : 0;
    const bool             skinSplit   = skin ? (RpSkinIsSplit(skin) != FALSE) : false;

    // ------------------------------------------------------------
    // 骨骼索引位宽探测（2026-11-24 17:1x）
    //
    // SA 的 skin 允许用 **8 位**骨骼索引（32 根骨时导出器常这么做），
    // 而 RpSkinGetVertexBoneIndices 返回的是 void 层面的指针：
    // 按 u32 读会把一个字节组（3,9,2）读成 133379 → 索引全部越界
    // ⇒ 日志里 94% 顶点"NoSkinWeight"，锚点错乱，布料被拉偏。
    //
    // 这里在初始化时用前 64 个顶点探测：如果 u32 读法几乎全部越界、
    // 而 u8 读法几乎全部合法，就判定为 8 位索引。
    // ------------------------------------------------------------
    bool indicesU8 = false;

    if (skinIndices && boneCount > 0 && numVerts > 0)
    {
        const RwUInt8* idx8Base = reinterpret_cast<const RwUInt8*>(skinIndices);

        // 关键判据：32 位索引的合法值一定 < numBones（通常 < 64）；
        // 若按 u32 读出来的最大值远超这个（比如 133379），说明数组其实是
        // 8 位存储（4 个 u8 被拼成一个 u32 读了）。
        unsigned int max32 = 0;
        unsigned int max8 = 0;

        const int probe = std::min(numVerts, 512);

        for (int v = 0; v < probe; ++v)
        {
            const RwUInt32* i32 = skinIndices + static_cast<size_t>(v) * 4;
            const RwUInt8*  i8  = idx8Base + static_cast<size_t>(v) * 4;

            for (int k = 0; k < 4; ++k)
            {
                if (i32[k] > max32) max32 = i32[k];
                if (i8[k]  > max8)  max8  = i8[k];
            }
        }

        if (max32 >= 256 && max8 < boneCount)
            indicesU8 = true;
    }

    if (skinWeights && skinIndices && boneCount > 0)
    {
        rig.hasSkinAnchor = true;
        rig.skinBoneCount = boneCount;
    }

    // ============================================================
    // ColorScan 诊断
    // ============================================================

    int legacyMarkers = 0;

    if (!colors)
    {
        GeoPhysLog("[ColorScan] ERROR: geometry->preLitLum is NULL");
    }
    else
    {
        int hairMarkers = 0;
        int clothMarkers = 0;
        int bodyMarkers = 0;
        int softMarkers = 0;
        int rigidOpaqueBlack = 0;
        int otherColors = 0;

        for (int v = 0; v < numVerts; ++v)
        {
            VertexPhysParams vp;
            const bool isPhys = ClassifyVertexColor(colors[v], vp);

            if (isPhys)
            {
                switch (vp.type)
                {
                case PhysType::Hair:     ++hairMarkers;  break;
                case PhysType::Cloth:    ++clothMarkers; break;
                case PhysType::Body:     ++bodyMarkers;  break;
                case PhysType::SoftBody: ++softMarkers;  break;
                default: break;
                }

                if (colors[v].red == 0x03) ++legacyMarkers;
            }
            else
            {
                const uint8_t r = colors[v].red;
                const uint8_t g = colors[v].green;
                const uint8_t b = colors[v].blue;
                const uint8_t a = colors[v].alpha;

                if (r == 0 && g == 0 && b == 0 && a == 255) ++rigidOpaqueBlack;
                else
                {
                    ++otherColors;

                    // 采样打印"看起来像刷了颜色、但没被识别"的顶点，
                    // 用来判断是不是通道被量化/压缩了
                    if (otherColors <= 12 && !(r == 0 && g == 0 && b == 0))
                    {
                        std::ostringstream oss;
                        oss << "[VER_82][Unclassified] vertex=" << v
                            << " RGBA=(" << static_cast<int>(r) << ","
                            << static_cast<int>(g) << ","
                            << static_cast<int>(b) << ","
                            << static_cast<int>(a) << ")";
                        GeoPhysLog(oss.str());
                    }
                }
            }

            if (g_colorDiag && (isPhys || (v < 16)))
            {
                std::ostringstream oss;
                oss << "[ColorScan] Vertex " << v
                    << " RGBA=(" << static_cast<int>(colors[v].red) << ","
                    << static_cast<int>(colors[v].green) << ","
                    << static_cast<int>(colors[v].blue) << ","
                    << static_cast<int>(colors[v].alpha) << ")";

                if (isPhys)
                {
                    const char* name =
                        vp.type == PhysType::Hair     ? "Hair" :
                        vp.type == PhysType::Cloth    ? "Cloth" :
                        vp.type == PhysType::Body     ? "Body" :
                                                        "SoftBody";

                    oss << " type=" << name
                        << " stiffness=" << vp.stiffness
                        << " return=" << vp.returnStrength
                        << " anchor=" << vp.anchorFactor;
                }

                GeoPhysLog(oss.str());
            }
        }

        std::ostringstream summary;
        summary << "[ColorScan] SUMMARY"
            << " vertices=" << numVerts
            << " hair=" << hairMarkers
            << " cloth=" << clothMarkers
            << " body=" << bodyMarkers
            << " soft=" << softMarkers
            << " rigidOpaqueBlack=" << rigidOpaqueBlack
            << " other=" << otherColors;
        GeoPhysLog(summary.str());

        // 颜色直方图：列出这块几何体里实际存在的所有颜色及数量。
        // 能一眼看出"某个颜色只覆盖了一部分顶点"或"颜色被量化成近邻值"。
        if (g_colorDiag)
        {
            std::map<uint32_t, int> histogram;

            for (int v = 0; v < numVerts; ++v)
            {
                const uint32_t key =
                    (static_cast<uint32_t>(colors[v].red) << 24) |
                    (static_cast<uint32_t>(colors[v].green) << 16) |
                    (static_cast<uint32_t>(colors[v].blue) << 8) |
                    static_cast<uint32_t>(colors[v].alpha);

                histogram[key] += 1;
            }

            std::vector<std::pair<int, uint32_t>> sorted;
            sorted.reserve(histogram.size());

            for (const auto& entry : histogram)
                sorted.push_back({ entry.second, entry.first });

            std::sort(sorted.begin(), sorted.end(),
                [](const std::pair<int, uint32_t>& a, const std::pair<int, uint32_t>& b)
                {
                    return a.first > b.first;
                });

            std::ostringstream hist;
            hist << "[VER_82][ColorHist] unique=" << histogram.size();

            for (size_t i = 0; i < sorted.size() && i < 12; ++i)
            {
                const uint32_t key = sorted[i].second;

                hist << " | (" << ((key >> 24) & 0xFF) << ","
                     << ((key >> 16) & 0xFF) << ","
                     << ((key >> 8) & 0xFF) << ","
                     << (key & 0xFF) << ")=" << sorted[i].first;
            }

            GeoPhysLog(hist.str());
        }

        if (legacyMarkers > 0)
        {
            std::ostringstream warn;
            warn << "[VER_82] WARNING: " << legacyMarkers
                << " vertices still use legacy marker R=3; "
                << "switch to R=0x40(64) Hair / R=0xC0(192) Body, B = anchor strength";
            GeoPhysLog(warn.str());
        }
    }

    // ============================================================
    // 顶点分类
    // ============================================================

    rig.vertexToParticle.assign(numVerts, -1);

    // 同位置的物理顶点必须合并成"同一个粒子"（这才是真正的焊接）：
    //   - 一个粒子只计算一次运动，不会互相覆盖（之前"后写的赢"）
    //   - 写回时把同一个位置写给名下所有顶点，所以不会裂开
    std::map<std::tuple<int, int, int>, int> particleByPosition;

    auto quantizePos = [](const RwV3d& pos)
    {
        return std::make_tuple(
            static_cast<int>(std::lround(pos.x * 10000.0f)),
            static_cast<int>(std::lround(pos.y * 10000.0f)),
            static_cast<int>(std::lround(pos.z * 10000.0f)));
    };

    int mergedPhysicsVertices = 0;

    for (int v = 0; v < numVerts; ++v)
    {
        VertexPhysParams vp;

        if (!colors || !ClassifyVertexColor(colors[v], vp))
        {
            ++rig.rigidCount;
            continue;
        }

        const auto posKey = quantizePos(verts[v]);
        auto existing = particleByPosition.find(posKey);

        if (existing != particleByPosition.end())
        {
            const int index = existing->second;
            auto& host = rig.particles[index];

            // 同一位置被刷成不同类型时，保留"更自由"的那个：
            // Body 让位给 Cloth / Hair / SoftBody
            if (static_cast<PhysType>(host.typeU32) == PhysType::Body &&
                vp.type != PhysType::Body)
            {
                --rig.bodyCount;

                switch (vp.type)
                {
                case PhysType::Hair:     ++rig.hairCount;  host.invMass = 1.0f; break;
                case PhysType::Cloth:    ++rig.clothCount; host.invMass = 1.1f; break;
                case PhysType::SoftBody: ++rig.softCount;  host.invMass = 1.4f; break;
                default: break;
                }

                host.typeU32 = static_cast<uint32_t>(vp.type);
                host.stiffness = (vp.type == PhysType::Cloth)
                    ? std::max(vp.stiffness, 0.55f) : vp.stiffness;
                host.returnStrength = vp.returnStrength;
                host.anchorFactor = std::clamp(vp.anchorFactor, 0.0f, 1.0f);
                host.hardAnchor = vp.hardAnchor;
            }

            host.weldVertices.push_back(v);
            rig.vertexToParticle[v] = index;
            ++mergedPhysicsVertices;
            continue;
        }

        VertexParticle p;
        p.geometryVertexIndex = v;
        p.restLocal = verts[v];

        // 世界空间：和骨骼锚点（inverseBind * boneMatrix）保持同一坐标系
        p.posWorld = rig.modelSpace ? p.restLocal : TransformPoint(*anchorLTM, p.restLocal);
        p.prevWorld = p.posWorld;
        p.anchorWorld = p.posWorld;

        p.typeU32 = static_cast<uint32_t>(vp.type);
        p.stiffness = vp.stiffness;
        p.returnStrength = vp.returnStrength;
        p.anchorFactor = std::clamp(vp.anchorFactor, 0.0f, 1.0f);
        p.hardAnchor = vp.hardAnchor;

        // 布料要保住面积：约束刚度设下限，否则大片布料会被重力拉成尖角
        if (vp.type == PhysType::Cloth)
            p.stiffness = std::max(p.stiffness, 0.55f);

        // Body 的 invMass 保持 0：不参与粒子约束，只受抖动弹簧驱动
        switch (vp.type)
        {
        case PhysType::Hair:     p.invMass = 1.0f; ++rig.hairCount;  break;
        case PhysType::Cloth:    p.invMass = 1.1f; ++rig.clothCount; break;
        case PhysType::Body:     p.invMass = 0.0f; ++rig.bodyCount;  break;
        case PhysType::SoftBody: p.invMass = 1.4f; ++rig.softCount;  break;
        default:                 p.invMass = 1.0f; break;
        }

        // 顶点骨骼权重（最多 4 根，和引擎蒙皮一致）
        if (skinWeights && skinIndices && boneCount > 0)
        {
            const RwMatrixWeights& w = skinWeights[v];

            const RwUInt32* idx32 = skinIndices + static_cast<size_t>(v) * 4;
            const RwUInt8*  idx8  = reinterpret_cast<const RwUInt8*>(skinIndices)
                                  + static_cast<size_t>(v) * 4;

            const float wArr[4] = { w.w0, w.w1, w.w2, skinSplit ? 0.0f : w.w3 };

            for (int k = 0; k < 4; ++k)
            {
                const RwUInt32 boneIdx = indicesU8 ? idx8[k] : idx32[k];

                if (wArr[k] > 1e-6f && boneIdx < boneCount)
                {
                    p.bones[k] = static_cast<int>(boneIdx);
                    p.weights[k] = wArr[k];

                    if (wArr[k] > p.dominantWeight)
                    {
                        p.dominantWeight = wArr[k];
                        p.dominantBone = static_cast<int>(boneIdx);
                    }
                }
            }
        }

        const int idx = static_cast<int>(rig.particles.size());
        rig.vertexToParticle[v] = idx;
        rig.particles.push_back(p);
        particleByPosition[posKey] = idx;

        switch (vp.type)
        {
        case PhysType::Hair:     rig.hairParticleSet.push_back(idx);  break;
        case PhysType::Cloth:    rig.clothParticleSet.push_back(idx); break;
        case PhysType::Body:     rig.bodyParticleSet.push_back(idx);  break;
        case PhysType::SoftBody: rig.softParticleSet.push_back(idx);  break;
        default: break;
        }
    }

    if (rig.particles.empty())
    {
        GeoPhysLog("[VER_82] ERROR: No physics particles");
        return false;
    }

    {
        std::ostringstream oss;
        oss << "[VER_82][Merge] mergedVertices=" << mergedPhysicsVertices
            << " particles=" << rig.particles.size();
        GeoPhysLog(oss.str());
    }

    // 诊断：没有有效骨骼权重的物理粒子（细分/新增顶点常见）
    {
        int noWeights = 0;
        std::ostringstream list;
        int shown = 0;

        for (const auto& p : rig.particles)
        {
            if (p.dominantBone >= 0) continue;

            ++noWeights;

            if (shown < 12)
            {
                list << " " << p.geometryVertexIndex;
                ++shown;
            }
        }

        std::ostringstream oss;
        oss << "[VER_82][NoSkinWeight] particles=" << noWeights
            << "/" << rig.particles.size()
            << " vertices=" << list.str();
        GeoPhysLog(oss.str());
    }

    // ------------------------------------------------------------
    // [SkinDiag] 抽样打印"原始"骨骼权重/索引
    //
    // 2026-11-24 17:0x：某份导出（18504）在 mod 里 94% 顶点读不到权重，
    // 但游戏自带蒙皮显示完全正常 ⇒ 需要确认是"文件里权重是 0"还是
    // "mod 的读取偏移/布局不对"。
    // 这里同时打印：几何体顶点数、骨骼数、split 标志、指针，
    // 以及若干布料顶点的 w0..w3 / idx0..idx3（原始值，不做任何加工）。
    // ------------------------------------------------------------
    {
        std::ostringstream head;
        head << std::fixed << std::setprecision(3);
        head << "[SkinDiag] geomVerts=" << numVerts
             << " bones=" << boneCount
             << " split=" << (skinSplit ? 1 : 0)
             << " idxFmt=" << (indicesU8 ? "u8" : "u32")
             << " weights=" << static_cast<const void*>(skinWeights)
             << " indices=" << static_cast<const void*>(skinIndices);

        GeoPhysLog(head.str());

        int printed = 0;

        for (const auto& p : rig.particles)
        {
            if (printed >= 8) break;

            // 优先看布料（裙子）顶点
            if (static_cast<PhysType>(p.typeU32) != PhysType::Cloth) continue;

            const int v = p.geometryVertexIndex;

            if (!skinWeights || !skinIndices || v < 0 || v >= numVerts) continue;

            const RwMatrixWeights& w = skinWeights[v];
            const RwUInt32* idx = skinIndices + static_cast<size_t>(v) * 4;
            const RwUInt8*  idx8dbg = reinterpret_cast<const RwUInt8*>(skinIndices)
                                    + static_cast<size_t>(v) * 4;

            std::ostringstream oss2;
            oss2 << std::fixed << std::setprecision(4);
            oss2 << "[SkinDiag] v=" << v
                 << " w=(" << w.w0 << "," << w.w1 << "," << w.w2 << "," << w.w3 << ")"
                 << " idx32=(" << idx[0] << "," << idx[1] << "," << idx[2] << "," << idx[3] << ")"
                 << " idx8=(" << static_cast<int>(idx8dbg[0]) << ","
                              << static_cast<int>(idx8dbg[1]) << ","
                              << static_cast<int>(idx8dbg[2]) << ","
                              << static_cast<int>(idx8dbg[3]) << ")"
                 << " bone=" << p.dominantBone
                 << " pos=(" << p.restLocal.x << "," << p.restLocal.y << "," << p.restLocal.z << ")";
            GeoPhysLog(oss2.str());

            ++printed;
        }
    }

    // 没有骨骼权重的顶点：继承"最近的、有权重的物理粒子"的权重。
    // 不能让它们走刚体兜底 —— 邻居走逐顶点蒙皮、它走刚体换算，
    // 两种结果不一致，顶点就会被拉开成尖刺。
    {
        int inherited = 0;

        for (auto& p : rig.particles)
        {
            if (p.dominantBone >= 0) continue;

            int best = -1;
            float bestDist = 1e9f;

            for (size_t j = 0; j < rig.particles.size(); ++j)
            {
                const auto& q = rig.particles[j];
                if (q.dominantBone < 0) continue;

                const float d2 = V3LenSq(V3Sub(q.restLocal, p.restLocal));
                if (d2 < bestDist) { bestDist = d2; best = static_cast<int>(j); }
            }

            if (best < 0) continue;

            const auto& q = rig.particles[best];

            for (int k = 0; k < 4; ++k)
            {
                p.bones[k] = q.bones[k];
                p.weights[k] = q.weights[k];
            }

            p.dominantBone = q.dominantBone;
            p.dominantWeight = q.dominantWeight;
            ++inherited;
        }

        std::ostringstream oss;
        oss << "[VER_82][WeightInherit] inherited=" << inherited;
        GeoPhysLog(oss.str());
    }

    // ============================================================
    // 顶点焊接
    //
    // RenderWare 几何体在 UV 缝 / 材质边界处会存在"位置相同、索引不同"的顶点。
    // 它们在 Blender 里看起来是同一个点，但物理上是独立粒子，而且彼此
    // **没有三角形连边**，所以平滑和差值限制都作用不到 —— 一抖就分家。
    // 这里把同位置的顶点都挂到同一个粒子下，写回时写同一个位置。
    // ============================================================
    {
        std::map<std::tuple<int, int, int>, std::vector<int>> byPosition;

        auto quantize = [](float value)
        {
            return static_cast<int>(std::lround(value * 10000.0f));   // 0.1mm
        };

        for (int v = 0; v < numVerts; ++v)
        {
            const RwV3d& pos = verts[v];
            byPosition[std::make_tuple(quantize(pos.x), quantize(pos.y), quantize(pos.z))]
                .push_back(v);
        }

        int weldedVertices = 0;
        int weldGroups = 0;
        int skippedOwnParticles = 0;

        for (auto& p : rig.particles)
        {
            const RwV3d& pos = p.restLocal;

            auto it = byPosition.find(std::make_tuple(
                quantize(pos.x), quantize(pos.y), quantize(pos.z)));

            if (it == byPosition.end()) continue;

            bool added = false;

            for (int v : it->second)
            {
                if (v == p.geometryVertexIndex) continue;

                // 关键修复：如果这个顶点本身也是物理粒子（比如披风顶点恰好
                // 与身体顶点同位置），绝不能把它焊进来 —— 否则两个粒子会
                // 写同一个顶点，后写的赢，那一侧就"看起来没有物理"。
                if (rig.vertexToParticle[v] >= 0)
                {
                    ++skippedOwnParticles;
                    continue;
                }

                p.weldVertices.push_back(v);
                ++weldedVertices;
                added = true;
            }

            if (added) ++weldGroups;
        }

        // 焊接进来的顶点现在"跟着物理粒子走"，不能再算作不动顶点，
        // 否则它们会把周围一圈物理顶点全标成"边界"（幅度 0），
        // 表现就是"只有一小块能抖"。
        for (size_t i = 0; i < rig.particles.size(); ++i)
        {
            for (int v : rig.particles[i].weldVertices)
            {
                if (v >= 0 && v < numVerts)
                    rig.vertexToParticle[v] = static_cast<int>(i);
            }
        }

        std::ostringstream oss;
        oss << "[VER_82][Weld] groups=" << weldGroups
            << " extraVertices=" << weldedVertices
            << " skippedParticles=" << skippedOwnParticles
            << " particles=" << rig.particles.size();
        GeoPhysLog(oss.str());

        // 诊断：哪些物理顶点是"硬锚定"（B=0），它们本来就不会动
        {
            int pinnedHair = 0, pinnedCloth = 0, pinnedBody = 0, pinnedSoft = 0;
            std::ostringstream list;
            int shown = 0;

            for (const auto& p : rig.particles)
            {
                if (!p.hardAnchor) continue;

                switch (static_cast<PhysType>(p.typeU32))
                {
                case PhysType::Hair:     ++pinnedHair;  break;
                case PhysType::Cloth:    ++pinnedCloth; break;
                case PhysType::Body:     ++pinnedBody;  break;
                case PhysType::SoftBody: ++pinnedSoft;  break;
                default: break;
                }

                if (shown < 12)
                {
                    list << " " << p.geometryVertexIndex;
                    ++shown;
                }
            }

            std::ostringstream pin;
            pin << "[VER_82][Pinned] hair=" << pinnedHair
                << " cloth=" << pinnedCloth
                << " body=" << pinnedBody
                << " soft=" << pinnedSoft
                << " vertices=" << list.str();
            GeoPhysLog(pin.str());
        }
    }

    // ============================================================
    // 类型清理：按"连通片多数票"把混进来的异类顶点改回主导类型
    //
    // 2026-11-23 实测（tk7Chi2.dff）：作者在 Blender 里把头发整片刷成 R=64，
    // 但导出后顶点色在材质/UV 缝处被插值，出现了 127 / 145 / 159 / 191 / 207
    // 这一串"混合值"（R 桶计数：64=1757、159=54、191=32、127=29、145=13 ...）。
    // 而 ClassifyVertexColor 用 ±4 容差匹配 0x40/0x80/0xC0：
    //     R=127 -> Cloth(128)      R=191 -> Body(192)
    // 于是**头发中间夹进了几十个"布料/身体"顶点**（实测 29 个）。
    // 后果：
    //   * Body 顶点 invMass=0（不动）、Cloth 顶点走软挂点/布料参数，
    //     同一片头发被从内部撕开；
    //   * Body 参与的边过去被 limitStretch 整条跳过 -> 16.2mm 的边被拉到
    //     392mm（[TriSpike] 实测 376mm），就是后脑勺那条细长尖刺。
    //
    // 处理：对每个连通片投票，若某类型占 >=70%，就把占比很小的异类顶点
    // （通常是插值出来的几个）改成主导类型，参数从最近的同类邻居拷贝。
    // 真正混合的片（没有任何类型到 70%）不会被改。
    // ============================================================
    {
        const int pc = static_cast<int>(rig.particles.size());

        std::vector<int> parent(static_cast<size_t>(pc));
        for (int i = 0; i < pc; ++i) parent[i] = i;

        auto findRootType = [&parent](int x) -> int
        {
            while (parent[x] != x) { parent[x] = parent[parent[x]]; x = parent[x]; }
            return x;
        };

        std::vector<std::vector<int>> adj(static_cast<size_t>(pc));

        for (int i = 0; i < numTris; ++i)
        {
            const int va = static_cast<int>(tris[i].vertIndex[0]);
            const int vb = static_cast<int>(tris[i].vertIndex[1]);
            const int vc = static_cast<int>(tris[i].vertIndex[2]);

            if (va < 0 || va >= numVerts || vb < 0 || vb >= numVerts ||
                vc < 0 || vc >= numVerts)
                continue;

            const int p3[3] = { rig.vertexToParticle[va],
                                rig.vertexToParticle[vb],
                                rig.vertexToParticle[vc] };

            for (int a = 0; a < 3; ++a)
            {
                for (int b = a + 1; b < 3; ++b)
                {
                    if (p3[a] < 0 || p3[b] < 0 || p3[a] == p3[b]) continue;

                    adj[static_cast<size_t>(p3[a])].push_back(p3[b]);
                    adj[static_cast<size_t>(p3[b])].push_back(p3[a]);

                    const int ra = findRootType(p3[a]);
                    const int rb = findRootType(p3[b]);
                    if (ra != rb) parent[rb] = ra;
                }
            }
        }

        std::map<int, std::vector<int>> typeComps;
        for (int i = 0; i < pc; ++i)
            typeComps[findRootType(i)].push_back(i);

        int flipped = 0;
        int flippedPieces = 0;
        std::ostringstream detail;

        for (auto& entry : typeComps)
        {
            auto& members = entry.second;
            if (members.size() < 4) continue;

            std::map<int, int> hist;
            for (int idx : members)
                hist[static_cast<int>(rig.particles[idx].typeU32)] += 1;

            int dom = -1, domCount = 0;
            for (const auto& h : hist)
                if (h.second > domCount) { domCount = h.second; dom = h.first; }

            if (dom < 0) continue;
            if (domCount * 10 < static_cast<int>(members.size()) * 7) continue;

            bool any = false;

            for (int idx : members)
            {
                auto& q = rig.particles[idx];
                if (static_cast<int>(q.typeU32) == dom) continue;

                int src = -1;
                float best = 1e9f;

                for (int nb : adj[static_cast<size_t>(idx)])
                {
                    if (static_cast<int>(rig.particles[nb].typeU32) != dom) continue;
                    const float d = V3LenSq(V3Sub(rig.particles[nb].restLocal, q.restLocal));
                    if (d < best) { best = d; src = nb; }
                }

                if (src < 0)
                {
                    for (int m : members)
                        if (static_cast<int>(rig.particles[m].typeU32) == dom) { src = m; break; }
                }

                if (src < 0) continue;

                const auto& host = rig.particles[src];

                detail << " " << q.geometryVertexIndex << "->" << dom;

                q.typeU32 = static_cast<uint32_t>(dom);
                q.stiffness = host.stiffness;
                q.returnStrength = host.returnStrength;
                q.anchorFactor = host.anchorFactor;
                q.hardAnchor = host.hardAnchor;

                switch (static_cast<PhysType>(dom))
                {
                case PhysType::Hair:
                    q.invMass = 1.0f;
                    break;
                case PhysType::Cloth:
                    q.invMass = 1.1f;
                    q.stiffness = std::max(q.stiffness, 0.55f);
                    break;
                case PhysType::Body:
                    q.invMass = 0.0f;
                    q.hardAnchor = false;
                    break;
                case PhysType::SoftBody:
                    q.invMass = 1.4f;
                    break;
                default:
                    q.invMass = 1.0f;
                    break;
                }

                ++flipped;
                any = true;
            }

            if (any) ++flippedPieces;
        }

        if (flipped > 0)
        {
            // 重建类型集合与计数
            rig.hairParticleSet.clear();
            rig.clothParticleSet.clear();
            rig.softParticleSet.clear();
            rig.bodyParticleSet.clear();
            rig.hairCount = rig.clothCount = rig.softCount = rig.bodyCount = 0;

            for (size_t i = 0; i < rig.particles.size(); ++i)
            {
                switch (static_cast<PhysType>(rig.particles[i].typeU32))
                {
                case PhysType::Hair:     rig.hairParticleSet.push_back(static_cast<int>(i));  ++rig.hairCount;  break;
                case PhysType::Cloth:    rig.clothParticleSet.push_back(static_cast<int>(i)); ++rig.clothCount; break;
                case PhysType::Body:     rig.bodyParticleSet.push_back(static_cast<int>(i));  ++rig.bodyCount;  break;
                case PhysType::SoftBody: rig.softParticleSet.push_back(static_cast<int>(i));  ++rig.softCount;  break;
                default: break;
                }
            }

            std::ostringstream oss;
            oss << "[VER_82][TypeFix] pieces=" << flippedPieces
                << " particles=" << flipped
                << " (混合顶点色落在别的类型码上 -> 改回该片主导类型)"
                << " first:" << detail.str().substr(0, 200);
            GeoPhysLog(oss.str());
        }
    }

    // ============================================================
    // 三角形拓扑约束
    // ============================================================

    std::vector<uint64_t> edges;

    // 三角形共享边 -> 对面顶点（用于抗弯约束）
    std::map<uint64_t, std::vector<int>> edgeOpposite;

    // 与 Rigid 顶点共享三角形的"边界粒子"——它们紧挨着不参与物理的
    // 发根/头部顶点，是天然的可锚定点
    std::vector<char> boundary(rig.particles.size(), 0);

    for (int i = 0; i < numTris; ++i)
    {
        const int va = static_cast<int>(tris[i].vertIndex[0]);
        const int vb = static_cast<int>(tris[i].vertIndex[1]);
        const int vc = static_cast<int>(tris[i].vertIndex[2]);

        if (va < 0 || va >= numVerts || vb < 0 || vb >= numVerts ||
            vc < 0 || vc >= numVerts)
            continue;

        const int pa = rig.vertexToParticle[va];
        const int pb = rig.vertexToParticle[vb];
        const int pc = rig.vertexToParticle[vc];

        const int pIdx[3] = { pa, pb, pc };
        int particleCount = 0;
        for (int k = 0; k < 3; ++k)
            if (pIdx[k] >= 0) ++particleCount;

        if (particleCount > 0 && particleCount < 3)
        {
            for (int k = 0; k < 3; ++k)
                if (pIdx[k] >= 0) boundary[pIdx[k]] = 1;
        }

        // 物理顶点 ↔ Rigid 顶点的挂点约束（可用 kEnablePins 关闭）
        if (kEnablePins)
        for (int k = 0; k < 3; ++k)
        {
            if (pIdx[k] < 0) continue;

            for (int m = 0; m < 3; ++m)
            {
                if (m == k || pIdx[m] >= 0) continue;

                const int rigidVertex = (m == 0) ? va : ((m == 1) ? vb : vc);

                VertexParticle::RigidPin pin;
                pin.restLocal = verts[rigidVertex];
                pin.restLength = V3Len(V3Sub(verts[rigidVertex],
                    rig.particles[pIdx[k]].restLocal));
                pin.rigidVertex = rigidVertex;

                // 同一个 rigid 顶点会被该物理顶点所在的每个三角形重复加入。
                // 不去重的话一个头发顶点可能挂 10 条相同 pin，每帧做 10 次
                // 硬投影（等于反复把同一个约束解 10 遍，还挤掉别的 pin 的
                // 迭代次数）。这里按几何体顶点索引去重。
                {
                    auto& hostPins = rig.particles[pIdx[k]].pins;

                    bool duplicate = false;
                    for (const auto& existing : hostPins)
                    {
                        if (existing.rigidVertex == rigidVertex)
                        {
                            duplicate = true;
                            break;
                        }
                    }

                    if (duplicate) continue;
                }

                // 记录这个 Rigid 顶点自己的骨骼权重
                // 2026-11-27：必须和粒子侧一样支持"8 位骨骼索引"。
                // 之前这里永远按 u32 读，8 位蒙皮模型的 idx 全部越界 →
                // pin.bones/weights 保持默认 → 挂点目标退化，挂点密集的布片被拉偏。
                if (skinWeights && skinIndices && boneCount > 0)
                {
                    const RwMatrixWeights& w = skinWeights[rigidVertex];
                    const float wArr[4] = { w.w0, w.w1, w.w2, skinSplit ? 0.0f : w.w3 };

                    const RwUInt32* idx32 = skinIndices + static_cast<size_t>(rigidVertex) * 4;
                    const RwUInt8*  idx8  = reinterpret_cast<const RwUInt8*>(skinIndices)
                                          + static_cast<size_t>(rigidVertex) * 4;

                    for (int kk = 0; kk < 4; ++kk)
                    {
                        const RwUInt32 boneIdx = indicesU8 ? idx8[kk] : idx32[kk];

                        if (wArr[kk] > 1e-6f && boneIdx < boneCount)
                        {
                            pin.bones[kk] = static_cast<int>(boneIdx);
                            pin.weights[kk] = wArr[kk];
                        }
                    }
                }

                if (pin.restLength > 1e-4f)
                    rig.particles[pIdx[k]].pins.push_back(pin);
            }
        }

        auto addEdge = [&](int a, int b)
        {
            if (a >= 0 && b >= 0)
            {
                if (a > b) std::swap(a, b);
                edges.push_back(
                    (static_cast<uint64_t>(a) << 32) |
                    static_cast<uint32_t>(b));
            }
        };

        addEdge(pa, pb);
        addEdge(pb, pc);
        addEdge(pc, pa);

        // 记录每条边对面的顶点，后面给软垂旗面加抗弯约束
        auto addOpposite = [&](int a, int b, int c)
        {
            if (a < 0 || b < 0 || c < 0) return;


            if (a > b) std::swap(a, b);

            const uint64_t key =
                (static_cast<uint64_t>(a) << 32) | static_cast<uint32_t>(b);

            edgeOpposite[key].push_back(c);
        };

        addOpposite(pa, pb, pc);
        addOpposite(pb, pc, pa);
        addOpposite(pc, pa, pb);
    }

    std::sort(edges.begin(), edges.end());
    edges.erase(std::unique(edges.begin(), edges.end()), edges.end());

    // 模型空间模式下刚体顶点是静止的：挂点目标就是它的静止位置。
    // 世界空间模式每帧会重算 targetWorld；这里的初值只在模型空间分支用到。
    for (auto& p : rig.particles)
    {
        for (auto& pin : p.pins)
            pin.targetWorld = pin.restLocal;
    }

    for (uint64_t e : edges)
    {
        const int a = static_cast<int>(e >> 32);
        const int b = static_cast<int>(e & 0xFFFFFFFF);

        const auto& A = rig.particles[a];
        const auto& B = rig.particles[b];

        const float rest = V3Len(V3Sub(B.restLocal, A.restLocal));
        if (rest < 1e-4f) continue;

        VertexConstraint c;
        c.a = a;
        c.b = b;
        c.restLength = rest;

        // 两端软度取平均：发根(G=0)硬、发梢(G 高)软，过渡自然
        c.stiffness = std::clamp(0.5f * (A.stiffness + B.stiffness), 0.05f, 1.0f);

        rig.constraints.push_back(c);
    }

    // ============================================================
    // 孤立头发片"缝合"（2026-11-23，修"有一部分头发完全没物理"）
    //
    // 现象：有几片头发在**网格拓扑上没有任何 pin、也没有硬锚点**
    // （本例 pid=2/3/4/7/10，52+27+27+43+52 = 201 个顶点），
    // 它们唯一的约束是弱锚点回拉，于是整片会飘到离头皮 20~30cm（[PieceDrift] 实测 174~304mm）。
    // 上一版的处理是"整片 hardAnchor 跟随骨骼"—— 尖刺没了，但那 201 个顶点
    // **完全没有物理**（用户实测"有一部分头发不动"）。
    //
    // 现在改成：把这些片**用距离约束缝合到最近的、有挂点的头发片上**，
    // 于是它们跟着主发片一起动（有物理），又不会被拉走（缝合边长 ≤8cm、且受 1.35× 限长）。
    // 每片的每个顶点最多缝 1 条，只缝给"非孤立片"的最近头发粒子。
    // 找不到 8cm 内的邻居时才退回上一版的"跟随骨骼"（hairpiecefix 兜底）。
    // ============================================================
    {
        const int pc = static_cast<int>(rig.particles.size());

        std::vector<int> parent(static_cast<size_t>(pc));
        for (int i = 0; i < pc; ++i) parent[i] = i;

        auto findRootS = [&parent](int x) -> int
        {
            while (parent[x] != x) { parent[x] = parent[parent[x]]; x = parent[x]; }
            return x;
        };

        for (const auto& c : rig.constraints)
        {
            const int ra = findRootS(c.a);
            const int rb = findRootS(c.b);
            if (ra != rb) parent[rb] = ra;
        }

        std::map<int, std::vector<int>> comps;
        for (int i = 0; i < pc; ++i) comps[findRootS(i)].push_back(i);

        struct CompStat { int hair = 0; int pins = 0; int hard = 0; };
        std::map<int, CompStat> stat;

        for (auto& entry : comps)
        {
            CompStat cs;
            for (int idx : entry.second)
            {
                const auto& p = rig.particles[idx];
                if (static_cast<PhysType>(p.typeU32) == PhysType::Hair) ++cs.hair;
                cs.pins += static_cast<int>(p.pins.size());
                if (p.hardAnchor) ++cs.hard;
            }
            stat[entry.first] = cs;
        }

        auto isLoose = [&stat](int root) -> bool
        {
            const auto& cs = stat[root];
            return cs.hair > 0 && cs.pins == 0 && cs.hard == 0;
        };

        // 宿主 = 非孤立片里的头发粒子
        std::vector<int> hosts;
        for (auto& entry : comps)
        {
            if (isLoose(entry.first)) continue;
            for (int idx : entry.second)
                if (static_cast<PhysType>(rig.particles[idx].typeU32) == PhysType::Hair)
                    hosts.push_back(idx);
        }

        int stitched = 0, piecesStitched = 0;
        float worstDist = 0.0f;
        std::set<uint64_t> added;

        if (!hosts.empty())
        for (auto& entry : comps)
        {
            if (!isLoose(entry.first)) continue;

            bool any = false;

            for (int idx : entry.second)
            {
                const auto& p = rig.particles[idx];
                if (static_cast<PhysType>(p.typeU32) != PhysType::Hair) continue;

                int best = -1;
                float bestD2 = 1e9f;

                for (int h : hosts)
                {
                    const float d2 = V3LenSq(V3Sub(rig.particles[h].restLocal, p.restLocal));
                    if (d2 < bestD2) { bestD2 = d2; best = h; }
                }

                if (best < 0) continue;

                const float d = std::sqrt(std::max(bestD2, 0.0f));
                if (d > 0.08f) continue;

                int a = idx;
                int b = best;
                if (a > b) std::swap(a, b);

                const uint64_t key = (static_cast<uint64_t>(a) << 32) |
                    static_cast<uint32_t>(b);

                if (!added.insert(key).second) continue;

                VertexConstraint sc;
                sc.a = a;
                sc.b = b;
                sc.restLength = std::max(d, 0.01f);
                sc.stiffness = 0.55f;

                rig.constraints.push_back(sc);
                ++stitched;
                any = true;
                if (d > worstDist) worstDist = d;
            }

            if (any) ++piecesStitched;
        }

        if (stitched > 0)
        {
            std::ostringstream oss;
            oss << std::fixed << std::setprecision(1);
            oss << "[VER_82][HairStitch] pieces=" << piecesStitched
                << " constraints=" << stitched
                << " maxDist=" << (worstDist * 1000.0f) << "mm"
                << " (孤立头发片缝到最近的头发片上，改成有物理)";
            GeoPhysLog(oss.str());
        }
    }

    // ============================================================
    // Body 分组：先按主导骨骼分组，再把**空间上相邻**的骨骼组合并
    //
    // 左右臀部通常绑在不同骨骼上，但视觉上应该同步抖动；
    // 不合并的话两半会各走各的（甚至一半位移接近 0，看起来"半边没物理"）。
    // ============================================================

    {
        std::map<int, RwV3d> centroidSum;
        std::map<int, int>   centroidCount;

        for (const auto& p : rig.particles)
        {
            if (static_cast<PhysType>(p.typeU32) != PhysType::Body) continue;

            const int g = (p.dominantBone >= 0) ? p.dominantBone : -1;

            centroidSum[g] = V3Add(centroidSum[g], p.restLocal);
            centroidCount[g] += 1;
        }

        std::map<int, int> parent;
        for (const auto& entry : centroidCount) parent[entry.first] = entry.first;

        auto findRoot = [&parent](int x)
        {
            while (parent[x] != x)
            {
                parent[x] = parent[parent[x]];
                x = parent[x];
            }
            return x;
        };

        std::vector<int> keys;
        std::vector<RwV3d> centers;

        for (const auto& entry : centroidCount)
        {
            keys.push_back(entry.first);

            RwV3d c = centroidSum[entry.first];
            const float inv = 1.0f / static_cast<float>(entry.second);
            centers.push_back(V3Scale(c, inv));
        }

        constexpr float kMergeDistance = 0.12f;

        for (size_t i = 0; i < keys.size(); ++i)
        {
            for (size_t j = i + 1; j < keys.size(); ++j)
            {
                if (V3Len(V3Sub(centers[i], centers[j])) > kMergeDistance) continue;

                const int ra = findRoot(keys[i]);
                const int rb = findRoot(keys[j]);
                if (ra != rb) parent[rb] = ra;
            }
        }

        int mergedGroups = 0;

        for (auto& p : rig.particles)
        {
            if (static_cast<PhysType>(p.typeU32) != PhysType::Body) continue;

            const int g = (p.dominantBone >= 0) ? p.dominantBone : -1;
            p.bodyGroup = findRoot(g);
        }

        for (const auto& entry : parent)
        {
            if (findRoot(entry.first) == entry.first) ++mergedGroups;
        }

        std::ostringstream oss;
        oss << "[VER_82][BodyMerge] bones=" << keys.size()
            << " groups=" << mergedGroups
            << " mergeDistance=" << kMergeDistance;
        GeoPhysLog(oss.str());
    }

    // ============================================================
    // AutoAnchor：保证每一片头发/布料至少有一个固定点
    //
    // 没有固定点的连通片会在重力下整片滑走，运动时又被"最大偏离距离"
    // 反复拽回头部，表现就是"被推着走 + 拉成一片"。
    // 规则：
    //   1) 该片已有硬锚定顶点 -> 不动
    //   2) 该片有紧邻 Rigid 顶点的边界顶点 -> 把边界顶点锚死（发根）
    //   3) 都没有 -> 锚死该片最高处（restLocal.z 最大）的顶点
    // ============================================================

    auto anchorParticles = [&](const std::vector<int>& members,
                               const std::vector<int>& picks,
                               const char* mode)
    {
        for (int idx : picks)
        {
            auto& p = rig.particles[idx];
            p.hardAnchor = true;
            p.anchorFactor = 0.0f;
        }

        std::ostringstream oss;
        oss << "[VER_82][AutoAnchor] particles=" << members.size()
            << " anchored=" << picks.size()
            << " mode=" << mode
            << " vertices=";
        for (size_t i = 0; i < picks.size() && i < 8; ++i)
            oss << (i ? "," : "") << rig.particles[picks[i]].geometryVertexIndex;
        GeoPhysLog(oss.str());
    };

    {
        const int particleCount = static_cast<int>(rig.particles.size());

        std::vector<int> parent(static_cast<size_t>(particleCount));
        for (int i = 0; i < particleCount; ++i) parent[i] = i;

        auto findRoot = [&parent](int x)
        {
            while (parent[x] != x)
            {
                parent[x] = parent[parent[x]];
                x = parent[x];
            }
            return x;
        };

        for (const auto& c : rig.constraints)
        {
            const int ra = findRoot(c.a);
            const int rb = findRoot(c.b);
            if (ra != rb) parent[rb] = ra;
        }

        std::vector<std::vector<int>> components(static_cast<size_t>(particleCount));
        for (int i = 0; i < particleCount; ++i)
            components[static_cast<size_t>(findRoot(i))].push_back(i);

        // ------------------------------------------------------------
        // 注册连通片：分配 pieceId + 每片统计（诊断 / "无锚点片固定"都要用）
        // ------------------------------------------------------------
        rig.pieces.clear();

        for (const auto& members : components)
        {
            if (members.empty()) continue;

            GeometryPhysics::HairPieceInfo info;
            info.count = static_cast<int>(members.size());

            for (int idx : members)
            {
                auto& p = rig.particles[idx];
                p.pieceId = static_cast<int>(rig.pieces.size());

                if (info.repVertex < 0) info.repVertex = p.geometryVertexIndex;

                switch (static_cast<PhysType>(p.typeU32))
                {
                case PhysType::Hair:  ++info.hairCount;  break;
                case PhysType::Cloth: ++info.clothCount; break;
                default: break;
                }

                info.pinLinks += static_cast<int>(p.pins.size());
                if (p.hardAnchor) ++info.hardCount;
                if (p.restSpan > info.maxSpan) info.maxSpan = p.restSpan;
            }

            rig.pieces.push_back(info);
        }

        // Body 分组 id 分配（piece / cluster 模式）
        int nextBodyGroup = 0;

        for (const auto& members : components)
        {
            if (members.empty()) continue;

            // ------------------------------------------------------------
            // 简化：Body 顶点一律 100% 幅度，不再做"环数渐隐"。
            // 之前的 ring 0 = 0 会把薄片状区域（臀部）几乎整片冻住，
            // 只剩个别顶点能动，反而在边界拉出细长的尖刺。
            // 接缝安全由"顶点焊接 + 相邻位移差限制"保证，不再靠压幅度。
            //
            // 2026-11-24 15:4x：同时给 Body 顶点分配 **bodyGroup**。
            // 原来 bodyGroup 恒为 -1 ⇒ 回退到 dominantBone ⇒ 胸和腹
            // （都挂脊椎骨）落进同一组 ⇒ 只能往同一个方向抖。
            // ------------------------------------------------------------
            std::vector<int> bodyMembers;

            for (int idx : members)
            {
                auto& p = rig.particles[idx];
                if (static_cast<PhysType>(p.typeU32) != PhysType::Body) continue;
                p.jiggleWeight = 1.0f;
                bodyMembers.push_back(idx);
            }

            if (bodyMembers.empty()) continue;

            if (g_bodyGroupMode == 2)
            {
                // ---- cluster：片内 k-means（锚点位置）----
                const int k = std::clamp(g_bodyGroupSplit, 1, 8);
                const int n = static_cast<int>(bodyMembers.size());

                if (k <= 1 || n <= k)
                {
                    for (int idx : bodyMembers)
                        rig.particles[idx].bodyGroup = nextBodyGroup;

                    ++nextBodyGroup;
                }
                else
                {
                    std::vector<RwV3d> pts(static_cast<size_t>(n));

                    for (int i = 0; i < n; ++i)
                        pts[static_cast<size_t>(i)] =
                            rig.particles[bodyMembers[static_cast<size_t>(i)]].anchorWorld;

                    // 确定性初始化：先取第一个点，再反复取"离已有中心最远"的点
                    std::vector<RwV3d> centers;
                    centers.reserve(static_cast<size_t>(k));
                    centers.push_back(pts[0]);

                    while (static_cast<int>(centers.size()) < k)
                    {
                        float bestD = -1.0f;
                        int bestI = 0;

                        for (int i = 0; i < n; ++i)
                        {
                            float dMin = 1e30f;

                            for (const auto& c : centers)
                            {
                                const float dx = pts[static_cast<size_t>(i)].x - c.x;
                                const float dy = pts[static_cast<size_t>(i)].y - c.y;
                                const float dz = pts[static_cast<size_t>(i)].z - c.z;
                                dMin = std::min(dMin, dx * dx + dy * dy + dz * dz);
                            }

                            if (dMin > bestD) { bestD = dMin; bestI = i; }
                        }

                        centers.push_back(pts[static_cast<size_t>(bestI)]);
                    }

                    std::vector<int> assign(static_cast<size_t>(n), 0);

                    for (int iter = 0; iter < 8; ++iter)
                    {
                        for (int i = 0; i < n; ++i)
                        {
                            float bestD = 1e30f;
                            int bestC = 0;

                            for (int c = 0; c < static_cast<int>(centers.size()); ++c)
                            {
                                const float dx = pts[static_cast<size_t>(i)].x - centers[c].x;
                                const float dy = pts[static_cast<size_t>(i)].y - centers[c].y;
                                const float dz = pts[static_cast<size_t>(i)].z - centers[c].z;
                                const float d = dx * dx + dy * dy + dz * dz;

                                if (d < bestD) { bestD = d; bestC = c; }
                            }

                            assign[static_cast<size_t>(i)] = bestC;
                        }

                        std::vector<RwV3d> sum(centers.size(), RwV3d{ 0.0f, 0.0f, 0.0f });
                        std::vector<int> cnt(centers.size(), 0);

                        for (int i = 0; i < n; ++i)
                        {
                            const int c = assign[static_cast<size_t>(i)];
                            sum[static_cast<size_t>(c)] = V3Add(sum[static_cast<size_t>(c)],
                                                                pts[static_cast<size_t>(i)]);
                            ++cnt[static_cast<size_t>(c)];
                        }

                        for (size_t c = 0; c < centers.size(); ++c)
                        {
                            if (cnt[c] > 0)
                                centers[c] = V3Scale(sum[c], 1.0f / static_cast<float>(cnt[c]));
                        }
                    }

                    // 空簇跳过（编号压缩），保证组 id 连续
                    std::map<int, int> clusterToGroup;

                    for (int i = 0; i < n; ++i)
                    {
                        const int c = assign[static_cast<size_t>(i)];

                        auto it = clusterToGroup.find(c);

                        if (it == clusterToGroup.end())
                        {
                            const int gid = nextBodyGroup++;
                            it = clusterToGroup.emplace(c, gid).first;
                        }

                        rig.particles[bodyMembers[static_cast<size_t>(i)]].bodyGroup = it->second;
                    }
                }
            }
            else if (g_bodyGroupMode == 0)
            {
                // ---- piece：一个连通片一组 ----
                for (int idx : bodyMembers)
                    rig.particles[idx].bodyGroup = nextBodyGroup;

                ++nextBodyGroup;
            }
        }

        // 注意：旧的"重复顶点同步硬锚定"(mirrored anchors) 已经彻底删除。
        // 它会因为某个 B=0 顶点而把同位置的其他物理顶点一起钉死，
        // 正是"刷了颜色却不动"的隐藏来源。

        // ------------------------------------------------------------
        // [ClothPiece] 每个布料连通片的诊断（建 rig 时打一次）
        //
        // 用途：左右裙片"权重/顶点色几乎相同但只有一边出错"时，
        // 直接对比两片的：顶点数、挂点数(pins)、软挂点、重心、尺寸。
        // 挂点=0 的片会直接掉下来/乱飘 —— 这一行能立刻看出来。
        // ------------------------------------------------------------
        {
            struct ClothPieceAgg
            {
                int   count = 0;
                int   pinned = 0;      // 有 pins 的顶点数
                int   pinLinks = 0;    // pins 总条数
                int   softPins = 0;
                int   hard = 0;
                RwV3d sum{ 0.0f, 0.0f, 0.0f };
                RwV3d mn{ 1e9f, 1e9f, 1e9f };
                RwV3d mx{ -1e9f, -1e9f, -1e9f };
            };

            std::map<int, ClothPieceAgg> agg;

            for (const auto& p : rig.particles)
            {
                if (static_cast<PhysType>(p.typeU32) != PhysType::Cloth) continue;

                ClothPieceAgg& a = agg[p.pieceId];

                ++a.count;
                a.pinLinks += static_cast<int>(p.pins.size());
                if (!p.pins.empty()) ++a.pinned;
                if (p.softPins) ++a.softPins;
                if (p.hardAnchor) ++a.hard;

                const RwV3d& q = p.restLocal;
                a.sum = V3Add(a.sum, q);
                a.mn = { std::min(a.mn.x, q.x), std::min(a.mn.y, q.y), std::min(a.mn.z, q.z) };
                a.mx = { std::max(a.mx.x, q.x), std::max(a.mx.y, q.y), std::max(a.mx.z, q.z) };
            }

            for (const auto& kv : agg)
            {
                const ClothPieceAgg& a = kv.second;

                if (a.count < 8) continue;

                const float inv = 1.0f / static_cast<float>(a.count);

                std::ostringstream oss;
                oss << std::fixed << std::setprecision(3);
                oss << "[ClothPiece] id=" << kv.first
                    << " n=" << a.count
                    << " pins=" << a.pinned << "/" << a.pinLinks
                    << " soft=" << a.softPins
                    << " hard=" << a.hard
                    << " c=(" << a.sum.x * inv << "," << a.sum.y * inv << "," << a.sum.z * inv << ")"
                    << " size=(" << (a.mx.x - a.mn.x) << "," << (a.mx.y - a.mn.y) << "," << (a.mx.z - a.mn.z) << ")";

                GeoPhysLog(oss.str());
            }
        }


        int bodyPinned = 0;
        for (const auto& p : rig.particles)
        {
            if (static_cast<PhysType>(p.typeU32) == PhysType::Body && p.hardAnchor)
                ++bodyPinned;
        }

        if (bodyPinned > 0)
        {
            std::ostringstream oss;
            oss << "[VER_82][AutoAnchor] WARNING body pinned=" << bodyPinned
                << " (这些身体顶点不会抖动)";
            GeoPhysLog(oss.str());
        }

        // ============================================================
        // 自然可伸长度：每个粒子离"最近锚点"的静止距离
        //
        // 锚点 = B=0 的硬锚定顶点，或挂点(pin)指向的刚体顶点。
        // 布料/头发被钉在锚点上，只能绕它旋转：离自己静止位置最远可以到
        // 2×restSpan（绕到锚点另一侧）。以前只把 B=0 顶点当锚点，
        // AutoAnchor 移除后几乎没有任何 B=0 顶点，restSpan 就一直是 0，
        // 于是所有布料被限制在 15cm 内、头发 10cm 内——
        // 表现正是"被无形的力提住，永远不会自然落下"。
        // ============================================================

        int pinRootedPieces = 0;
        int unpinnedPieces  = 0;
        int softPinPieces   = 0;
        float maxSpan = 0.0f;

        for (const auto& members : components)
        {
            if (members.empty()) continue;

            std::vector<RwV3d> anchorPoints;
            std::vector<RwV3d> pinPoints;
            bool hasPin = false;

            for (int idx : members)
            {
                const auto& p = rig.particles[idx];

                if (p.hardAnchor)
                    anchorPoints.push_back(p.restLocal);

                for (const auto& pin : p.pins)
                {
                    anchorPoints.push_back(pin.restLocal);
                    pinPoints.push_back(pin.restLocal);
                    hasPin = true;
                }
            }

            // 判断"挂点是不是一条竖边"（旗子的升旗边）。
            // 竖边被硬缝死时，重力与这条边平行，布面根本翻不下来；
            // 横边（横幅上沿、披风肩线）则必须保持硬连接。
            bool softHoist = false;

            if (hasPin && pinPoints.size() >= 2)
            {
                RwV3d lo = pinPoints.front();
                RwV3d hi = pinPoints.front();

                for (const auto& q : pinPoints)
                {
                    lo.x = std::min(lo.x, q.x); lo.y = std::min(lo.y, q.y); lo.z = std::min(lo.z, q.z);
                    hi.x = std::max(hi.x, q.x); hi.y = std::max(hi.y, q.y); hi.z = std::max(hi.z, q.z);
                }

                const float spanZ = hi.z - lo.z;
                const float spanH = std::max(hi.x - lo.x, hi.y - lo.y);

                softHoist = spanZ > spanH * 1.5f && spanZ > 0.05f;
            }

            // 竖向挂点的布料用软挂点（clothPinStiffness），布面才能从挂点滑开垂落。
            if (hasPin && softHoist) ++softPinPieces;

            const int rootAnchors = 0;

            // 诊断：较大的布料片单独打印尺寸/挂点数/是否软挂点
            {
                int clothInPiece = 0;
                int pinnedInPiece = 0;
                RwV3d lo{}, hi{};
                bool first = true;

                for (int idx : members)
                {
                    const auto& p = rig.particles[idx];

                    if (static_cast<PhysType>(p.typeU32) == PhysType::Cloth)
                        ++clothInPiece;

                    if (!p.pins.empty())
                        ++pinnedInPiece;

                    if (first) { lo = hi = p.restLocal; first = false; }
                    else
                    {
                        lo.x = std::min(lo.x, p.restLocal.x);
                        lo.y = std::min(lo.y, p.restLocal.y);
                        lo.z = std::min(lo.z, p.restLocal.z);
                        hi.x = std::max(hi.x, p.restLocal.x);
                        hi.y = std::max(hi.y, p.restLocal.y);
                        hi.z = std::max(hi.z, p.restLocal.z);
                    }
                }

                if (clothInPiece >= 8)
                {
                    float pieceMaxSpan = 0.0f;

                    for (int idx : members)
                    {
                        float best = 1e9f;

                        for (const auto& a : anchorPoints)
                        {
                            const float d = V3Len(V3Sub(rig.particles[idx].restLocal, a));
                            if (d < best) best = d;
                        }

                        if (best < 1e8f && best > pieceMaxSpan)
                            pieceMaxSpan = best;
                    }

                    // 给软垂的旗面一个极小的初始褶皱种子：
                    // 完美平面的布料在无风时只有"整体旋转"一个平衡态，
                    // 1cm 量级的扰动才能让它起褶、看起来像布。
                    if (softHoist)
                    {
                        const float sx = hi.x - lo.x;
                        const float sy = hi.y - lo.y;
                        const float sz = hi.z - lo.z;

                        RwV3d normal = { 0.0f, 0.0f, 0.0f };

                        if (sx <= sy && sx <= sz)      normal.x = 1.0f;
                        else if (sy <= sz)             normal.y = 1.0f;
                        else                           normal.z = 1.0f;

                        if (anchorLTM)
                            normal = TransformDir(*anchorLTM, normal);

                        const float nlen = V3Len(normal);

                        if (nlen > 1e-4f)
                        {
                            normal = V3Scale(normal, 1.0f / nlen);

                            int seedIndex = 0;

                            for (int idx : members)
                            {
                                auto& p = rig.particles[idx];

                                if (static_cast<PhysType>(p.typeU32) != PhysType::Cloth)
                                    continue;

                                const float phase =
                                    1.7f * static_cast<float>(seedIndex++) + 0.3f;

                                p.seedOffset = V3Scale(normal, 0.012f * std::sin(phase));
                            }
                        }
                    }

                    std::ostringstream oss;
                    oss << "[VER_82][Span] piece cloth=" << clothInPiece
                        << " pinned=" << pinnedInPiece
                        << " soft=" << (softHoist ? 1 : 0)
                        << " root=" << rootAnchors
                        << " maxSpan=" << pieceMaxSpan
                        << " limit=" << (pieceMaxSpan * 2.0f + 0.2f)
                        << " size=(" << (hi.x - lo.x) << ","
                        << (hi.y - lo.y) << ","
                        << (hi.z - lo.z) << ")";
                    GeoPhysLog(oss.str());
                }
            }

            // 整片半径：完全没有锚点时用它兜底，
            // 至少让布料能自然下落/摆动，而不是被 15cm 钉死。
            RwV3d centroid{};

            for (int idx : members)
                centroid = V3Add(centroid, rig.particles[idx].restLocal);

            centroid = V3Scale(centroid, 1.0f / static_cast<float>(members.size()));

            float radius = 0.0f;

            for (int idx : members)
            {
                const float d = V3Len(V3Sub(rig.particles[idx].restLocal, centroid));
                if (d > radius) radius = d;
            }

            if (hasPin) ++pinRootedPieces;
            else if (anchorPoints.empty()) ++unpinnedPieces;

            for (int idx : members)
            {
                auto& p = rig.particles[idx];
                p.pinRooted = hasPin;
                p.softPins  = hasPin && softHoist &&
                    (static_cast<PhysType>(p.typeU32) == PhysType::Cloth);

                if (p.hardAnchor) { p.restSpan = 0.0f; continue; }

                float best = 1e9f;

                for (const auto& a : anchorPoints)
                {
                    const float d = V3Len(V3Sub(p.restLocal, a));
                    if (d < best) best = d;
                }

                // 无锚点（例如整片都是布料、没有刚性边框）：用整片半径兜底
                if (best > 1e8f) best = radius;

                p.restSpan = best;
                if (best > maxSpan) maxSpan = best;

                // 接缝渐隐：离挂点越近，越跟随刚性边的静止姿势。
                // 没有它，第一排布料会直接绕挂点硬折，出现"一刀切断"的观感。
                if (p.softPins)
                {
                    const float t = std::clamp(1.0f - best / 0.35f, 0.0f, 1.0f);
                    p.seamFollow = 0.030f * t;
                }
            }
        }

        {
            std::ostringstream oss;
            oss << "[VER_82][Span] pinRootedPieces=" << pinRootedPieces
                << " softPinPieces=" << softPinPieces
                << " unpinnedPieces=" << unpinnedPieces
                << " maxRestSpan=" << maxSpan << "m";
            GeoPhysLog(oss.str());
        }

        // ============================================================
        // 诊断：逐个"头发连通片"输出规模/挂点/硬锚点/可动半径。
        // 目的：确认尖刺是不是来自"既没有 pin 也没有硬锚点"的片
        //       （它们只靠很弱的锚点回拉吊着，restSpan 退化成整片半径）。
        // ============================================================
        if (rig.hairDiagnostics)
        {
            int printed = 0;

            for (const auto& info : rig.pieces)
            {
                if (info.hairCount == 0) continue;
                if (printed >= 40) break;

                ++printed;

                // span 要在 restSpan 算完之后再统计（注册时还没算）
                float pieceSpan = 0.0f;
                for (const auto& q : rig.particles)
                {
                    if (q.pieceId != printed - 1) continue;
                    if (q.restSpan > pieceSpan) pieceSpan = q.restSpan;
                }

                std::ostringstream oss;
                oss << "[VER_82][HairPiece] #" << printed
                    << " pid=" << (printed - 1)
                    << " id=" << info.repVertex
                    << " n=" << info.count
                    << " hair=" << info.hairCount
                    << " cloth=" << info.clothCount
                    << " pinLinks=" << info.pinLinks
                    << " pinned=" << ((info.pinLinks > 0) ? 1 : 0)
                    << " hard=" << info.hardCount
                    << " span=" << pieceSpan << "m";
                GeoPhysLog(oss.str());
            }
        }
    }

    // ============================================================
    // 轻度抗弯：共享一条边的两个三角形，其"对面顶点"的距离在折弯时会缩短。
    // 用一个很软的约束阻止它塌成一条硬折线，弯曲就会分散到几排布料上，
    // 而不是在挂点处折断、布面和刚性边重叠。
    // 只对"竖向挂点"的软垂旗面加，避免把披风/裙摆变硬。
    // ============================================================
    {
        std::set<uint64_t> addedPairs;
        int bendingConstraints = 0;

        for (const auto& entry : edgeOpposite)
        {
            if (entry.second.size() != 2) continue;

            const int c = entry.second[0];
            const int d = entry.second[1];

            if (c < 0 || d < 0 || c == d) continue;

            if (!rig.particles[c].softPins || !rig.particles[d].softPins) continue;

            const int lo = std::min(c, d);
            const int hi = std::max(c, d);
            const uint64_t pairKey =
                (static_cast<uint64_t>(lo) << 32) | static_cast<uint32_t>(hi);

            if (!addedPairs.insert(pairKey).second) continue;

            const float rest = V3Len(V3Sub(rig.particles[d].restLocal,
                                           rig.particles[c].restLocal));

            if (rest < 1e-4f) continue;

            VertexConstraint bc;
            bc.a = c;
            bc.b = d;
            bc.restLength = rest;
            bc.stiffness  = 0.08f;   // 每帧 10 次迭代后约 0.57，软弯

            rig.constraints.push_back(bc);
            ++bendingConstraints;
        }

        if (bendingConstraints > 0)
        {
            std::ostringstream oss;
            oss << "[VER_82][Bending] constraints=" << bendingConstraints;
            GeoPhysLog(oss.str());
        }
    }

    rig.jiggleScratch.assign(rig.particles.size(), RwV3d{ 0.0f, 0.0f, 0.0f });
    rig.jiggleCount.assign(rig.particles.size(), 0);
    rig.diagScratch.assign(rig.particles.size(), 0.0f);

    // ------------------------------------------------------------
    // 布料平均边长（2026-11-23 08:2x）
    //
    // 世界碰撞的 skin 原来固定 1cm，而"物件自己的网格碰撞体"上的三角形
    // 往往和布料网格同尺度（3~5cm）。skin 比网格间距薄 ⇒ 布从三角形缝隙里漏过去。
    // 这里统计布料-布料约束的平均静止长度，EntityPhysics 用它算
    // ownCol 的 skin = clamp(0.45 × 平均边长, 4cm, 12cm)。
    // ------------------------------------------------------------
    {
        float sum = 0.0f;
        int   cnt = 0;

        for (const auto& c : rig.constraints)
        {
            const PhysType ta = static_cast<PhysType>(rig.particles[c.a].typeU32);
            const PhysType tb = static_cast<PhysType>(rig.particles[c.b].typeU32);

            if (ta != PhysType::Cloth || tb != PhysType::Cloth) continue;

            sum += c.restLength;
            ++cnt;
        }

        rig.clothMeanEdge = (cnt > 0) ? (sum / static_cast<float>(cnt)) : 0.0f;
    }

    // ------------------------------------------------------------
    // [ClothEdge] 布料最长的约束边（建 rig 时打一次）
    //
    // 用途：定位"某一侧裙片被水平拉伸"这类问题。
    // 如果左片里存在一条连到很远顶点的边（restLength 异常长，
    // 或两端的 piece 不同/跨越身体中线），拉伸就是它造成的。
    // ------------------------------------------------------------
    {
        struct EdgeInfo
        {
            float rest = 0.0f;
            int   a = -1;
            int   b = -1;
        };

        std::vector<EdgeInfo> longest;

        for (const auto& c : rig.constraints)
        {
            const PhysType ta = static_cast<PhysType>(rig.particles[c.a].typeU32);
            const PhysType tb = static_cast<PhysType>(rig.particles[c.b].typeU32);

            if (ta != PhysType::Cloth || tb != PhysType::Cloth) continue;

            EdgeInfo e;
            e.rest = c.restLength;
            e.a = c.a;
            e.b = c.b;
            longest.push_back(e);
        }

        std::sort(longest.begin(), longest.end(),
            [](const EdgeInfo& x, const EdgeInfo& y) { return x.rest > y.rest; });

        const float mean = (rig.clothMeanEdge > 1e-6f) ? rig.clothMeanEdge : 1.0f;

        for (size_t i = 0; i < longest.size() && i < 5; ++i)
        {
            const EdgeInfo& e = longest[i];
            const auto& A = rig.particles[e.a];
            const auto& B = rig.particles[e.b];

            std::ostringstream oss;
            oss << std::fixed << std::setprecision(3);
            oss << "[ClothEdge] rest=" << (e.rest * 1000.0f) << "mm"
                << " x" << (e.rest / mean) << "mean"
                << " pieceA=" << A.pieceId
                << " pieceB=" << B.pieceId
                << " A=(" << A.restLocal.x << "," << A.restLocal.y << "," << A.restLocal.z << ")"
                << " B=(" << B.restLocal.x << "," << B.restLocal.y << "," << B.restLocal.z << ")";
            GeoPhysLog(oss.str());
        }
    }

    {
        std::ostringstream hc;
        hc << std::fixed << std::setprecision(3);
        hc << "[HairCfg] ride=" << rig.hairRide
           << " damp=" << rig.hairDamping
           << " pull=" << rig.hairPull
           << " follow=" << rig.hairFollow
           << " acc=" << g_hairAcc
           << " decay=" << g_hairDecay
           << " rot=" << g_hairRot
           << " rigidSkin=" << (rig.hairRigidSkin ? 1 : 0)
           << " bend=" << (g_hairBend ? 1 : 0)
           << " pieces=" << rig.pieces.size();
        GeoPhysLog(hc.str());
    }

    AddHairBendConstraints(rig);

    rig.initialized = true;

    std::ostringstream oss;
    oss << "[VER_82] hair=" << rig.hairCount
        << " cloth=" << rig.clothCount
        << " body=" << rig.bodyCount
        << " soft=" << rig.softCount
        << " rigid=" << rig.rigidCount
        << " total=" << numVerts
        << " constraints=" << rig.constraints.size()
        << " skinAnchor=" << (rig.hasSkinAnchor ? "YES" : "NO")
        << " bones=" << rig.skinBoneCount
        << " splitSkin=" << (skinSplit ? "YES" : "NO");

    report = oss.str();
    GeoPhysLog("[VER_82] ==== SUCCESS ==== " + report);

    // 模型空间模式：计算每个顶点"离固定点有多远"，
    // 风只吹自由边缘，靠近固定处的顶点几乎不动（避免整片一起平移）
    if (rig.modelSpace)
    {
        // 用实例的世界矩阵反算：世界"下"和世界 X/Y 在模型空间里的方向。
        // 这样模型是 Z-up 还是 Y-up 都能正确取到重力方向。
        if (anchorLTM)
        {
            RwMatrix inv;
            if (InvertMatrix(*anchorLTM, inv))
            {
                const RwV3d worldDown = { 0.0f, 0.0f, -1.0f };
                const RwV3d worldX    = { 1.0f, 0.0f, 0.0f };
                const RwV3d worldY    = { 0.0f, 1.0f, 0.0f };

                RwV3d down = TransformDir(inv, worldDown);
                RwV3d wX   = TransformDir(inv, worldX);
                RwV3d wY   = TransformDir(inv, worldY);

                const float ld = V3Len(down);
                const float lx = V3Len(wX);
                const float ly = V3Len(wY);

                if (ld > 1e-4f) rig.gravityDir = V3Scale(down, 1.0f / ld);
                if (lx > 1e-4f) rig.windDirX   = V3Scale(wX, 1.0f / lx);
                if (ly > 1e-4f) rig.windDirY   = V3Scale(wY, 1.0f / ly);

                std::ostringstream gos;
                gos << "[MapPhysics][Gravity] modelSpace gravityDir=("
                    << rig.gravityDir.x << "," << rig.gravityDir.y << "," << rig.gravityDir.z
                    << ") windX=(" << rig.windDirX.x << "," << rig.windDirX.y << "," << rig.windDirX.z
                    << ") windY=(" << rig.windDirY.x << "," << rig.windDirY.y << "," << rig.windDirY.z
                    << ")";
                GeoPhysLog(gos.str());
            }
        }

        float maxDist = 0.0f;
        int   fixedCount = 0;

        for (const auto& p : rig.particles)
        {
            if (p.invMass <= 0.0f) ++fixedCount;
        }

        if (fixedCount > 0)
        {
            for (auto& p : rig.particles)
            {
                if (p.invMass <= 0.0f) { p.freedom = 0.0f; continue; }

                float best = 1e9f;

                for (const auto& q : rig.particles)
                {
                    if (q.invMass > 0.0f) continue;

                    const float d = V3Len(V3Sub(p.restLocal, q.restLocal));
                    if (d < best) best = d;
                }

                p.freedom = best;
                if (best > maxDist) maxDist = best;
            }

            if (maxDist > 1e-4f)
            {
                for (auto& p : rig.particles)
                    p.freedom = std::clamp(p.freedom / maxDist, 0.10f, 1.0f);
            }
        }
    }

    // Body 粒子：计算到"未刷色顶点（边界）"的渐隐系数，
    // 对所有物理粒子计算（人物/车辆/地图物件通用）：
    // 让位移在刷色边界处平滑衰减，避免与静态网格之间出现硬边。
    // （人物 / 车辆 / 地图物件通用）
    {
        int fadeCount = 0;

        for (const auto& p : rig.particles)
            if (p.invMass >= 0.0f) ++fadeCount;

        if (fadeCount > 0 && geometry)
        {
            const int nv2 = static_cast<int>(geometry->numVertices);
            const RwV3d* vtx = RpMorphTargetGetVertices(&geometry->morphTarget[0]);

            if (vtx)
            {
                // 收集"非物理顶点"作为边界参考
                std::vector<RwV3d> border;
                border.reserve(1024);

                for (int v = 0; v < nv2; ++v)
                {
                    if (rig.vertexToParticle[v] < 0)
                        border.push_back(vtx[v]);
                }

                const float fadeRange = 0.06f;   // 6 cm 内渐隐

                for (auto& p : rig.particles)
                {
                    // 所有物理粒子都参与边界渐隐

                    float best = 1e9f;

                    for (const auto& b : border)
                    {
                        const float d = V3LenSq(V3Sub(p.restLocal, b));
                        if (d < best) best = d;
                    }

                    const float d = (best < 1e8f) ? std::sqrt(best) : fadeRange;

                    p.boundaryFade = std::clamp(d / fadeRange, 0.0f, 1.0f);
                    p.boundaryFade = p.boundaryFade * p.boundaryFade * (3.0f - 2.0f * p.boundaryFade);
                }

                std::ostringstream bos;
                bos << "[VER_82][BoundaryFade] particles=" << fadeCount << " border=" << border.size()
                    << " range=" << fadeRange;
                GeoPhysLog(bos.str());
            }
        }
    }

    return true;
}


static int g_stepFrame = 0;

void (*g_worldCollideFn)(RwV3d& point, const RwV3d& prev) = nullptr;

// ============================================================
//  天气风力系统
//
//  每个天气类型可以单独配置：风向(度)、风力强度、湍流频率(Hz)。
//  之前风只在"地图物件模型空间"分支里生效，人物 rig 完全没风；
//  这里把它做成通用的、按天气取参数的世界空间风力。
// ============================================================
namespace
{
    constexpr int kMaxWeatherId = 23;

    WeatherWindConfig g_weatherWind[kMaxWeatherId + 1];
    bool              g_weatherWindSet[kMaxWeatherId + 1] = {};

    WeatherWindConfig g_defaultWeatherWind;
    float             g_windMultiplier = 1.0f;
    float             g_windTime = 0.0f;

    int GetCurrentWeatherId()
    {
        int w = static_cast<int>(CWeather::NewWeatherType);

        if (w < 0 || w > kMaxWeatherId)
            w = static_cast<int>(CWeather::ForcedWeatherType);

        if (w < 0 || w > kMaxWeatherId)
            w = 0;

        return w;
    }
}

void SetWeatherWind(int weatherId, float dirDeg, float strength, float turbFreq)
{
    if (weatherId < 0 || weatherId > kMaxWeatherId) return;

    g_weatherWind[weatherId].dirDeg   = dirDeg;
    g_weatherWind[weatherId].strength = std::max(0.0f, strength);
    g_weatherWind[weatherId].turbFreq = std::max(0.0f, turbFreq);
    g_weatherWindSet[weatherId] = true;
}

void SetDefaultWeatherWind(float dirDeg, float strength, float turbFreq)
{
    g_defaultWeatherWind.dirDeg   = dirDeg;
    g_defaultWeatherWind.strength = std::max(0.0f, strength);
    g_defaultWeatherWind.turbFreq = std::max(0.0f, turbFreq);
}

void SetWindMultiplier(float mul)
{
    g_windMultiplier = std::clamp(mul, 0.0f, 10.0f);
}

void AdvanceWindTime(float deltaTime)
{
    g_windTime += std::clamp(deltaTime, 0.0f, 0.1f);
}

void SampleWindAcceleration(const RwV3d& pos, RwV3d& outAccel)
{
    outAccel = { 0.0f, 0.0f, 0.0f };

    const int weatherId = GetCurrentWeatherId();

    const WeatherWindConfig& cfg = g_weatherWindSet[weatherId]
        ? g_weatherWind[weatherId]
        : g_defaultWeatherWind;

    const float strength = cfg.strength * g_windMultiplier;
    if (strength <= 0.0001f) return;

    const float rad = cfg.dirDeg * 3.14159265358979f / 180.0f;

    const float dirX = std::cos(rad);
    const float dirY = std::sin(rad);

    // 湍流：按位置错开相位，两个八度，避免整片同步摆动
    float turb = 0.0f;

    if (cfg.turbFreq > 0.0001f)
    {
        const float phase = pos.x * 0.7f + pos.y * 0.9f + pos.z * 0.4f;
        const float w1 = 6.28318530717959f * cfg.turbFreq;

        turb = 0.55f * std::sin(w1 * g_windTime + phase)
             + 0.30f * std::sin(w1 * 1.9f * g_windTime + phase * 1.7f + 1.3f);
    }

    const float gust = 1.0f + 0.6f * turb;

    outAccel.x = dirX * strength * gust;
    outAccel.y = dirY * strength * gust;
    outAccel.z = strength * 0.15f * turb;
}

void StepGeometryPhysics(
    GeometryPhysics& rig,
    RpAtomic* atomic,
    float deltaTime)
{
    if (!rig.initialized || rig.particles.empty()) return;

    // 允许的最大位移：粒子只能绕最近锚点旋转到另一侧，即 2×restSpan。
    // 以前是"离自己的静止位置不超过 restSpan + 15cm"；AutoAnchor 移除后
    // restSpan 常常是 0，等于只给 15cm —— 整片布料几乎不能动，
    // 看起来就是"被无形的力提住，永远落不下来"。
    auto particleLimit = [](const VertexParticle& p) -> float
    {
        const PhysType t = static_cast<PhysType>(p.typeU32);

        if (t == PhysType::Hair || t == PhysType::Cloth || t == PhysType::SoftBody)
        {
            const float lead = (t == PhysType::Hair) ? 0.10f : 0.20f;
            const float cap  = (t == PhysType::Hair) ? 4.00f
                             : (t == PhysType::Cloth) ? 10.0f : 3.0f;

            return std::clamp(std::max(p.restSpan, 0.0f) * 2.0f + lead, lead, cap);
        }

        return MaxAnchorDistance(t);
    };

    // ============================================================
    // 模型空间模式（静态地图物件：共享几何体，风摆）
    // 没有骨骼、没有世界变换，直接把顶点在模型空间里吹动
    // ============================================================
    if (rig.modelSpace)
    {
        const float dt = std::clamp(deltaTime, 1.0f / 240.0f, 1.0f / 20.0f) * 0.5f;  // 两个子步
        const float wind = rig.windStrength;
        const float airDrag = std::pow(0.96f, dt * 60.0f);   // 空气阻力（按帧时间标定）

        for (int sub = 0; sub < 2; ++sub)
        {
            const float t = (static_cast<float>(g_stepFrame) * 2.0f + static_cast<float>(sub)) * dt;

            // 低频阵风：整片同向
            const float gust = 0.65f + 0.35f * std::sin(t * 0.55f);

            const RwV3d windAcc =
            {
                2.4f * wind * gust * std::sin(t * 0.33f),
                1.2f * wind * gust * std::sin(t * 0.27f + 1.3f),
                0.0f
            };

            // 世界空间的风向量 → 模型空间
            const RwV3d windModel = V3Add(
                V3Scale(rig.windDirX, windAcc.x),
                V3Scale(rig.windDirY, windAcc.y));

            for (auto& p : rig.particles)
            {
                if (p.invMass <= 0.0f) continue;   // 固定点保持不动

                RwV3d vel = V3Sub(p.posWorld, p.prevWorld);
                vel = V3Scale(vel, airDrag);       // 空气阻力

                // 湍流：中等空间频率，让波在布面上传播
                // （相邻顶点相位略有差异 → 有起伏，而不是整体平移）
                const float phaseA = t * 1.9f + p.restLocal.z * 2.4f + p.restLocal.x * 1.6f;
                const float phaseB = t * 2.7f - p.restLocal.z * 3.3f + p.restLocal.y * 1.2f;

                const float turbX = std::sin(phaseA) * 0.9f + std::sin(phaseB * 0.8f) * 0.5f;
                const float turbY = std::sin(phaseB) * 0.6f;
                const float turbZ = std::sin(phaseA * 0.9f + 1.7f) * 0.4f;

                // 风在自由边缘更强（freedom：0 固定 → 1 最自由）
                // 软度（G）与边界渐隐共同决定受风强度：
                //   G 越小（越硬）摆得越少；靠近未刷色边界的地方渐隐到 0。
                const float softnessN = std::clamp(1.0f - p.stiffness, 0.0f, 1.0f);
                const float gain = (0.20f + 0.80f * p.freedom) * wind
                                 * (0.35f + 0.65f * softnessN)
                                 * p.boundaryFade;

                // 形状记忆：有挂点的布料完全不要——那股回拉正是"把布料提在
                // 静止姿势的隐形力"；没有挂点的片保留一点，只防止被吹散。
                const RwV3d toRest = V3Sub(p.restLocal, p.posWorld);
                const float shapeMemory = p.pinRooted ? 0.0f : 3.5f;

                // 重力沿"实例反算出来的下方向"
                const PhysType msType = static_cast<PhysType>(p.typeU32);

                float gravityScale = 9.8f * (0.30f + 0.70f * p.freedom);

                if (msType == PhysType::Cloth)
                    gravityScale *= rig.clothGravity;

                const RwV3d acc =
                {
                    windModel.x * gain + rig.windDirX.x * turbX + rig.windDirY.x * turbY
                        + rig.gravityDir.x * gravityScale + toRest.x * shapeMemory,

                    windModel.y * gain + rig.windDirX.y * turbX + rig.windDirY.y * turbY
                        + rig.gravityDir.y * gravityScale + toRest.y * shapeMemory,

                    windModel.z * gain + rig.windDirX.z * turbX + rig.windDirY.z * turbY
                        + rig.gravityDir.z * gravityScale + toRest.z * shapeMemory
                };

                p.prevWorld = p.posWorld;

                RwV3d next = V3Add(V3Add(p.posWorld, vel), V3Scale(acc, dt * dt));

                // 速度限幅，防止数值抖动
                RwV3d delta = V3Sub(next, p.posWorld);
                const float len = V3Len(delta);
                if (len > 0.08f) next = V3Add(p.posWorld, V3Scale(delta, 0.08f / len));

                p.posWorld = next;
            }

            // 模型空间用较少的迭代：太硬会让下摆变成刚体整体平移
            const int iters = std::max(4, rig.iterations / 2);

            for (int it = 0; it < iters; ++it)
            {
                for (auto& c : rig.constraints)
                {
                    auto& A = rig.particles[c.a];
                    auto& B = rig.particles[c.b];

                    const float wa = (A.invMass <= 0.0f) ? 0.0f : A.invMass;
                    const float wb = (B.invMass <= 0.0f) ? 0.0f : B.invMass;
                    const float ws = wa + wb;
                    if (ws < 1e-6f) continue;

                    RwV3d d = V3Sub(B.posWorld, A.posWorld);
                    const float d2 = V3LenSq(d);
                    if (d2 < 1e-10f) continue;

                    const float dLen = std::sqrt(d2);
                    const float diff = (dLen - c.restLength) / dLen;
                    const float wA = (wa / ws) * c.stiffness;
                    const float wB = (wb / ws) * c.stiffness;

                    A.posWorld = V3Add(A.posWorld, V3Scale(d, diff * wA));
                    B.posWorld = V3Sub(B.posWorld, V3Scale(d, diff * wB));
                }

                // 挂点约束：模型空间里刚体顶点是静止的，目标就是它的静止位置。
                // 以前模型空间分支完全不解挂点，布料只能靠"形状记忆"吊着，
                // 这也是它不会自然垂下来的原因之一。
                for (auto& p : rig.particles)
                {
                    if (p.pins.empty() || p.invMass <= 0.0f) continue;

                    const float alpha = p.softPins ? rig.clothPinStiffness : 1.0f;
                    if (alpha <= 0.0f) continue;

                    RwV3d wantSum = { 0.0f, 0.0f, 0.0f };
                    float weightSum = 0.0f;

                    for (const auto& pin : p.pins)
                    {
                        RwV3d d = V3Sub(p.posWorld, pin.targetWorld);
                        const float len = V3Len(d);

                        if (len < 1e-5f) continue;

                        const RwV3d want = V3Add(pin.targetWorld,
                            V3Scale(d, pin.restLength / len));

                        // 与主分支一致：多挂点时求加权一致解，而不是"最后一条赢"
                        const float w = 1.0f /
                            std::max(pin.restLength * pin.restLength, 1e-6f);

                        wantSum = V3Add(wantSum, V3Scale(want, w));
                        weightSum += w;
                    }

                    if (weightSum <= 1e-6f) continue;

                    const RwV3d want = V3Scale(wantSum, 1.0f / weightSum);

                    p.posWorld = (alpha >= 0.999f)
                        ? want
                        : V3Add(p.posWorld,
                            V3Scale(V3Sub(want, p.posWorld), alpha));
                }
            }
        }

        // 限幅：有挂点的片允许绕挂点转到另一侧（2×restSpan）；
        // 没有挂点的片仍然只允许小幅漂移，避免整片被吹走。
        for (auto& p : rig.particles)
        {
            if (p.invMass <= 0.0f) continue;

            const float limit = p.pinRooted ? particleLimit(p) : 0.55f;
            const RwV3d toRest = V3Sub(p.posWorld, p.restLocal);
            const float len = V3Len(toRest);

            if (len > limit)
                p.posWorld = V3Add(p.restLocal, V3Scale(toRest, limit / len));
        }

        ++g_stepFrame;

        // 诊断：模型空间模式的实际位移量（判断"没动"还是"动了但没画出来"）
        if ((g_stepFrame % 900) == 0)
        {
            float maxOffset = 0.0f;
            int moved = 0;

            for (const auto& p : rig.particles)
            {
                const float d = V3Len(V3Sub(p.posWorld, p.restLocal));
                if (d > 0.0005f) ++moved;
                if (d > maxOffset) maxOffset = d;
            }

            std::ostringstream oss;
            oss << "[MapPhysics][StepMap] particles=" << rig.particles.size()
                << " moved=" << moved
                << " maxOffset=" << maxOffset << "m"
                << " wind=" << rig.windStrength;
            GeoPhysLog(oss.str());
        }

        return;
    }

    if (!atomic) return;

    RpGeometry* geometry = RpAtomicGetGeometry(atomic);
    if (!geometry) return;

    if (deltaTime <= 0.0f) deltaTime = 1.0f / 60.0f;
    if (deltaTime > 0.1f)  deltaTime = 0.1f;

    RpSkin* skin = RpSkinGeometryGetSkin(geometry);
    RpHAnimHierarchy* hierarchy = RpSkinAtomicGetHAnimHierarchy(atomic);

    RwMatrix* boneMatrices = hierarchy
        ? RpHAnimHierarchyGetMatrixArray(hierarchy) : nullptr;
    const RwMatrix* inverseBinds = skin
        ? RpSkinGetSkinToBoneMatrices(skin) : nullptr;

    const RwUInt32 maxBone = std::min<RwUInt32>(rig.skinBoneCount, kMaxBones);

    const bool haveSkinAnchor =
        rig.hasSkinAnchor && boneMatrices && inverseBinds && maxBone > 0;

    // 无蒙皮模型（车辆 / 物体 / 建筑）：用原子的世界矩阵当锚点变换
    RwMatrix* atomicLTM = RwFrameGetLTM(RpAtomicGetFrame(atomic));

    RwMatrix* clumpLTM = nullptr;
    {
        RpClump* clump = RpAtomicGetClumpMacro(atomic);
        if (clump) clumpLTM = RwFrameGetLTM(RpClumpGetFrame(clump));
    }

    // 非蒙皮模型（车辆部件 / 道具 / 建筑）的世界锚点必须用**原子自己的 frame**。
    // clump 根节点只适合"整个模型就是一个原子"的情况；车辆/飞机是多 atomic
    // 模型，车门/机翼/布料各有自己的局部偏移。用 clump 会让物理粒子停在
    // 根节点附近，而渲染在部件上——碰撞检测点也就和布料视觉位置错开了。
    // 蒙皮模型仍然用骨骼矩阵（上面 already haveSkinAnchor 分支）。
    RwMatrix* anchorLTM = atomicLTM ? atomicLTM : clumpLTM;

    if (!haveSkinAnchor && !anchorLTM) return;

    // 每帧重置拉伸修正诊断计数（在 limitStretch 之前）
    rig.diagBodyEdgeFixes = 0;
    rig.diagPinEdgeFixes  = 0;
    rig.diagMaxExcess     = 0.0f;

    // 每帧重置头发惯性诊断计数
    rig.hfPops     = 0;
    rig.hfClamped  = 0;
    rig.hfMaxMove  = 0.0f;
    rig.hfMaxAcc   = 0.0f;
    rig.hfMaxSpeed = 0.0f;
    rig.hfRotDeg   = 0.0f;
    rig.hfMaxImpulse = 0.0f;
    rig.hfImpulseClamped = 0;
    rig.diagWorldCorrClamped = 0;
    rig.diagMaxWorldCorr = 0.0f;
    rig.diagHitKind = 0;
    rig.diagHitModel = -1;
    WorldCollision_ResetGroundLiftRejected();

    // ============================================================
    // 头发骨骼的"本帧旋转增量"（只有 hairrotrans=1 才用）
    //
    // 所有头发粒子共用同一根骨骼（HairRebind 的结果），所以可以只算一次：
    //   D = inv(boneMatrix_prev) * boneMatrix_cur     （世界空间，行向量）
    // 把一个"相对头部固定"的偏移从上一帧搬到本帧。
    // ============================================================
    RwMatrix hairRotDelta{};
    bool haveHairRotDelta = false;

    if (rig.hairRotTransport && haveSkinAnchor &&
        rig.hairBoneIndex >= 0 &&
        rig.hairBoneIndex < static_cast<int>(maxBone))
    {
        const RwMatrix& cur = boneMatrices[rig.hairBoneIndex];

        if (rig.hairBoneMatValid)
        {
            RwMatrix invPrev;

            if (InvertMatrix(rig.hairBoneMatPrev, invPrev))
            {
                RwMatrixMultiply(&hairRotDelta, &invPrev, &cur);

                // 只取旋转部分：平移由 anchorMove 单独处理
                hairRotDelta.pos = { 0.0f, 0.0f, 0.0f };
                hairRotDelta.flags = rwMATRIXTYPEORTHONORMAL;
                haveHairRotDelta = true;

                // 本帧头部骨骼转了多少度（诊断用：坡面/转身突变会看到 >5°/帧）
                const float trace =
                    hairRotDelta.right.x + hairRotDelta.up.y + hairRotDelta.at.z;
                const float cosT = std::clamp((trace - 1.0f) * 0.5f, -1.0f, 1.0f);
                rig.hfRotDeg = std::acos(cosT) * 57.2957795f;
            }
        }

        rig.hairBoneMatPrev = cur;
        rig.hairBoneMatValid = true;
    }
    else
    {
        rig.hairBoneMatValid = false;
    }

    // A/B 诊断开关状态（一次性日志，方便确认这次跑的是哪一组）
    if (!rig.hairTestLogged)
    {
        rig.hairTestLogged = true;

        std::ostringstream oss;
        oss << "[VER_82][HairTest] pins=" << (rig.hairPinsEnabled ? 1 : 0)
            << " capsuleMask=" << (rig.hairCapsuleMaskEnabled ? 1 : 0)
            << " rootStretchExempt=" << (rig.hairRootStretchExempt ? 1 : 0)
            << " pieceFix=" << (rig.hairPieceFixEnabled ? 1 : 0)
            << " rotrans=" << (rig.hairRotTransport ? 1 : 0)
            << " inertia=" << (rig.hairInertiaEnabled ? 1 : 0)
            << " worldCollide=" << (rig.hairWorldCollideEnabled ? 1 : 0)
            << " diag=" << (rig.hairDiagnostics ? 1 : 0);
        GeoPhysLog(oss.str());
    }

    // ============================================================
    // 头发兜底骨骼绑定（读不到逐顶点权重时）
    //
    // 现象：日志一直是 [NoSkinWeight] particles=N/N + [WeightInherit] inherited=0，
    // 但 skinAnchor=YES。也就是说克隆皮肤里的逐顶点权重读不出来（DFF 里是好的：
    // tools/hair_bonebind.py 可以验证 tk7Chi2 的头发 100% 绑在 neck 上）。
    // 结果 anchorWorld / pin.targetWorld / 写回全都退化成 atomicLTM 刚体分支，
    // 而引擎渲染用的是骨骼矩阵（颈骨）。两者差一个"颈骨相对人物根的动画位移 δ"，
    // 于是：
    //   * 静止时发根被钉在"绑定姿势的头皮"上，画面却画在"动画后的头皮"上，
    //     差 δ，看起来就是后脑勺还剩一小块凸起；
    //   * 起步/停步时颈部动画 δ 突变，头发被整体拽一下（方向由 δ 决定，
    //     所以会看到"起步往前拉、停步往后拉"这种反直觉的拉扯）。
    //
    // 处理：读不到权重时，按"骨骼绑定位置最近"把头发绑到最合适的那根骨骼上。
    // 候选骨骼只取上半身（头/颈/上胸/脊）：长发垂到肩时最近点可能是肩关节，
    // 绑到手臂骨上会让头发跟着手臂飞，必须排除。
    // 只对 Hair 生效，Cloth / Body / SoftBody 保持原来的刚体锚点。
    // ============================================================
    if (haveSkinAnchor && !rig.hairRebindDone)
    {
        rig.hairRebindDone = true;

        // 骨骼的绑定(模型空间)位置 = skinToBone 逆矩阵的平移
        std::vector<RwV3d> bindPos(maxBone, RwV3d{ 0.0f, 0.0f, 0.0f });
        std::vector<char>  bindValid(maxBone, 0);

        for (RwUInt32 b = 0; b < maxBone; ++b)
        {
            RwMatrix inv;
            if (InvertMatrix(inverseBinds[b], inv))
            {
                bindPos[b] = inv.pos;
                bindValid[b] = 1;
            }
        }

        // 骨骼节点 ID -> 骨骼索引（和骨胶囊那段一致）
        std::map<int, int> boneByNodeId;
        if (hierarchy && hierarchy->pNodeInfo)
        {
            for (RwInt32 i = 0; i < hierarchy->numNodes; ++i)
            {
                const RpHAnimNodeInfo& info = hierarchy->pNodeInfo[i];
                int idx = i;
                if (info.nodeIndex >= 0 && static_cast<RwUInt32>(info.nodeIndex) < maxBone)
                    idx = info.nodeIndex;
                if (idx >= 0 && static_cast<RwUInt32>(idx) < maxBone)
                    boneByNodeId[info.nodeID] = idx;
            }
        }

        // 候选骨骼按"优先顺序"排：头骨 > 颈 > 上胸 > 脊。
        //   模型作者在 Blender 里把头发全部刷在 Head 上（DFF 里 Head/Neck 的
        //   绑定位置只差 4mm，光看坐标分不出来，以 Blender 为准）。
        //
        //   **所有头发必须绑同一根骨骼**：引擎渲染色头发用的是它自己的权重
        //   （一根骨骼），插件写回用的是"粒子自己的骨骼矩阵"的逆。如果相邻粒子
        //   绑到不同骨骼，写回就会差一个 M_engine·M_ours⁻¹ —— 跑动/前倾时
        //   那几个顶点会被整体甩出 10~30cm，在头皮上拉出"尖刺"。
        //   所以这里先投票选出唯一一根，再全部绑上。
        std::vector<int> candidates;
        const int upperBodyIds[] =
        {
            BONE_HEAD, BONE_NECK, BONE_UPPERTORSO, BONE_SPINE1,
        };

        for (int id : upperBodyIds)
        {
            auto it = boneByNodeId.find(id);
            if (it != boneByNodeId.end() && bindValid[it->second])
                candidates.push_back(it->second);
        }

        if (candidates.empty())
        {
            for (RwUInt32 b = 0; b < maxBone; ++b)
                if (bindValid[b]) candidates.push_back(static_cast<int>(b));
        }

        int rebound = 0;
        int reboundPins = 0;
        std::map<int, int> usedBones;

        // ---- 第一遍：给每个头发粒子投一票（谁最合适）----
        int   winnerBone = -1;
        int   winnerVotes = 0;
        float winnerDist = 1e9f;

        for (auto& p : rig.particles)
        {
            if (static_cast<PhysType>(p.typeU32) != PhysType::Hair) continue;
            if (p.dominantBone >= 0) continue;          // 有真权重就不动
            if (candidates.empty()) continue;

            // 先求"最近的候选距离"，再按优先级取第一根"足够近"的骨骼：
            // 头骨/颈骨绑定位置只差几毫米，所以"差不多近"时按优先级取头骨。
            float minDist = 1e9f;

            for (int b : candidates)
            {
                const float d = V3LenSq(V3Sub(p.restLocal, bindPos[b]));
                if (d < minDist) minDist = d;
            }

            if (minDist > 1e8f) continue;

            int bestBone = candidates.front();

            for (int b : candidates)
            {
                const float d = V3LenSq(V3Sub(p.restLocal, bindPos[b]));

                if (d <= minDist * 1.5625f)     // 1.25²
                {
                    bestBone = b;
                    break;
                }
            }

            usedBones[bestBone] += 1;

            if (usedBones[bestBone] > winnerVotes ||
                (usedBones[bestBone] == winnerVotes && minDist < winnerDist))
            {
                winnerVotes = usedBones[bestBone];
                winnerBone = bestBone;
                winnerDist = minDist;
            }
        }

        // ---- 第二遍：整片头发统一绑到这一根骨骼（关键：不能逐顶点换骨）----
        int pinsForced = 0;

        if (winnerBone >= 0)
        for (auto& p : rig.particles)
        {
            if (static_cast<PhysType>(p.typeU32) != PhysType::Hair) continue;
            if (p.dominantBone >= 0) continue;

            for (int k = 0; k < 4; ++k) { p.bones[k] = -1; p.weights[k] = 0.0f; }
            p.bones[0] = winnerBone;
            p.weights[0] = 1.0f;
            p.dominantBone = winnerBone;
            p.dominantWeight = 1.0f;
            ++rebound;

            // 挂点目标也必须用同一根骨骼。
            // 旧代码只改"读不到权重的 pin"，保留了自己读到权重的 pin —— 那些权重
            // 可能是错位/别的骨骼（读权重这条路整体不可信），于是同一个发根上
            // 会出现"锚点=Head、pin=别的骨骼"的互相拉扯：pin 每轮硬投影、边长约束
            // 又把发根拉回来，中间的三角形被拉成尖刺（[PieceDrift] 里 edge=9~24 的
            // 那些边就是这种"被两个约束对拉"的边）。
            // 现在全部 pin 统一到整片头发的骨骼。
            for (auto& pin : p.pins)
            {
                if (pin.bones[0] >= 0) ++pinsForced;

                for (int k = 0; k < 4; ++k)
                {
                    pin.bones[k] = -1;
                    pin.weights[k] = 0.0f;
                }

                pin.bones[0] = winnerBone;
                pin.weights[0] = 1.0f;
                ++reboundPins;
            }
        }

        rig.hairBoneIndex = winnerBone;

        std::ostringstream oss;
        oss << "[VER_82][HairRebind] particles=" << rebound
            << " pins=" << reboundPins
            << " pinsForced=" << pinsForced
            << " candidates=" << candidates.size()
            << " bone=" << winnerBone;

        // 顺便把骨骼的 GTA nodeID 打出来（8 = Head，5 = Neck），方便核对
        for (const auto& entry : boneByNodeId)
        {
            if (entry.second == winnerBone)
            {
                oss << "(nodeId=" << entry.first << ")";
                break;
            }
        }

        oss << " votes=";

        int shown = 0;
        for (const auto& entry : usedBones)
        {
            if (shown++) oss << ",";
            oss << entry.first << ":" << entry.second;
            if (shown >= 8) break;
        }

        oss << " (读不到逐顶点权重 -> 整片头发统一绑一根骨骼)";
        GeoPhysLog(oss.str());
    }

    // ============================================================
    // 无锚点头发连通片：改成跟随骨骼（刚性）
    //
    // 现象/证据（轮次0 日志的 [HairPiece]）：
    //   这个 ped 的头发有 5 个连通片既没有 pin（pinLinks=0）也没有硬锚点：
    //   id=12113(52)、12348(52)、12424(27)、12439(27)、12555(43)，共 201 个顶点；
    //   它们的 restSpan 退化成"整片半径"（0.16~0.20m）⇒ 自由上限
    //   particleLimit = 2·span+0.1 ≈ 0.49m，而唯一的约束只有很弱的锚点回拉
    //   （pull = anchorFactor²·returnStrength ≈ 0.014/迭代）。
    //   结果：整片以重力/惯性稳态悬挂在离锚点 0.2~0.25m 的位置
    //   （[Step] hair max=0.254m 正是这个量级），而且"整片一起平移"，
    //   所以"离邻居平均位置"的尖刺比看不出来 —— 它就是那条细长的刀片。
    //
    // 处理：这些片在网格拓扑上没有任何附着，在引擎里也是 100% 刚性跟随骨骼，
    // 因此物理上唯一正确的做法就是让它们跟随骨骼，而不是给它们编造一个附着点。
    // 只影响"既无 pin 也无硬锚点"的片（本例 201/1473 个顶点），其余头发照常物理。
    // ini 的 hairpiecefix=0 可关闭做对照。
    // ============================================================
    if (rig.hairPieceFixEnabled && !rig.pieces.empty() && !rig.hairPieceFixDone)
    {
        rig.hairPieceFixDone = true;

        int fixedPieces = 0;
        int fixedParticles = 0;

        for (size_t pi = 0; pi < rig.pieces.size(); ++pi)
        {
            const auto& info = rig.pieces[pi];

            if (info.hairCount <= 0) continue;
            if (info.pinLinks > 0 || info.hardCount > 0) continue;

            ++fixedPieces;

            for (auto& p : rig.particles)
            {
                if (p.pieceId != static_cast<int>(pi)) continue;
                if (static_cast<PhysType>(p.typeU32) != PhysType::Hair) continue;

                p.hardAnchor = true;
                p.anchorFactor = 0.0f;
                p.restSpan = 0.0f;
                ++fixedParticles;
            }
        }

        if (fixedPieces > 0)
        {
            std::ostringstream oss;
            oss << "[VER_82][HairPieceFix] pieces=" << fixedPieces
                << " particles=" << fixedParticles
                << " (无 pin/无硬锚点的头发片改为跟随骨骼)";
            GeoPhysLog(oss.str());
        }
    }

    // 锚点 = 顶点在"当前骨骼姿态"下的世界位置（与引擎蒙皮完全一致）
    // ------------------------------------------------------------
    // 头发"统一骨骼"解析（2026-11-26）
    //
    // 4.4 修好 8 位骨骼索引后，头发开始出现"靠近发根的一段被向后拉伸"：
    // 因为每个发丝顶点现在按**自己的**权重混合骨骼矩阵，发根附近几个顶点
    // 的权重分布不同（有的偏 Head、有的偏 Neck/Spine），角色一移动，
    // 相邻顶点的锚点就按不同骨骼走 → 它们之间的间距被拉开 = 拉伸。
    //
    // 4.3 之前之所以"正常"，恰恰是因为索引读错、所有权重都无效，
    // 整片头发退回原子矩阵（刚性整片跟随），所以不会拉扯。
    //
    // 这里恢复"整片刚性"的行为，但用**正确的骨骼**：对整片头发投票，
    // 选出权重总和最大的那根骨骼，所有发丝粒子 + 全部挂点都用它的矩阵。
    // 单骨骼矩阵是正交的，长度守恒，不可能把位移放大。
    // ------------------------------------------------------------
    if (rig.hairRigidSkin && haveSkinAnchor && rig.hairPieceBone.empty())
    {
        rig.hairPieceBone.assign(rig.pieces.size(), -1);

        for (size_t pi = 0; pi < rig.pieces.size(); ++pi)
        {
            if (rig.pieces[pi].hairCount <= 0) continue;

            std::map<int, float> votes;
            RwV3d centroid = { 0.0f, 0.0f, 0.0f };
            int   cnt = 0;

            for (const auto& bp : rig.particles)
            {
                if (bp.pieceId != static_cast<int>(pi)) continue;
                if (static_cast<PhysType>(bp.typeU32) != PhysType::Hair) continue;

                for (int k = 0; k < 4; ++k)
                {
                    const int b = bp.bones[k];
                    if (b < 0 || b >= static_cast<int>(maxBone)) continue;
                    if (bp.weights[k] <= 0.0f) continue;
                    votes[b] += bp.weights[k];
                }

                centroid = V3Add(centroid, bp.anchorWorld);
                ++cnt;
            }

            int   bone  = -1;
            float bestW = 0.0f;
            const char* via = "vote";

            for (const auto& kv : votes)
                if (kv.second > bestW) { bestW = kv.second; bone = kv.first; }

            if (bone < 0 && cnt > 0 && hierarchy)
            {
                via = "nearest";
                centroid = V3Scale(centroid, 1.0f / static_cast<float>(cnt));

                float bestD = 1e30f;

                for (RwInt32 i = 0; i < hierarchy->numNodes; ++i)
                {
                    const float dx = boneMatrices[i].pos.x - centroid.x;
                    const float dy = boneMatrices[i].pos.y - centroid.y;
                    const float dz = boneMatrices[i].pos.z - centroid.z;
                    const float d2 = dx * dx + dy * dy + dz * dz;

                    if (d2 < bestD) { bestD = d2; bone = i; }
                }
            }

            rig.hairPieceBone[pi] = bone;

            std::ostringstream pb;
            pb << "[PieceBind] piece=" << pi
               << " hair=" << rig.pieces[pi].hairCount
               << " bone=" << bone << " via=" << via;
            GeoPhysLog(pb.str());
        }
    }


    int placedCount = 0;
    int blendFallbacks = 0;
    int fallbackSamples[3] = { -1, -1, -1 };

    for (auto& p : rig.particles)
    {
        RwMatrix blended;
        bool usedFallback = false;

        // 每片头发用各自的骨骼（2026-11-27）：尾巴片会绑到尾骨，而不是头骨
        const int pieceBone =
            (p.pieceId >= 0 && p.pieceId < static_cast<int>(rig.hairPieceBone.size()))
                ? rig.hairPieceBone[p.pieceId] : -1;

        const bool hairRigid =
            rig.hairRigidSkin &&
            static_cast<PhysType>(p.typeU32) == PhysType::Hair &&
            pieceBone >= 0 &&
            pieceBone < static_cast<int>(maxBone);

        if (hairRigid)
        {
            RwMatrixMultiply(&blended,
                &inverseBinds[pieceBone], &boneMatrices[pieceBone]);
        }

        if (hairRigid ||
            (haveSkinAnchor &&
             BuildVertexMatrix(boneMatrices, inverseBinds, p, maxBone, blended, &usedFallback)))
        {
            p.anchorWorld = TransformPoint(blended, p.restLocal);

            // 挂点目标：必须用"该 Rigid 顶点自己的骨骼权重"算，
            // 不能借用物理顶点的矩阵（两者权重可能完全不同）
            for (auto& pin : p.pins)
            {
                if (hairRigid)
                {
                    // 统一骨骼：挂点也用同一矩阵，保证发根与发丝不会相对漂移
                    pin.targetWorld = TransformPoint(blended, pin.restLocal);
                    continue;
                }

                VertexParticle tmp;
                tmp.restLocal = pin.restLocal;

                for (int k = 0; k < 4; ++k)
                {
                    tmp.bones[k] = pin.bones[k];
                    tmp.weights[k] = pin.weights[k];
                }

                RwMatrix pinMatrix;

                if (BlendVertexMatrix(boneMatrices, inverseBinds, tmp, maxBone, pinMatrix))
                    pin.targetWorld = TransformPoint(pinMatrix, pin.restLocal);
                else
                    pin.targetWorld = TransformPoint(blended, pin.restLocal);
            }

            if (usedFallback)
            {
                ++blendFallbacks;
                if (blendFallbacks <= 3) fallbackSamples[blendFallbacks - 1] = p.geometryVertexIndex;
            }
        }
        else if (anchorLTM)
        {
            p.anchorWorld = TransformPoint(*anchorLTM, p.restLocal);

            for (auto& pin : p.pins)
                pin.targetWorld = TransformPoint(*anchorLTM, pin.restLocal);
        }
        else
            p.anchorWorld = p.posWorld;

        // 第一帧必须用"蒙皮位置"摆放粒子。
        // Init 里用的是 clumpLTM * restLocal，和蒙皮位置能差一米多，
        // 不校正的话整条发丝会从头骨一路拉到那个错误位置——就是那条"刀片"。
        if (!p.placed)
        {
            p.posWorld = V3Add(p.anchorWorld, p.seedOffset);
            p.prevWorld = p.posWorld;
            p.placed = true;
            ++placedCount;
        }
        else
        {
            // 传送 / 重生 / 模型重载保护。
            //
            // 这里以前写死 1.5m：任何布料粒子离静止位置超过 1.5m 就被瞬移回去。
            // 旗子/披风正常垂落本来就超过 1.5m，于是每帧被拽回、再落下——
            // 表现出来就是"垂到一半被无形的力提住 + 一直弹跳"。
            // 现在只在位移大到不可能是正常摆动时才瞬移（并且不超过粒子自己的自由半径）。
            const float snapDistance = std::max(8.0f, particleLimit(p) * 2.0f);

            if (V3LenSq(V3Sub(p.posWorld, p.anchorWorld)) > snapDistance * snapDistance)
            {
                p.posWorld = p.anchorWorld;
                p.prevWorld = p.anchorWorld;
            }
        }
    }

    if (placedCount > 0)
    {
        std::ostringstream oss;
        oss << "[VER_82][Place] particles=" << placedCount
            << " placed at skinned anchors; blendFallback=" << blendFallbacks
            << "/" << rig.particles.size();
        GeoPhysLog(oss.str());

        // 求逆自检：M -> M^-1 往返误差应该是 0（浮点误差级别）
        if (!rig.particles.empty())
        {
            RwMatrix m;
            if (BuildVertexMatrix(boneMatrices, inverseBinds, rig.particles.front(), maxBone, m, nullptr))
            {
                RwMatrix inv;
                if (InvertMatrix(m, inv))
                {
                    std::ostringstream chk;
                    chk << std::fixed << std::setprecision(6);
                    chk << "[VER_82][InverseCheck] residual="
                        << InverseResidual(m, inv, rig.particles.front().restLocal);
                    GeoPhysLog(chk.str());
                }
            }
        }

        if (blendFallbacks > 0)
        {
            for (int s = 0; s < 3; ++s)
            {
                if (fallbackSamples[s] < 0) continue;

                for (const auto& p : rig.particles)
                {
                    if (p.geometryVertexIndex != fallbackSamples[s]) continue;

                    std::ostringstream dbg;
                    dbg << "[VER_82][Blend] vertex=" << p.geometryVertexIndex
                        << " bones=" << p.bones[0] << "," << p.bones[1] << ","
                        << p.bones[2] << "," << p.bones[3]
                        << " weights=" << p.weights[0] << "," << p.weights[1] << ","
                        << p.weights[2] << "," << p.weights[3]
                        << " dominant=" << p.dominantBone;
                    GeoPhysLog(dbg.str());
                    break;
                }
            }
        }
    }

    // 碰撞体：优先按 GTA 人物骨骼 ID 生成"人体胶囊"。
    // 以前按骨骼数组的相邻索引配对，但那个顺序不对应真实关节，
    // 腿部经常没有胶囊——裙子/披风就从大腿中间穿过去了。
    if (rig.colliders.empty() && haveSkinAnchor && maxBone >= 2)
    {
        std::map<int, int> boneByNodeId;

        if (hierarchy && hierarchy->pNodeInfo)
        {
            for (RwInt32 i = 0; i < hierarchy->numNodes; ++i)
            {
                const RpHAnimNodeInfo& info = hierarchy->pNodeInfo[i];

                int idx = i;

                if (info.nodeIndex >= 0 &&
                    static_cast<RwUInt32>(info.nodeIndex) < maxBone)
                {
                    idx = info.nodeIndex;
                }

                if (idx >= 0 && static_cast<RwUInt32>(idx) < maxBone)
                    boneByNodeId[info.nodeID] = idx;
            }
        }

        auto addBoneCapsule = [&](int idA, int idB, float radius, int group)
        {
            auto ia = boneByNodeId.find(idA);
            auto ib = boneByNodeId.find(idB);

            if (ia == boneByNodeId.end() || ib == boneByNodeId.end()) return;
            // ia == ib 表示球体（用来补臀部/胸部的体积）

            // ini caps kip：按部位跳过（A/B 用）
            if (group != 0 && (g_capsuleSkipMask & group) != 0) return;

            PhysicsCapsule cap;
            cap.boneA = ia->second;
            cap.boneB = ib->second;
            cap.group = group;

            // 半径为 0 的"占位胶囊"直接跳过（骨骼存在但这一段不需要挡）
            if (radius <= 0.0f) return;

            // 倍率：整体(bodycolliderscale) × 部位(capsgroupscale*) × 索引(capsulescale)
            float groupScale = 1.0f;

            const auto gsIt = g_capsuleGroupScale.find(group);
            if (gsIt != g_capsuleGroupScale.end())
                groupScale = gsIt->second;

            cap.radius = radius * g_bodyColliderScale * groupScale + g_bodyColliderMargin;

            rig.colliders.push_back(cap);
        };

        // 髋 / 大腿 / 小腿：裙子最容易穿模的地方
        addBoneCapsule(BONE_PELVIS1,     BONE_PELVIS1,    0.13f, 1);   // 真正的骨盆（有些模型只有这根）
        addBoneCapsule(BONE_PELVIS,      BONE_PELVIS,     0.13f, 1);
        addBoneCapsule(BONE_PELVIS,      BONE_LEFTHIP,    0.12f, 2);
        addBoneCapsule(BONE_PELVIS,      BONE_RIGHTHIP,   0.12f, 2);
        addBoneCapsule(BONE_LEFTHIP,     BONE_LEFTKNEE,   0.11f, 4);
        addBoneCapsule(BONE_RIGHTHIP,    BONE_RIGHTKNEE,  0.11f, 4);
        addBoneCapsule(BONE_LEFTKNEE,    BONE_LEFTANKLE,  0.09f, 16);
        addBoneCapsule(BONE_RIGHTKNEE,   BONE_RIGHTANKLE, 0.09f, 16);

        // 关节球：髋/膝只有两根胶囊相交，关节外侧那一圈是"空洞"，
        // 快速摆腿时布料正好从那里漏进去
        addBoneCapsule(BONE_LEFTHIP,     BONE_LEFTHIP,    0.115f, 2);
        addBoneCapsule(BONE_RIGHTHIP,    BONE_RIGHTHIP,   0.115f, 2);
        addBoneCapsule(BONE_LEFTKNEE,    BONE_LEFTKNEE,   0.09f, 8);
        addBoneCapsule(BONE_RIGHTKNEE,   BONE_RIGHTKNEE,  0.09f, 8);

        // 臀部 / 腰 / 胸：用球体补体积，不靠盲目放大腿部胶囊
        addBoneCapsule(BONE_UPPERTORSO,  BONE_UPPERTORSO,  0.14f, 32);

        // 躯干：披风 / 上衣
        addBoneCapsule(BONE_PELVIS,      BONE_SPINE1,      0.13f, 32);
        addBoneCapsule(BONE_SPINE1,      BONE_UPPERTORSO,  0.13f, 32);

        // 脖子 / 头：头发
        addBoneCapsule(BONE_NECK,        BONE_HEAD,        0.10f, 64);

        // 手臂 / 前臂：袖子、长袖外套
        addBoneCapsule(BONE_LEFTSHOULDER,  BONE_LEFTELBOW,  0.07f, 128);
        addBoneCapsule(BONE_RIGHTSHOULDER, BONE_RIGHTELBOW, 0.07f, 128);
        addBoneCapsule(BONE_LEFTELBOW,     BONE_LEFTWRIST,  0.065f, 128);
        addBoneCapsule(BONE_RIGHTELBOW,    BONE_RIGHTWRIST, 0.065f, 128);

        // 脚 / 鞋：长裙下摆、奔跑抬脚时会碰到
        addBoneCapsule(BONE_LEFTANKLE,     BONE_LEFTFOOT,   0.07f, 256);
        addBoneCapsule(BONE_RIGHTANKLE,    BONE_RIGHTFOOT,  0.07f, 256);

        const int pedCapsules = static_cast<int>(rig.colliders.size());

        // 没有人物骨骼 ID（车辆 / 其它 skinned 模型）：退回相邻骨骼胶囊
        if (rig.colliders.empty())
        {
            for (RwUInt32 i = 0; i + 1 < maxBone && rig.colliders.size() < 20; ++i)
            {
                const float seg = V3Len(V3Sub(boneMatrices[i + 1].pos, boneMatrices[i].pos));

                if (seg < 0.01f || seg > 0.5f) continue;

                PhysicsCapsule cap;
                cap.boneA = static_cast<int>(i);
                cap.boneB = static_cast<int>(i + 1);
                cap.radius = std::clamp(0.35f * seg, 0.03f, 0.10f)
                    * g_bodyColliderScale + g_bodyColliderMargin;

                rig.colliders.push_back(cap);
            }
        }

        // ------------------------------------------------------------
        // 按索引倍率（ini: capsulescale=1.0,1.2,...）
        // 在整体/部位倍率之后，对最终半径再乘一次。
        // 索引对应 [Colliders] radii= 的打印顺序（0-based）。
        // ------------------------------------------------------------
        if (!g_capsuleIndexScale.empty())
        {
            const size_t n = std::min(g_capsuleIndexScale.size(), rig.colliders.size());

            for (size_t i = 0; i < n; ++i)
            {
                const float s = std::clamp(g_capsuleIndexScale[i], 0.0f, 3.0f);

                rig.colliders[i].radius *= s;
            }
        }

        std::ostringstream oss;
        oss << "[VER_82][Colliders] capsules=" << rig.colliders.size()
            << " pedBoneCapsules=" << pedCapsules
            << " bones=" << maxBone
            << " scale=" << g_bodyColliderScale
            << " margin=" << g_bodyColliderMargin;

        // 逐根打印半径：穿模时先看是哪一段（按列表顺序：骨盆/髋/大腿/小腿/关节/躯干/头/臂/脚）
        oss << " radii=";

        for (size_t i = 0; i < rig.colliders.size(); ++i)
        {
            if (i) oss << ",";
            oss << std::fixed << std::setprecision(3) << rig.colliders[i].radius;
        }

        GeoPhysLog(oss.str());
    }

    const bool hadColliderPrev = rig.colliderPrevValid;

    for (auto& cap : rig.colliders)
    {
        // 扫掠碰撞需要上一帧端点；第一帧直接对齐，避免把"从原点飞过来"算成运动
        if (hadColliderPrev)
        {
            cap.aPrevWorld = cap.aWorld;
            cap.bPrevWorld = cap.bWorld;
        }

        const int ba = cap.boneA;
        const int bb = cap.boneB;

        if (haveSkinAnchor &&
            ba >= 0 && bb >= 0 &&
            ba < static_cast<int>(maxBone) && bb < static_cast<int>(maxBone))
        {
            cap.aWorld = boneMatrices[ba].pos;
            cap.bWorld = boneMatrices[bb].pos;
        }
        else if (anchorLTM)
        {
            cap.aWorld = TransformPoint(*anchorLTM, cap.aLocal);
            cap.bWorld = TransformPoint(*anchorLTM, cap.bLocal);
        }

        if (!hadColliderPrev)
        {
            cap.aPrevWorld = cap.aWorld;
            cap.bPrevWorld = cap.bWorld;
        }
    }

    rig.colliderPrevValid = true;

    // ============================================================
    // 头发 × 自身骨胶囊：**静息姿态就已经重叠**的胶囊不能推
    //
    // 骨胶囊只是身体的中轴近似：头/颈胶囊是"半径 12cm 的球"，上胸是
    // "半径 16cm 的球"，都远粗于真实身体表面。头发是从头皮里长出来的，
    // 天生就在这些胶囊内部 —— 每帧照样往外推，等于给整片头发加了一个
    // 恒定的向外膨胀力：离线用 tk7Chi2.dff 复现，静止时头发被推离原始
    // 发型轮廓 平均 29mm / 最大 95mm（后脑+肩部），正是"后脑勺局部凸起"。
    //
    // 处理：初始化时用静止姿态做一次配对检测，凡是"建模时就埋在胶囊里"
    // 的 (粒子, 胶囊) 对直接屏蔽。头发本来就贴着头/胸，不需要被推开；
    // 真正在胶囊外面的头发（垂到肩背、跑动时的发梢）仍然照常碰撞。
    // 只对 Hair 生效，Cloth / Body / SoftBody 完全不受影响。
    // ============================================================
    if (!rig.capsuleMaskBuilt && rig.hairCapsuleMaskEnabled &&
        !rig.colliders.empty() && rig.colliders.size() <= 32)
    {
        rig.capsuleMaskBuilt = true;

        int maskedPairs = 0;
        int maskedHair = 0;
        int maskedCloth = 0;
        int clothPairs = 0;
        float deepest = 0.0f;
        float deepestCloth = 0.0f;

        for (auto& p : rig.particles)
        {
            const PhysType pt = static_cast<PhysType>(p.typeU32);

            const bool isHair  = (pt == PhysType::Hair);
            const bool isCloth = (pt == PhysType::Cloth);

            if (!isHair && !isCloth) continue;
            if (isCloth && !g_clothCapsuleMaskEnabled) continue;

            const float threshold = isCloth ? g_clothCapsuleMaskDepth : 0.005f;

            uint32_t mask = 0;
            bool any = false;
            float deepestHere = 0.0f;

            for (size_t ci = 0; ci < rig.colliders.size(); ++ci)
            {
                const auto& cap = rig.colliders[ci];
                if (cap.radius <= 0.0f) continue;

                // 关键：布料实际碰撞用的是 (radius + clothCollisionMargin)，
                // 屏蔽判定必须用同一个有效半径，否则"在有效半径内、原始半径外"
                // 的那圈粒子不会被屏蔽，仍然每帧被推 → 残留抖动。
                const float effRadius = cap.radius + (isCloth ? rig.clothCollisionMargin : 0.0f);

                RwV3d closest;
                const float d2 = ClosestSegSq(p.anchorWorld, cap.aWorld,
                                              cap.bWorld, closest);

                const float depth = effRadius - std::sqrt(std::max(d2, 0.0f));

                if (depth > threshold)
                {
                    mask |= (1u << ci);
                    any = true;
                    if (depth > deepestHere) deepestHere = depth;
                }
            }

            if (isHair)
            {
                p.capsuleIgnore = mask;
                maskedPairs += static_cast<int>(p.pins.size()) * 0;
                for (int k = 0; k < 32; ++k) if (mask & (1u << k)) ++maskedPairs;
                if (any) ++maskedHair;
                if (deepestHere > deepest) deepest = deepestHere;
            }
            else
            {
                p.capsuleIgnore |= mask;
                for (int k = 0; k < 32; ++k) if (mask & (1u << k)) ++clothPairs;
                if (any) ++maskedCloth;
                if (deepestHere > deepestCloth) deepestCloth = deepestHere;
            }
        }

        std::ostringstream oss;
        oss << "[VER_82][HairCapsuleMask] capsules=" << rig.colliders.size()
            << " hairParticles=" << maskedHair
            << " maskedPairs=" << maskedPairs
            << " deepest=" << (deepest * 1000.0f) << "mm"
            << " (静息姿态已重叠的胶囊不再推头发)";
        GeoPhysLog(oss.str());

        std::ostringstream co;
        co << "[VER_82][ClothCapsuleMask] capsules=" << rig.colliders.size()
           << " clothParticles=" << maskedCloth
           << " maskedPairs=" << clothPairs
           << " deepest=" << (deepestCloth * 1000.0f) << "mm"
           << " threshold=" << (g_clothCapsuleMaskDepth * 1000.0f) << "mm"
           << " (静息姿态已重叠的胶囊不再推布料)";
        GeoPhysLog(co.str());
    }

    // ------------------------------------------------------------
    // 布料"静息深度软碰撞"表（每个 rig 只建一次）
    //
    // 与上面的 mask 不同：这里不做 0/1 屏蔽，而是记录**具体深度**，
    // 运行时用 (有效半径 - 静息深度) 作为碰撞半径 ⇒ 静止时推力恰好为 0。
    // ------------------------------------------------------------
    if (!rig.clothCapBuilt && g_clothCapsuleMaskEnabled &&
        !rig.colliders.empty() && rig.colliders.size() <= 32)
    {
        rig.clothCapBuilt = true;

        const size_t nCaps = rig.colliders.size();

        rig.clothCapDepth.assign(rig.particles.size() * nCaps, 0.0f);

        int   clothCount   = 0;
        int   overlapped   = 0;
        float deepestSoft  = 0.0f;

        for (size_t pi = 0; pi < rig.particles.size(); ++pi)
        {
            const auto& p = rig.particles[pi];

            if (static_cast<PhysType>(p.typeU32) != PhysType::Cloth) continue;

            ++clothCount;

            bool any = false;

            for (size_t ci = 0; ci < nCaps; ++ci)
            {
                const auto& cap = rig.colliders[ci];
                if (cap.radius <= 0.0f) continue;

                const float effRadius = cap.radius + rig.clothCollisionMargin;

                RwV3d closest;
                const float d2 = ClosestSegSq(p.anchorWorld, cap.aWorld,
                                              cap.bWorld, closest);

                float depth = effRadius - std::sqrt(std::max(d2, 0.0f));

                // 只保留"静息时确实埋进去"的那部分，并限幅避免极端值
                depth = std::clamp(depth, 0.0f, 0.30f);

                rig.clothCapDepth[pi * nCaps + ci] = depth;

                if (depth > 0.0f)
                {
                    any = true;
                    if (depth > deepestSoft) deepestSoft = depth;
                }
            }

            if (any) ++overlapped;
        }

        std::ostringstream so;
        so << std::fixed << std::setprecision(1);
        so << "[VER_82][ClothCapSoft] capsules=" << nCaps
           << " clothParticles=" << clothCount
           << " overlapped=" << overlapped
           << " deepest=" << (deepestSoft * 1000.0f) << "mm"
           << " margin=" << (rig.clothCollisionMargin * 1000.0f) << "mm"
           << " (布料只用超过静息深度的部分做碰撞)";
        GeoPhysLog(so.str());

        std::ostringstream io;
        io << "[VER_82][HairInertia] frameTransport=ON frameAccel=ON"
              " pinSolve=consensus maxFrameAccel=40m/s2"
              " (角色平移/旋转不再当成头发速度)";
        GeoPhysLog(io.str());
    }

    const float h  = deltaTime;
    const float h2 = h * h;
    const RwV3d gravityBase = { 0.0f, 0.0f, -rig.gravity };

    // 阻尼按 60fps 标定，再按实际帧时间换算：否则 30fps 与 144fps 手感完全不同。
    // 布料单独用更轻的阻力（clothDamping），否则会像泡在水里，落不下来。
    const float dampingHair  = std::pow(std::clamp(rig.hairDamping, 0.0f, 1.0f), h * 60.0f);
    const float dampingCloth = std::pow(std::clamp(rig.clothDamping, 0.0f, 1.0f), h * 60.0f);

    const float maxStep   = rig.maxVelocity * h;
    const float maxStepSq = maxStep * maxStep;

    for (auto& p : rig.particles)
    {
        if (p.invMass > 0.0f) continue;   // 自由粒子
        // Body 由下面的抖动弹簧驱动
        if (static_cast<PhysType>(p.typeU32) == PhysType::Body) continue;

        p.posWorld  = p.anchorWorld;
        p.prevWorld = p.posWorld;
    }

    // ============================================================
    // Body（身体软组织）：按"主导骨骼"分组的抖动
    //
    // 关键：同组顶点共用同一个位移向量。
    // 如果每个顶点各自积分，相邻顶点锚点不同、相位不同，就会被拉开成尖刺。
    // 分组后每组整体平移，形状保持不变；组与组之间靠 jiggleWeight 渐隐过渡。
    // ============================================================
    {
        std::map<int, int>   groupCount;
        std::map<int, RwV3d> groupSum;
        std::map<int, float> groupSoft;
        std::map<int, float> groupAmp;

        // ---- 根运动加速度：落脚/起跳/急停带来的整机冲击 ----
        RwV3d rootAcc = { 0.0f, 0.0f, 0.0f };

        if (anchorLTM)
        {
            const RwV3d rootPos = anchorLTM->pos;

            if (!rig.rootInit)
            {
                rig.rootPrevPos = rootPos;
                rig.rootVel = { 0.0f, 0.0f, 0.0f };
                rig.rootInit = true;
            }

            const float invDtRoot = (deltaTime > 1e-6f) ? (1.0f / deltaTime) : 60.0f;

            const RwV3d rv = V3Scale(V3Sub(rootPos, rig.rootPrevPos), invDtRoot);
            rootAcc = V3Scale(V3Sub(rv, rig.rootVel), invDtRoot);

            rig.rootPrevPos = rootPos;
            rig.rootVel = rv;

            // 传送/重载会产生巨大的瞬时加速度，钳一下
            const float raLen = V3Len(rootAcc);
            if (raLen > 40.0f)
                rootAcc = V3Scale(rootAcc, 40.0f / raLen);
        }

        for (auto& p : rig.particles)
        {
            if (static_cast<PhysType>(p.typeU32) != PhysType::Body) continue;

            const int g = (p.bodyGroup >= 0)
                ? p.bodyGroup
                : ((p.dominantBone >= 0) ? p.dominantBone : -1);

            groupCount[g] += 1;
            groupSum[g] = V3Add(groupSum[g], p.anchorWorld);
            groupSoft[g] += p.stiffness;
            groupAmp[g] = std::max(groupAmp[g],
                std::clamp(1.0f - p.anchorFactor, 0.0f, 1.0f));
        }

        for (auto& entry : groupCount)
        {
            const int g = entry.first;
            const int count = entry.second;

            if (count <= 0) continue;

            const RwV3d anchorAvg = V3Scale(groupSum[g], 1.0f / static_cast<float>(count));

            GeometryPhysics::BodyGroupState& state = rig.bodyGroups[g];

            if (!state.init)
            {
                state.prev      = anchorAvg;
                state.anchorVel = { 0.0f, 0.0f, 0.0f };
                state.offset    = { 0.0f, 0.0f, 0.0f };
                state.vel       = { 0.0f, 0.0f, 0.0f };
                state.init      = true;
            }

            const float invDt = (deltaTime > 1e-6f) ? (1.0f / deltaTime) : 60.0f;

            // 传送/重生/换模型：整组锚点瞬移。把它当加速度会灌进一个巨大冲量，
            // 之后靠阻尼衰减 —— 表现为"角色走路时身上一直在抖"。
            const float jumpLen = V3Len(V3Sub(anchorAvg, state.prev));

            RwV3d anchorVel = V3Scale(V3Sub(anchorAvg, state.prev), invDt);
            RwV3d anchorAcc = V3Scale(V3Sub(anchorVel, state.anchorVel), invDt);

            state.prev      = anchorAvg;
            state.anchorVel = anchorVel;

            if (jumpLen > 0.5f)
            {
                anchorAcc       = { 0.0f, 0.0f, 0.0f };
                state.offset    = { 0.0f, 0.0f, 0.0f };
                state.vel       = { 0.0f, 0.0f, 0.0f };
                state.accSmooth = { 0.0f, 0.0f, 0.0f };
                ++rig.bodyResets;
            }

            // ------------------------------------------------------------------
            // 驱动加速度：**一阶低通 + 限幅**
            //
            // 锚点加速度是骨骼矩阵的二阶差分。25~30fps、骨骼动画每帧都在变，
            // 这个差分里混着大量高频噪声（头发那边实测能到数百 m/s²）。
            // 直接拿它驱动弹簧 = 把动画噪声放大成"身上乱抖"。
            // 低通 0.35（≈3~4 帧记忆）保留真实的起步/急停/落脚冲击，
            // 限幅 25 m/s² 防止尖峰把弹簧打飞。
            //
            // 同时**去掉原来的 +0.8*rootAcc**：anchorAcc 本身已包含根节点运动
            // （骨骼矩阵是世界空间的），再加一次就是重复计 —— 冲击被放大 1.8 倍，
            // 这也是抖动的来源之一。
            // ------------------------------------------------------------------
            state.accSmooth = V3Add(state.accSmooth,
                V3Scale(V3Sub(anchorAcc, state.accSmooth), 0.35f));

            {
                constexpr float kMaxBodyDrive = 25.0f;   // m/s²
                const float dl = V3Len(state.accSmooth);

                if (dl > kMaxBodyDrive)
                    state.accSmooth = V3Scale(state.accSmooth, kMaxBodyDrive / dl);

                if (dl > rig.bodyMaxDrive) rig.bodyMaxDrive = dl;
            }

            const float soft = std::clamp(groupSoft[g] / static_cast<float>(count), 0.05f, 1.0f);

            // 手感参数（Q 弹的关键是低阻尼 + 中等刚度）
            //   k：越大越"紧"，越小越"慢而软"
            //   c：阻尼，越小回弹次数越多（ζ = c / (2√k)）
            //
            // 2026-11-24 15:3x：新增 ini 旋钮 bodyk / bodyc，>0 时直接覆盖公式，
            // 这样"又快又弹"（k 大 + c 小）才可能实现 —— 只靠 G 的话 k 和 c
            // 是同向变化的，做不到。
            const float k = (g_bodyK > 0.0f) ? g_bodyK : (30.0f + 120.0f * soft);
            const float c = (g_bodyC > 0.0f) ? g_bodyC : (1.5f + 5.0f * soft);

            // 驱动的加速度放大：角色移动/蹲起时的带动更明显（ini: bodydrive）
            const float driveScale = g_bodyDrive;

            // 驱动 = 本组锚点加速度（已滤波、已限幅，且不重复叠加根加速度）
            //
            // 2026-11-24 15:4x：每组再绕一个**确定性随机轴**旋转一点角度。
            // 原因：左胸/右胸/腹的锚点都挂同一根脊椎骨时，加速度完全相同，
            // 分组再细也会同步；旋转后各自方向不同，看起来才自然。
            // 强度由 ini 的 bodyrandom 控制（0 = 完全同步）。
            RwV3d drive = state.accSmooth;

            if (g_bodyRandom > 0.0f)
            {
                unsigned int h = static_cast<unsigned int>(g) * 2654435761u + 0x9E3779B9u;

                auto nextRand = [&h]() -> float
                {
                    h ^= h << 13;
                    h ^= h >> 17;
                    h ^= h << 5;
                    return static_cast<float>(h & 0xFFFFFFu) / static_cast<float>(0xFFFFFFu);
                };

                RwV3d axis = { nextRand() * 2.0f - 1.0f,
                               nextRand() * 2.0f - 1.0f,
                               nextRand() * 2.0f - 1.0f };

                const float axisLen = V3Len(axis);

                if (axisLen > 1e-4f)
                {
                    axis = V3Scale(axis, 1.0f / axisLen);

                    const float ang = g_bodyRandom * 0.7f;    // 最多≈40°
                    const float ca = std::cos(ang);
                    const float sa = std::sin(ang);

                    const RwV3d cross = { axis.y * drive.z - axis.z * drive.y,
                                          axis.z * drive.x - axis.x * drive.z,
                                          axis.x * drive.y - axis.y * drive.x };

                    const float dot = V3Dot(axis, drive);

                    drive = V3Add(V3Add(V3Scale(drive, ca), V3Scale(cross, sa)),
                                  V3Scale(axis, dot * (1.0f - ca)));
                }
            }

            // ------------------------------------------------------------------
            // 固定子步积分
            //
            // 显式欧拉 + 每帧可变 dt 的问题：k 最大 150（ω≈12.2 rad/s），
            // dt=45ms（22fps）时 dt·ω≈0.55，已经贴近稳定边界；再叠加噪声驱动
            // 就会"嗡嗡"抖。这里把弹簧按 1/120s 子步推进（每帧 1~4 步，
            // 内部 dt 恒定），刚度/阻尼的响应就不再随帧率变化。
            // ------------------------------------------------------------------
            constexpr float kSpringStep = 1.0f / 120.0f;

            int springSteps = static_cast<int>(
                std::lround(deltaTime / kSpringStep));

            springSteps = std::clamp(springSteps, 1, 4);

            const float hs = deltaTime / static_cast<float>(springSteps);

            for (int si = 0; si < springSteps; ++si)
            {
                RwV3d acc = V3Sub(V3Scale(state.offset, -k),
                                  V3Scale(state.vel, c));

                acc = V3Sub(acc, V3Scale(drive, driveScale));

                state.vel    = V3Add(state.vel, V3Scale(acc, hs));
                state.offset = V3Add(state.offset, V3Scale(state.vel, hs));
            }

            // 幅度 = 锚定强度(B) × 软度(G)：G 越小（越硬）抖动越小
            // bodyamp 是整体幅度倍率（ini: bodyamp，默认 1.0）
            const float softnessN = std::clamp(1.0f - soft, 0.0f, 1.0f);
            const float amp = 0.070f * std::clamp(groupAmp[g], 0.0f, 1.0f)
                                     * (0.15f + 0.85f * softnessN)
                                     * g_bodyAmp;

            const float offSq = V3LenSq(state.offset);

            if (offSq > amp * amp)
            {
                if (amp <= 1e-6f)
                {
                    state.offset = { 0.0f, 0.0f, 0.0f };
                    state.vel    = { 0.0f, 0.0f, 0.0f };
                }
                else
                {
                    // 只做"几何钳制 + 去掉向外的速度分量"：
                    // 旧实现每帧把位移硬钳回球面、再把速度砍半 —— 钳制量与速度
                    // 不同步，会形成"推到边界→砍掉→再推"的锯齿（也是抖动的来源）。
                    const float len = std::sqrt(offSq);
                    const RwV3d dir = V3Scale(state.offset, 1.0f / len);

                    state.offset = V3Scale(dir, amp);

                    const float outward = dir.x * state.vel.x +
                                          dir.y * state.vel.y +
                                          dir.z * state.vel.z;

                    if (outward > 0.0f)
                        state.vel = V3Sub(state.vel, V3Scale(dir, outward));
                }
            }

            if (V3Len(state.offset) > rig.bodyMaxOffset)
                rig.bodyMaxOffset = V3Len(state.offset);
            if (V3Len(state.vel) > rig.bodyMaxVel)
                rig.bodyMaxVel = V3Len(state.vel);
        }

        // 诊断：Body 弹簧的驱动/位移/速度峰值（每 120 帧）
    //   drive  = 限幅后的锚点加速度（m/s²）：正常走动 < 10，急停/落脚 15~25
    //   offset = 当前最大位移（m）：正常 < 0.07（amp 上限）
    //   vel    = 最大速度（m/s）：正常 < 1.0；抖动时会出现 2~5
    //   resets = 本帧被判定为"传送/瞬移"而清零的组数
    ++rig.bodyDiagFrame;

    if (rig.hairDiagnostics && (rig.bodyDiagFrame % 120) == 0)
    {
        std::ostringstream bo;
        bo << std::fixed << std::setprecision(3);
        bo << "[VER_82][BodySpring] dt=" << (deltaTime * 1000.0f) << "ms"
           << " n=" << rig.bodyCount
           << " groups=" << rig.bodyGroups.size()
           << " drive=" << rig.bodyMaxDrive << "m/s2"
           << " offset=" << rig.bodyMaxOffset << "m"
           << " vel=" << rig.bodyMaxVel << "m/s"
           << " resets=" << rig.bodyResets;
        GeoPhysLog(bo.str());

        rig.bodyMaxDrive  = 0.0f;
        rig.bodyMaxOffset = 0.0f;
        rig.bodyMaxVel    = 0.0f;
        rig.bodyResets    = 0;
    }

    // 诊断：每个骨骼组的顶点数与当前位移（用于定位"半边不抖"这类问题）
        if ((g_stepFrame % 900) == 0)
        {
            std::ostringstream oss;
            oss << "[VER_82][BodyGroups] groups=" << groupCount.size();

            int shown = 0;

            for (const auto& entry : groupCount)
            {
                if (shown++ >= 6) break;

                const int g = entry.first;
                auto git = rig.bodyGroups.find(g);

                float avgAmp = 0.0f;
                auto ait = groupAmp.find(g);
                if (ait != groupAmp.end())
                    avgAmp = 0.070f * std::clamp(ait->second, 0.0f, 1.0f);

                oss << " g" << g << "(n=" << entry.second
                    << ",off=" << (git != rig.bodyGroups.end()
                        ? V3Len(git->second.offset) : 0.0f)
                    << ",amp=" << avgAmp << ")";
            }

            GeoPhysLog(oss.str());
        }

        for (auto& p : rig.particles)
        {
            if (static_cast<PhysType>(p.typeU32) != PhysType::Body) continue;

            const int g = (p.bodyGroup >= 0)
                ? p.bodyGroup
                : ((p.dominantBone >= 0) ? p.dominantBone : -1);
            auto it = rig.bodyGroups.find(g);

            if (it == rig.bodyGroups.end())
                p.jiggleOffset = { 0.0f, 0.0f, 0.0f };
            else
                p.jiggleOffset = V3Scale(it->second.offset, p.jiggleWeight);
        }

        // 邻域平滑：个别顶点的主导骨骼和邻居不同时，会拿到别的组的位移，
        // 表现为"单点尖刺"。沿 Body-Body 约束边平均即可消除。
        //
        // 2026-11-24 15:4x：只在**同组内**平滑，次数由 ini bodysmooth 控制
        // （原来 12 次跨组平均会把左胸/右胸/腹的差异直接抹平）。
        const size_t particleCount = rig.particles.size();

        if (rig.jiggleScratch.size() != particleCount)
        {
            rig.jiggleScratch.assign(particleCount, RwV3d{ 0.0f, 0.0f, 0.0f });
            rig.jiggleCount.assign(particleCount, 0);
        }

        for (int pass = 0; pass < std::clamp(g_bodySmooth, 0, 12); ++pass)
        {
            std::fill(rig.jiggleScratch.begin(), rig.jiggleScratch.end(),
                RwV3d{ 0.0f, 0.0f, 0.0f });
            std::fill(rig.jiggleCount.begin(), rig.jiggleCount.end(), 0);

            for (size_t i = 0; i < particleCount; ++i)
            {
                auto& p = rig.particles[i];
                if (static_cast<PhysType>(p.typeU32) != PhysType::Body) continue;

                rig.jiggleScratch[i] = p.jiggleOffset;
                rig.jiggleCount[i] = 1;
            }

            for (const auto& c : rig.constraints)
            {
                auto& A = rig.particles[c.a];
                auto& Bp = rig.particles[c.b];

                if (static_cast<PhysType>(A.typeU32) != PhysType::Body) continue;
                if (static_cast<PhysType>(Bp.typeU32) != PhysType::Body) continue;

                // 跨组不平滑：否则左胸/右胸/腹的位移会被平均成同一个方向
                if (g_bodyGroupMode != 1 && A.bodyGroup != Bp.bodyGroup) continue;

                rig.jiggleScratch[c.a] = V3Add(rig.jiggleScratch[c.a], Bp.jiggleOffset);
                rig.jiggleCount[c.a] += 1;

                rig.jiggleScratch[c.b] = V3Add(rig.jiggleScratch[c.b], A.jiggleOffset);
                rig.jiggleCount[c.b] += 1;
            }

            for (size_t i = 0; i < particleCount; ++i)
            {
                if (rig.jiggleCount[i] <= 0) continue;

                auto& p = rig.particles[i];
                if (static_cast<PhysType>(p.typeU32) != PhysType::Body) continue;

                p.jiggleOffset = V3Scale(rig.jiggleScratch[i],
                    1.0f / static_cast<float>(rig.jiggleCount[i]));
            }
        }

        // 相邻顶点的位移差硬限制：无论前面怎么分组/平滑，相邻两点的位移
        // 最多差 kMaxDelta（默认 4mm）。这是"网格不会被撕开"的硬保证。
        constexpr float kMaxDelta = 0.004f;

        for (int pass = 0; pass < 3; ++pass)
        {
            for (const auto& c : rig.constraints)
            {
                auto& A = rig.particles[c.a];
                auto& Bp = rig.particles[c.b];

                if (static_cast<PhysType>(A.typeU32) != PhysType::Body) continue;
                if (static_cast<PhysType>(Bp.typeU32) != PhysType::Body) continue;
                if (A.hardAnchor || Bp.hardAnchor) continue;

                RwV3d diff = V3Sub(Bp.jiggleOffset, A.jiggleOffset);
                const float len = V3Len(diff);

                if (len <= kMaxDelta) continue;

                const RwV3d excess = V3Scale(diff, (len - kMaxDelta) / len * 0.5f);

                A.jiggleOffset  = V3Add(A.jiggleOffset, excess);
                Bp.jiggleOffset = V3Sub(Bp.jiggleOffset, excess);
            }
        }

        for (auto& p : rig.particles)
        {
            if (static_cast<PhysType>(p.typeU32) != PhysType::Body) continue;

            // 边界渐隐：靠近未刷色区域的顶点位移逐渐减到 0，避免硬边
            p.posWorld  = V3Add(p.anchorWorld, V3Scale(p.jiggleOffset, p.boundaryFade));
        }
    }

    // 允许的最大位移在函数开头统一计算（particleLimit）

    for (auto& p : rig.particles)
    {
        if (p.invMass <= 0.0f) continue;

        const PhysType physType = static_cast<PhysType>(p.typeU32);

        // 布料用独立的重力/阻尼，其余类型沿用 GravityScale
        const float gravMul = (physType == PhysType::Cloth)
            ? rig.clothGravity : GravityScale(physType);
        const float maxDist = particleLimit(p);
        const float damping = (physType == PhysType::Cloth) ? dampingCloth : dampingHair;

        // ============================================================
        // 布料惯性继承（跑动不穿模的关键之一）
        //
        // 布料原来只靠"锚点回拉"去追身体：每帧 12 次迭代各修正
        // anchorFactor*returnStrength 的比例，稳态下来布料会**永久滞后**
        // 十几厘米（冲刺时裙子被留在身后，腿自然就从裙子里穿过去了）。
        // 这里把锚点这一帧的位移按 clothInertia 的比例同时加到 pos/prev 上：
        // 相当于布料在一个跟着身体走的坐标系里做自己的摆动，
        // 剩下的 (1-clothInertia) 仍然交给 Verlet 自然滞后 + 回拉，
        // 所以跑动时不会"粘"在身上，站定时也不会漂。
        // ============================================================
        if (physType == PhysType::Cloth && rig.clothInertia > 0.0f)
        {
            if (p.anchorPrevValid)
            {
                const RwV3d anchorMove = V3Sub(p.anchorWorld, p.anchorPrevWorld);
                const float moveLen = V3Len(anchorMove);

                // 超过 0.8m/帧当成传送/重生，不要跟着飞
                if (moveLen > 1e-6f && moveLen < 0.8f)
                {
                    const RwV3d ride = V3Scale(anchorMove,
                        std::clamp(rig.clothInertia, 0.0f, 0.95f));

                    p.posWorld  = V3Add(p.posWorld, ride);
                    p.prevWorld = V3Add(p.prevWorld, ride);
                }
            }
        }

        // ============================================================
        // 头发：角色整体运动的正确补偿（frame transport）
        //
        // 世界空间 Verlet 里 pos/prev 都是世界坐标，角色自己的位移原本
        // **完全没有进入粒子状态**：每帧头发被动地"落后 v·h"，只能靠
        // 位置式锚点回拉（anchorFactor²·returnStrength/迭代）去追。
        // 单粒子的稳态解是 d = v·h / pull每帧，一个"永久、正比于速度、
        // 反比于帧率"的滞后。整片头发由边长约束把这个滞后摊开，实测
        // （离线复刻 tk7Chi2）：6m/s 匀速时相对滞后 34mm 且**永不回落**，
        // 而静止时只有 8mm；游戏里还要再乘上
        //   * 锚点是"人物空间的刚体变换"（本模型权重读不到，见
        //     [NoSkinWeight]），转身时锚点速度是 ω×r，r 到头顶有 1.7m；
        //   * 帧时间最大 1/20s（delta 上限）；
        //   * 每粒子自由上限 2·restSpan+0.1 把整片推到同一半径。
        // 这几点合起来就是日志里 [Step] hair max 长期 0.55~0.59m、
        // 视频里"一张被向后拉平的板"。
        //
        // 修复：把锚点自己的位移同时加到 pos 和 prev（整体平移+旋转不再被
        // 当成"头发速度"），再把锚点加速度反号当惯性力。这样
        //   匀速  -> 不滞后（不会再无限向后拉伸）
        //   加速  -> 向后滞后（有真实惯性，离线测峰值 45mm @0.15s）
        //   急停  -> 向前甩，然后被约束/回拉带回来
        //   转身  -> 锚点加速度里含向心/切向项，产生侧向惯性
        // 回拉强度保持原样，没有用参数掩盖问题。
        // ============================================================
        RwV3d hairFrameAcc = { 0.0f, 0.0f, 0.0f };

        if (physType == PhysType::Hair && p.anchorPrevValid)
        {
            const RwV3d anchorMove = V3Sub(p.anchorWorld, p.anchorPrevWorld);
            const float moveLen = V3Len(anchorMove);

            // 传送/重生/换模型：完全不跟随，惯性状态重同步。
            // 阈值按"12 m/s 当量"给，低帧率（h 大）时自动放宽。
            const float teleportLen = std::max(0.8f, 12.0f * h);

            if (moveLen > 1e-6f && moveLen < teleportLen)
            {
                // 2026-11-24 18:5x：不再 100% 全额跟随，改成和布料一样的比例：
                //   ride = anchorMove × hairRide（默认 0.6）
                // 剩下的 (1-hairRide) 交给 Verlet 自然滞后 ⇒ 冲刺时头发向后拖，
                // 而不是靠加速度项来回推（旧行为看起来是"向前拉伸"）。
                const RwV3d ride = V3Scale(anchorMove,
                    std::clamp(rig.hairRide, 0.0f, 1.0f));

                p.posWorld  = V3Add(p.posWorld, ride);
                p.prevWorld = V3Add(p.prevWorld, ride);

                // 可选（hairrotrans=1）：把 offset 也跟着头部旋转一起搬。
                // 只补偿平移时，大幅俯仰/转身会让发丝"保持世界朝向"，
                // 相对头部看起来就是被甩出去了；跟着骨骼旋转搬更接近刚体随动，
                // 真正的滞后交给下面的伪加速度。
                if (haveHairRotDelta)
                {
                    const RwV3d relCur = V3Sub(p.posWorld, p.anchorWorld);
                    const RwV3d relRot = TransformDir(hairRotDelta, relCur);
                    const RwV3d relNow = V3Add(relCur, V3Scale(V3Sub(relRot, relCur), g_hairRot));
                    p.posWorld = V3Add(p.anchorWorld, relNow);

                    const RwV3d relPrevCur = V3Sub(p.prevWorld, p.anchorPrevWorld);
                    const RwV3d relPrevRot = TransformDir(hairRotDelta, relPrevCur);
                    const RwV3d relPrev = V3Add(relPrevCur, V3Scale(V3Sub(relPrevRot, relPrevCur), g_hairRot));
                    p.prevWorld = V3Add(p.anchorPrevWorld, relPrev);
                }

                const float invH = 1.0f / std::max(h, 1e-5f);
                const RwV3d avel = V3Scale(anchorMove, invH);

                if (rig.hfMaxMove < moveLen) rig.hfMaxMove = moveLen;

                if (!p.anchorFrameValid)
                {
                    p.anchorFrameVel = avel;
                    p.anchorFrameValid = true;

                    p.frameAccSmooth = { 0.0f, 0.0f, 0.0f };
                    p.frameAccSmoothValid = true;
                }
                else
                {
                    // ------------------------------------------------------------------
                    // 伪加速度 = 锚点速度的差分。三个保护，缺一不可：
                    //
                    // 1) 用"原始上一帧速度"做差分（不做状态低通）：低通会把
                    //    姿势突变的冲量摊到后面好几帧，等于持续推着头发飞。
                    // 2) 单帧锚点位移超过"12 m/s 当量"就当姿势突变/传送：
                    //    只搬位置、不注入冲量。走路/跑步的锚点位移是 2~12 cm/帧，
                    //    上坡换动作/脚踩上斜坡时骨骼姿态可能一帧跳十几厘米。
                    // 3) 伪加速度自身再做一次轻低通(0.5)并限幅 20 m/s²：
                    //    把剩余的高频抖动压掉，同时保留起步/急停的真实冲量
                    //    （急停的真实加速度约 50~70 m/s²，被削到 20 仍然足够甩）。
                    // ------------------------------------------------------------------
                    const RwV3d aRaw = V3Scale(V3Sub(avel, p.anchorFrameVel), invH);
                    p.anchorFrameVel = avel;

                    // 刚性突变判别（按"加速度"而不是"位移"）：
                    //   真实起步/急停 ≤ ~50 m/s²；上坡换姿态 / 坡面转向让整片锚点
                    //   在一帧里转 10~20°，等价加速度 300~700 m/s²。
                    //   这种情况头发应该**整片跟着刚性搬走**（含旋转，见 hairrotrans），
                    //   绝不能把它当成惯性注入，否则旋转的切向分量会把两侧的发丝
                    //   往相反方向甩 —— 就是"上坡炸开"。
                    const bool posePop = (V3Len(aRaw) > 120.0f);

                    if (posePop)
                    {
                        ++rig.hfPops;
                        p.frameAccSmooth = { 0.0f, 0.0f, 0.0f };
                        p.frameAccSmoothValid = true;
                    }
                    else
                    {
                        if (!p.frameAccSmoothValid)
                        {
                            p.frameAccSmooth = aRaw;
                            p.frameAccSmoothValid = true;
                        }
                        else
                        {
                            p.frameAccSmooth = V3Add(p.frameAccSmooth,
                                V3Scale(V3Sub(aRaw, p.frameAccSmooth), 0.5f));
                        }

                        const float kMaxPseudoAccel = g_hairAcc;   // m/s²
                        const float kImpulseDecay = g_hairDecay;    // 用掉之后衰减

                        const float alen = V3Len(p.frameAccSmooth);

                        if (alen > kMaxPseudoAccel)
                        {
                            p.frameAccSmooth =
                                V3Scale(p.frameAccSmooth, kMaxPseudoAccel / alen);
                            ++rig.hfClamped;
                        }

                        hairFrameAcc = p.frameAccSmooth;

                        // 冲量衰减：一次"事件"只给一份有限冲量，不会在后面几十帧继续推
                        p.frameAccSmooth = V3Scale(p.frameAccSmooth, 0.6f);

                        if (alen > rig.hfMaxAcc) rig.hfMaxAcc = alen;
                    }
                }
            }
            else if (moveLen >= teleportLen)
            {
                // 传送：惯性状态直接重同步，避免下一帧甩出一个假冲量
                p.anchorFrameVel = { 0.0f, 0.0f, 0.0f };
                p.frameAccSmooth = { 0.0f, 0.0f, 0.0f };
                p.frameAccSmoothValid = true;
                ++rig.hfPops;
            }
        }
        else if (physType == PhysType::Hair)
        {
            p.anchorFrameVel = { 0.0f, 0.0f, 0.0f };
            p.frameAccSmooth = { 0.0f, 0.0f, 0.0f };
            p.frameAccSmoothValid = true;
        }

        // A/B：hairinertia=0 时只做 transport，不注入任何伪惯性冲量
        if (physType == PhysType::Hair && !rig.hairInertiaEnabled)
            hairFrameAcc = { 0.0f, 0.0f, 0.0f };

        p.anchorPrevWorld = p.anchorWorld;
        p.anchorPrevValid = true;

        RwV3d vel = V3Sub(p.posWorld, p.prevWorld);
        vel = V3Scale(vel, damping);
        vel = V3Add(vel, V3Scale(gravityBase, h2 * gravMul));

        // ============================================================
        // 伪惯性力（有界）：-a_frame 积分成"伪速度"，再注入 verlet。
        //
        // 为什么不直接 -a*h²：
        //   30fps 时 h=33ms，12 m/s² 每帧就有 0.013 m/帧 ≈ 0.4 m/s；
        //   快速转身/坡面姿态修正会让伪加速度**连续饱和**，几帧就顶到 4~8 m/s，
        //   头发被整体甩出去（日志里 speed 最高 8.0m/s，正好是 maxVelocity）。
        //   有了 frameImpulseVel 这个"伪速度"状态，并把它钳在 2 m/s、每帧衰减 0.9，
        //   任何姿势抖动最多只能给头发 2 m/s 的甩动 —— 物理上完全够（甩头发就 1~2 m/s），
        //   但不可能再累积成爆炸。
        // ============================================================
        // 伪惯性力：-a_frame * h²（重力是真实力，保留）
        // 02:44 那版把它改成"伪速度 + 2m/s 上限"，结果急停前冲的手感被削回去了，
        // 这里恢复成 02:38 你认可的形式（a*h² + 用后 0.6 衰减）。
        if (physType == PhysType::Hair && V3LenSq(hairFrameAcc) > 1e-8f)
            vel = V3Sub(vel, V3Scale(hairFrameAcc, h2));

        // 风：按当前天气取样（头发/布料受影响，身体组织不受风）
        if (physType == PhysType::Hair || physType == PhysType::Cloth)
        {
            RwV3d windAcc;
            SampleWindAcceleration(p.posWorld, windAcc);

            // 越靠外（freedom 越大）受风越明显；布料/头发各有受风倍率
            float windGain = 0.35f + 0.65f * p.freedom;

            if (physType == PhysType::Cloth)
                windGain *= rig.clothWind;

            // ini: hairwind —— 头发受风倍率（布料是 clothWind=1.8；
            // 细长的发片/刷成头发的布片需要调大才会飘）
            if (physType == PhysType::Hair)
                windGain *= rig.hairWind;

            // 人物（蒙皮）布料的风向单独取反（ini: pedclothwindflip）
            if (physType == PhysType::Cloth && rig.hasSkinAnchor && g_pedClothWindFlip)
                windGain = -windGain;

            vel = V3Add(vel, V3Scale(windAcc, h2 * windGain));
        }

        const float vSq = V3LenSq(vel);
        if (vSq > maxStepSq && vSq > 1e-12f)
            vel = V3Scale(vel, maxStep / std::sqrt(vSq));

        if (physType == PhysType::Hair)
        {
            const float speed = V3Len(vel) / std::max(h, 1e-5f);
            if (speed > rig.hfMaxSpeed) rig.hfMaxSpeed = speed;
        }

        p.prevWorld = p.posWorld;
        p.posWorld  = V3Add(p.posWorld, vel);

        // 安全网：只限制离锚点的最大距离，不再瞬移回锚点
        // （以前这里直接 reset 到 anchorWorld，跑动时会把头发反复拽到头上，
        //   看起来就是"被推着走 + 拉成一片"）
        const RwV3d toAnchor = V3Sub(p.anchorWorld, p.posWorld);
        const float d2 = V3LenSq(toAnchor);

        if (d2 > maxDist * maxDist)
        {
            // 几何钳制：pos 和 prev 一起移，避免"钳制量"变成速度（同世界碰撞那个 bug）
            const float d = std::sqrt(d2);
            const RwV3d clamped = V3Add(p.anchorWorld, V3Scale(toAnchor, -maxDist / d));
            const RwV3d corr = V3Sub(clamped, p.posWorld);

            p.posWorld  = clamped;
            p.prevWorld = V3Add(p.prevWorld, corr);
        }
    }

    for (int it = 0; it < rig.iterations; ++it)
    {
        for (auto& c : rig.constraints)
        {
            auto& A = rig.particles[c.a];
            auto& B = rig.particles[c.b];

            const float wa = (A.invMass <= 0.0f) ? 0.0f : A.invMass;
            const float wb = (B.invMass <= 0.0f) ? 0.0f : B.invMass;
            const float ws = wa + wb;
            if (ws < 1e-6f) continue;

            RwV3d d = V3Sub(B.posWorld, A.posWorld);
            const float d2 = V3LenSq(d);
            if (d2 < 1e-10f) continue;

            const float dLen = std::sqrt(d2);
            const float diff = (dLen - c.restLength) / dLen;
            const float wA = (wa / ws) * c.stiffness;
            const float wB = (wb / ws) * c.stiffness;

            A.posWorld = V3Add(A.posWorld, V3Scale(d,  diff * wA));
            B.posWorld = V3Add(B.posWorld, V3Scale(d, -diff * wB));
        }

        // 挂点约束：物理顶点与相邻的 rigid 顶点保持静止距离，
        // 这就是"披风挂在肩线上"的来源（不再依赖自动锚定）
        //
        // 修复要点：一个顶点可能同时连着多个 rigid 顶点（实测最多 10 条，
        // 目标点之间最大相距 50mm）。以前是逐条硬投影，最后一条把前面的
        // 结果整个覆盖掉 —— 顶点位置由 pin 在数组里的顺序决定，头皮附近
        // 于是出现"局部凸起/换姿势换个形状"。现在求一次加权一致解：
        //     want_i = target_i + normalize(pos-target_i)*restLength_i
        //     pos   += (Σ w_i·want_i / Σ w_i - pos) · alpha
        // 只有一条 pin 时与原来的硬投影逐位等价，布料行为不变。
        if (kEnablePins)
        for (auto& p : rig.particles)
        {
            if (p.pins.empty()) continue;
            if (p.invMass <= 0.0f) continue;

            // A/B：完全关闭头发的挂点约束（对照实验用，ini 的 hairpins=0）
            if (static_cast<PhysType>(p.typeU32) == PhysType::Hair &&
                !rig.hairPinsEnabled)
                continue;

            const float alpha = p.softPins ? rig.clothPinStiffness : 1.0f;
            if (alpha <= 0.0f) continue;

            if (p.pins.size() == 1)
            {
                const auto& pin = p.pins.front();

                RwV3d d = V3Sub(p.posWorld, pin.targetWorld);
                const float len = V3Len(d);

                if (len < 1e-5f) continue;

                const RwV3d target = V3Add(pin.targetWorld,
                    V3Scale(d, pin.restLength / len));

                p.posWorld = (alpha >= 0.999f)
                    ? target
                    : V3Add(p.posWorld,
                        V3Scale(V3Sub(target, p.posWorld), alpha));
                continue;
            }

            RwV3d wantSum = { 0.0f, 0.0f, 0.0f };
            float weightSum = 0.0f;

            for (const auto& pin : p.pins)
            {
                RwV3d d = V3Sub(p.posWorld, pin.targetWorld);
                const float len = V3Len(d);

                if (len < 1e-5f) continue;

                const RwV3d want = V3Add(pin.targetWorld,
                    V3Scale(d, pin.restLength / len));

                // 短连杆权重更高：紧贴头皮的挂点定位置，长连杆只做弱约束
                const float w = 1.0f /
                    std::max(pin.restLength * pin.restLength, 1e-6f);

                wantSum = V3Add(wantSum, V3Scale(want, w));
                weightSum += w;
            }

            if (weightSum <= 1e-6f) continue;

            const RwV3d want = V3Scale(wantSum, 1.0f / weightSum);

            p.posWorld = V3Add(p.posWorld,
                V3Scale(V3Sub(want, p.posWorld), alpha));
        }

        // 锚定：B=0 -> 完全固定；B=255 -> 完全不拉（只靠约束和重力）
        for (auto& p : rig.particles)
        {
            if (p.invMass <= 0.0f) continue;

            if (p.hardAnchor)
            {
                p.posWorld = p.anchorWorld;
                continue;
            }

            float pull = p.anchorFactor * p.returnStrength;
            // 身体组织：几乎不受重力，但必须有足够回位力，
            // 否则整块胸/臀会一直往下坠，被拉成尖锥
            const PhysType pullType = static_cast<PhysType>(p.typeU32);

            if (pullType == PhysType::Body)
            {
                pull = std::max(pull * 0.5f, 0.02f);
            }
            else if (pullType == PhysType::Hair)
            {
                // 头发：锚定拉力做平方衰减。
                // B=200 时 anchorFactor≈0.22，平方后≈0.05 —— 发丝能跟着头
                // 缓慢回位，但不会被每帧拽回原位（否则看起来"僵着不动"）。
                pull *= p.anchorFactor;

                // ini: hairpull —— <1 更自由（更像布料），>1 更贴身
                pull *= rig.hairPull;

                // ini: hairfollow —— 有挂点的发片向静止姿势回拉的倍率
                // 布料是 clothFollow(0)：挂了点就不回拉，只靠挂点+约束+重力。
                // 设 0 可让头发（或刷成头发的布片）也变成自由垂坠。
                if (p.pinRooted)
                    pull *= rig.hairFollow;
            }
            else if (pullType == PhysType::Cloth)
            {
                // 布料：有挂点的片默认完全不回拉（clothFollow=0），
                // 让它只靠挂点 + 约束 + 重力自然垂坠/摆动。
                // 没有挂点的片保留一部分跟随，否则人物走动时整片会被留在原地。
                pull *= p.pinRooted ? rig.clothFollow : 0.60f;
                pull += p.seamFollow;   // 挂点附近的渐隐过渡，避免硬折
            }

            if (pull <= 0.0f) continue;

            p.posWorld = V3Add(
                p.posWorld,
                V3Scale(V3Sub(p.anchorWorld, p.posWorld), std::clamp(pull, 0.0f, 1.0f)));
        }

        for (size_t ci = 0; ci < rig.colliders.size(); ++ci)
        {
            auto& cap = rig.colliders[ci];

            for (size_t pi = 0; pi < rig.particles.size(); ++pi)
            {
                auto& p = rig.particles[pi];
                if (p.invMass <= 0.0f) continue;

                // 建模时就已经埋在胶囊里的头发不推（见 HairCapsuleMask）
                if (ci < 32 && (p.capsuleIgnore & (1u << ci))) continue;

                // 布料与身体之间额外留 1.5cm 安全距离：
                // 胶囊是骨骼中轴，真实大腿网格表面比它大一圈；
                // 只对 Cloth 加，头发/身体胶囊半径不变。
                const bool isClothP = (static_cast<PhysType>(p.typeU32) == PhysType::Cloth);

                const float margin = isClothP ? rig.clothCollisionMargin : 0.0f;

                // 布料：扣掉"静息时就已经埋进去的深度"。
                // 静止姿态下 effRadius - depth == 当前距离 ⇒ 推力为 0（不抖）；
                // 只有比静息更深（身体压过来 / 布料被拉进身体）时才往外推。
                float softRadius = cap.radius + margin;

                if (isClothP &&
                    rig.clothCapDepth.size() == rig.particles.size() * rig.colliders.size())
                {
                    const float restDepth = rig.clothCapDepth[pi * rig.colliders.size() + ci];
                    softRadius -= restDepth;
                }

                // 布料单独在后面处理（要汇总多根胶囊的推出量再平均）
                if (isClothP) continue;

                CollideCapsule(p.posWorld, cap.aWorld, cap.bWorld, cap.radius);
            }
        }

        // ------------------------------------------------------------
        // 布料 vs 胶囊：**汇总后一次性平均推出**
        //
        // 2026-11-24 17:4x：原来是在"每根胶囊"的循环里直接推，
        // 臀/胯位置骨盆+左右大腿的胶囊互相重叠，粒子被 A 推出去、又陷进 B，
        // 每帧来回 → 猛烈抖动（把胶囊半径设 0 才会消失，但那样也没碰撞了）。
        // 现在把"所有胶囊对本粒子的推出向量"加起来取平均，一帧只应用一次：
        //   - 方向互相抵消时不会过冲（不会左右弹）
        //   - 单根胶囊时行为和原来一致
        //   - 静息深度折扣保持不变（静止时推力≈0）
        // ------------------------------------------------------------
        for (size_t pi = 0; pi < rig.particles.size(); ++pi)
        {
            auto& p = rig.particles[pi];

            if (p.invMass <= 0.0f) continue;
            if (static_cast<PhysType>(p.typeU32) != PhysType::Cloth) continue;

            RwV3d pushSum = { 0.0f, 0.0f, 0.0f };
            int   pushCount = 0;

            const bool haveSoft =
                (rig.clothCapDepth.size() == rig.particles.size() * rig.colliders.size());

            for (size_t ci = 0; ci < rig.colliders.size(); ++ci)
            {
                const auto& cap = rig.colliders[ci];

                if (ci < 32 && (p.capsuleIgnore & (1u << ci))) continue;

                float softRadius = cap.radius + rig.clothCollisionMargin;

                if (haveSoft)
                    softRadius -= rig.clothCapDepth[pi * rig.colliders.size() + ci];

                if (softRadius <= 0.002f) continue;

                RwV3d closest;
                const float d2 = ClosestSegSq(p.posWorld, cap.aWorld, cap.bWorld, closest);
                const float dist = std::sqrt(std::max(d2, 0.0f));

                if (dist >= softRadius || dist < 1e-5f) continue;

                const float pen = softRadius - dist;

                pushSum.x += (p.posWorld.x - closest.x) / dist * pen;
                pushSum.y += (p.posWorld.y - closest.y) / dist * pen;
                pushSum.z += (p.posWorld.z - closest.z) / dist * pen;

                ++pushCount;
            }

            if (pushCount > 0)
            {
                p.posWorld = V3Add(p.posWorld,
                    V3Scale(pushSum, 1.0f / static_cast<float>(pushCount)));
            }
        }
    }

    // 最大拉伸限制：把每条边硬性限制在 restLength 的 1.35 倍以内。
    //
    // === 2026-11-23 修复：不再"整条边跳过" ===
    // 以前：Body 参与的边直接 continue；两端都是 pinned/hardAnchor 的边因为
    // 权重和为 0 也跳过。这些边**永远不做长度修正**，实际被拉到 9~24 倍
    // （[TriSpike] 实测 16.2mm -> 392mm）—— 那就是后脑勺的细长尖刺。
    // 现在按"可动性"分配权重，只要有一端能动能修，这条边就收敛到 1.35x：
    //   Body（invMass=0，由抖动弹簧单独驱动） -> 0     （不动，但边照样限长）
    //   hardAnchor                            -> 0
    //   挂了 pin 的头发发根                    -> 0.35  （半固定：pin 会把它拉回
    //       头皮，但它也必须承担一部分长度修正，否则"两个发根之间"的边无解）
    //   其它                                   -> 1
    auto mobilityOf = [&rig](const VertexParticle& q) -> float
    {
        if (q.hardAnchor) return 0.0f;
        if (q.invMass <= 0.0f) return 0.0f;            // Body
        if (!rig.hairRootStretchExempt) return 1.0f;   // 对照实验 C
        if (static_cast<PhysType>(q.typeU32) == PhysType::Hair && !q.pins.empty())
            return 0.35f;
        return 1.0f;
    };

    auto limitStretch = [&rig, &mobilityOf](int passes)
    {
        for (int pass = 0; pass < passes; ++pass)
        {
            for (auto& c : rig.constraints)
            {
                auto& A = rig.particles[c.a];
                auto& B = rig.particles[c.b];

                const bool hairEdge =
                    static_cast<PhysType>(A.typeU32) == PhysType::Hair &&
                    static_cast<PhysType>(B.typeU32) == PhysType::Hair;

                const float stretchFac  = hairEdge ? rig.hairStretch : rig.clothStretch;
                const float stretchStep = hairEdge ? rig.hairStretchStep : rig.clothStretchStep;

                const float limit = c.restLength * stretchFac;

                RwV3d d = V3Sub(B.posWorld, A.posWorld);
                const float len = V3Len(d);

                if (len <= limit || len < 1e-6f) continue;

                const float wa = mobilityOf(A);
                const float wb = mobilityOf(B);
                const float ws = wa + wb;

                if (ws <= 1e-6f) continue;

                // 每次最多修正 15mm：以前"一步把顶点拽走 30cm"会变成可见的弹跳
                const float excess = std::min(len - limit, stretchStep);
                const RwV3d dir = V3Scale(d, 1.0f / len);

                A.posWorld = V3Add(A.posWorld, V3Scale(dir, excess * (wa / ws)));
                B.posWorld = V3Sub(B.posWorld, V3Scale(dir, excess * (wb / ws)));

                // 诊断：这条边是靠"Body 端点不动 / 发根半固定"才修回来的
                if (static_cast<PhysType>(A.typeU32) == PhysType::Body ||
                    static_cast<PhysType>(B.typeU32) == PhysType::Body)
                    ++rig.diagBodyEdgeFixes;

                if (static_cast<PhysType>(A.typeU32) == PhysType::Hair &&
                    static_cast<PhysType>(B.typeU32) == PhysType::Hair &&
                    (!A.pins.empty() || !B.pins.empty()))
                    ++rig.diagPinEdgeFixes;

                if (excess > rig.diagMaxExcess) rig.diagMaxExcess = excess;
            }
        }
    };

    limitStretch(2);

    // 跨模型碰撞：头发 / 布料 与附近的其他实体（人物、车辆、地图物件）
    // 只做一次（不是每次迭代），控制开销；软体不做。
    if (g_worldCollideFn)
    {
        for (auto& p : rig.particles)
        {
            if (p.invMass <= 0.0f) continue;

            const PhysType t = static_cast<PhysType>(p.typeU32);
            if (t != PhysType::Hair && t != PhysType::Cloth) continue;

            // A/B：hairworldcollide=0 时头发不参与跨实体/地面碰撞
            if (t == PhysType::Hair && !rig.hairWorldCollideEnabled) continue;

            // 高速子步进：这一帧位移大时，沿路径分几次检测，避免"穿过"薄物体
            const RwV3d prevPos = p.prevWorld;
            const RwV3d delta = V3Sub(p.posWorld, prevPos);
            const float moveLen = V3Len(delta);

            int steps = 1 + static_cast<int>(moveLen / 0.05f);
            steps = std::clamp(steps, 1, std::max(1, g_collisionSubsteps));

            bool hitAny = false;
            RwV3d correction = { 0.0f, 0.0f, 0.0f };

            for (int s = 1; s <= steps; ++s)
            {
                const float k = static_cast<float>(s) / static_cast<float>(steps);
                RwV3d sample = V3Add(prevPos, V3Scale(delta, k));

                g_worldCollideFn(sample, prevPos);

                if (WorldCollision_LastHit())
                {
                    correction = V3Sub(sample, p.posWorld);
                    hitAny = true;
                    break;
                }
            }

            // ============================================================
            // 接触响应（2026-11-23 修复"上坡头发炸开"）
            //
            // 旧实现是 `p.posWorld = sample;`：把粒子**直接瞬移到接触点**，
            // 但 prevWorld 没动 ⇒ verlet 把"瞬移量"当成速度。
            // 上坡时发梢常常压在"前方/侧面的坡面"上，每帧被向上抬几厘米，
            // 等效速度 = 抬升量/h ≈ 1~3 m/s，连着几帧就把头发顶到 clamp 球面 ⇒ 炸开。
            // 而且抬得越深踢得越猛（正反馈）。这就是"只有上坡炸"的原因。
            //
            // 现在：
            //   1) 位置修正只当作**几何修正**：pos 和 prev 一起移，不产生速度；
            //   2) 单帧修正量硬上限 5cm，任何离谱采样（坡面/屋顶/墙角）都不会瞬移；
            //   3) 摩擦/障碍物速度仍然作用在**残余**速度上（布料被扫开的手感保留）。
            // ============================================================
            if (hitAny)
            {
                // 头发边只有 ~2cm，修正量必须比它还小，否则单帧就"撕开"一格。
                // 布料边大得多，保持 5cm。
                const float maxCorr =
                    (t == PhysType::Hair) ? 0.015f : 0.05f;

                const float corrLen = V3Len(correction);

                if (corrLen > maxCorr)
                {
                    correction = V3Scale(correction, maxCorr / corrLen);
                    ++rig.diagWorldCorrClamped;
                }

                if (corrLen > rig.diagMaxWorldCorr)
                {
                    rig.diagMaxWorldCorr = corrLen;

                    WorldCollisionHitInfo hi;
                    WorldCollision_LastHitInfo(hi);
                    rig.diagHitKind = hi.kind;
                    rig.diagHitModel = hi.modelId;
                }

                p.posWorld  = V3Add(p.posWorld, correction);
                p.prevWorld = V3Add(p.prevWorld, correction);

                RwV3d impulse = { 0.0f, 0.0f, 0.0f };
                WorldCollision_LastImpulse(impulse);

                p.prevWorld = V3Sub(p.prevWorld, V3Scale(impulse, 0.6f));

                RwV3d vel = V3Sub(p.posWorld, p.prevWorld);
                vel.x *= 0.55f;
                vel.y *= 0.55f;
                vel.z *= 0.85f;
                p.prevWorld = V3Sub(p.posWorld, vel);

                // ============================================================
                // 布料的**有界碰撞冲量**（2026-11-23 06:5x）
                //
                // 2.3 版是 `p.posWorld = sample`：把粒子直接瞬移到接触点，
                // prev 不动 ⇒ verlet 把"瞬移量"当成速度 ⇒ 布料被明显顶开。
                // 03:04 为了修"上坡头发炸开"改成"几何修正 + prev 同步"，
                // 副作用是布料对静止物体几乎没反应（推出 ≤5cm/帧、再被摩擦削掉 45%）。
                //
                // 这里只给**布料**补一个显式、有上限的接触冲量：
                //   kick  = 朝里钻的法向速度 × 0.75，上限 3.0 m/s
                //   sweep = 本帧穿透修正量 / dt × 0.9，上限 2.5 m/s
                // 两个上限保证不会像旧实现那样"穿透越深踢得越猛"（那是炸开的来源）。
                // 头发保持纯几何修正不动。
                // ============================================================
                if (t == PhysType::Cloth)
                {
                    ++rig.diagClothContacts;

                    const float corrLen2 = V3Len(correction);

                    if (corrLen2 > 1e-5f)
                    {
                        constexpr float kClothRestitution = 0.75f;
                        constexpr float kMaxClothKick     = 3.00f;
                        constexpr float kClothSweepGain   = 0.90f;
                        constexpr float kMaxClothSweep    = 2.50f;

                        const RwV3d n = V3Scale(correction, 1.0f / corrLen2);

                        const float frameH = std::max(
                            std::clamp(deltaTime, 1.0f / 240.0f, 1.0f / 20.0f), 1e-5f);

                        const float invH = 1.0f / frameH;

                        RwV3d v2 = V3Sub(p.posWorld, p.prevWorld);

                        const float into =
                            -(n.x * v2.x + n.y * v2.y + n.z * v2.z) * invH;

                        float kick = 0.0f;

                        if (into > 0.0f)
                            kick = std::min(into * kClothRestitution, kMaxClothKick);

                        const float sweep = std::min(
                            corrLen2 * invH * kClothSweepGain, kMaxClothSweep);

                        const float add = kick + sweep;

                        if (add > 0.0f)
                        {
                            v2 = V3Add(v2, V3Scale(n, add * frameH));
                            p.prevWorld = V3Sub(p.posWorld, v2);

                            if (add > rig.diagClothKick)
                                rig.diagClothKick = add;
                        }
                    }
                }
            }
        }
    }

    // 世界碰撞会移动单个顶点（可能最多 5cm），之后立刻再限一次边长：
    // 让邻居跟着一起挪，避免"一个顶点被推走、周围不动"形成局部尖刺。
    // （这一步很便宜：只遍历一次约束）
    limitStretch(1);

    // 约束求解之后再做一次距离限幅：软约束可能把粒子推出很远，
    // 导致胸/臀被拉成尖锥（日志里出现过 body max=1.05m）
    for (auto& p : rig.particles)
    {
        if (p.invMass <= 0.0f) continue;

        const float limit = particleLimit(p);
        const RwV3d toAnchor = V3Sub(p.anchorWorld, p.posWorld);
        const float d2 = V3LenSq(toAnchor);

        if (d2 > limit * limit)
        {
            // 同上：几何钳制不算速度
            const float d = std::sqrt(d2);
            const RwV3d clamped = V3Add(p.anchorWorld, V3Scale(toAnchor, -limit / d));
            const RwV3d corr = V3Sub(clamped, p.posWorld);

            p.posWorld  = clamped;
            p.prevWorld = V3Add(p.prevWorld, corr);
        }
    }

    // ============================================================
    // 最终碰撞修正（静态 + 扫掠 CCD）
    //
    // 放在所有约束和限幅之后：碰撞把粒子推出去，边长约束 / 姿势回拉
    // 又可能把它拉回身体里。分四段：
    //   1) 静态：用本帧姿态把已经穿进去的粒子推出来（2 遍，重叠胶囊收敛）；
    //   2) 扫掠：粒子这一帧的轨迹 vs 胶囊这一帧扫过的体积（防隧穿）；
    //   3) 拉伸收尾：碰撞会把单个顶点推出去，收回边长尖刺；
    //   4) 最终静态：保证最后位置不在任何胶囊内部。
    //
    // 关键修正：静态碰撞也要把法线记下来。以前只记 CCD 的法线，
    // 风把裙子压在腿上时，静态推出改了 pos 却没同步 prev，
    // 下一帧隐式速度就带着"被推出去"的分量 → 弹开、再被风压回，表现为抖动/穿模。
    // ============================================================
    if (!rig.colliders.empty())
    {
        int staticHits = 0;
        int sweptHits = 0;
        int contactParticles = 0;
        float maxPen = 0.0f;
        float finalPen = 0.0f;
        int worstCap = -1;

        if (rig.ccdNormalScratch.size() != rig.particles.size())
            rig.ccdNormalScratch.assign(rig.particles.size(), RwV3d{ 0.0f, 0.0f, 0.0f });

        for (auto& n : rig.ccdNormalScratch)
            n = { 0.0f, 0.0f, 0.0f };

        // 布料额外安全距离：只对 Cloth 生效，不动头发/身体胶囊
        auto clothMargin = [&rig](const VertexParticle& p) -> float
        {
            return (static_cast<PhysType>(p.typeU32) == PhysType::Cloth)
                ? rig.clothCollisionMargin : 0.0f;
        };

        // 1) 静态修正（两遍：髋球 / 大腿 / 骨盆球互相重叠）
        for (int pass = 0; pass < 2; ++pass)
        {
            for (size_t ci = 0; ci < rig.colliders.size(); ++ci)
            {
                const auto& cap = rig.colliders[ci];
                if (cap.radius <= 0.0f) continue;

                for (size_t i = 0; i < rig.particles.size(); ++i)
                {
                    auto& p = rig.particles[i];

                    if (p.invMass <= 0.0f) continue;
                    if (p.hardAnchor) continue;
                    if (ci < 32 && (p.capsuleIgnore & (1u << ci))) continue;

                    RwV3d n = { 0.0f, 0.0f, 0.0f };

                    const float pen = ResolveCapsule(
                        p.posWorld, cap.aWorld, cap.bWorld,
                        cap.radius + clothMargin(p), &n);

                    if (pen <= 0.0f) continue;

                    rig.ccdNormalScratch[i] = V3Add(rig.ccdNormalScratch[i], n);

                    if (pass == 0)
                    {
                        ++staticHits;

                        if (pen > maxPen)
                        {
                            maxPen = pen;
                            worstCap = static_cast<int>(ci);
                        }
                    }
                }
            }
        }

        // 2) 扫掠 CCD（胶囊端点已在每个子步插值）
        for (size_t ci = 0; ci < rig.colliders.size(); ++ci)
        {
            const auto& cap = rig.colliders[ci];
            if (cap.radius <= 0.0f) continue;

            for (size_t i = 0; i < rig.particles.size(); ++i)
            {
                auto& p = rig.particles[i];

                if (p.invMass <= 0.0f) continue;
                if (p.hardAnchor) continue;
                if (ci < 32 && (p.capsuleIgnore & (1u << ci))) continue;

                RwV3d n = { 0.0f, 0.0f, 0.0f };

                const int hits = CollideCapsuleSwept(
                    p.posWorld, p.prevWorld, cap, n, clothMargin(p));

                if (hits <= 0) continue;

                ++sweptHits;
                rig.ccdNormalScratch[i] = V3Add(rig.ccdNormalScratch[i], n);
            }
        }

        // 3) 碰撞会把单个顶点推出去，相邻边可能被拉开：再收一次拉伸上限
        limitStretch(1);

        // 4) 收尾静态：扫掠之后再做一遍，保证最终位置不在任何胶囊内部
        for (size_t ci = 0; ci < rig.colliders.size(); ++ci)
        {
            const auto& cap = rig.colliders[ci];
            if (cap.radius <= 0.0f) continue;

            for (size_t i = 0; i < rig.particles.size(); ++i)
            {
                auto& p = rig.particles[i];

                if (p.invMass <= 0.0f) continue;
                if (p.hardAnchor) continue;
                if (ci < 32 && (p.capsuleIgnore & (1u << ci))) continue;

                RwV3d n = { 0.0f, 0.0f, 0.0f };

                const float pen = ResolveCapsule(
                    p.posWorld, cap.aWorld, cap.bWorld,
                    cap.radius + clothMargin(p), &n);

                if (pen <= 0.0f) continue;

                rig.ccdNormalScratch[i] = V3Add(rig.ccdNormalScratch[i], n);

                if (pen > finalPen) finalPen = pen;
            }
        }

        // 5) 速度一致性：接触粒子只去掉"法向"速度，保留切向。
        //    不再只处理 vn < 0：被碰撞推出时法向速度是朝外的，
        //    保留它会让裙子下一帧弹开、再被风压回，形成抖动。
        for (size_t i = 0; i < rig.particles.size(); ++i)
        {
            const float nl = V3Len(rig.ccdNormalScratch[i]);

            if (nl < 1e-4f) continue;

            auto& p = rig.particles[i];
            const RwV3d n = V3Scale(rig.ccdNormalScratch[i], 1.0f / nl);

            RwV3d v = V3Sub(p.posWorld, p.prevWorld);
            v = V3Sub(v, V3Scale(n, V3Dot(v, n)));

            p.prevWorld = V3Sub(p.posWorld, v);

            ++contactParticles;
        }

        if ((g_stepFrame % 600) == 0)
        {
            std::ostringstream oss;
            oss << "[VER_82][CCD] capsules=" << rig.colliders.size()
                << " static=" << staticHits
                << " swept=" << sweptHits
                << " contactParticles=" << contactParticles
                << " margin=" << (rig.clothCollisionMargin * 1000.0f) << "mm"
                << " maxPen=" << (maxPen * 1000.0f) << "mm"
                << " finalPen=" << (finalPen * 1000.0f) << "mm"
                << " worst=" << (worstCap + 1);   // 1-based，对应 [Colliders] radii= 的顺序
            GeoPhysLog(oss.str());
        }
    }

    // 诊断：布料碰撞反应（每 rig 每 120 帧）
    //   hits  = 这 120 帧里布料粒子的接触次数（0 = 根本没碰到东西）
    //   kick  = 接触冲量峰值（m/s）：<0.3 看不出来，1.0~1.5 明显，>2 夸张
    //
    // 2026-11-23 11:0x：另外累计"实际位移方向"（pos - prev），
    // 因为"风向量相同但看起来相反"用风向是分辨不出来的。
    for (const auto& p : rig.particles)
    {
        if (p.invMass <= 0.0f) continue;

        const PhysType t = static_cast<PhysType>(p.typeU32);
        const RwV3d d = V3Sub(p.posWorld, p.prevWorld);

        if (t == PhysType::Cloth)
        {
            rig.diagClothDisp = V3Add(rig.diagClothDisp, d);
            ++rig.diagClothDispN;
        }
        else if (t == PhysType::Hair)
        {
            rig.diagHairDisp = V3Add(rig.diagHairDisp, d);
            ++rig.diagHairDispN;
        }
    }

    ++rig.clothDiagFrame;

    if (rig.hairDiagnostics && rig.clothCount > 0 && (rig.clothDiagFrame % 120) == 0)
    {
        std::ostringstream ko;
        ko << std::fixed << std::setprecision(2);
        ko << "[VER_82][ClothKick] modelId=" << rig.modelId
           << " hits=" << rig.diagClothContacts
           << " kick=" << rig.diagClothKick << "m/s"
           << " corr=" << (rig.diagMaxWorldCorr * 1000.0f) << "mm"
           << " clamp=" << rig.diagWorldCorrClamped
           << " kind=" << rig.diagHitKind << ":" << rig.diagHitModel
           << " obs=" << WorldCollision_ObstacleCount()
           << " selfObs=" << WorldCollision_OwnerObstacleCount()
           << " caps=" << WorldCollision_CapsuleCount()
           << " tris=" << WorldCollision_TriangleCount();

        // 诊断：最近的布料粒子到最近骨胶囊的**表面距离**。
        //   只有几十 mm 却没碰撞 ⇒ 判定/margin 问题；
        //   上百 mm 以上 ⇒ 角色确实没挨到布（几何问题，不是代码问题）。
        if (rig.clothCount > 0)
        {
            float capMin = -1.0f;

            for (const auto& p : rig.particles)
            {
                if (static_cast<PhysType>(p.typeU32) != PhysType::Cloth) continue;

                const float d = WorldCollision_NearestCapsuleDistance(p.posWorld);

                if (d >= 0.0f && (capMin < 0.0f || d < capMin)) capMin = d;
            }

            if (capMin >= 0.0f)
                ko << " capDist=" << (capMin * 1000.0f) << "mm";
        }

        // 建筑收集统计（每个 rig 都能看到：扇区/访问/收下/盒球/面/薄墙板）
        {
            WorldBuildingStats bld{};
            WorldCollision_GetBuildingStats(bld);

            ko << " bldS=" << bld.sectors << "/" << bld.visited << "/"
               << bld.accepted << "/" << bld.obs << "/" << bld.tris
               << "/" << bld.slabs;
        }

        // ------------------------------------------------------------
        // 诊断：布料 vs 头发**实际吃到的风力向量**，以及各自当前速度方向。
        //
        // 用户反馈"人物布料的风向和其他物理是反的"。风是同一段代码：
        //   SampleWindAcceleration(pos) -> vel += windAcc * h2 * windGain
        // 所以这里把两边的 windAcc / windGain / freedom / 当前速度都打出来：
        //   windC 与 windH 方向相反 ⇒ 采样本身就不同（位置/天气无关？基本不可能，
        //                              除非某一边是 modelSpace 路径）
        //   windC 与 windH 相同、velC 反向 ⇒ 不是风，是约束/胶囊碰撞把它顶回去了
        // ------------------------------------------------------------
        {
            int ci = -1;
            int hi = -1;

            for (size_t i = 0; i < rig.particles.size(); ++i)
            {
                const PhysType t = static_cast<PhysType>(rig.particles[i].typeU32);

                if (rig.particles[i].invMass <= 0.0f) continue;

                if (t == PhysType::Cloth && ci < 0) ci = static_cast<int>(i);
                if (t == PhysType::Hair && hi < 0) hi = static_cast<int>(i);
            }

            if (ci >= 0 || hi >= 0)
            {
                ko << " |";

                auto logOne = [&](const char* tag, int idx)
                {
                    if (idx < 0) return;

                    const auto& p = rig.particles[static_cast<size_t>(idx)];

                    RwV3d w{};
                    SampleWindAcceleration(p.posWorld, w);

                    const RwV3d v = V3Sub(p.posWorld, p.prevWorld);

                    ko << " " << tag
                       << " wind=(" << w.x << "," << w.y << "," << w.z << ")"
                       << " free=" << p.freedom
                       << " af=" << p.anchorFactor
                       << " vel=(" << v.x << "," << v.y << "," << v.z << ")";
                };

                logOne("cloth", ci);
                logOne("hair", hi);
            }
        }

        // 实际平均位移方向（每帧 pos-prev 的累计 / 帧数）
        if (rig.diagClothDispN > 0)
            ko << " dC=(" << (rig.diagClothDisp.x / rig.diagClothDispN) << ","
               << (rig.diagClothDisp.y / rig.diagClothDispN) << ","
               << (rig.diagClothDisp.z / rig.diagClothDispN) << ")";

        if (rig.diagHairDispN > 0)
            ko << " dH=(" << (rig.diagHairDisp.x / rig.diagHairDispN) << ","
               << (rig.diagHairDisp.y / rig.diagHairDispN) << ","
               << (rig.diagHairDisp.z / rig.diagHairDispN) << ")";

        rig.diagClothDisp = { 0.0f, 0.0f, 0.0f };
        rig.diagHairDisp = { 0.0f, 0.0f, 0.0f };
        rig.diagClothDispN = 0;
        rig.diagHairDispN = 0;
        GeoPhysLog(ko.str());

        rig.diagClothKick = 0.0f;
        rig.diagClothContacts = 0;
        rig.diagMaxWorldCorr = 0.0f;
        rig.diagWorldCorrClamped = 0;
    }

    ++g_stepFrame;

    // 头发惯性诊断：每 60 帧报告锚点位移/伪加速度/姿势突变计数
    //   move  = 本帧最大的锚点位移（cm 量级正常，>15cm 说明姿势在跳）
    //   acc   = 本帧最大的伪加速度（限幅前）
    //   pop   = 被判成"姿势突变/传送"的粒子数（这些不注入冲量）
    //   clamp = 伪加速度被限幅的粒子数
    if (rig.hairDiagnostics && !rig.hairParticleSet.empty() &&
        (g_stepFrame % 60) == 0)
    {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(1);
        oss << "[VER_82][HairFrame] frame=" << g_stepFrame
            << " h=" << (h * 1000.0f) << "ms"
            << " move=" << (rig.hfMaxMove * 1000.0f) << "mm"
            << " acc=" << rig.hfMaxAcc << "m/s2"
            << " pop=" << rig.hfPops
            << " clamp=" << rig.hfClamped
            << " speed=" << rig.hfMaxSpeed << "m/s"
            << " rot=" << rig.hfRotDeg << "deg"
            << " iv=" << rig.hfMaxImpulse << "m/s"
            << " ivClamp=" << rig.hfImpulseClamped
            << " wcorr=" << (rig.diagMaxWorldCorr * 1000.0f) << "mm"
            << " wclamp=" << rig.diagWorldCorrClamped
            << " glift=" << WorldCollision_GroundLiftRejected()
            << " wsrc=" << rig.diagHitKind << ":" << rig.diagHitModel;
        GeoPhysLog(oss.str());
    }

    // 拉伸修正诊断：每 600 帧报告"这一帧靠 Body/发根端点修掉了多少次超长边"
    if ((g_stepFrame % 600) == 0 &&
        (rig.diagBodyEdgeFixes > 0 || rig.diagPinEdgeFixes > 0))
    {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(1);
        oss << "[VER_82][StretchFix] frame=" << g_stepFrame
            << " bodyEdges=" << rig.diagBodyEdgeFixes
            << " pinEdges=" << rig.diagPinEdgeFixes
            << " maxExcess=" << (rig.diagMaxExcess * 1000.0f) << "mm";
        GeoPhysLog(oss.str());
    }

    // ============================================================
    // 尖刺诊断（每 120 帧）
    //
    // 尖刺 = 某个顶点离"它邻居的平均位置"很远。用局部间距归一化：
    //     drift  = |pos - avg(neighbor pos)|
    //     space  = 平均邻居距离（该顶点到各邻居的距离平均）
    //     ratio  = drift / space
    // 平滑曲面 ratio≈0.1~0.3；尖刺 > 1。ratio 与模型尺寸无关。
    //
    // 命中时打印最坏顶点的全部线索，用来判断是"哪条约束在拉"：
    //   vert / d(|pos-anchor|) / pins(条数) / pinErr(每个 pin 目标离自身锚点多远)
    //   pinW(每个 pin 的权重和，≠1 说明读到的权重是垃圾) / bones
    //   deg(邻居数) / edgeMax(最大边长比) / pinned(是否挂点根) / pinRooted(整片是否有挂点)
    //   soft / hard / ignore(胶囊屏蔽位数) / drift / space
    // ============================================================
    if (rig.hairDiagnostics && !rig.hairParticleSet.empty() &&
        (g_stepFrame % 120) == 0 &&
        rig.jiggleScratch.size() == rig.particles.size() &&
        rig.jiggleCount.size() == rig.particles.size() &&
        rig.diagScratch.size() == rig.particles.size())
    {
        std::fill(rig.jiggleScratch.begin(), rig.jiggleScratch.end(),
            RwV3d{ 0.0f, 0.0f, 0.0f });
        std::fill(rig.jiggleCount.begin(), rig.jiggleCount.end(), 0);
        std::fill(rig.diagScratch.begin(), rig.diagScratch.end(), 0.0f);

        for (const auto& c : rig.constraints)
        {
            auto& A = rig.particles[c.a];
            auto& Bp = rig.particles[c.b];

            if (static_cast<PhysType>(A.typeU32) != PhysType::Hair) continue;
            if (static_cast<PhysType>(Bp.typeU32) != PhysType::Hair) continue;

            rig.jiggleScratch[c.a] = V3Add(rig.jiggleScratch[c.a], Bp.posWorld);
            rig.jiggleCount[c.a] += 1;
            rig.diagScratch[c.a] += V3Len(V3Sub(A.posWorld, Bp.posWorld));

            rig.jiggleScratch[c.b] = V3Add(rig.jiggleScratch[c.b], A.posWorld);
            rig.jiggleCount[c.b] += 1;
            rig.diagScratch[c.b] += V3Len(V3Sub(Bp.posWorld, A.posWorld));
        }

        int worstIdx = -1;
        int spikeCount = 0;
        float worstRatio = 0.0f;
        float worstDrift = 0.0f;
        float worstSpace = 0.0f;

        for (int idx : rig.hairParticleSet)
        {
            const int n = rig.jiggleCount[idx];
            if (n < 2) continue;

            const auto& p = rig.particles[idx];

            const RwV3d avg = V3Scale(rig.jiggleScratch[idx], 1.0f / n);
            const float drift = V3Len(V3Sub(p.posWorld, avg));
            const float space = std::max(rig.diagScratch[idx] / n, 1e-4f);
            const float ratio = drift / space;

            if (ratio > 1.0f) ++spikeCount;

            if (ratio > worstRatio)
            {
                worstRatio = ratio;
                worstIdx = idx;
                worstDrift = drift;
                worstSpace = space;
            }
        }

        rig.diagSpikeRatio = worstRatio;

        if (worstIdx >= 0 && worstRatio > 1.0f)
        {
            const auto& p = rig.particles[worstIdx];

            // 该顶点的最大边长比
            float edgeMax = 0.0f;
            for (const auto& c : rig.constraints)
            {
                if (c.a != worstIdx && c.b != worstIdx) continue;
                const auto& A = rig.particles[c.a];
                const auto& Bp = rig.particles[c.b];
                const float len = V3Len(V3Sub(Bp.posWorld, A.posWorld));
                if (c.restLength > 1e-5f)
                {
                    const float r = len / c.restLength;
                    if (r > edgeMax) edgeMax = r;
                }
            }

            float selfW = 0.0f;
            for (int k = 0; k < 4; ++k) selfW += p.weights[k];

            std::ostringstream oss;
            oss << std::fixed << std::setprecision(4);
            oss << "[VER_82][HairSpike] frame=" << g_stepFrame
                << " ratio=" << worstRatio
                << " spikes=" << spikeCount
                << " drift=" << (worstDrift * 1000.0f) << "mm"
                << " space=" << (worstSpace * 1000.0f) << "mm"
                << " vert=" << p.geometryVertexIndex
                << " d=" << (V3Len(V3Sub(p.posWorld, p.anchorWorld)) * 1000.0f) << "mm"
                << " deg=" << rig.jiggleCount[worstIdx]
                << " edgeMax=" << edgeMax
                << " pins=" << p.pins.size()
                << " pinW=" << selfW
                << " pinned=" << (p.pins.empty() ? 0 : 1)
                << " pinRooted=" << (p.pinRooted ? 1 : 0)
                << " hard=" << (p.hardAnchor ? 1 : 0)
                << " soft=" << (p.softPins ? 1 : 0)
                << " ignore=0x" << std::hex << p.capsuleIgnore << std::dec
                << " bones=";
            for (int k = 0; k < 4; ++k)
                oss << p.bones[k] << "(" << p.weights[k] << ")";

            // 每个 pin：目标离自身锚点多远 / 它自己用的骨骼和权重和
            oss << " pinErr=";
            {
                int shown = 0;
                for (const auto& pin : p.pins)
                {
                    if (shown) oss << ",";
                    float pw = 0.0f;
                    for (int k = 0; k < 4; ++k) pw += pin.weights[k];
                    oss << (V3Len(V3Sub(pin.targetWorld, p.anchorWorld)) * 1000.0f)
                        << "mm/len" << (pin.restLength * 1000.0f) << "/w" << pw;
                    if (++shown >= 4) break;
                }
            }

            GeoPhysLog(oss.str());
        }
    }

    // ============================================================
    // 逐连通片漂移诊断（每 300 帧）
    //
    // 为什么需要它：一条"整片飞出去"的头发刀片，**每个顶点相对邻居的偏移都很小**
    // （整片一起平移），所以 [HairSpike] 的尖刺比看不出来。必须按片统计：
    //   drift = max |posWorld - anchorWorld|（该片所有粒子）
    //   edge  = max 边长/静止边长
    // 只要某片的 drift 明显大于同类片，它就是那条刀片。
    // ============================================================
    if (rig.hairDiagnostics && !rig.pieces.empty() && (g_stepFrame % 300) == 0)
    {
        struct PieceStat { int pid; int n; int pinned; float drift; float edge; };

        std::vector<PieceStat> stats;
        stats.reserve(rig.pieces.size());

        for (size_t pi = 0; pi < rig.pieces.size(); ++pi)
        {
            const auto& info = rig.pieces[pi];
            if (info.hairCount <= 0) continue;

            float drift = 0.0f;

            for (const auto& p : rig.particles)
            {
                if (p.pieceId != static_cast<int>(pi)) continue;
                if (static_cast<PhysType>(p.typeU32) != PhysType::Hair) continue;

                const float d = V3Len(V3Sub(p.posWorld, p.anchorWorld));
                if (d > drift) drift = d;
            }

            PieceStat st;
            st.pid = static_cast<int>(pi);
            st.n = info.hairCount;
            st.pinned = (info.pinLinks > 0 || info.hardCount > 0) ? 1 : 0;
            st.drift = drift;
            st.edge = 0.0f;
            stats.push_back(st);
        }

        for (auto& st : stats)
        {
            for (const auto& c : rig.constraints)
            {
                const auto& A = rig.particles[c.a];
                const auto& Bp = rig.particles[c.b];
                if (A.pieceId != st.pid || Bp.pieceId != st.pid) continue;
                if (static_cast<PhysType>(A.typeU32) != PhysType::Hair) continue;
                if (c.restLength < 1e-5f) continue;

                const float r = V3Len(V3Sub(Bp.posWorld, A.posWorld)) / c.restLength;
                if (r > st.edge) st.edge = r;
            }
        }

        std::sort(stats.begin(), stats.end(),
            [](const PieceStat& a, const PieceStat& b) { return a.drift > b.drift; });

        std::ostringstream oss;
        oss << std::fixed << std::setprecision(3);
        oss << "[VER_82][PieceDrift] frame=" << g_stepFrame << " pieces=" << stats.size();

        int shown = 0;
        for (const auto& st : stats)
        {
            if (shown >= 6) break;
            oss << " | #" << st.pid
                << " n=" << st.n
                << " pin=" << st.pinned
                << " drift=" << (st.drift * 1000.0f) << "mm"
                << " edge=" << st.edge;
            ++shown;
        }

        GeoPhysLog(oss.str());
    }

    if ((g_stepFrame % 900) == 0)
    {
        int hFree = 0, cFree = 0, bFree = 0, sFree = 0;
        float hMax = 0, cMax = 0, bMax = 0, sMax = 0;

        for (auto& p : rig.particles)
        {
            const PhysType t = static_cast<PhysType>(p.typeU32);
            const float d = V3Len(V3Sub(p.posWorld, p.anchorWorld));

            if (t == PhysType::Body)
            {
                ++bFree;
                if (d > bMax) bMax = d;
                continue;
            }

            if (p.invMass <= 0.0f) continue;

            switch (t)
            {
            case PhysType::Hair:     ++hFree; if (d > hMax) hMax = d; break;
            case PhysType::Cloth:    ++cFree; if (d > cMax) cMax = d; break;
            case PhysType::SoftBody: ++sFree; if (d > sMax) sMax = d; break;
            default: break;
            }
        }

        std::ostringstream oss;
        oss << "[VER_82][Step] frame=" << g_stepFrame
            << " hair(" << hFree << ",max=" << hMax << "m)"
            << " cloth(" << cFree << ",max=" << cMax << "m)"
            << " body(" << bFree << ",max=" << bMax << "m)"
            << " soft(" << sFree << ",max=" << sMax << "m)"
            << " hspike=" << rig.diagSpikeRatio;
        GeoPhysLog(oss.str());
    }
}


// 逐顶点颜色诊断开关（声明在 GeometryPhysics.h）
void GeometryPhysics_SetColorDiag(bool on)
{
    g_colorDiag = on;
}

// 扫掠碰撞子步上限（声明在 GeometryPhysics.h）：1~4，默认 2
void GeometryPhysics_SetCollisionSubsteps(int steps)
{
    g_collisionSubsteps = std::clamp(steps, 1, 4);
}

// Body（R=0xC0）独立手感旋钮：k/c/drive/amp
// 传 0 表示"恢复自动（由顶点色 G 推导）"
void GeometryPhysics_SetBodyTuning(float k, float c, float drive, float amp)
{
    g_bodyK = (k > 0.0f) ? k : 0.0f;
    g_bodyC = (c > 0.0f) ? c : 0.0f;
    g_bodyDrive = (drive > 0.0f) ? drive : 1.30f;
    g_bodyAmp = (amp > 0.0f) ? amp : 1.0f;
}

// Body 分组方式（ini: bodygroup / bodygroupsplit / bodyrandom / bodysmooth）
//   mode: 0=piece（连通片） 1=bone（旧） 2=cluster（片内 k-means）
void GeometryPhysics_SetBodyGrouping(int mode, int split, float random, int smooth)
{
    g_bodyGroupMode = std::clamp(mode, 0, 2);
    g_bodyGroupSplit = std::clamp(split, 1, 8);
    g_bodyRandom = std::clamp(random, 0.0f, 1.0f);
    g_bodySmooth = std::clamp(smooth, 0, 12);
}

// 布料胶囊屏蔽（ini: clothcapsulemask / clothcapsulemaskdepth）
//   enabled: 静息姿态已埋在胶囊里的布料顶点，不再被该胶囊推（防抖）
//   depth  : 判定阈值（米），越大越保守（屏蔽更少）
void GeometryPhysics_SetClothCapsuleMask(int enabled, float depth)
{
    g_clothCapsuleMaskEnabled = (enabled != 0);
    g_clothCapsuleMaskDepth = std::clamp(depth, 0.0f, 0.10f);
}

// 按部位跳过人体胶囊（ini: capskip，位掩码）
//   1=骨盆球 2=髋连接/髋球 4=大腿 8=膝 16=小腿 32=躯干 64=颈头 128=手臂 256=脚
void GeometryPhysics_SetCapsuleSkipMask(int mask)
{
    g_capsuleSkipMask = mask;
}

// 胶囊半径按部位倍率（ini: capsgroupscale*）
void GeometryPhysics_SetCapsuleGroupScale(int group, float scale)
{
    g_capsuleGroupScale[group] = std::clamp(scale, 0.0f, 3.0f);
}

// 胶囊半径按索引倍率（ini: capsulescale=1.0,1.2,...）
void GeometryPhysics_SetCapsuleIndexScale(const std::vector<float>& scales)
{
    g_capsuleIndexScale.clear();

    for (float s : scales)
        g_capsuleIndexScale.push_back(std::clamp(s, 0.0f, 3.0f));
}

bool ApplyGeometryPhysics(
    GeometryPhysics& rig,
    RpAtomic* atomic)
{
    if (!rig.initialized || !atomic) return false;
    RpGeometry* geometry = RpAtomicGetGeometry(atomic);
    if (!geometry) return false;

    // 模型空间模式：顶点就是最终值，直接写回
    if (rig.modelSpace)
    {
        if (!RpGeometryLock(geometry, rpGEOMETRYLOCKVERTICES)) return false;

        RwV3d* verts = RpMorphTargetGetVertices(&geometry->morphTarget[0]);
        if (!verts) { RpGeometryUnlock(geometry); return false; }

        const int numVerts = RpGeometryGetNumVertices(geometry);

        for (auto& p : rig.particles)
        {
            if (p.geometryVertexIndex < 0 || p.geometryVertexIndex >= numVerts) continue;

            const RwV3d pos = p.posWorld;
            verts[p.geometryVertexIndex] = pos;

            for (int weldIndex : p.weldVertices)
            {
                if (weldIndex >= 0 && weldIndex < numVerts)
                    verts[weldIndex] = pos;
            }
        }

        RpGeometryUnlock(geometry);
        return true;
    }

    RpSkin* skin = RpSkinGeometryGetSkin(geometry);
    RpHAnimHierarchy* hierarchy = RpSkinAtomicGetHAnimHierarchy(atomic);

    RwMatrix* boneMatrices = hierarchy
        ? RpHAnimHierarchyGetMatrixArray(hierarchy) : nullptr;
    const RwMatrix* inverseBinds = skin
        ? RpSkinGetSkinToBoneMatrices(skin) : nullptr;

    const RwUInt32 maxBone = std::min<RwUInt32>(rig.skinBoneCount, kMaxBones);

    const bool haveSkinAnchor =
        rig.hasSkinAnchor && boneMatrices && inverseBinds && maxBone > 0;

    // 无蒙皮模型：用原子的世界矩阵做刚体换算（长度严格守恒）
    RwMatrix* atomicLTM = RwFrameGetLTM(RpAtomicGetFrame(atomic));

    RwMatrix rigidInv{};
    bool haveRigidInv = false;

    // 无论如何都准备好"刚体换算"的备用方案：
    // 有些顶点（尤其是细分/新加出来的）没有有效骨骼权重，
    // 走不了逐顶点矩阵；以前这种情况会被 continue 直接跳过，
    // 于是"粒子在动、画面不动"。
    if (atomicLTM)
        haveRigidInv = InvertMatrix(*atomicLTM, rigidInv);

    if (!haveSkinAnchor && !haveRigidInv) return false;

    if (!RpGeometryLock(geometry, rpGEOMETRYLOCKVERTICES | rpGEOMETRYLOCKNORMALS)) return false;

    RwV3d* verts = RpMorphTargetGetVertices(&geometry->morphTarget[0]);
    if (!verts) { RpGeometryUnlock(geometry); return false; }

    const int numVerts = RpGeometryGetNumVertices(geometry);
    if (numVerts <= 0) { RpGeometryUnlock(geometry); return false; }

    float maxOffsetApplied = 0.0f;

    for (auto& p : rig.particles)
    {
        if (p.geometryVertexIndex < 0 || p.geometryVertexIndex >= numVerts)
            continue;

        RwV3d finalPos{};
        bool haveFinalPos = false;

        if (haveSkinAnchor)
        {
            RwMatrix blended;
            RwMatrix inverse;

            if (BuildVertexMatrix(boneMatrices, inverseBinds, p, maxBone, blended, nullptr) &&
                InvertMatrix(blended, inverse))
            {
                finalPos = TransformPoint(inverse, p.posWorld);
                haveFinalPos = true;
            }
        }

        // 没有骨骼权重 / 矩阵不可逆 —— 退回刚体换算，绝不静默跳过
        if (!haveFinalPos && haveRigidInv)
        {
            finalPos = V3Add(p.restLocal,
                TransformDir(rigidInv, V3Sub(p.posWorld, p.anchorWorld)));
            haveFinalPos = true;
        }

        if (!haveFinalPos)
            continue;

        if (!std::isfinite(finalPos.x) ||
            !std::isfinite(finalPos.y) ||
            !std::isfinite(finalPos.z))
        {
            continue;
        }

        float maxOffset = 2.0f;

        switch (static_cast<PhysType>(p.typeU32))
        {
        // 写回上限只是"失控保护"，不能当作正常限制：
        // 披风摆动 0.9m、头发摆动 1.4m 时若被截断，每个顶点的截断方向不同，
        // 三角形就会被拉成尖刺。真正的安全由"边长 ≤135%"负责。
        case PhysType::Hair:     maxOffset = 2.00f; break;
        case PhysType::Cloth:
        {
            // 大面料（旗帜/披风）翻转时，离静止位置可能超过 3m。
            // 硬截断会按"每个顶点截断方向不同"把布压扁、把三角形拉成尖刺，
            // 所以按"绕锚点旋转"的自然半径给上限。
            maxOffset = std::clamp(p.restSpan * 2.0f + 0.5f, 2.0f, 12.0f);
            break;
        }
        case PhysType::Body:     maxOffset = 0.05f; break;   // 胸/臀：最多 5cm
        case PhysType::SoftBody: maxOffset = 1.00f; break;
        default: break;
        }

        RwV3d offset = V3Sub(finalPos, p.restLocal);
        const float offSq = V3LenSq(offset);
        const float maxSq = maxOffset * maxOffset;

        if (offSq > maxSq)
        {
            const float len = std::sqrt(offSq);
            offset  = V3Scale(offset, maxOffset / len);
            finalPos = V3Add(p.restLocal, offset);
        }

        if (V3LenSq(offset) > maxOffsetApplied * maxOffsetApplied)
            maxOffsetApplied = V3Len(offset);

        {
            // 一次性诊断：世界位移和写回局部位移应该基本 1:1
            static bool g_offsetLogged = false;
            if (!g_offsetLogged && static_cast<PhysType>(p.typeU32) == PhysType::Hair)
            {
                g_offsetLogged = true;

                std::ostringstream oss;
                oss << std::fixed << std::setprecision(5);
                oss << "[VER_82][Offset] vertex=" << p.geometryVertexIndex
                    << " worldDelta=" << V3Len(V3Sub(p.posWorld, p.anchorWorld))
                    << " localDelta=" << V3Len(offset);
                GeoPhysLog(oss.str());
            }
        }

        verts[p.geometryVertexIndex] = finalPos;

        // 焊接顶点写同一个位置，保证接缝两侧不会分开
        for (int weldIndex : p.weldVertices)
        {
            if (weldIndex >= 0 && weldIndex < numVerts)
                verts[weldIndex] = finalPos;
        }
    }

    // ============================================================
    // 渲染三角形扫描（每 60 帧）
    //
    // 这是唯一"直接量屏幕上看到的那个三角形"的诊断：在所有写回都做完之后，
    // 用**局部空间**（= 引擎马上要拿去蒙皮的空间）算每个三角形的边长，
    // 和"静止时的边长"比：
    //     ratio = |cur| / |rest|      （rest 用 restLocal；刚体顶点没被动过，就是它本身）
    //     abs   = |cur| - |rest|      （绝对拉伸，米）
    // 只有 ratio 大 **且** abs 大才是看得见的刀片；ratio 大但 abs 很小（1mm→14mm）
    // 是退化短边，不影响画面。
    //
    // 为什么需要它：前面所有诊断都在"世界空间 / 粒子图"上，而尖刺是在写回后的
    // 局部空间里被引擎蒙皮画出来的。粒子图上"整片一起漂移"看不出来，
    // "两个顶点都被 pin 钉住"的边又会被 limitStretch 跳过，只有这里能量到。
    // ============================================================
    if (rig.hairDiagnostics && (g_stepFrame % 60) == 0 &&
        geometry->triangles && geometry->numTriangles > 0)
    {
        auto restOf = [&](int v) -> RwV3d
        {
            const int pi = rig.vertexToParticle[v];
            if (pi >= 0 && pi < static_cast<int>(rig.particles.size()))
                return rig.particles[pi].restLocal;
            return verts[v];      // 刚体顶点不会被写回，当前位置就是静止位置
        };

        float worstAbs = 0.0f, worstRatioBig = 0.0f;
        int worstAbsTri = -1, worstRatioTri = -1;
        int worstAbsEdge[2] = { -1, -1 };
        float worstAbsRest = 0.0f, worstAbsCur = 0.0f;
        float worstRatioVal = 0.0f, worstRatioRest = 0.0f, worstRatioCur = 0.0f;
        int worstRatioEdge[2] = { -1, -1 };
        int hairTriCount = 0;

        const int numTris2 = static_cast<int>(geometry->numTriangles);

        for (int t = 0; t < numTris2; ++t)
        {
            const RpTriangle& tri = geometry->triangles[t];
            int vi[3] = { static_cast<int>(tri.vertIndex[0]),
                          static_cast<int>(tri.vertIndex[1]),
                          static_cast<int>(tri.vertIndex[2]) };

            bool hasHair = false;
            for (int k = 0; k < 3; ++k)
            {
                const int pi = (vi[k] >= 0 && vi[k] < numVerts)
                    ? rig.vertexToParticle[vi[k]] : -1;
                if (pi >= 0 &&
                    static_cast<PhysType>(rig.particles[pi].typeU32) == PhysType::Hair)
                {
                    hasHair = true;
                    break;
                }
            }

            if (!hasHair) continue;
            ++hairTriCount;

            for (int e = 0; e < 3; ++e)
            {
                const int a = vi[e];
                const int b = vi[(e + 1) % 3];

                if (a < 0 || a >= numVerts || b < 0 || b >= numVerts) continue;

                const RwV3d cur = V3Sub(verts[b], verts[a]);
                const RwV3d rest = V3Sub(restOf(b), restOf(a));

                const float lenCur = V3Len(cur);
                const float lenRest = V3Len(rest);

                const float absStretch = lenCur - lenRest;

                if (absStretch > worstAbs)
                {
                    worstAbs = absStretch;
                    worstAbsTri = t;
                    worstAbsEdge[0] = a;
                    worstAbsEdge[1] = b;
                    worstAbsRest = lenRest;
                    worstAbsCur = lenCur;
                }

                // 只统计"够长的边"的比值，避免 0.5mm 短边放大噪声
                if (lenRest >= 0.002f)
                {
                    const float ratio = lenCur / lenRest;
                    if (ratio > worstRatioBig)
                    {
                        worstRatioBig = ratio;
                        worstRatioTri = t;
                        worstRatioVal = ratio;
                        worstRatioRest = lenRest;
                        worstRatioCur = lenCur;
                        worstRatioEdge[0] = a;
                        worstRatioEdge[1] = b;
                    }
                }
            }
        }

        if (worstAbsTri >= 0)
        {
            auto kindOf = [&](int v) -> int
            {
                return (v >= 0 && v < numVerts) ? rig.vertexToParticle[v] : -1;
            };

            // 每个端点的"身份"：类型 / 挂点数 / 是否硬锚定 / 离锚点多远 / 写回位移
            auto endInfo = [&](int v) -> std::string
            {
                std::ostringstream e;
                e << std::fixed << std::setprecision(1);

                const int pi = (v >= 0 && v < numVerts) ? rig.vertexToParticle[v] : -1;

                if (pi < 0)
                {
                    e << "v" << v << ":rigid/dl0.0";
                    return e.str();
                }

                const auto& q = rig.particles[pi];
                const char* tn = "R";
                switch (static_cast<PhysType>(q.typeU32))
                {
                case PhysType::Hair:     tn = "H"; break;
                case PhysType::Cloth:    tn = "C"; break;
                case PhysType::Body:     tn = "B"; break;
                case PhysType::SoftBody: tn = "S"; break;
                default: break;
                }

                const float da = V3Len(V3Sub(q.posWorld, q.anchorWorld)) * 1000.0f;
                const float dl = ((v >= 0 && v < numVerts)
                    ? V3Len(V3Sub(verts[v], restOf(v))) : 0.0f) * 1000.0f;

                e << "v" << v << ":" << tn
                  << "/p" << q.pins.size()
                  << (q.hardAnchor ? "/hard" : "")
                  << "/da" << da << "mm/dl" << dl << "mm";
                return e.str();
            };

            std::ostringstream oss;
            oss << std::fixed << std::setprecision(1);
            oss << "[VER_82][TriSpike] frame=" << g_stepFrame
                << " hairTris=" << hairTriCount
                << " worstAbs=" << (worstAbs * 1000.0f) << "mm"
                << "(cur=" << (worstAbsCur * 1000.0f)
                << "/rest=" << (worstAbsRest * 1000.0f) << "mm)"
                << " tri=" << worstAbsTri
                << " v=" << worstAbsEdge[0] << "," << worstAbsEdge[1]
                << " part=" << kindOf(worstAbsEdge[0]) << "," << kindOf(worstAbsEdge[1])
                << " worstRatio=";


            if (worstRatioTri >= 0)
            {
                oss << worstRatioVal
                    << "(cur=" << (worstRatioCur * 1000.0f)
                    << "/rest=" << (worstRatioRest * 1000.0f) << "mm)"
                    << " v=" << worstRatioEdge[0] << "," << worstRatioEdge[1]
                    << " part=" << kindOf(worstRatioEdge[0]) << "," << kindOf(worstRatioEdge[1]);
            }
            else
            {
                oss << "0";
            }

            oss << " eA=" << endInfo(worstAbsEdge[0])
                << " eB=" << endInfo(worstAbsEdge[1]);

            // 只有真的异常才打（>15mm 绝对拉伸 且 比值 >1.6）
            if (worstAbs > 0.015f && worstRatioBig > 1.6f)
                GeoPhysLog(oss.str());
        }
    }

    // 位移会让包围球偏小，稍微撑大避免边缘被裁掉
    if (rig.boundingRadius > 0.0f && maxOffsetApplied > 0.0f)
    {
        geometry->morphTarget[0].boundingSphere.radius =
            rig.boundingRadius + maxOffsetApplied;
    }

    // ============================================================
    // 法线重算：顶点动过之后，静止时的法线会让明暗出现硬边，
    // 看起来像模型被切碎。这里按三角形面积加权累加面法线再归一化。
    // ============================================================
    if (rig.recomputeNormals && geometry->morphTarget[0].normals)
    {
        RwV3d* normals = geometry->morphTarget[0].normals;
        const RpTriangle* tris = geometry->triangles;
        const int numTris = geometry->numTriangles;

        // 只重算"物理顶点"的法线，其余顶点保持模型原有法线，
        // 避免整张模型的光照被改掉（自定义平滑组/手写法线会丢）。
        if (rig.normalScratch.size() != static_cast<size_t>(numVerts))
            rig.normalScratch.assign(static_cast<size_t>(numVerts), RwV3d{ 0.0f, 0.0f, 0.0f });

        RwV3d* accum = rig.normalScratch.data();

        for (int i = 0; i < numVerts; ++i)
            accum[i] = { 0.0f, 0.0f, 0.0f };

        for (int t = 0; t < numTris; ++t)
        {
            const RpTriangle& tri = tris[t];

            const int i0 = static_cast<int>(tri.vertIndex[0]);
            const int i1 = static_cast<int>(tri.vertIndex[1]);
            const int i2 = static_cast<int>(tri.vertIndex[2]);

            if (i0 < 0 || i0 >= numVerts ||
                i1 < 0 || i1 >= numVerts ||
                i2 < 0 || i2 >= numVerts)
                continue;

            const RwV3d e1 = { verts[i1].x - verts[i0].x,
                               verts[i1].y - verts[i0].y,
                               verts[i1].z - verts[i0].z };
            const RwV3d e2 = { verts[i2].x - verts[i0].x,
                               verts[i2].y - verts[i0].y,
                               verts[i2].z - verts[i0].z };

            const RwV3d n =
            {
                e1.y * e2.z - e1.z * e2.y,
                e1.z * e2.x - e1.x * e2.z,
                e1.x * e2.y - e1.y * e2.x
            };

            accum[i0].x += n.x; accum[i0].y += n.y; accum[i0].z += n.z;
            accum[i1].x += n.x; accum[i1].y += n.y; accum[i1].z += n.z;
            accum[i2].x += n.x; accum[i2].y += n.y; accum[i2].z += n.z;
        }

        for (int i = 0; i < numVerts; ++i)
        {
            if (rig.vertexToParticle[i] < 0) continue;

            const float lenSq = accum[i].x * accum[i].x
                              + accum[i].y * accum[i].y
                              + accum[i].z * accum[i].z;

            if (lenSq < 1e-12f) continue;

            const float inv = 1.0f / std::sqrt(lenSq);
            normals[i].x = accum[i].x * inv;
            normals[i].y = accum[i].y * inv;
            normals[i].z = accum[i].z * inv;
        }
    }

    RpGeometryUnlock(geometry);

    {
        // 一次性证据：确认物理结果真的写回了渲染用的几何体
        static bool g_applyConfirmed = false;
        if (!g_applyConfirmed)
        {
            g_applyConfirmed = true;

            std::ostringstream oss;
            oss << "[VER_82][Apply] first write-back geometry=" << geometry
                << " particles=" << rig.particles.size()
                << " hair=" << rig.hairCount;
            GeoPhysLog(oss.str());
        }
    }

    return true;
}


bool AddDefaultColliders(GeometryPhysics& rig, RpClump* clump)
{
    (void)clump;

    if (!rig.initialized) return false;

    // 有骨骼的模型改用"骨骼链胶囊"（在 StepGeometryPhysics 里生成），
    // 这里的写死占位胶囊只留给无蒙皮的刚体（车辆/道具）
    if (rig.hasSkinAnchor) return true;

    const struct { RwV3d a; RwV3d b; float r; } defs[] =
    {
        { {0, 0.06f, 0}, {0, -0.06f, 0}, 0.10f },
        { {0, 0.05f, 0}, {0, -0.05f, 0}, 0.05f },
        { {0, 0.10f, 0}, {0, -0.10f, 0}, 0.13f },
        { {0, 0.10f, 0}, {0, -0.10f, 0}, 0.12f },
        { {0, 0.08f, 0}, {0, -0.08f, 0}, 0.14f },
    };

    for (const auto& d : defs)
    {
        PhysicsCapsule c;
        c.aLocal = d.a;
        c.bLocal = d.b;
        c.radius = d.r;
        rig.colliders.push_back(c);
    }

    return true;
}

// 头发跟随比例 / 阻尼 / 回拉（ini: hairride / hairdamping / hairpull）
// 头发受风倍率 / 有挂点回拉倍率（ini: hairwind / hairfollow）
void GeometryPhysics_SetHairWindFollow(float wind, float follow)
{
    g_hairWind   = std::clamp(wind, 0.0f, 5.0f);
    g_hairFollow = std::clamp(follow, 0.0f, 1.0f);
}

void GeometryPhysics_SetHairRide(float ride, float damping, float pull)
{
    g_hairRide    = std::clamp(ride, 0.0f, 1.0f);
    g_hairDamping = std::clamp(damping, 0.50f, 0.999f);
    g_hairPull    = std::clamp(pull, 0.0f, 3.0f);
}

// 头发统一骨骼（ini: hairrigidskin，1 = 整片刚性随动，0 = 逐顶点混合）
void GeometryPhysics_SetHairRigidSkin(int enabled)
{
    g_hairRigidSkin = (enabled != 0);
}

// 拉伸上限（ini: hairstretch/hairstretchstep/clothstretch/clothstretchstep）
void GeometryPhysics_SetStretchLimits(float hairFac, float hairStep, float clothFac, float clothStep)
{
    g_hairStretch      = std::clamp(hairFac, 1.0f, 3.0f);
    g_hairStretchStep  = std::clamp(hairStep, 0.001f, 0.10f);
    g_clothStretch     = std::clamp(clothFac, 1.0f, 3.0f);
    g_clothStretchStep = std::clamp(clothStep, 0.001f, 0.10f);
}

void GeometryPhysics_SetHairBend(int enabled)
{
    g_hairBend = (enabled != 0);
}

// 头发惯性参数（ini: hairacc / hairdecay / hairrot）
void GeometryPhysics_SetHairInertiaParams(float acc, float decay, float rot)
{
    g_hairAcc   = std::clamp(acc, 0.0f, 60.0f);
    g_hairDecay = std::clamp(decay, 0.0f, 1.0f);
    g_hairRot   = std::clamp(rot, 0.0f, 1.0f);
}
