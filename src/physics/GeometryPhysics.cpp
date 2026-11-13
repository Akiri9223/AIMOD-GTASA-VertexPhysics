#include "GeometryPhysics.h"
#include "common.h"
#include "WorldCollision.h"

#include "rpskin.h"
#include "rphanim.h"
#include "CWeather.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iomanip>
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

        // 日志精简：超过 4 MB 截断
        static bool s_sizeChecked = false;

        if (!s_sizeChecked)
        {
            s_sizeChecked = true;

            WIN32_FILE_ATTRIBUTE_DATA info{};

            if (GetFileAttributesExA(path.c_str(), GetFileExInfoStandard, &info) &&
                info.nFileSizeHigh == 0 && info.nFileSizeLow > (4u * 1024u * 1024u))
            {
                std::ofstream trunc(path.c_str(), std::ios::out | std::ios::trunc);
            }
        }

        std::ofstream f(path.c_str(), std::ios::out | std::ios::app);
        if (f.is_open())
        {
            f << text << std::endl;
        }
    }

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
        case PhysType::Cloth:    return 0.80f;
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

    void CollideCapsule(RwV3d& p, const RwV3d& a, const RwV3d& b, float r)
    {
        RwV3d closest;
        float d2 = ClosestSegSq(p, a, b, closest);

        if (d2 < r * r && d2 > 1e-12f)
        {
            float d = std::sqrt(d2);
            RwV3d n = V3Scale(V3Sub(p, closest), 1.0f / d);
            p = V3Add(closest, V3Scale(n, r));
        }
        else if (d2 < 1e-12f)
        {
            p = V3Add(closest, { r, 0.0f, 0.0f });
        }
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

            if (isPhys || (v < 16))
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
            const RwUInt32* idx = skinIndices + static_cast<size_t>(v) * 4;

            const float wArr[4] = { w.w0, w.w1, w.w2, skinSplit ? 0.0f : w.w3 };

            for (int k = 0; k < 4; ++k)
            {
                if (wArr[k] > 1e-6f && idx[k] < boneCount)
                {
                    p.bones[k] = static_cast<int>(idx[k]);
                    p.weights[k] = wArr[k];

                    if (wArr[k] > p.dominantWeight)
                    {
                        p.dominantWeight = wArr[k];
                        p.dominantBone = static_cast<int>(idx[k]);
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
    // 三角形拓扑约束
    // ============================================================

    std::vector<uint64_t> edges;

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

                // 记录这个 Rigid 顶点自己的骨骼权重
                if (skinWeights && skinIndices && boneCount > 0)
                {
                    const RwMatrixWeights& w = skinWeights[rigidVertex];
                    const RwUInt32* idx = skinIndices + static_cast<size_t>(rigidVertex) * 4;
                    const float wArr[4] = { w.w0, w.w1, w.w2, skinSplit ? 0.0f : w.w3 };

                    for (int kk = 0; kk < 4; ++kk)
                    {
                        if (wArr[kk] > 1e-6f && idx[kk] < boneCount)
                        {
                            pin.bones[kk] = static_cast<int>(idx[kk]);
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
    }

    std::sort(edges.begin(), edges.end());
    edges.erase(std::unique(edges.begin(), edges.end()), edges.end());

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

        for (const auto& members : components)
        {
            if (members.empty()) continue;

            // ------------------------------------------------------------
            // 简化：Body 顶点一律 100% 幅度，不再做"环数渐隐"。
            // 之前的 ring 0 = 0 会把薄片状区域（臀部）几乎整片冻住，
            // 只剩个别顶点能动，反而在边界拉出细长的尖刺。
            // 接缝安全由"顶点焊接 + 相邻位移差限制"保证，不再靠压幅度。
            // ------------------------------------------------------------
            for (int idx : members)
            {
                auto& p = rig.particles[idx];
                if (static_cast<PhysType>(p.typeU32) != PhysType::Body) continue;
                p.jiggleWeight = 1.0f;
            }

            // AutoAnchor 已彻底移除：
            // 物理片与不动网格之间现在有"挂点约束"（pins），
            // 靠三角形连接自然吊住，不再需要任何自动锚定。
            // 顶点色里的 B=0 仍然表示"固定"（hardAnchor）。
        }

        // 注意：旧的"重复顶点同步硬锚定"(mirrored anchors) 已经彻底删除。
        // 它会因为某个 B=0 顶点而把同位置的其他物理顶点一起钉死，
        // 正是"刷了颜色却不动"的隐藏来源。

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
        // 自然可伸长度：每个粒子离"同片锚定顶点"的静止距离
        // 头发/布料的位移不会超过这个距离 + 少量余量，
        // 从根上杜绝"发丝被拉成刀片"。
        // ============================================================

        float maxSpan = 0.0f;

        for (const auto& members : components)
        {
            if (members.empty()) continue;

            std::vector<int> anchors;

            for (int idx : members)
            {
                if (rig.particles[idx].hardAnchor) anchors.push_back(idx);
            }

            if (anchors.empty()) continue;

            for (int idx : members)
            {
                auto& p = rig.particles[idx];

                if (p.hardAnchor) { p.restSpan = 0.0f; continue; }

                float best = 1e9f;

                for (int a : anchors)
                {
                    const float d = V3Len(V3Sub(p.restLocal, rig.particles[a].restLocal));
                    if (d < best) best = d;
                }

                if (best < 1e8f)
                {
                    p.restSpan = best;
                    if (best > maxSpan) maxSpan = best;
                }
            }
        }

        if (maxSpan > 0.0f)
        {
            std::ostringstream oss;
            oss << "[VER_82][Span] maxRestSpan=" << maxSpan << "m";
            GeoPhysLog(oss.str());
        }
    }

    rig.jiggleScratch.assign(rig.particles.size(), RwV3d{ 0.0f, 0.0f, 0.0f });
    rig.jiggleCount.assign(rig.particles.size(), 0);

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

    // ============================================================
    // 模型空间模式（静态地图物件：共享几何体，风摆）
    // 没有骨骼、没有世界变换，直接把顶点在模型空间里吹动
    // ============================================================
    if (rig.modelSpace)
    {
        const float dt = std::clamp(deltaTime, 1.0f / 240.0f, 1.0f / 20.0f) * 0.5f;  // 两个子步
        const float wind = rig.windStrength;

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
                vel = V3Scale(vel, 0.96f);         // 空气阻力

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

                // 形状记忆：很轻，只防止塌成一团
                const RwV3d toRest = V3Sub(p.restLocal, p.posWorld);

                // 重力沿"实例反算出来的下方向"
                const float gravityScale = 9.8f * (0.30f + 0.70f * p.freedom);

                const RwV3d acc =
                {
                    windModel.x * gain + rig.windDirX.x * turbX + rig.windDirY.x * turbY
                        + rig.gravityDir.x * gravityScale + toRest.x * 3.5f,

                    windModel.y * gain + rig.windDirX.y * turbX + rig.windDirY.y * turbY
                        + rig.gravityDir.y * gravityScale + toRest.y * 3.5f,

                    windModel.z * gain + rig.windDirX.z * turbX + rig.windDirY.z * turbY
                        + rig.gravityDir.z * gravityScale + toRest.z * 3.5f
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
            }
        }

        // 限幅：别偏离静止位置太远（防止整片被吹走）
        for (auto& p : rig.particles)
        {
            if (p.invMass <= 0.0f) continue;

            const RwV3d toRest = V3Sub(p.posWorld, p.restLocal);
            const float len = V3Len(toRest);

            if (len > 0.55f)
                p.posWorld = V3Add(p.restLocal, V3Scale(toRest, 0.55f / len));
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

    RwMatrix* anchorLTM = clumpLTM ? clumpLTM : atomicLTM;

    if (!haveSkinAnchor && !anchorLTM) return;

    // 锚点 = 顶点在"当前骨骼姿态"下的世界位置（与引擎蒙皮完全一致）
    int placedCount = 0;
    int blendFallbacks = 0;
    int fallbackSamples[3] = { -1, -1, -1 };

    for (auto& p : rig.particles)
    {
        RwMatrix blended;
        bool usedFallback = false;

        if (haveSkinAnchor &&
            BuildVertexMatrix(boneMatrices, inverseBinds, p, maxBone, blended, &usedFallback))
        {
            p.anchorWorld = TransformPoint(blended, p.restLocal);

            // 挂点目标：必须用"该 Rigid 顶点自己的骨骼权重"算，
            // 不能借用物理顶点的矩阵（两者权重可能完全不同）
            for (auto& pin : p.pins)
            {
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
            p.posWorld = p.anchorWorld;
            p.prevWorld = p.anchorWorld;
            p.placed = true;
            ++placedCount;
        }
        else if (V3LenSq(V3Sub(p.posWorld, p.anchorWorld)) > 1.5f * 1.5f)
        {
            // 传送 / 重生 / 模型重载这类大幅位移：直接贴合，别把发丝留在原地
            p.posWorld = p.anchorWorld;
            p.prevWorld = p.anchorWorld;
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

    // 碰撞体：骨骼驱动的胶囊（第一次运行时按相邻骨骼生成），
    // 其余情况用原子的世界矩阵换算本地胶囊
    if (rig.colliders.empty() && haveSkinAnchor && maxBone >= 2)
    {
        for (RwUInt32 i = 0; i + 1 < maxBone && rig.colliders.size() < 20; ++i)
        {
            const float seg = V3Len(V3Sub(boneMatrices[i + 1].pos, boneMatrices[i].pos));

            if (seg < 0.01f || seg > 0.5f) continue;

            PhysicsCapsule cap;
            cap.boneA = static_cast<int>(i);
            cap.boneB = static_cast<int>(i + 1);
            cap.radius = std::clamp(0.35f * seg, 0.03f, 0.10f);

            rig.colliders.push_back(cap);
        }

        std::ostringstream oss;
        oss << "[VER_82][Colliders] skeleton capsules=" << rig.colliders.size()
            << " bones=" << maxBone;
        GeoPhysLog(oss.str());
    }

    for (auto& cap : rig.colliders)
    {
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
    }

    const float h  = deltaTime;
    const float h2 = h * h;
    const RwV3d gravityBase = { 0.0f, 0.0f, -rig.gravity };
    const float damping   = std::clamp(rig.damping, 0.0f, 1.0f);
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

            const RwV3d anchorVel = V3Scale(V3Sub(anchorAvg, state.prev), invDt);
            const RwV3d anchorAcc = V3Scale(V3Sub(anchorVel, state.anchorVel), invDt);

            state.prev      = anchorAvg;
            state.anchorVel = anchorVel;

            const float soft = std::clamp(groupSoft[g] / static_cast<float>(count), 0.05f, 1.0f);

            // 手感参数（Q 弹的关键是低阻尼 + 中等刚度）
            //   k：越大越"紧"，越小越"慢而软"
            //   c：阻尼，越小回弹次数越多（ζ = c / (2√k)）
            const float k = 30.0f + 120.0f * soft;   // 软 -> 65，硬 -> 150
            const float c = 1.5f + 5.0f * soft;      // 软 -> 3.0（ζ≈0.19，很弹），硬 -> 6.5

            // 驱动的加速度放大：角色移动/蹲起时的带动更明显
            const float driveScale = 1.30f;

            RwV3d acc = V3Sub(V3Scale(state.offset, -k), V3Scale(state.vel, c));

            // 驱动 = 本组锚点加速度 + 整机根加速度（后者让臀部这类
            // 本身很平稳的部位也能被落脚/起跳带起来）
            const RwV3d drive = V3Add(anchorAcc, V3Scale(rootAcc, 0.8f));
            acc = V3Sub(acc, V3Scale(drive, driveScale));

            state.vel    = V3Add(state.vel, V3Scale(acc, deltaTime));
            state.offset = V3Add(state.offset, V3Scale(state.vel, deltaTime));

            // 幅度 = 锚定强度(B) × 软度(G)：G 越小（越硬）抖动越小
            const float softnessN = std::clamp(1.0f - soft, 0.0f, 1.0f);
            const float amp = 0.070f * std::clamp(groupAmp[g], 0.0f, 1.0f)
                                     * (0.15f + 0.85f * softnessN);

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
                    const float len = std::sqrt(offSq);
                    state.offset = V3Scale(state.offset, amp / len);
                    state.vel    = V3Scale(state.vel, 0.5f);
                }
            }
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
        // 表现为"单点尖刺"。沿 Body-Body 约束边平均两遍即可消除。
        const size_t particleCount = rig.particles.size();

        if (rig.jiggleScratch.size() != particleCount)
        {
            rig.jiggleScratch.assign(particleCount, RwV3d{ 0.0f, 0.0f, 0.0f });
            rig.jiggleCount.assign(particleCount, 0);
        }

        for (int pass = 0; pass < 12; ++pass)
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

    // 允许的最大位移：类型上限，且不超过"自己的自然长度"
    auto particleLimit = [](const VertexParticle& p) -> float
    {
        const PhysType t = static_cast<PhysType>(p.typeU32);
        float limit = MaxAnchorDistance(t);

        if ((t == PhysType::Hair || t == PhysType::Cloth || t == PhysType::SoftBody) &&
            p.restSpan > 0.0f)
        {
            const float slack = (t == PhysType::Hair) ? 0.10f : 0.15f;
            limit = std::min(limit, p.restSpan + slack);
        }

        return limit;
    };

    for (auto& p : rig.particles)
    {
        if (p.invMass <= 0.0f) continue;

        const PhysType physType = static_cast<PhysType>(p.typeU32);

        const float gravMul = GravityScale(physType);
        const float maxDist = particleLimit(p);

        RwV3d vel = V3Sub(p.posWorld, p.prevWorld);
        vel = V3Scale(vel, damping);
        vel = V3Add(vel, V3Scale(gravityBase, h2 * gravMul));

        // 风：按当前天气取样（头发/布料受影响，身体组织不受风）
        if (physType == PhysType::Hair || physType == PhysType::Cloth)
        {
            RwV3d windAcc;
            SampleWindAcceleration(p.posWorld, windAcc);

            // 越靠外（freedom 越大）受风越明显
            const float windGain = 0.35f + 0.65f * p.freedom;
            vel = V3Add(vel, V3Scale(windAcc, h2 * windGain));
        }

        const float vSq = V3LenSq(vel);
        if (vSq > maxStepSq && vSq > 1e-12f)
            vel = V3Scale(vel, maxStep / std::sqrt(vSq));

        p.prevWorld = p.posWorld;
        p.posWorld  = V3Add(p.posWorld, vel);

        // 安全网：只限制离锚点的最大距离，不再瞬移回锚点
        // （以前这里直接 reset 到 anchorWorld，跑动时会把头发反复拽到头上，
        //   看起来就是"被推着走 + 拉成一片"）
        const RwV3d toAnchor = V3Sub(p.anchorWorld, p.posWorld);
        const float d2 = V3LenSq(toAnchor);

        if (d2 > maxDist * maxDist)
        {
            const float d = std::sqrt(d2);
            p.posWorld = V3Add(p.anchorWorld, V3Scale(toAnchor, -maxDist / d));
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
        if (kEnablePins)
        for (auto& p : rig.particles)
        {
            if (p.pins.empty()) continue;
            if (p.invMass <= 0.0f) continue;

            for (const auto& pin : p.pins)
            {
                RwV3d d = V3Sub(p.posWorld, pin.targetWorld);
                const float len = V3Len(d);

                if (len < 1e-5f) continue;

                p.posWorld = V3Add(pin.targetWorld,
                    V3Scale(d, pin.restLength / len));
            }
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
            }

            if (pull <= 0.0f) continue;

            p.posWorld = V3Add(
                p.posWorld,
                V3Scale(V3Sub(p.anchorWorld, p.posWorld), std::clamp(pull, 0.0f, 1.0f)));
        }

        for (auto& cap : rig.colliders)
        {
            for (auto& p : rig.particles)
            {
                if (p.invMass <= 0.0f) continue;
                CollideCapsule(p.posWorld, cap.aWorld, cap.bWorld, cap.radius);
            }
        }
    }

    // 最大拉伸限制：软约束在重力下会把相邻顶点拉开（披风边缘就会出现
    // 尖刺/扇状），这里把每条边的长度硬性限制在 restLength 的 1.35 倍以内。
    for (int pass = 0; pass < 2; ++pass)
    {
        for (auto& c : rig.constraints)
        {
            auto& A = rig.particles[c.a];
            auto& B = rig.particles[c.b];

            if (static_cast<PhysType>(A.typeU32) == PhysType::Body) continue;
            if (static_cast<PhysType>(B.typeU32) == PhysType::Body) continue;

            const float limit = c.restLength * 1.35f;

            RwV3d d = V3Sub(B.posWorld, A.posWorld);
            const float len = V3Len(d);

            if (len <= limit || len < 1e-6f) continue;

            const float wa = (A.hardAnchor || A.invMass <= 0.0f) ? 0.0f : 1.0f;
            const float wb = (B.hardAnchor || B.invMass <= 0.0f) ? 0.0f : 1.0f;
            const float ws = wa + wb;

            if (ws <= 0.0f) continue;

            const float excess = len - limit;
            const RwV3d dir = V3Scale(d, 1.0f / len);

            A.posWorld = V3Add(A.posWorld, V3Scale(dir, excess * (wa / ws)));
            B.posWorld = V3Sub(B.posWorld, V3Scale(dir, excess * (wb / ws)));
        }
    }

    // 跨模型碰撞：头发 / 布料 与附近的其他实体（人物、车辆、地图物件）
    // 只做一次（不是每次迭代），控制开销；软体不做。
    if (g_worldCollideFn)
    {
        for (auto& p : rig.particles)
        {
            if (p.invMass <= 0.0f) continue;

            const PhysType t = static_cast<PhysType>(p.typeU32);
            if (t != PhysType::Hair && t != PhysType::Cloth) continue;

            // 高速子步进：这一帧位移大时，沿路径分几次检测，避免"穿过"薄物体
            const RwV3d prevPos = p.prevWorld;
            const RwV3d delta = V3Sub(p.posWorld, prevPos);
            const float moveLen = V3Len(delta);

            int steps = 1 + static_cast<int>(moveLen / 0.05f);
            steps = std::clamp(steps, 1, 4);

            bool hitAny = false;

            for (int s = 1; s <= steps; ++s)
            {
                const float k = static_cast<float>(s) / static_cast<float>(steps);
                RwV3d sample = V3Add(prevPos, V3Scale(delta, k));

                g_worldCollideFn(sample, prevPos);

                if (WorldCollision_LastHit())
                {
                    p.posWorld = sample;
                    hitAny = true;
                    break;
                }
            }

            // 接触响应：摩擦 + 把障碍物的速度传给粒子（布料被"扫开"而不是被粘住）
            if (hitAny)
            {
                RwV3d impulse = { 0.0f, 0.0f, 0.0f };
                WorldCollision_LastImpulse(impulse);

                p.prevWorld = V3Sub(p.prevWorld, V3Scale(impulse, 0.6f));

                RwV3d vel = V3Sub(p.posWorld, p.prevWorld);
                vel.x *= 0.55f;
                vel.y *= 0.55f;
                vel.z *= 0.85f;
                p.prevWorld = V3Sub(p.posWorld, vel);
            }
        }
    }

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
            const float d = std::sqrt(d2);
            p.posWorld = V3Add(p.anchorWorld, V3Scale(toAnchor, -limit / d));
        }
    }

    ++g_stepFrame;

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
            << " soft(" << sFree << ",max=" << sMax << "m)";
        GeoPhysLog(oss.str());
    }
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
        case PhysType::Cloth:    maxOffset = 2.00f; break;
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
