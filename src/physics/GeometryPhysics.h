#pragma once

#include "RenderWare.h"
#include <map>
#include <string>
#include <vector>
#include <cstdint>

enum class PhysType : uint8_t {
    Rigid    = 0,
    Hair     = 1,
    Cloth    = 2,
    SoftBody = 3,
    Body     = 4,
};

// ============================================================
// 顶点色约定（三通道，互不干扰，不使用 Alpha）
//
//   R = 物理类型（离散）
//       0x00 (0)   Rigid     完全刚性，不创建粒子，跟随骨骼
//       0x40 (64)  Hair      头发物理
//       0x80 (128) Cloth     布料物理
//       0xC0 (192) Body      身体软组织（跟随骨骼 + 抖动）
//       0xFE (254) SoftBody  通用软体（注意：不能再用 0xFF，会和暖白预光照色撞车）
//       0x03 (3)   Hair      旧版标记的兼容别名
//
//   G = 软度（连续，0 = 最硬 -> 255 = 最软）
//       同时决定约束刚度 stiffness 和回位强度 returnStrength
//
//   B = 锚定强度（连续，0 = 完全固定 -> 255 = 完全自由）
//       anchorFactor = 1 - B/255
//
// 发根用 R=0x40,G=0,B=0（强锚定但仍参与 Hair-Hair 约束，不会断开），
// 发梢用 R=0x40,G=高,B=高；纯刚性部位用 R=0。
// ============================================================

struct VertexPhysParams {
    PhysType type          = PhysType::Rigid;
    float    stiffness     = 1.0f;   // 约束刚度
    float    returnStrength = 0.5f;  // 向锚点回归的软度系数
    float    anchorFactor  = 1.0f;   // 0 = 完全固定, 1 = 完全自由
    bool     hardAnchor    = false;  // B==0：每帧强制贴合锚点（旧版标记不置位）
};

// 解析顶点色。返回 false 表示该顶点不参与物理（Rigid / 未标记）。
bool ClassifyVertexColor(const RwRGBA& color, VertexPhysParams& out);

struct VertexParticle {
    int      geometryVertexIndex = 0;

    RwV3d    restLocal{};
    RwV3d    posWorld{};
    RwV3d    prevWorld{};
    RwV3d    anchorWorld{};

    float    invMass        = 1.0f;    uint32_t typeU32 = 0;

    // 模型空间模式专用：离固定点的归一化距离（0 = 被固定，1 = 最自由）
    // 用来让风只吹动自由边缘，靠近固定处的顶点几乎不动
    float    freedom        = 1.0f;
    float    boundaryFade   = 1.0f;   // Body：到"未刷色边界"的渐隐系数

    // === 顶点色驱动的参数 ===
    float    stiffness      = 0.9f;   // R/G：约束刚度
    float    returnStrength = 0.5f;   // G：锚点回归强度
    float    anchorFactor   = 0.0f;   // B：0 = 完全固定, 1 = 完全自由
    bool     hardAnchor     = false;  // B==0：每帧强制贴合锚点

    // === 每顶点骨骼混合（最多 4 骨骼，和引擎蒙皮完全一致）===
    int      bones[4]   = { -1, -1, -1, -1 };
    float    weights[4] = { 0.0f, 0.0f, 0.0f, 0.0f };

    // 权重最大的骨骼（混合矩阵退化时用它兜底，正交矩阵不会放大位移）
    int      dominantBone   = -1;
    float    dominantWeight = 0.0f;

    // 是否已经用"骨骼姿态下的锚点"摆放过初值
    // （不能用 clump 世界矩阵摆放，和蒙皮位置差一米多，会拉成一片）
    bool     placed = false;

    // === 身体软组织专用：锚点加速度驱动的阻尼弹簧（不做粒子约束）===
    bool     anchorInit = false;
    RwV3d    anchorPrev{};
    RwV3d    anchorVel{};
    RwV3d    jiggleOffset{};
    RwV3d    jiggleVel{};
    float    jiggleWeight = 1.0f;   // 边缘渐隐权重（0 = 完全跟随骨骼）

    // 静止状态下到"同片锚定顶点"的最近距离：头发/布料的自然可伸长度
    float    restSpan = 0.0f;

    // 身体抖动分组（相邻骨骼组会合并成同一个 id）
    int      bodyGroup = -1;

    // 顶点焊接：RenderWare 会把 UV 缝/材质边界处的一个点拆成多个索引顶点，
    // 这些顶点位置相同但不共享三角形。物理必须让它们拿到同一个位置，
    // 否则接缝两侧会被拉开（表现就是"物理顶点和周围分离"）。
    std::vector<int> weldVertices;

    // 与 Rigid（不参与物理）顶点之间的挂点约束：
    // 物理片贴着不动网格的那一圈就是靠它吊住的，不需要任何"自动锚定"。
    struct RigidPin
    {
        RwV3d restLocal{};      // Rigid 顶点的静止位置
        float restLength = 0.0f;
        RwV3d targetWorld{};    // 每帧根据"该 Rigid 顶点自己的骨骼权重"算出的世界位置

        // Rigid 顶点自身的骨骼权重（不能借用物理顶点的矩阵，两者权重可能完全不同）
        int   bones[4]   = { -1, -1, -1, -1 };
        float weights[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
    };

    std::vector<RigidPin> pins;
};

struct VertexConstraint {
    int   a = -1;
    int   b = -1;
    float restLength = 0.1f;
    float stiffness  = 0.9f;
};

struct PhysicsCapsule {
    RwV3d aLocal{};
    RwV3d bLocal{};
    float radius = 0.10f;
    RwV3d aWorld{};
    RwV3d bWorld{};

    // >=0 表示这根胶囊由骨骼驱动：每帧直接用两根骨骼的世界位置当端点
    int   boneA = -1;
    int   boneB = -1;
};

struct GeometryPhysics {
    bool initialized = false;

    // 模型空间模式：用于"共享几何体"的静态地图物件（旗帜 / 灌木 / 挂布）
    // 物理在模型空间里做，所有实例呈现同一份形变（不需要逐实例克隆）
    bool  modelSpace   = false;
    float windStrength = 1.0f;

    // 模型空间模式下：世界"下"和风向在模型空间里的方向
    // 由实例的世界矩阵反算，避免猜 Z-up / Y-up
    RwV3d gravityDir = { 0.0f, 0.0f, -1.0f };
    RwV3d windDirX   = { 1.0f, 0.0f, 0.0f };
    RwV3d windDirY   = { 0.0f, 1.0f, 0.0f };

    // 身体抖动按骨骼分组：同组顶点共用同一个位移，避免相邻点被拉开成尖刺
    struct BodyGroupState
    {
        bool  init = false;
        RwV3d prev{};
        RwV3d anchorVel{};
        RwV3d offset{};
        RwV3d vel{};
    };

    std::map<int, BodyGroupState> bodyGroups;

    // 根运动（整机/骨骼根）的加速度：用来驱动"落脚、起跳"这类冲击，
    // 让骨盆这种本身很平稳的部位也能抖起来
    bool   rootInit = false;
    RwV3d  rootPrevPos{};
    RwV3d  rootVel{};

    // 抖动平滑用的临时缓冲（避免每帧分配）
    std::vector<RwV3d> jiggleScratch;
    std::vector<int>   jiggleCount;
    std::vector<RwV3d> normalScratch;

    float gravity     = 10.0f;
    float damping     = 0.92f;
    bool  recomputeNormals = true;   // 顶点动过之后重算法线，避免明暗硬边
    float maxVelocity = 8.0f;
    int   iterations  = 12;

    std::vector<VertexParticle> particles;
    std::vector<int>            vertexToParticle;

    std::vector<VertexConstraint> constraints;
    std::vector<PhysicsCapsule>   colliders;

    std::vector<int> hairParticleSet;
    std::vector<int> clothParticleSet;
    std::vector<int> softParticleSet;
    std::vector<int> bodyParticleSet;

    int hairCount   = 0;
    int clothCount  = 0;
    int softCount   = 0;
    int bodyCount   = 0;
    int rigidCount  = 0;

    // === 骨骼锚定（通用） ===
    bool     hasSkinAnchor = false;
    RwUInt32 skinBoneCount = 0;

    // 原始包围球半径（位移后适度撑大，避免边缘裁剪）
    float    boundingRadius = 0.0f;
};

bool InitGeometryPhysics(
    GeometryPhysics& rig,
    RpGeometry* geometry,
    const RwMatrix* anchorLTM,
    std::string& report);

void StepGeometryPhysics(
    GeometryPhysics& rig,
    RpAtomic* atomic,
    float deltaTime);

bool ApplyGeometryPhysics(
    GeometryPhysics& rig,
    RpAtomic* atomic);

bool AddDefaultColliders(GeometryPhysics& rig, RpClump* clump);

// 外部（EntityPhysics）提供的跨模型碰撞回调：把点从其他实体的碰撞体里推出去。
// prev 为粒子上一帧位置（用于判断进入方向）。为 nullptr 时表示不启用。
extern void (*g_worldCollideFn)(RwV3d& point, const RwV3d& prev);

// ============================================================
//  天气风力系统
//  每个天气类型可单独配置：风向(度)、风力强度、湍流频率(Hz)
// ============================================================
struct WeatherWindConfig
{
    float dirDeg   = 0.0f;    // 风向：世界 XY 平面，0 = +X（东），90 = +Y（北）
    float strength = 0.0f;    // 风力强度（m/s^2 量级，0 = 无风）
    float turbFreq = 0.0f;    // 湍流频率（Hz，0 = 无湍流）
};

// 配置某个天气的风（weatherId 0..23）
void SetWeatherWind(int weatherId, float dirDeg, float strength, float turbFreq);

// 配置兜底风（未单独配置的天气使用）
void SetDefaultWeatherWind(float dirDeg, float strength, float turbFreq);

// 全局风力倍率（对应 ini 的 wind）
void SetWindMultiplier(float mul);

// 每帧推进风的时间（在步骤里调用）
void AdvanceWindTime(float deltaTime);

// 取某个世界位置上的风加速度
void SampleWindAcceleration(const RwV3d& pos, RwV3d& outAccel);
