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

    // 该粒子所在的整片布料/头发是否有"挂点(pin)"钉在刚体上。
    // 有挂点的片不需要每帧被拉回静止姿势——那股回拉正是"把布料提上去的隐形力"。
    bool     pinRooted      = false;

    // 挂点是"软连接"：允许布料在重力下从挂点滑开一点，从而自然垂落。
    // 只给"挂在竖杆上的旗面"这类片开启（见 InitGeometryPhysics 的方向判定）。
    bool     softPins       = false;

    // 接缝渐隐：靠近挂点/刚性边的一圈布料向静止姿势的跟随强度。
    // 让布面从"固定边"平滑地弯过去，而不是在第一排布料处硬折。
    float    seamFollow     = 0.0f;

    // 初始褶皱种子（世界空间偏移，很小）。完美平面的布料在无风时
    // 只有"整体旋转"这一个平衡态，加一点扰动才会起褶、看起来像布。
    RwV3d    seedOffset{};

    // === 每顶点骨骼混合（最多 4 骨骼，和引擎蒙皮完全一致）===
    int      bones[4]   = { -1, -1, -1, -1 };
    float    weights[4] = { 0.0f, 0.0f, 0.0f, 0.0f };

    // 权重最大的骨骼（混合矩阵退化时用它兜底，正交矩阵不会放大位移）
    int      dominantBone   = -1;
    float    dominantWeight = 0.0f;

    // 是否已经用"骨骼姿态下的锚点"摆放过初值
    // （不能用 clump 世界矩阵摆放，和蒙皮位置差一米多，会拉成一片）
    bool     placed = false;

    // === 布料惯性继承 ===
    // 锚点上一帧的世界位置。角色跑动时整片布料必须"跟着身体走"，
    // 只靠挂点/回拉去追会永久滞后（腿就从裙子里穿过去）。
    RwV3d    anchorPrevWorld{};
    bool     anchorPrevValid = false;

    // === 头发惯性（frame transport）===
    // 世界空间 Verlet 里，角色自己的位移不是头发的速度。
    // 每帧把锚点自身的位移同时加到 pos/prev 上（把整体平移+旋转从"头发速度"
    // 里扣掉），再把锚点加速度反号当惯性力：加速时滞后、匀速时不滞后、
    // 急停时向前甩。见 StepGeometryPhysics 的 hair transport 段。
    RwV3d    anchorFrameVel{};        // 上一帧的锚点速度（原始，未滤波）
    bool     anchorFrameValid = false;

    // 平滑后的"伪加速度"（= 本帧锚点加速度的低通）。用它驱动惯性力，
    // 而不是直接用二阶差分：姿势/动画突变时二阶差分会在单帧里冒出上百 m/s²，
    // 那个冲量直接进 verlet 就是"上坡头发炸开"。低通 + 限幅把它压成一次
    // 平滑的、有界的冲量（见 StepGeometryPhysics）。
    RwV3d    frameAccSmooth{};
    bool     frameAccSmoothValid = false;

    // 有界"伪速度"（m/s）：伪惯性力累积成的速度，硬上限 kMaxImpulseVel。
    // 没有它的时候，30fps + 快速转身/坡面修正会让伪加速度连续饱和，
    // 每帧注入 0.4~0.7 m/s，头发被顶到 4~8 m/s —— 就是"上坡炸开"。
    RwV3d    frameImpulseVel{};

    // 静息姿态下就**已经在胶囊内部**的骨骼胶囊（按 bit 屏蔽）。
    // 头发从头皮里长出来，天生位于头/颈/胸胶囊内部；如果照样往外推，
    // 站立不动时整片头发会被推离头皮 4~9cm（即"后脑勺局部凸起"）。
    // 只对 Hair 设置；Cloth / Body / SoftBody 保持原样。
    uint32_t capsuleIgnore = 0;

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

    // 诊断：所在连通片（Init 时按约束并查集分配，-1 = 未分配）
    int      pieceId = -1;

    // 与 Rigid（不参与物理）顶点之间的挂点约束：
    // 物理片贴着不动网格的那一圈就是靠它吊住的，不需要任何"自动锚定"。
    struct RigidPin
    {
        RwV3d restLocal{};      // Rigid 顶点的静止位置
        float restLength = 0.0f;
        RwV3d targetWorld{};    // 每帧根据"该 Rigid 顶点自己的骨骼权重"算出的世界位置

        // 该 Rigid 顶点自己的骨骼权重（不能借用物理顶点的矩阵，两者权重可能完全不同）
        int   bones[4]   = { -1, -1, -1, -1 };
        float weights[4] = { 0.0f, 0.0f, 0.0f, 0.0f };

        // 目标 Rigid 顶点的几何体索引：同一个顶点会被相邻三角形反复加入，
        // 必须去重（一个头发顶点最多挂 10 条重复 pin，等于反复硬投影同一个目标）
        int   rigidVertex = -1;
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

    // 上一帧的两个端点：扫掠碰撞要考虑骨骼自己的运动
    RwV3d aPrevWorld{};
    RwV3d bPrevWorld{};

    // >=0 表示这根胶囊由骨骼驱动：每帧直接用两根骨骼的世界位置当端点
    int   boneA = -1;
    int   boneB = -1;

    // 部位分组（用于 ini caps kip 逐个部位做 A/B）：
    //   1=骨盆球 2=髋/骨盆连接 4=大腿 8=膝 16=小腿 32=躯干 64=颈头 128=手臂 256=脚
    int   group = 0;
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
        RwV3d accSmooth{};     // 低通后的锚点加速度（驱动弹簧用，滤掉动画噪声）
    };

    std::map<int, BodyGroupState> bodyGroups;

    // 根运动（整机/骨骼根）的加速度：用来驱动"落脚、起跳"这类冲击，
    // 让骨盆这种本身很平稳的部位也能抖起来
    // Body 弹簧诊断（每帧累计，定期打印 [BodySpring]）
    float  bodyMaxDrive  = 0.0f;
    float  bodyMaxOffset = 0.0f;
    float  bodyMaxVel    = 0.0f;
    int    bodyResets    = 0;
    int    bodyDiagFrame = 0;   // 每 rig 自己的帧计数（g_stepFrame 是全局的，
                                // 多 rig 时 % 120 只会命中其中一个，导致诊断抽样偏斜）

    bool   rootInit = false;
    RwV3d  rootPrevPos{};
    RwV3d  rootVel{};

    // 抖动平滑用的临时缓冲（避免每帧分配）
    std::vector<RwV3d> jiggleScratch;
    std::vector<int>   jiggleCount;
    std::vector<RwV3d> normalScratch;

    // 扫掠碰撞的"接触法线"累加缓冲（每个粒子一个，最终一次性修速度）
    std::vector<RwV3d> ccdNormalScratch;

    float gravity     = 10.0f;
    float damping     = 0.92f;
    bool  recomputeNormals = true;   // 顶点动过之后重算法线，避免明暗硬边
    float maxVelocity = 8.0f;
    int   iterations  = 12;

    // === 布料手感（由 ini 的 clothgravity / clothfollow / clothdamping 设置）===
    // 布料默认是"自由垂坠"的：只靠挂点+约束+重力，不再被静止姿势回拉。
    float clothGravity = 1.0f;    // 重力倍率（1 = 与头发相同）
    float clothFollow  = 0.0f;    // 有挂点的布料向静止姿势回拉的强度（0 = 完全自由）
    float clothDamping = 0.95f;   // 每帧阻尼（60fps 基准；越小越快静止）
    float clothPinStiffness = 0.15f;  // 竖向挂点的布料：每帧挂点修正比例（1 = 硬缝死）
    float clothWind = 1.8f;       // 布料受风倍率（比头发更吃风，旗帜才飘得起来）

    // 布料惯性继承：锚点这一帧的位移有多少比例直接带着布料一起走。
    //   0    = 旧行为（布料只靠挂点/回拉去追身体，跑动时永久滞后十几厘米）
    //   0.6  = 推荐：布料跟着身体走，另外 40% 留给自然滞后/摆动
    //   1.0  = 完全跟死（布料像焊在身体上，跑动时不会滞后也不会飘）
    float clothInertia = 0.60f;

    // 布料与身体胶囊之间的安全距离（米）。只加到 Cloth 粒子，
    // 不动头发/身体胶囊本身，避免把头发也撑开。
    //   0     = 贴着胶囊表面（容易露出腿部网格）
    //   0.015 = 推荐：裙子离腿 1.5cm，视觉上不再"贴进肉里"
    // 布料 vs 自身骨胶囊的安全距离（ini: characterclothselfmargin）
    float clothCollisionMargin = 0.015f;

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

    // 骨骼胶囊的"上一帧端点"是否已初始化（用于扫掠碰撞）
    bool     colliderPrevValid = false;

    // 头发的"静息姿态胶囊屏蔽"是否已经算过（每个 rig 只算一次）
    bool     capsuleMaskBuilt = false;

    // ------------------------------------------------------------
    // 布料的"静息深度软碰撞"（2026-11-24 17:4x）
    //
    // 布料贴在臀/腿上时，静息姿态就已经埋在胶囊里。硬推到表面的话，
    // 每帧"推出 → 锚点拉回"会形成剧烈抖动。
    // 这里记录每个布料顶点在静息时埋进每根胶囊的深度，
    // 运行时只把"超过静息深度"的部分推回去 ⇒ 静止零推力、不抖，
    // 但身体靠近布料时仍会把它推开（碰撞依然有效）。
    //   clothCapDepth 索引 = particleIndex * colliders.size() + capsuleIndex
    // ------------------------------------------------------------
    bool     clothCapBuilt = false;
    std::vector<float> clothCapDepth;

    // 头发兜底骨骼绑定是否已经算过（读不到逐顶点权重时按最近骨骼绑定）
    bool     hairRebindDone = false;

    // === 头发 A/B 诊断开关（由 ini 配置，默认全开 = 正常行为）===
    //   hairPinsEnabled        : false = 完全不解头发挂点约束（只看边长/锚点/惯性/碰撞）
    //   hairCapsuleMaskEnabled : false = 不屏蔽"静息姿态已重叠"的胶囊（恢复旧的膨胀行为）
    //   hairRootStretchExempt  : false = 拉伸修正不再豁免挂了 pin 的头发发根
    //   hairPieceFixEnabled    : true  = 把"既没有 pin 也没有硬锚点"的头发连通片
    //                                    改成跟随骨骼（引擎里它们本来也是刚性的）
    //   hairDiagnostics        : true  = 输出 [HairPiece] / [PieceDrift] / [HairSpike] 诊断
    bool     hairPinsEnabled = true;
    bool     hairCapsuleMaskEnabled = true;
    bool     hairRootStretchExempt = true;
    bool     hairPieceFixEnabled = true;
    bool     hairDiagnostics = true;

    // 连通片表（Init 填，诊断 + "无锚点片固定"用）
    struct HairPieceInfo
    {
        int   repVertex = -1;   // 该片第一个顶点的几何体索引（日志里当 id 用）
        int   count = 0;
        int   hairCount = 0;
        int   clothCount = 0;
        int   pinLinks = 0;
        int   hardCount = 0;
        float maxSpan = 0.0f;
    };

    std::vector<HairPieceInfo> pieces;

    // [HairTest] 一次性日志标记
    bool     hairTestLogged = false;

    // 头发惯性/transport 的诊断计数（每帧累计，定期打印 [HairFrame]）
    int      hfPops = 0;          // 被判定为"姿势突变/传送"的粒子数
    int      hfClamped = 0;       // 伪加速度被限幅的粒子数
    float    hfMaxMove = 0.0f;    // 本帧最大锚点位移（米）
    float    hfMaxAcc = 0.0f;     // 本帧最大伪加速度（m/s²）
    float    hfMaxSpeed = 0.0f;   // 本帧最大粒子速度（m/s）
    float    hfRotDeg = 0.0f;     // 本帧头部骨骼旋转角（度/帧）
    float    hfMaxImpulse = 0.0f; // 本帧最大伪速度（m/s，钳制前）
    int      hfImpulseClamped = 0;// 伪速度被钳到 2 m/s 的粒子数

    // 世界碰撞位置修正诊断（每帧累计）
    int      diagWorldCorrClamped = 0;   // 修正量被 5cm 上限截断的次数
    float    diagMaxWorldCorr = 0.0f;    // 本帧最大修正量（米）
    int      diagHitKind = 0;            // 最坏修正来自：1=地面 2=三角面 3=球 4=盒
    int      diagHitModel = -1;          // 该障碍物的 modelId

    // 布料网格的平均静止边长（Init 时算）：给"物件自己的网格碰撞体"算自适应 skin
    float    clothMeanEdge = 0.0f;

    // 1 = 这块布料的**风向取反**（只对人物布料使用，ini: pedclothwindflip）。
    // 人物的布料和头发共用同一段风力代码，但实测人物布料看起来是反的，
    // 而车辆/地图布料同向 —— 所以给"蒙皮布料"单独留一个方向开关。
    bool     clothWindFlip = false;

    // 诊断：布料 / 头发"实际位移方向"的累计（每帧 pos-prev 累加）
    // 用来分辨"风向量相同但看起来相反"这种观感问题：谁真的往哪边动。
    RwV3d    diagClothDisp{};
    RwV3d    diagHairDisp{};
    int      diagClothDispN = 0;
    int      diagHairDispN = 0;

    // 诊断：相对锚点的平均偏移（真正的"往哪边飘"）+ 每帧锚点位移（身体运动）
    RwV3d    diagClothOff{};
    RwV3d    diagHairOff{};
    RwV3d    diagClothAnchorMove{};
    int      diagClothOffN = 0;
    int      diagHairOffN = 0;

    // === 布料碰撞反应（2026-11-23 06:5x）===
    // 03:04 把"瞬移到接触点"改成"几何修正 + prev 同步"之后，布料碰撞反应被削到
    // 几乎看不见（推出 ≤5cm/帧，还被摩擦削 45%）。这里补一个**有上限**的
    // 接触冲量，只给 Cloth，头发保持纯几何修正（避免上坡炸开复发）。
    int      diagClothContacts = 0;      // 本帧布料接触次数
    float    diagClothKick = 0.0f;       // 本帧布料碰撞冲量峰值（m/s）
    int      clothDiagFrame = 0;         // 每 rig 帧计数（诊断打印用）

    // 模型 id（诊断用，EntityPhysics 创建时写入）
    int      modelId = -1;

    // （可选，A/B）把 offset 跟着头部旋转一起搬：ini 的 hairrotrans
    bool     hairRotTransport = true;
    // A/B：伪惯性力总开关（hairinertia=0 时只做 transport、不注入冲量）
    bool     hairInertiaEnabled = true;
    // A/B：头发是否参与跨实体/地面碰撞（hairworldcollide=0 时跳过头发）
    bool     hairWorldCollideEnabled = true;

    // ------------------------------------------------------------
    // 头发"跟随身体"的比例（ini: hairride，默认 0.6）
    //
    // 布料用 ride = anchorMove × clothInertia(0.6)：每帧只跟随身体 60% 的
    // 位移，剩下 40% 交给 Verlet 自然滞后 ⇒ 跑动时自然向后拖。
    // 头发原来是 100% 全额跟随（anchorMove 直接加到 pos/prev），
    // 匀速时完全不落后，只有加速度项在推，所以跑动时看起来"向前拉伸"。
    // 这里给头发同样的比例控制：1.0 = 旧行为，0.6 = 与布料一致。
    // ------------------------------------------------------------
    float    hairRide = 0.6f;

    // 头发的每帧阻尼（ini: hairdamping，默认 0.98 = 与布料一致）
    // 旧值是 rig.damping(0.92)：阻尼更大 ⇒ 摆动更快衰减、看起来更"僵"。
    float    hairDamping = 0.98f;

    // 头发锚点回拉倍率（ini: hairpull，默认 1.0 = 原行为）
    // <1 更"自由"（更像布料），>1 更贴身。
    float    hairPull = 1.0f;

    // 头发受风倍率（ini: hairwind，默认 1.0）
    // 布料是 clothWind(1.8)：细长的布/发片当头发时，加大这个才会"飘"起来。
    float    hairWind = 1.0f;

    // 有挂点(pin)的头发向静止姿势回拉的倍率（ini: hairfollow，默认 1.0）
    // 布料用 clothFollow(0)：挂了点就完全不回拉，只靠挂点+约束+重力。
    // 这里设 0 可让头发也变成"自由垂坠"（适合刷成头发的布片）。
    float    hairFollow = 1.0f;
    int      hairBoneIndex = -1;         // 头发统一绑定的骨骼索引
    RwMatrix hairBoneMatPrev{};          // 上一帧该骨骼的世界矩阵
    bool     hairBoneMatValid = false;

    // "无锚点片改为跟随骨骼"是否已经做过（每个 rig 一次）
    bool     hairPieceFixDone = false;

    // 尖刺诊断：每 120 帧算一次"最坏粒子的尖刺比"，供 [Step] 打印
    float    diagSpikeRatio = 0.0f;

    // 拉伸修正诊断（每帧累计，每 600 帧打印一次）
    int      diagBodyEdgeFixes = 0;   // 靠"Body 端点不动"修回来的边次数
    int      diagPinEdgeFixes  = 0;   // 靠"发根半固定"修回来的边次数
    float    diagMaxExcess     = 0.0f;// 本帧修掉的最大超出量（米）

    // 诊断用的每粒子浮点缓冲（邻居距离累加）
    std::vector<float> diagScratch;
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
//  布料手感（全局默认值）
//  在 ini 加载时调用一次；之后建立的 rig 都会继承这套参数。
//    gravityScale ：布料重力倍率，1 = 与头发相同
//    followStrength：有挂点的布料向静止姿势回拉的强度，0 = 完全自由垂坠
//    dampingPer60 ：每帧阻尼（60fps 基准），越接近 1 越飘
//    pinStiffness ：竖向挂点布料的挂点修正比例（1 = 硬缝死，0.15 = 自然垂落）
//    windScale    ：布料受风倍率（1 = 与头发相同，1.8 = 旗帜更吃风）
//    inertia      ：布料惯性继承（0 = 旧行为，0.6 = 跑动时跟着身体走）
//    collisionMargin ：布料与身体胶囊的安全距离（米，只对 Cloth 生效）
// ============================================================
void SetClothTuning(float gravityScale, float followStrength,
                    float dampingPer60, float pinStiffness, float windScale,
                    float inertia, float collisionMargin);

// ============================================================
//  人体胶囊（自身碰撞）调参
//     radiusScale  ：所有胶囊半径的倍率（1 = 原始设计值）
//     radiusMargin ：所有胶囊半径的额外加成（米，0 = 贴着设计半径）
//  两者合成后的半径在 [Colliders] 日志里逐根打印，方便对着日志调。
// ============================================================
void SetBodyColliderTuning(float radiusScale, float radiusMargin);

// 逐顶点颜色诊断开关（ini: colordiag，默认 0）
// 打开后会为每个物理顶点写一行 [ColorScan] Vertex —— 只用于排查，
// 正常游玩务必关闭，否则一个 rig 就可能刷出几千行日志把游戏拖卡。
void GeometryPhysics_SetColorDiag(bool on);

// 每粒子每帧的扫掠碰撞子步上限（ini: collsteps，默认 2，范围 1~4）。
// 只影响"单帧位移很大"时的防穿墙采样精度（车辆高速布料最相关），
// 不影响重力/风/约束/碰撞响应本身。
void GeometryPhysics_SetCollisionSubsteps(int steps);

// Body（R=0xC0 身体抖动）独立手感旋钮（ini: bodyk / bodyc / bodydrive / bodyamp）
//   k       : 弹簧刚度（越大抖得越快）。0 = 由顶点色 G 推导（原行为）
//   c       : 阻尼（越小回弹次数越多、越"弹"）。0 = 由 G 推导
//   drive   : 骨骼加速度驱动倍率（越大动作带动的抖动越明显），默认 1.30
//   amp     : 幅度倍率（在 B 通道幅度上再乘），默认 1.0
// 参考：k=180,c=3.0 → ω≈13.4rad/s(2.1Hz)、ζ≈0.11（又快又弹）
void GeometryPhysics_SetBodyTuning(float k, float c, float drive, float amp);

// 头发跟随比例 / 阻尼 / 回拉（ini: hairride / hairdamping / hairpull）
//   ride    : 每帧跟随身体位移的比例（1.0=旧行为；0.6=与布料一致，跑动时自然后拖）
//   damping : 每帧阻尼（60fps 基准，0.98≈布料；旧值 0.92 更"僵"）
//   pull    : 锚点回拉倍率（<1 更自由，>1 更贴身；默认 1.0）
void GeometryPhysics_SetHairRide(float ride, float damping, float pull);

// 头发受风倍率 / 有挂点回拉倍率（ini: hairwind / hairfollow）
//   wind   : 0~5，默认 1.0（布料 clothWind=1.8；细长发片想更飘就调大）
//   follow : 0~1，默认 1.0；设 0 = 挂了点也完全不回拉（与布料 clothFollow 一致）
void GeometryPhysics_SetHairWindFollow(float wind, float follow);

// Body 分组方式（决定"左胸/右胸/腹能不能各往各的方向抖"）
//   mode   : 0=piece（每个连通片一组，推荐） 1=bone（旧：按主导骨骼） 2=cluster（片内 k-means）
//   split  : cluster 模式的簇数（左右胸+腹 = 3）
//   random : 每组驱动方向随机旋转强度 0~1（0=完全同步，0.25≈10°）
//   smooth : 邻域平滑次数（原来固定 12，越小越"各自为政"，建议 2~3）
void GeometryPhysics_SetBodyGrouping(int mode, int split, float random, int smooth);

// 布料胶囊屏蔽（ini: clothcapsulemask / clothcapsulemaskdepth）
// 布料贴在臀部/大腿上时，静止姿态就已经在胶囊内部：每帧被推出、又被锚点拉回
// ⇒ 表现为"臀部位置猛烈抖动"。开启后这些"建模时已重叠"的胶囊不再推该顶点，
// 其余情况照常碰撞。
//   enabled : 1=开启（默认），0=关闭
//   depth   : 判定阈值（米，默认 0.008）；越大越保守
void GeometryPhysics_SetClothCapsuleMask(int enabled, float depth);

// 按部位跳过人体胶囊（ini: capskip，位掩码）
//   1=骨盆球 2=髋连接/髋球 4=大腿 8=膝 16=小腿 32=躯干 64=颈头 128=手臂 256=脚
// 例：capskip=3 跳过所有骨盆/髋部胶囊（A/B 用）
void GeometryPhysics_SetCapsuleSkipMask(int mask);

// 胶囊半径"按部位"倍率（ini: capsgroupscale<part>）
//   group 用与 capskip 相同的位：1骨盆 2髋 4大腿 8膝 16小腿 32躯干 64颈头 128手臂 256脚
// 最终半径 = 基础半径 × bodycolliderscale × 部位倍率 × 索引倍率 + bodycollidermargin
void GeometryPhysics_SetCapsuleGroupScale(int group, float scale);

// 胶囊半径"按索引"倍率（ini: capsulescale=1.0,1.2,...）
// 索引顺序 = [Colliders] 日志里 radii= 的顺序（0-based）
void GeometryPhysics_SetCapsuleIndexScale(const std::vector<float>& scales);

// 人物布料的风向翻转（ini: pedclothwindflip，默认 1 = 取反）
// 只对**蒙皮布料**（人物）生效；车辆/地图的刚体布料不受影响。
void SetPedClothWindFlip(int enabled);
// ============================================================
//  头发 A/B 诊断开关（ini 的 hairpins / haircapsulemask /
//  hairrootexempt / hairdiag）。默认 1,1,1,1 = 正常行为。
//  目的是让"关闭某一项"的对照实验**不需要重新编译**。
// ============================================================
void SetHairDebugOptions(int pinsEnabled, int capsuleMaskEnabled,
                         int rootStretchExempt, int pieceFixEnabled,
                         int rotTransport, int inertiaEnabled,
                         int worldCollide, int diagnostics);


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
