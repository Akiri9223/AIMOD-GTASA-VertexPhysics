#include "WorldCollision.h"

#include "common.h"
#include "RenderWare.h"
#include "CPools.h"
#include "CEntity.h"
#include "CPed.h"
#include "CVehicle.h"
#include "CObject.h"
#include "CColModel.h"
#include "CCollisionData.h"
#include "CColSphere.h"
#include "CColBox.h"
#include "CWorld.h"
#include "CBuilding.h"
#include "CSector.h"
#include "CPtrNodeSingleLink.h"
#include "CVector.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <map>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

namespace
{
    // ------------------------------------------------------------
    // 实体局部 -> 世界 的基（右/上/前 + 位置）
    //
    // 2026-11-23 07:2x：为了"建筑布料碰自己所属建筑的碰撞体"，
    // 障碍物需要能带朝向（建筑的 col model 在建筑局部空间里，
    // 而 IPL 里的物件经常是旋转过的）。旧的 AddBox 只会加 pos，
    // 那等于把局部盒子当成世界轴对齐盒子 —— 朝向一变就落在错位置。
    // ------------------------------------------------------------
    struct Basis
    {
        float rx = 1.0f, ry = 0.0f, rz = 0.0f;
        float ux = 0.0f, uy = 1.0f, uz = 0.0f;
        float ax = 0.0f, ay = 0.0f, az = 1.0f;
        float px = 0.0f, py = 0.0f, pz = 0.0f;

        void LocalToWorld(float lx, float ly, float lz,
                          float& wx, float& wy, float& wz) const
        {
            wx = px + rx * lx + ux * ly + ax * lz;
            wy = py + ry * lx + uy * ly + ay * lz;
            wz = pz + rz * lx + uz * ly + az * lz;
        }

        void WorldToLocal(float wx, float wy, float wz,
                          float& lx, float& ly, float& lz) const
        {
            const float dx = wx - px;
            const float dy = wy - py;
            const float dz = wz - pz;

            lx = rx * dx + ry * dy + rz * dz;
            ly = ux * dx + uy * dy + uz * dz;
            lz = ax * dx + ay * dy + az * dz;
        }
    };

    Basis GetEntityBasis(CEntity* entity)
    {
        Basis b;

        if (!entity) return b;

        const CVector& pos = entity->GetPosition();

        b.px = pos.x; b.py = pos.y; b.pz = pos.z;

        if (entity->m_matrix)
        {
            const CMatrix& m = *entity->m_matrix;

            b.rx = m.right.x; b.ry = m.right.y; b.rz = m.right.z;
            b.ux = m.up.x;    b.uy = m.up.y;    b.uz = m.up.z;
            b.ax = m.at.x;    b.ay = m.at.y;    b.az = m.at.z;
        }
        else
        {
            // 只有 heading（绕 Z）：GTA 约定 heading=0 朝 +Y
            const float h = entity->m_placement.m_fHeading;
            const float c = std::cos(h);
            const float s = std::sin(h);

            b.rx = c;   b.ry = s;   b.rz = 0.0f;
            b.ux = 0.0f; b.uy = 0.0f; b.uz = 1.0f;
            b.ax = -s;  b.ay = c;   b.az = 0.0f;
        }

        auto normalize = [](float& x, float& y, float& z) -> bool
        {
            const float len2 = x * x + y * y + z * z;
            if (len2 < 1e-8f) return false;
            const float inv = 1.0f / std::sqrt(len2);
            x *= inv; y *= inv; z *= inv;
            return true;
        };

        if (!normalize(b.rx, b.ry, b.rz) ||
            !normalize(b.ux, b.uy, b.uz) ||
            !normalize(b.ax, b.ay, b.az))
        {
            b = Basis{};
            b.px = pos.x; b.py = pos.y; b.pz = pos.z;
        }

        return b;
    }

    struct Obstacle
    {
        enum class Kind { Box, Sphere };

        Kind  kind = Kind::Box;

        // Box：**实体局部空间**的范围 + 实体基（basis 把局部换到世界）
        float minX = 0.0f, minY = 0.0f, minZ = 0.0f;
        float maxX = 0.0f, maxY = 0.0f, maxZ = 0.0f;
        Basis basis;

        // Sphere
        float cx = 0.0f, cy = 0.0f, cz = 0.0f, radius = 0.0f;

        // 该障碍物本帧的速度（实体位置帧间差）
        float vx = 0.0f, vy = 0.0f, vz = 0.0f;

        // 1 = 只修正"刚穿进去"的浅穿透，深陷不管。
        // 用于"布料所属建筑自己的碰撞体"：布往往挂在物件上/穿过它，
        // 深的粒子如果也被硬推到表面，整片布会被掀飞。
        int shallowOnly = 0;

        int modelId = -1;   // 诊断：来自哪个模型

        // 碰撞体"膨胀量"（米，2026-11-23 13:2x，ini: carmargin）
        //   只对**载具**设置：SA 车的 col 盒常常比可见车身小/偏，布料会穿进
        //   车身外板。把盒/球向外撑 grow 米，能补上这段差距（成本为零）。
        float grow = 0.0f;
    };

    std::vector<Obstacle> g_obstacles;

    // ---- 逐三角面碰撞 ----
    struct WorldTriangle
    {
        float ax, ay, az;
        float bx, by, bz;
        float cx, cy, cz;

        float minX, minY, minZ;
        float maxX, maxY, maxZ;

        // 单位法线（2026-11-23 08:2x 加：扫掠检测和"按 prev 侧推出"都要用）
        float nx, ny, nz;

        float vx, vy, vz;
        int modelId = -1;

        // 额外的 skin（米，2026-11-23 13:3x）：按"哪个碰撞体"给三角面加余量
        //   carmargin / mapselfmargin 会写到这里；0 = 不加
        float skin = 0.0f;

        // 1 = 来自"布料所属物件自己的碰撞体/网格"：自适应 skin + 扫掠检测
        int ownCol = 0;
    };

    std::vector<WorldTriangle> g_triangles;

    // ------------------------------------------------------------
    // 三角面空间网格（2026-11-23 07:5x）
    //
    // 原来 CollidePoint 是**线性扫所有三角形**：地图布料 990 粒子 × 4 子步
    // × 900 面 ≈ 360 万次/帧 ⇒ [MapInst] step=23ms、帧时间掉到 15fps，
    // 布一帧飞一米，什么碰撞都追不上。
    //
    // 这里按 0.4m 均匀网格给三角形建索引，CollidePoint 只测"该点所在格"里的面。
    // 大面（跨格太多）放进 wide 列表，并且**追加到每一格**，保证不漏。
    // ------------------------------------------------------------
    constexpr float kTriCellSize = 0.40f;
    constexpr float kTriCellPad  = 0.06f;    // >= 最大 margin
    constexpr int   kTriMaxWide  = 8;        // wide 面超过这个数就退回线性扫描

    std::unordered_map<uint32_t, std::vector<int>> g_triCells;
    std::vector<int> g_triWide;
    std::vector<int> g_triAllIdx;
    bool             g_triGridValid = false;

    inline uint32_t CellKey(int ix, int iy, int iz)
    {
        return static_cast<uint32_t>(ix) * 73856093u ^
               static_cast<uint32_t>(iy) * 19349663u ^
               static_cast<uint32_t>(iz) * 83492791u;
    }

    // 建索引（收集完三角形之后、Step 之前调用一次）
    void BuildTriangleGrid()
    {
        g_triCells.clear();
        g_triWide.clear();
        g_triAllIdx.clear();
        g_triGridValid = false;

        const int n = static_cast<int>(g_triangles.size());

        if (n == 0) return;

        g_triAllIdx.resize(static_cast<size_t>(n));
        for (int i = 0; i < n; ++i) g_triAllIdx[static_cast<size_t>(i)] = i;

        for (int i = 0; i < n; ++i)
        {
            const WorldTriangle& t = g_triangles[static_cast<size_t>(i)];

            const int x0 = static_cast<int>(std::floor((t.minX - kTriCellPad) / kTriCellSize));
            const int x1 = static_cast<int>(std::floor((t.maxX + kTriCellPad) / kTriCellSize));
            const int y0 = static_cast<int>(std::floor((t.minY - kTriCellPad) / kTriCellSize));
            const int y1 = static_cast<int>(std::floor((t.maxY + kTriCellPad) / kTriCellSize));
            const int z0 = static_cast<int>(std::floor((t.minZ - kTriCellPad) / kTriCellSize));
            const int z1 = static_cast<int>(std::floor((t.maxZ + kTriCellPad) / kTriCellSize));

            const long long nx = static_cast<long long>(x1) - x0 + 1;
            const long long ny = static_cast<long long>(y1) - y0 + 1;
            const long long nz = static_cast<long long>(z1) - z0 + 1;

            if (nx <= 0 || ny <= 0 || nz <= 0) continue;

            if (nx * ny * nz > 27)
            {
                g_triWide.push_back(i);

                if (static_cast<int>(g_triWide.size()) > kTriMaxWide)
                    return;          // 大面太多：不建网格，退回线性扫描
                continue;
            }

            for (int ix = x0; ix <= x1; ++ix)
                for (int iy = y0; iy <= y1; ++iy)
                    for (int iz = z0; iz <= z1; ++iz)
                        g_triCells[CellKey(ix, iy, iz)].push_back(i);
        }

        // 大面追加到每一格（数量 <= kTriMaxWide，代价可接受）
        if (!g_triWide.empty())
        {
            for (auto& kv : g_triCells)
                kv.second.insert(kv.second.end(), g_triWide.begin(), g_triWide.end());
        }

        g_triGridValid = true;
    }

    bool  g_haveGround = false;
    int   g_groundLiftRejected = 0;   // 地面抬升量过大被拒绝的次数（诊断）

    // 诊断：最近一次命中的来源
    int   g_hitKind = 0;
    int   g_hitModel = -1;
    float g_hitDepth = 0.0f;

    // 被模拟的实体自己（必须排除，否则它自己的 col model 会把头发顶开）
    CEntity* g_ownerEntity = nullptr;
bool     g_includeOwner = false;
int      g_ownerObstacleCount = 0;   // 本次收集里来自 owner 自己的障碍数（诊断）

// ---- 实体胶囊（人物骨胶囊）注册表 ----
struct CapsuleObstacle
{
    float ax, ay, az;
    float bx, by, bz;
    float radius;
    float vx, vy, vz;
    int   modelId;
};

std::map<CEntity*, std::vector<CapsuleObstacle>> g_entityCapsules;
std::vector<CapsuleObstacle> g_capsules;      // 本次 Collect 的（已按距离筛）
std::map<CEntity*, int> g_capsuleStamp;       // 存活戳：多久没更新就丢
int g_collectCounter = 0;
constexpr int kCapsuleStaleCollects = 90;
    float g_groundZ = 0.0f;
    bool  g_lastHit = false;
    RwV3d g_lastImpulse = { 0.0f, 0.0f, 0.0f };

    // 逐粒子地面采样缓存（0.25 m 网格）：披风很长时要采很多点，
    // 同一格内复用同一次查询
    std::map<int, float> g_groundCache;

    float GroundZAt(float x, float y, float z)
    {
        const int qx = static_cast<int>(std::floor(x * 4.0f));
        const int qy = static_cast<int>(std::floor(y * 4.0f));
        const int key = (qx << 16) ^ (qy & 0xFFFF);

        auto it = g_groundCache.find(key);
        if (it != g_groundCache.end()) return it->second;

        bool ok = false;
        CEntity* hit = nullptr;

        const float gz = CWorld::FindGroundZFor3DCoord(x, y, z, &ok, &hit);
        const float result = ok ? gz : g_groundZ;

        g_groundCache[key] = result;
        return result;
    }

    int             g_maxObstacles = 400;   // 障碍球/盒总数上限（ini: maxobstacles）

    // 三角面总预算。2026-11-23 07:5x：
    // "自己物件的 col model" 经常上千个面（test.col 有 1152），
    // 原来的 900 上限会**把尾巴裁掉**——正好漏掉布料经过的那一段，
    // 表现就是"布直接穿过去、连停顿都没有"。
    // 所以给这个预算做成变量：普通障碍仍用 900，"自己物件"临时放宽到 2600。
    constexpr int   kMaxTrianglesDefault = 900;
    constexpr int   kMaxTrianglesWide    = 2600;
    int             g_trianglesCap       = kMaxTrianglesDefault;

    constexpr float kIgnoreRadius = 0.15f;

    // ------------------------------------------------------------
    // "自己物件的顶面托盘"（2026-11-23 08:1x，ini: mapselfbox）
    //
    // 现象：竖直的布板穿过水平空心圆环（洞）——洞里没有三角形，
    // 网格碰撞只能在管壁上挡左右两条窄带，视觉上就是"什么都没挡"。
    //
    // 这里给物件补一个**实心顶面**：布粒子只要落在物件的 XY 范围内、
    // 且低于顶面（在 band 以内），就抬到顶面之上。
    // 只向上推（不往下、不往侧面），所以不会把挂在上面的布掀飞、也不抖。
    // ------------------------------------------------------------
    struct OwnTopPlate
    {
        bool  active = false;
        float minX = 0.0f, minY = 0.0f, maxX = 0.0f, maxY = 0.0f;
        float topZ = 0.0f;
        float band = 0.8f;      // 低于顶面多少以内算"压在物件上"
    };

    OwnTopPlate g_ownTopPlate;

    void SetOwnTopPlate(float minX, float minY, float maxX, float maxY,
                        float topZ, float band)
    {
        g_ownTopPlate.active = true;
        g_ownTopPlate.minX = minX;
        g_ownTopPlate.minY = minY;
        g_ownTopPlate.maxX = maxX;
        g_ownTopPlate.maxY = maxY;
        g_ownTopPlate.topZ = topZ;
        g_ownTopPlate.band = (band > 0.05f) ? band : 0.05f;
    }
    // ------------------------------------------------------------
    // 各类碰撞体的 margin（2026-11-23 13:3x，都是 ini 可调）
    //   groundmargin : 地面竖直查询的抬升余量（原硬编码 2cm）
    //   objmargin    : 普通物件（CObject）的盒/球膨胀量
    //   mapselfmargin : 地图物件"自己"的 col 盒/球/三角面余量
    //   carmargin    : 载具盒/球膨胀量 + 载具渲染网格三角面余量
    // ------------------------------------------------------------
    float g_groundMargin = 0.02f;
    float g_objectMargin = 0.02f;
    float g_mapObjMargin = 0.02f;

    // ---------------- 静态建筑（CBuilding）碰撞 ----------------
    // 2026-11-23 13:5x，ini: buildingcol / buildingmargin
    //   CBuilding 不在任何池里，只能按 50m 扇区的建筑链表枚举。
    //   只取布料周围 radius 内的建筑，按包围球裁剪，每栋有三角形预算。
    bool  g_buildingCol = true;
    float g_buildingMargin = 0.02f;
    int   g_buildingTriBudget = 1600;   // 一次 Collect 内所有建筑合计
    int   g_buildingTrisPer   = 500;    // 单栋上限（ini: buildingtrisper，一堵墙通常 100~300 面）

    // 最近一次建筑收集的计数（诊断用，由 EntityPhysics 打印）
    WorldBuildingStats g_bldStats{};

    constexpr float kGroundMargin = 0.02f;   // 兼容旧代码引用（会被 g_groundMargin 覆盖使用）

    // "只修正浅穿透"的深度上限（米）：再深就不认为是"刚穿进去"，
    // 而是布料本来就长在物件里（挂点/旗杆附近），不能硬推。
    constexpr float kShallowDepth = 0.15f;

    // 建筑 col 球/盒是否只修"浅穿透"（ini: buildingboxshallow，默认 1）
    // 楼梯/走廊的 col 常是一整块大盒子，布料深陷进去时"往最近的面推出"
    // 会把人甩到上/侧面。开启后深于 kShallowDepth 的穿透直接忽略。
    int  g_buildingBoxShallow = 1;

    // "自己物件的网格碰撞体"专用的 skin（米）。
    // 2026-11-23 08:2x：不再固定 4cm，而是按布料网格分辨率自适应
    // （EntityPhysics 每帧用 clamp(0.45×平均边长, 4cm, 12cm) 设置）。
    // 原因：skin 比布料网格间距薄时，布会从三角形缝隙里漏过去。
    float g_ownColSkin = 0.04f;

    // 载具"渲染网格"碰撞开关（2026-11-23 12:5x，ini: carmesh）
    // SA 的载具 col model 只有盒+球，没有三角面 ⇒ 想精细只能读渲染网格。
    bool g_vehicleMesh = true;

    // 载具碰撞体的"膨胀量"（2026-11-23 13:2x，ini: carmargin）
    // SA 车的 col 盒常常比可见车身小/偏 ⇒ 布料会穿进车身外板。
    // 把载具的盒/球向外撑 grow 米（成本为零），能补上这段差距。
    float g_carMargin = 0.02f;

    // 扫掠（穿越）检测总开关（ini: clothsweep）
    bool  g_ownColSweep = true;

    // 局部空间盒子 + 实体基
    void AddBoxLocal(const Basis& basis, float lminx, float lminy, float lminz,
                     float lmaxx, float lmaxy, float lmaxz,
                     float vx, float vy, float vz, int shallowOnly, int modelId)
    {
        if (static_cast<int>(g_obstacles.size()) >= g_maxObstacles) return;

        if (lmaxx - lminx <= 0.001f ||
            lmaxy - lminy <= 0.001f ||
            lmaxz - lminz <= 0.001f) return;

        Obstacle o;
        o.kind = Obstacle::Kind::Box;

        o.minX = lminx; o.minY = lminy; o.minZ = lminz;
        o.maxX = lmaxx; o.maxY = lmaxy; o.maxZ = lmaxz;
        o.basis = basis;

        o.vx = vx; o.vy = vy; o.vz = vz;
        o.shallowOnly = shallowOnly;
        o.modelId = modelId;

        g_obstacles.push_back(o);
    }

    // 老接口：世界轴对齐盒子（basis 只有平移，局部=世界-pos）
    void AddBox(const CVector& pos, const CBox& box, float vx, float vy, float vz)
    {
        Basis b;
        b.px = pos.x; b.py = pos.y; b.pz = pos.z;

        AddBoxLocal(b,
                    std::min(box.m_vecMin.x, box.m_vecMax.x),
                    std::min(box.m_vecMin.y, box.m_vecMax.y),
                    std::min(box.m_vecMin.z, box.m_vecMax.z),
                    std::max(box.m_vecMin.x, box.m_vecMax.x),
                    std::max(box.m_vecMin.y, box.m_vecMax.y),
                    std::max(box.m_vecMin.z, box.m_vecMax.z),
                    vx, vy, vz, 0, -1);
    }

    // 世界坐标球心
    void AddSphereWorld(float cx, float cy, float cz, float radius,
                        float vx, float vy, float vz, int shallowOnly, int modelId)
    {
        if (static_cast<int>(g_obstacles.size()) >= g_maxObstacles) return;
        if (radius <= 0.001f) return;

        Obstacle o;
        o.kind = Obstacle::Kind::Sphere;

        o.cx = cx; o.cy = cy; o.cz = cz;
        o.radius = radius;

        o.vx = vx; o.vy = vy; o.vz = vz;
        o.shallowOnly = shallowOnly;
        o.modelId = modelId;

        g_obstacles.push_back(o);
    }

    void AddSphere(const CVector& pos, const CSphere& sphere, float vx, float vy, float vz)
    {
        AddSphereWorld(sphere.m_vecCenter.x + pos.x,
                       sphere.m_vecCenter.y + pos.y,
                       sphere.m_vecCenter.z + pos.z,
                       sphere.m_fRadius, vx, vy, vz, 0, -1);
    }

    // ============================================================
    // 逐三角形：读取 m_pColData 的压缩顶点 + 三角形索引
    //
    // 压缩顶点是 int16，缩放系数不同版本可能不同。这里用
    // "包围盒自动标定"：拿顶点原始范围去和 CColModel 的包围盒对比，
    // 反推缩放系数，避免猜错比例导致整块碰撞体错位。
    // ============================================================
    void AddTriangles(CEntity* entity, CColModel* col, const CVector& pos,
                      const RwV3d& center, float radius,
                      float velX, float velY, float velZ,
                      const Basis* basis = nullptr, int ownCol = 0,
                      const float* forcedScale = nullptr,
                      float extraSkin = 0.0f)
    {
        const int modelIdOfEntity = entity
            ? static_cast<int>(entity->m_nModelIndex) : -1;

        CCollisionData* data = col->m_pColData;
        if (!data) return;
        if (!data->m_pVertices || !data->m_pTriangles) return;
        if (data->m_nNumTriangles == 0) return;

        const int triCount = static_cast<int>(data->m_nNumTriangles);

        float rawMin[3] = { 1e9f, 1e9f, 1e9f };
        float rawMax[3] = { -1e9f, -1e9f, -1e9f };

        for (int t = 0; t < triCount; ++t)
        {
            const CColTriangle& tri = data->m_pTriangles[t];
            const unsigned short idx[3] = { tri.m_nVertA, tri.m_nVertB, tri.m_nVertC };

            for (int k = 0; k < 3; ++k)
            {
                const CompressedVector& v = data->m_pVertices[idx[k]];

                const float c[3] = { static_cast<float>(v.x),
                                     static_cast<float>(v.y),
                                     static_cast<float>(v.z) };

                for (int a = 0; a < 3; ++a)
                {
                    if (c[a] < rawMin[a]) rawMin[a] = c[a];
                    if (c[a] > rawMax[a]) rawMax[a] = c[a];
                }
            }
        }

        // 用包围盒范围反推缩放
        float scaleSum = 0.0f;
        int   scaleCount = 0;

        const float boxExtent[3] =
        {
            std::fabs(col->m_boundBox.m_vecMax.x - col->m_boundBox.m_vecMin.x),
            std::fabs(col->m_boundBox.m_vecMax.y - col->m_boundBox.m_vecMin.y),
            std::fabs(col->m_boundBox.m_vecMax.z - col->m_boundBox.m_vecMin.z)
        };

        for (int a = 0; a < 3; ++a)
        {
            const float rawExtent = rawMax[a] - rawMin[a];

            if (rawExtent > 1.0f && boxExtent[a] > 1e-4f)
            {
                scaleSum += boxExtent[a] / rawExtent;
                ++scaleCount;
            }
        }

        float scale = (scaleCount > 0) ? (scaleSum / scaleCount) : (1.0f / 128.0f);
        scale = std::clamp(scale, 1.0f / 512.0f, 1.0f / 8.0f);

        // 强制缩放（自己物件的 col）：包围盒不可信时自动标定会算出离谱的比例，
        // 顶点被甩到几十米外 ⇒ 全部被裁剪掉 ⇒ "一个三角面都没加进来"。
        // COL 的压缩顶点是 int16，标准比例就是 1/128。
        if (forcedScale && *forcedScale > 0.0f)
            scale = *forcedScale;

        // 基（可选）：给了就按实体朝向把顶点换到世界（建筑/IPL 物件常带旋转）。
        // 没给则维持旧行为：pos.x/y/z 只做平移。
        //
        // 注意：有 basis 时**不能**先把平移加进 base，否则 LocalToWorld 会再加一次
        // （顶点跑到两倍坐标上 —— 表现就是"加了碰撞体但完全没效果"）。
        const float offX = basis ? 0.0f : pos.x;
        const float offY = basis ? 0.0f : pos.y;
        const float offZ = basis ? 0.0f : pos.z;

        const float baseX = col->m_boundBox.m_vecMin.x - rawMin[0] * scale + offX;
        const float baseY = col->m_boundBox.m_vecMin.y - rawMin[1] * scale + offY;
        const float baseZ = col->m_boundBox.m_vecMin.z - rawMin[2] * scale + offZ;

        const float cullRadius = radius + 0.6f;

        for (int t = 0; t < triCount; ++t)
        {
            if (static_cast<int>(g_triangles.size()) >= g_trianglesCap) break;

            const CColTriangle& tri = data->m_pTriangles[t];
            const unsigned short idx[3] = { tri.m_nVertA, tri.m_nVertB, tri.m_nVertC };

            float vx[3], vy[3], vz[3];
            bool  valid = true;

            for (int k = 0; k < 3; ++k)
            {
                const CompressedVector& v = data->m_pVertices[idx[k]];

                const float lx = baseX + static_cast<float>(v.x) * scale;
                const float ly = baseY + static_cast<float>(v.y) * scale;
                const float lz = baseZ + static_cast<float>(v.z) * scale;

                if (basis)
                {
                    basis->LocalToWorld(lx, ly, lz, vx[k], vy[k], vz[k]);
                }
                else
                {
                    vx[k] = lx; vy[k] = ly; vz[k] = lz;
                }

                const float dx = vx[k] - center.x;
                const float dy = vy[k] - center.y;
                const float dz = vz[k] - center.z;

                if (dx * dx + dy * dy + dz * dz > cullRadius * cullRadius) valid = false;
            }

            if (!valid) continue;

            WorldTriangle wt;
            wt.ax = vx[0]; wt.ay = vy[0]; wt.az = vz[0];
            wt.bx = vx[1]; wt.by = vy[1]; wt.bz = vz[1];
            wt.cx = vx[2]; wt.cy = vy[2]; wt.cz = vz[2];

            wt.minX = std::min({ vx[0], vx[1], vx[2] });
            wt.minY = std::min({ vy[0], vy[1], vy[2] });
            wt.minZ = std::min({ vz[0], vz[1], vz[2] });

            wt.maxX = std::max({ vx[0], vx[1], vx[2] });
            wt.maxY = std::max({ vy[0], vy[1], vy[2] });
            wt.maxZ = std::max({ vz[0], vz[1], vz[2] });

            // 单位法线（扫掠检测 / prev 侧推出要用；退化面给零法线，通道会自然跳过）
            {
                const float e1x = wt.bx - wt.ax, e1y = wt.by - wt.ay, e1z = wt.bz - wt.az;
                const float e2x = wt.cx - wt.ax, e2y = wt.cy - wt.ay, e2z = wt.cz - wt.az;

                float nx = e1y * e2z - e1z * e2y;
                float ny = e1z * e2x - e1x * e2z;
                float nz = e1x * e2y - e1y * e2x;

                const float nlen = std::sqrt(nx * nx + ny * ny + nz * nz);

                if (nlen > 1e-9f)
                {
                    nx /= nlen; ny /= nlen; nz /= nlen;
                }
                else
                {
                    nx = ny = nz = 0.0f;
                }

                wt.nx = nx; wt.ny = ny; wt.nz = nz;
            }

            wt.vx = velX; wt.vy = velY; wt.vz = velZ;
            wt.modelId = modelIdOfEntity;
            wt.ownCol = ownCol;
            wt.skin   = extraSkin;

            g_triangles.push_back(wt);
        }
    }

    void AddEntity(CEntity* entity, const RwV3d& center, float radius)
    {
        if (!entity || static_cast<int>(g_obstacles.size()) >= g_maxObstacles) return;

        // 按实体指针判断"自己"：
        //   人物(ped) 必须排除 —— 身体由骨胶囊代表，否则会自己顶自己；
        //   载具可以**包含**（g_includeOwner）—— 车体就是 col model，
        //   不包含的话车上的布会直接穿进车身（用户要求"载具布料自碰撞"）。
        const bool isOwner = (entity == g_ownerEntity);

        // 载具判定要在 col 分支外拿到（下面给"渲染网格"用）
        const bool vehicleLike = (entity->m_nType == ENTITY_TYPE_VEHICLE);

        // 这辆车会不会额外加"渲染网格"（见本函数末尾那段）。
        // 只有加了网格，粗盒子才应该退成"浅穿透"：否则盒子会把布顶在包围盒
        // 表面（比车身外扩 10~20cm），网格永远碰不到 ⇒ 看起来还是穿模。
        const bool hasRenderMesh =
            (!isOwner && vehicleLike && g_vehicleMesh && entity->m_pRwObject != nullptr);

        if (isOwner)
        {
            if (!g_includeOwner) return;
            ++g_ownerObstacleCount;
        }

        CColModel* col = entity->GetColModel();
        if (!col) return;

        const CVector& pos = entity->GetPosition();

        // 该实体本帧位移 -> 速度（接触时把速度传给布料，让它被"扫开"）
        static std::map<CEntity*, CVector> s_prevPos;

        float velX = 0.0f, velY = 0.0f, velZ = 0.0f;

        auto prevIt = s_prevPos.find(entity);

        if (prevIt != s_prevPos.end())
        {
            velX = pos.x - prevIt->second.x;
            velY = pos.y - prevIt->second.y;
            velZ = pos.z - prevIt->second.z;
        }

        if (s_prevPos.size() > 4096) s_prevPos.clear();
        s_prevPos[entity] = pos;

        const float dx = pos.x - center.x;
        const float dy = pos.y - center.y;
        const float dz = pos.z - center.z;

        const float distSq = dx * dx + dy * dy + dz * dz;

        // 0.15m 距离启发式：只在"不知道 owner 是谁"时兜底。
        // 一旦 owner 已知（地图实例现在传建筑指针），就完全信任指针判断 ——
        // 否则"走到旗子跟前"的人物/车辆会恰好落进以布料质心为圆心、
        // 半径 15cm 的圈里被丢掉：越贴近越没有碰撞反馈。
        if (g_ownerEntity == nullptr &&
            distSq < kIgnoreRadius * kIgnoreRadius) return;   // 自己

        const float boundRadius = col->m_boundSphere.m_fRadius + radius;
        if (distSq > boundRadius * boundRadius) return;

        // 优先用游戏自带的精细碰撞体：球 + 盒（**按实体朝向**，2026-11-23 09:2x）
        //
        // 旧代码只加 pos，等于把"实体局部盒子/球心"当成世界坐标：车一转向，
        // 碰撞体就落在完全错误的位置（该挡的没挡、不该挡的挡），
        // 这也是"车布穿车身/莫名浮空"的主要来源之一。
        const Basis basis = GetEntityBasis(entity);

        CCollisionData* data = col->m_pColData;
        bool usedDetail = false;

        // 人物 col 球/盒**保留**（2026-11-23 14:2x 按用户要求改回）：
        // 之前试过"人物只用骨胶囊"，结果角色和布料的碰撞反馈变弱了。
        // 现在两者都生效：粗 col 给"整块推开"，骨胶囊给"贴身人形"。
        const size_t obStart = g_obstacles.size();

        if (data)
        {
            const int modelId = static_cast<int>(entity->m_nModelIndex);

            // ------------------------------------------------------------
            // "薄壳型"碰撞：自己（owner）和**所有载具**都按同一套精细规则处理
            // （2026-11-23 12:4x）
            //
            // 以前只有 owner 走精细管线，别的车（人物布/地图布看到的车）：
            //   盒/球全额推出 ⇒ 粗盒子把布顶到盒面，精细三角面永远没机会生效；
            //   三角面 skin=1cm 且无扫掠 ⇒ 薄车壳直接穿过去。
            //
            // 现在：载具（不管是不是自己）——
            //   有三角面时：盒/球只修**浅穿透(≤15cm)**，让三角面负责精确贴合；
            //   没有三角面时：盒/球保持全额推出（否则布会陷进车里出不来）。
            //   三角面一律标 ownCol：自适应 skin + 扫掠检测 + prev 侧推出。
            // ------------------------------------------------------------
            const bool vehicleLike = (entity->m_nType == ENTITY_TYPE_VEHICLE);
            const bool shellLike   = (isOwner || vehicleLike);
            const bool hasTris     = (data->m_nNumTriangles > 0);

            const int shallow    = (shellLike && (hasTris || hasRenderMesh)) ? 1 : 0;
            const int ownColFlag = shellLike ? 1 : 0;

            for (int i = 0; i < data->m_nNumSpheres; ++i)
            {
                const CSphere& s = data->m_pSpheres[i];

                float wx = 0.0f, wy = 0.0f, wz = 0.0f;
                basis.LocalToWorld(s.m_vecCenter.x, s.m_vecCenter.y, s.m_vecCenter.z,
                                   wx, wy, wz);

                AddSphereWorld(wx, wy, wz, s.m_fRadius, velX, velY, velZ, shallow, modelId);
                usedDetail = true;
            }

            for (int i = 0; i < data->m_nNumBoxes; ++i)
            {
                const CBox& b = data->m_pBoxes[i];

                AddBoxLocal(basis,
                            std::min(b.m_vecMin.x, b.m_vecMax.x),
                            std::min(b.m_vecMin.y, b.m_vecMax.y),
                            std::min(b.m_vecMin.z, b.m_vecMax.z),
                            std::max(b.m_vecMin.x, b.m_vecMax.x),
                            std::max(b.m_vecMin.y, b.m_vecMax.y),
                            std::max(b.m_vecMin.z, b.m_vecMax.z),
                            velX, velY, velZ, shallow, modelId);
                usedDetail = true;
            }

            if (hasTris)
            {
                AddTriangles(entity, col, pos, center, radius, velX, velY, velZ,
                             &basis, ownColFlag);
                usedDetail = true;
            }
        }

        // 没有精细数据（或没有球/盒）时退回整体包围盒
        if (!usedDetail)
        {
            Obstacle o;
            o.kind = Obstacle::Kind::Box;

            o.minX = std::min(col->m_boundBox.m_vecMin.x, col->m_boundBox.m_vecMax.x) + pos.x;
            o.minY = std::min(col->m_boundBox.m_vecMin.y, col->m_boundBox.m_vecMax.y) + pos.y;
            o.minZ = std::min(col->m_boundBox.m_vecMin.z, col->m_boundBox.m_vecMax.z) + pos.z;

            o.maxX = std::max(col->m_boundBox.m_vecMin.x, col->m_boundBox.m_vecMax.x) + pos.x;
            o.maxY = std::max(col->m_boundBox.m_vecMin.y, col->m_boundBox.m_vecMax.y) + pos.y;
            o.maxZ = std::max(col->m_boundBox.m_vecMin.z, col->m_boundBox.m_vecMax.z) + pos.z;

            if (o.maxX - o.minX > 0.001f &&
                o.maxY - o.minY > 0.001f &&
                o.maxZ - o.minZ > 0.001f)
            {
                g_obstacles.push_back(o);
            }
        }

        // 载具 / 普通物件的盒/球统一"膨胀"（ini: carmargin / objmargin）：
        //   col 盒经常比可见模型小或偏移，布料会穿进外板/外壳。
        float growAmount = 0.0f;

        if (vehicleLike)
            growAmount = g_carMargin;
        else if (entity->m_nType == ENTITY_TYPE_OBJECT)
            growAmount = g_objectMargin;

        if (growAmount > 0.0f && g_obstacles.size() > obStart)
        {
            for (size_t k = obStart; k < g_obstacles.size(); ++k)
                g_obstacles[k].grow = growAmount;
        }

        // ------------------------------------------------------------
        // 载具的**渲染网格**（2026-11-23 12:5x）
        //
        // SA 的载具 col model 只有盒 + 球，`m_nNumTriangles` 基本是 0，
        // 所以"人物布 / 地图布撞别的车"永远只碰到那个粗盒子 ⇒ 不精细。
        // 这里对**非 owner 的载具**额外把渲染网格的三角形加进来，标 ownCol
        // ⇒ 自适应 skin + 扫掠检测 + prev 侧推出（和车布自己同一条管线）。
        // 排除 owner 自己：车布不能撞自己那面布的网格。
        // ------------------------------------------------------------
        if (hasRenderMesh)
        {
            // 每台车只放**有限个**三角形进来（2026-11-23 12:5x）
            //   车的渲染网格 1000~3000 面，直接全加会让"地图物件布料"的
            //   三角形总数涨到 3000+（地图路径还是每个实例各建一次空间网格）
            //   ⇒ 990 粒子的 test.dff 特别卡。这里给每台车一个独立预算，
            //   用完就停，并且**不动**全局 g_trianglesCap。
            constexpr int kVehicleMeshTris = 600;

            const int before  = static_cast<int>(g_triangles.size());
            const int capSave = g_trianglesCap;

            g_trianglesCap = before + kVehicleMeshTris;

            // 裁剪半径收紧到布料半径本身（离布更远的面没必要进来）
            WorldCollision_AddEntityOwnMesh(entity, center, radius,
                                            nullptr, 0, nullptr, g_carMargin);

            g_trianglesCap = capSave;
        }
    }

    // ============================================================
    //  静态建筑（CBuilding）碰撞：按扇区枚举（2026-11-23 13:5x）
    //
    //  CBuilding 不在任何池里（Ped/Vehicle/Object 都看不到），
    //  只能遍历 CWorld 的 50m 扇区建筑链表。
    //  只收布料 radius 范围内的建筑，按包围球裁剪；
    //  三角形标 ownCol（自适应 skin + 扫掠），并带"每栋/总计"预算，避免卡顿。
    // ============================================================
    void CollectBuildings(const RwV3d& center, float radius, CEntity* exclude)
    {
        if (!g_buildingCol || !CWorld::ms_aSectors) return;

        // 诊断计数（每 240 次收集打一条）
        static int s_diagCalls = 0;
        int dSectors = 0, dVisited = 0, dNoCol = 0, dCulled = 0, dAccepted = 0;
        int dObs = 0, dTris = 0;
        int dSlabs = 0;   // 加了多少个"薄墙实心板"

        // 最近一栋"收下"和最近一栋"被裁掉"的模型 + 距离（诊断：墙到底在不在里面）
        float dNearAccD2 = -1.0f;
        int   dNearAccModel = -1, dNearAccObs = 0, dNearAccTris = 0;
        float dNearCullD2 = -1.0f;
        int   dNearCullModel = -1;

        // 2026-11-23 14:0x：向四周各多扫 1 个扇区。
        // 原因：CBuilding 注册在**它自己原点**所在的扇区，而长墙/大房子的原点
        // 常在几十米外的地块角上 —— 只扫布料附近 1~2 个扇区会漏掉那面墙。
        const int ix0 = std::clamp(CWorld::GetSectorIndexX(center.x - radius) - 1, 0, NUMSECTORS_X - 1);
        const int ix1 = std::clamp(CWorld::GetSectorIndexX(center.x + radius) + 1, 0, NUMSECTORS_X - 1);
        const int iy0 = std::clamp(CWorld::GetSectorIndexY(center.y - radius) - 1, 0, NUMSECTORS_Y - 1);
        const int iy1 = std::clamp(CWorld::GetSectorIndexY(center.y + radius) + 1, 0, NUMSECTORS_Y - 1);

        int triBudget = g_buildingTriBudget;
        bool stopAll = false;

        for (int iy = iy0; iy <= iy1 && !stopAll; ++iy)
        {
            for (int ix = ix0; ix <= ix1 && !stopAll; ++ix)
            {
                CSector* sec = CWorld::GetSector(ix, iy);
                if (!sec) continue;

                ++dSectors;

                for (CPtrNodeSingleLink* n = sec->m_buildingList.GetNode(); n; n = n->pNext)
                {
                    ++dVisited;

                    CBuilding* b = reinterpret_cast<CBuilding*>(n->pItem);
                    if (!b || b == exclude) continue;

                    CColModel* col = b->GetColModel();

                    if (!col)
                    {
                        ++dNoCol;
                        continue;
                    }

                    const CVector& bp = b->GetPosition();

                    // ------------------------------------------------------------
                    // 裁剪：用 col 的**包围盒**（变换到世界 AABB），算点到盒的距离。
                    // 2026-11-23 14:1x：原来用 col->m_boundSphere，但 COL 的 boundSphere
                    // 经常偏移/偏小（长墙尤其明显），会把贴着布料的建筑误裁掉 ——
                    // 日志里"最近收下的建筑在 8.8m 外"就是这个原因。
                    // 包围盒在盒内距离为 0，不会误裁。
                    // ------------------------------------------------------------
                    const Basis cullBasis = GetEntityBasis(b);

                    const float bx0 = std::min(col->m_boundBox.m_vecMin.x, col->m_boundBox.m_vecMax.x);
                    const float by0 = std::min(col->m_boundBox.m_vecMin.y, col->m_boundBox.m_vecMax.y);
                    const float bz0 = std::min(col->m_boundBox.m_vecMin.z, col->m_boundBox.m_vecMax.z);
                    const float bx1 = std::max(col->m_boundBox.m_vecMin.x, col->m_boundBox.m_vecMax.x);
                    const float by1 = std::max(col->m_boundBox.m_vecMin.y, col->m_boundBox.m_vecMax.y);
                    const float bz1 = std::max(col->m_boundBox.m_vecMin.z, col->m_boundBox.m_vecMax.z);

                    float wmin[3] = { 1e9f, 1e9f, 1e9f };
                    float wmax[3] = { -1e9f, -1e9f, -1e9f };
                    bool  boxValid = (bx1 - bx0 > 0.001f || by1 - by0 > 0.001f || bz1 - bz0 > 0.001f);

                    if (boxValid)
                    {
                        for (int ci = 0; ci < 8; ++ci)
                        {
                            const float lx = (ci & 1) ? bx1 : bx0;
                            const float ly = (ci & 2) ? by1 : by0;
                            const float lz = (ci & 4) ? bz1 : bz0;

                            float wx = 0.0f, wy = 0.0f, wz = 0.0f;
                            cullBasis.LocalToWorld(lx, ly, lz, wx, wy, wz);

                            const float w[3] = { wx, wy, wz };

                            for (int k = 0; k < 3; ++k)
                            {
                                if (w[k] < wmin[k]) wmin[k] = w[k];
                                if (w[k] > wmax[k]) wmax[k] = w[k];
                            }
                        }
                    }
                    else
                    {
                        // 包围盒无效：退回"原点 + 保守半径"
                        wmin[0] = wmax[0] = bp.x;
                        wmin[1] = wmax[1] = bp.y;
                        wmin[2] = wmax[2] = bp.z;
                        wmax[0] += 8.0f; wmax[1] += 8.0f; wmax[2] += 8.0f;
                    }

                    float boxDx = 0.0f, boxDy = 0.0f, boxDz = 0.0f;

                    if (center.x < wmin[0]) boxDx = wmin[0] - center.x;
                    else if (center.x > wmax[0]) boxDx = center.x - wmax[0];

                    if (center.y < wmin[1]) boxDy = wmin[1] - center.y;
                    else if (center.y > wmax[1]) boxDy = center.y - wmax[1];

                    if (center.z < wmin[2]) boxDz = wmin[2] - center.z;
                    else if (center.z > wmax[2]) boxDz = center.z - wmax[2];

                    const float dx = boxDx;
                    const float dy = boxDy;
                    const float dz = boxDz;

                    const float reach = radius + 1.0f;

                    if (dx * dx + dy * dy + dz * dz > reach * reach)
                    {
                        ++dCulled;

                        const float d2 = dx * dx + dy * dy + dz * dz;
                        if (dNearCullD2 < 0.0f || d2 < dNearCullD2)
                        {
                            dNearCullD2 = d2;
                            dNearCullModel = static_cast<int>(b->m_nModelIndex);
                        }

                        continue;
                    }

                    CCollisionData* data = col->m_pColData;

                    if (!data)
                    {
                        ++dNoCol;
                        continue;
                    }

                    ++dAccepted;

                    int trisThis = 0;

                    const Basis basis = GetEntityBasis(b);
                    const int modelId = static_cast<int>(b->m_nModelIndex);

                    const size_t obStart = g_obstacles.size();

                    for (int i = 0; i < data->m_nNumSpheres; ++i)
                    {
                        if (static_cast<int>(g_obstacles.size()) >= g_maxObstacles) break;

                        const CSphere& s = data->m_pSpheres[i];
                        if (s.m_fRadius <= 0.01f) continue;

                        float wx = 0.0f, wy = 0.0f, wz = 0.0f;
                        basis.LocalToWorld(s.m_vecCenter.x, s.m_vecCenter.y, s.m_vecCenter.z,
                                           wx, wy, wz);

                        AddSphereWorld(wx, wy, wz, s.m_fRadius, 0.0f, 0.0f, 0.0f, 0, modelId);
                    }

                    for (int i = 0; i < data->m_nNumBoxes; ++i)
                    {
                        if (static_cast<int>(g_obstacles.size()) >= g_maxObstacles) break;

                        const CBox& b2 = data->m_pBoxes[i];

                        AddBoxLocal(basis,
                                    std::min(b2.m_vecMin.x, b2.m_vecMax.x),
                                    std::min(b2.m_vecMin.y, b2.m_vecMax.y),
                                    std::min(b2.m_vecMin.z, b2.m_vecMax.z),
                                    std::max(b2.m_vecMin.x, b2.m_vecMax.x),
                                    std::max(b2.m_vecMin.y, b2.m_vecMax.y),
                                    std::max(b2.m_vecMin.z, b2.m_vecMax.z),
                                    0.0f, 0.0f, 0.0f, 0, modelId);
                    }

                    if (g_buildingMargin > 0.0f && g_obstacles.size() > obStart)
                    {
                        for (size_t k = obStart; k < g_obstacles.size(); ++k)
                            g_obstacles[k].grow = g_buildingMargin;
                    }

                    // 深穿透保护（ini: buildingboxshallow，默认 1）。
                    // 楼梯/走廊的 col 常常是**一整块大盒子**；布料陷进去十几厘米时，
                    // "往最近的面推出"会选到上/侧面 → 披风被莫名甩向上方或一侧。
                    // 开启后，深于 kShallowDepth 的穿透直接忽略（交给三角面挡）。
                    if (g_buildingBoxShallow && g_obstacles.size() > obStart)
                    {
                        for (size_t k = obStart; k < g_obstacles.size(); ++k)
                            g_obstacles[k].shallowOnly = 1;
                    }

                    dObs += static_cast<int>(g_obstacles.size() - obStart);

                    // ---- 三角面（墙、楼梯、屋檐、栏杆）----
                    //
                    // 2026-11-23 14:1x：给建筑的三角面**放宽裁剪半径**。
                    // 原来传的是布料半径（2.37m），只收"三个顶点都在 2.37m 内"的面，
                    // 一堵墙只能捞到 4 个面（日志 near=17615:0+4@0.00m），根本挡不住。
                    // 墙是连续曲面，需要把附近 ~4.5m 内的面都拿进来才连成一片。
                    const float bldTriRadius = radius + 2.0f;

                    const int trisBefore = static_cast<int>(g_triangles.size());

                    if (data->m_nNumTriangles > 0 && triBudget > 0 &&
                        data->m_pTriangles && data->m_pVertices)
                    {
                        static const float kColScale = 1.0f / 128.0f;

                        const int before  = static_cast<int>(g_triangles.size());
                        const int capSave = g_trianglesCap;

                        const int perBuilding = std::min(g_buildingTrisPer, triBudget);

                        g_trianglesCap = std::min(capSave, before + perBuilding);

                        AddTriangles(b, col, b->GetPosition(), center, bldTriRadius,
                                     0.0f, 0.0f, 0.0f, &basis, 1, &kColScale,
                                     g_buildingMargin);

                        const int added = static_cast<int>(g_triangles.size()) - before;
                        triBudget -= added;
                        dTris += added;
                        trisThis = added;

                        g_trianglesCap = capSave;
                    }

                    // ------------------------------------------------------------
                    // 薄墙补"实心板"（2026-11-23 14:2x，按**三角形实际 AABB** 判断）
                    //
                    // 低模墙常常只有 4~6 个零厚度三角面。布料一旦停在它里侧
                    // （离面超过 skin 4~8cm），就再也检测不到 ⇒ 看起来"和墙穿模"。
                    // 之前用 col->m_boundBox 判断板状，但那个包围盒是压缩单位(1/128)，
                    // 尺寸放大 128 倍 ⇒ 条件永不成立、板子没加上（near=17615:0+5）。
                    // 现在改用刚加进来的三角形的世界 AABB，与缩放无关。
                    // ------------------------------------------------------------
                    if (trisThis > 0)
                    {
                        float tmin[3] = { 1e9f, 1e9f, 1e9f };
                        float tmax[3] = { -1e9f, -1e9f, -1e9f };

                        for (int ti = trisBefore; ti < static_cast<int>(g_triangles.size()); ++ti)
                        {
                            const WorldTriangle& wt = g_triangles[ti];

                            const float xs[3] = { wt.ax, wt.bx, wt.cx };
                            const float ys[3] = { wt.ay, wt.by, wt.cy };
                            const float zs[3] = { wt.az, wt.bz, wt.cz };

                            for (int k = 0; k < 3; ++k)
                            {
                                if (xs[k] < tmin[0]) tmin[0] = xs[k];
                                if (xs[k] > tmax[0]) tmax[0] = xs[k];
                                if (ys[k] < tmin[1]) tmin[1] = ys[k];
                                if (ys[k] > tmax[1]) tmax[1] = ys[k];
                                if (zs[k] < tmin[2]) tmin[2] = zs[k];
                                if (zs[k] > tmax[2]) tmax[2] = zs[k];
                            }
                        }

                        const float dxw = tmax[0] - tmin[0];
                        const float dyw = tmax[1] - tmin[1];
                        const float dzw = tmax[2] - tmin[2];

                        const float thinDim = std::min({ dxw, dyw, dzw });
                        const float longDim = std::max({ dxw, dyw, dzw });

                        // 判定放宽（2026-11-23 14:2x）：
                        //   ① 板状：薄 < 1.2m 且长 > 1.0m
                        //   ② 或者"低模"（这一栋只贡献了 <=16 个面）
                        //      —— 低模墙/栅栏的三角面常常跨两三个轴，
                        //      单看薄维会漏掉，所以对低模直接给一个实心代理盒。
                        const bool slabLike = (thinDim < 1.2f && longDim > 1.0f);
                        const bool lowPoly  = (trisThis <= 16 && longDim > 0.5f);

                        if ((slabLike || lowPoly) &&
                            static_cast<int>(g_obstacles.size()) < g_maxObstacles)
                        {
                            Obstacle o;
                            o.kind = Obstacle::Kind::Box;

                            o.minX = tmin[0]; o.minY = tmin[1]; o.minZ = tmin[2];
                            o.maxX = tmax[0]; o.maxY = tmax[1]; o.maxZ = tmax[2];

                            o.basis = Basis{};   // 单位基：min/max 就是世界 AABB

                            o.grow = g_buildingMargin;
                            o.shallowOnly = 0;
                            o.modelId = static_cast<int>(b->m_nModelIndex);

                            g_obstacles.push_back(o);
                            ++dSlabs;
                        }
                    }

                    // 记录"最近一栋收下的建筑"（模型 + 它贡献了多少盒/球/面）
                    {
                        const float d2 = dx * dx + dy * dy + dz * dz;

                        if (dNearAccD2 < 0.0f || d2 < dNearAccD2)
                        {
                            dNearAccD2 = d2;
                            dNearAccModel = modelId;
                            dNearAccObs = static_cast<int>(g_obstacles.size() - obStart);
                            dNearAccTris = trisThis;
                        }
                    }

                    if (triBudget <= 0)
                    {
                        stopAll = true;
                        break;
                    }
                }
            }
        }

        ++s_diagCalls;

        // 把计数暴露给 EntityPhysics（由它在 [MapInst] collide 里打印，避免这里依赖日志函数）
        g_bldStats.sectors  = dSectors;
        g_bldStats.visited  = dVisited;
        g_bldStats.noCol    = dNoCol;
        g_bldStats.culled   = dCulled;
        g_bldStats.accepted = dAccepted;
        g_bldStats.obs      = dObs;
        g_bldStats.tris     = dTris;
        g_bldStats.slabs    = dSlabs;

        g_bldStats.nearModel = dNearAccModel;
        g_bldStats.nearObs   = dNearAccObs;
        g_bldStats.nearTris  = dNearAccTris;
        g_bldStats.nearDist  = (dNearAccD2 >= 0.0f) ? std::sqrt(dNearAccD2) : -1.0f;

        g_bldStats.cullModel = dNearCullModel;
        g_bldStats.cullDist  = (dNearCullD2 >= 0.0f) ? std::sqrt(dNearCullD2) : -1.0f;
    }

    template <typename PoolType>
    void CollectPool(PoolType* pool, const RwV3d& center, float radius)
    {
        if (!pool) return;

        for (int i = 0; i < pool->m_nSize; ++i)
        {
            auto* entity = pool->GetAt(i);
            if (!entity) continue;

            AddEntity(entity, center, radius);
        }
    }
}

void WorldCollision_Collect(const RwV3d& center, float radius, CEntity* owner,
                            bool includeOwner)
{
    g_ownerEntity = owner;
    g_includeOwner = includeOwner;
    g_ownerObstacleCount = 0;
    g_obstacles.clear();
    g_triangles.clear();
    g_capsules.clear();
    g_groundCache.clear();

    // 顶面托盘是"每个物件自己的"，换 rig 必须清掉
    g_ownTopPlate.active = false;

    // 三角面列表被清空了，旧索引必须作废，否则会拿旧下标去索引新列表（撞错面）。
    // 之后要么 EntityPhysics 调 BuildTriangleGrid，要么退回线性扫描。
    g_triGridValid = false;
    g_triCells.clear();
    g_triWide.clear();
    g_triAllIdx.clear();

    if (radius <= 0.05f) radius = 0.05f;

    // 地面：直接问游戏（含静态地图、建筑、道路）
    {
        bool ok = false;
        CEntity* hit = nullptr;

        const float groundZ = CWorld::FindGroundZFor3DCoord(
            center.x, center.y, center.z + 1.0f, &ok, &hit);

        g_haveGround = ok;
        g_groundZ = groundZ;
    }

    CollectPool(CPools::ms_pPedPool, center, radius);
    CollectPool(CPools::ms_pVehiclePool, center, radius);
    CollectPool(CPools::ms_pObjectPool, center, radius);

    // 静态建筑（CBuilding）：扇区链表枚举（owner 自己会被排除）
    //
    // 2026-11-23 14:4x：**给建筑预留独立三角形额度**。
    // 之前建筑和"载具渲染网格"共用 g_trianglesCap（900），实体路径里
    // 车网格先把预算吃满 ⇒ 人物/载具布料一个建筑面都拿不到
    // （只有地图路径因为收集时机不同才碰巧能拿到）。
    {
        const int capSave = g_trianglesCap;
        const int need    = static_cast<int>(g_triangles.size()) + g_buildingTriBudget;

        g_trianglesCap = std::max(capSave, need);

        CollectBuildings(center, radius, owner);

        g_trianglesCap = capSave;
    }

    // ------------------------------------------------------------
    // 人物骨胶囊（2026-11-23 09:2x）
    // 注册表里每帧由 ped rig 更新；这里按距离筛进本次碰撞集合。
    // owner 自己的胶囊不加（它自己的 Step 里已经用 rig.colliders 碰过）。
    // ------------------------------------------------------------
    for (const auto& kv : g_entityCapsules)
    {
        if (kv.first == g_ownerEntity) continue;

        for (const auto& cap : kv.second)
        {
            const float mx = (cap.ax + cap.bx) * 0.5f - center.x;
            const float my = (cap.ay + cap.by) * 0.5f - center.y;
            const float mz = (cap.az + cap.bz) * 0.5f - center.z;

            const float reach = radius + cap.radius + 0.6f;

            if (mx * mx + my * my + mz * mz > reach * reach) continue;

            if (g_capsules.size() >= 256) break;
            g_capsules.push_back(cap);
        }
    }

    // 丢掉很久没更新的注册（人物销毁/走远后没人来注销）
    ++g_collectCounter;

    for (auto it = g_capsuleStamp.begin(); it != g_capsuleStamp.end(); )
    {
        if (g_collectCounter - it->second > kCapsuleStaleCollects)
        {
            g_entityCapsules.erase(it->first);
            it = g_capsuleStamp.erase(it);
        }
        else
        {
            ++it;
        }
    }
}

void WorldCollision_SetEntityCapsules(CEntity* entity,
                                      const WorldCapsule* capsules, int count)
{
    if (!entity) return;

    if (!capsules || count <= 0)
    {
        g_entityCapsules.erase(entity);
        g_capsuleStamp.erase(entity);
        return;
    }

    if (g_entityCapsules.size() > 256 &&
        g_entityCapsules.find(entity) == g_entityCapsules.end())
    {
        g_entityCapsules.clear();
        g_capsuleStamp.clear();
    }

    auto& list = g_entityCapsules[entity];
    list.clear();

    for (int i = 0; i < count; ++i)
    {
        const WorldCapsule& s = capsules[i];
        if (s.radius <= 0.0f) continue;

        CapsuleObstacle o;
        o.ax = s.ax; o.ay = s.ay; o.az = s.az;
        o.bx = s.bx; o.by = s.by; o.bz = s.bz;
        o.radius = s.radius;
        o.vx = s.vx; o.vy = s.vy; o.vz = s.vz;
        o.modelId = s.modelId;

        list.push_back(o);
    }

    g_capsuleStamp[entity] = g_collectCounter;
}

void WorldCollision_ClearEntityCapsules(CEntity* entity)
{
    if (!entity) return;
    g_entityCapsules.erase(entity);
    g_capsuleStamp.erase(entity);
}

int WorldCollision_CapsuleCount()
{
    return static_cast<int>(g_capsules.size());
}

float WorldCollision_NearestCapsuleDistance(const RwV3d& p)
{
    float best = -1.0f;

    for (const auto& cp : g_capsules)
    {
        const float abx = cp.bx - cp.ax, aby = cp.by - cp.ay, abz = cp.bz - cp.az;
        const float abLenSq = abx * abx + aby * aby + abz * abz;

        const float apx = p.x - cp.ax;
        const float apy = p.y - cp.ay;
        const float apz = p.z - cp.az;

        float t = 0.0f;

        if (abLenSq > 1e-8f)
            t = std::clamp((apx * abx + apy * aby + apz * abz) / abLenSq, 0.0f, 1.0f);

        const float qx = cp.ax + abx * t;
        const float qy = cp.ay + aby * t;
        const float qz = cp.az + abz * t;

        const float dx = p.x - qx;
        const float dy = p.y - qy;
        const float dz = p.z - qz;

        const float d = std::sqrt(dx * dx + dy * dy + dz * dz);

        if (best < 0.0f || d < best) best = d;
    }

    return best;
}

// 前向声明：CollidePoint 里要用（定义在后面）
bool SegmentHitsTriangle(const WorldTriangle& t,
                         float p0x, float p0y, float p0z,
                         float p1x, float p1y, float p1z,
                         float& outT, float& entrySign);

void WorldCollision_CollidePoint(RwV3d& point, const RwV3d& prev, float margin)
{
    g_lastHit = false;
    g_lastImpulse = { 0.0f, 0.0f, 0.0f };
    g_hitKind = 0;
    g_hitModel = -1;
    g_hitDepth = 0.0f;

    // 地面碰撞：只有接近地面时才逐点采样（长披风会覆盖较大面积）
    if (g_haveGround && point.z < g_groundZ + 1.2f)
    {
        const float gz = GroundZAt(point.x, point.y, point.z + 0.6f);

        if (point.z < gz + g_groundMargin)
        {
            // 2026-11-23：坡面/墙角/屋顶上 FindGroundZFor3DCoord 可能返回一个
            // 比粒子高出很多的高度。旧代码直接 point.z = gz 会把粒子瞬移到
            // 那个高度（上坡炸开的直接触发点）。这里把"抬升量"限制在 0.30m，
            // 超过就认为采样不可信，本帧不修正（下一帧换点再试）。
            const float lift = gz + g_groundMargin - point.z;

            if (lift <= 0.30f)
            {
                // 单帧最多抬 5cm：坡面/长披风都能在几帧内收敛，
                // 但不会一帧把粒子瞬移到坡面上（那会被当成撕开）。
                point.z += std::min(lift, 0.05f);
                g_lastHit = true;
                g_hitKind = 1;
                g_hitModel = -1;
                g_hitDepth = lift;
            }
            else
            {
                ++g_groundLiftRejected;
            }
        }
    }

    // ------------------------------------------------------------
    // 顶面托盘：物件对布料来说是"实心的"（mapselfbox）
    // 只向上推：布粒子落在物件 XY 范围内、且低于顶面（band 以内）就抬到顶面之上。
    // 不往下、不往侧面推，所以挂在上面的布不会被掀飞也不会抖。
    // ------------------------------------------------------------
    if (g_ownTopPlate.active &&
        point.x > g_ownTopPlate.minX && point.x < g_ownTopPlate.maxX &&
        point.y > g_ownTopPlate.minY && point.y < g_ownTopPlate.maxY)
    {
        const float target = g_ownTopPlate.topZ + margin;

        if (point.z < target && point.z > g_ownTopPlate.topZ - g_ownTopPlate.band)
        {
            const float lift = std::min(target - point.z, 0.05f);

            point.z += lift;

            g_lastHit = true;
            g_hitKind = 6;
            g_hitModel = -1;
            g_hitDepth = lift;
        }
    }

    // 逐三角面碰撞
    // 候选三角形：网格可用时只扫"该点所在格"里的面（大面已追加到每一格）；
    // 网格没建（或大面过多）时退回全扫。判定逻辑本身完全不变。
    const int*  candIdx = g_triAllIdx.data();
    int         candCount = static_cast<int>(g_triAllIdx.size());

    if (g_triGridValid)
    {
        const int ix = static_cast<int>(std::floor(point.x / kTriCellSize));
        const int iy = static_cast<int>(std::floor(point.y / kTriCellSize));
        const int iz = static_cast<int>(std::floor(point.z / kTriCellSize));

        auto it = g_triCells.find(CellKey(ix, iy, iz));

        if (it != g_triCells.end())
        {
            candIdx = it->second.data();
            candCount = static_cast<int>(it->second.size());
        }
        else
        {
            candIdx = g_triWide.data();
            candCount = static_cast<int>(g_triWide.size());
        }
    }

    for (int candI = 0; candI < candCount; ++candI)
    {
        const int triIndex = candIdx[candI];

        if (triIndex < 0 || triIndex >= static_cast<int>(g_triangles.size())) continue;

        const WorldTriangle& t = g_triangles[static_cast<size_t>(triIndex)];

        // "自己物件"的网格碰撞体用更厚的 skin：通用 margin 只有 1cm，
        // 而地图布料的网格边长常在 3cm 以上，1cm 会让布直接从面上漏过去。
        const float triMargin = ((t.ownCol != 0) ? std::max(margin, g_ownColSkin) : margin)
                              + t.skin;

        // ------------------------------------------------------------
        // 扫掠（穿越）检测：先做，只对"自己物件的网格"
        //
        // 薄壳 + 小 skin 时，"点到面距离"永远抓不到已经跨过去的粒子；
        // 这里检查 prev→point 这一小步有没有穿过三角形，命中就推回入射侧。
        // 只开给 ownCol，避免影响头发/人物那些已经调好的行为。
        // ------------------------------------------------------------
        if (t.ownCol != 0 && g_ownColSweep)
        {
            float tHit = 0.0f, entrySign = 1.0f;

            if (SegmentHitsTriangle(t, prev.x, prev.y, prev.z,
                                    point.x, point.y, point.z, tHit, entrySign))
            {
                const float hx = prev.x + (point.x - prev.x) * tHit;
                const float hy = prev.y + (point.y - prev.y) * tHit;
                const float hz = prev.z + (point.z - prev.z) * tHit;

                point.x = hx + t.nx * entrySign * triMargin;
                point.y = hy + t.ny * entrySign * triMargin;
                point.z = hz + t.nz * entrySign * triMargin;

                g_lastHit = true;
                g_lastImpulse = { t.vx, t.vy, t.vz };
                g_hitKind = 2;
                g_hitModel = t.modelId;
                g_hitDepth = triMargin;

                continue;
            }
        }

        if (point.x < t.minX - triMargin || point.x > t.maxX + triMargin) continue;
        if (point.y < t.minY - triMargin || point.y > t.maxY + triMargin) continue;
        if (point.z < t.minZ - triMargin || point.z > t.maxZ + triMargin) continue;

        // 点到三角形最近点（Ericson 的算法）
        const float ax = t.ax, ay = t.ay, az = t.az;
        const float bx = t.bx, by = t.by, bz = t.bz;
        const float cx = t.cx, cy = t.cy, cz = t.cz;

        const float abx = bx - ax, aby = by - ay, abz = bz - az;
        const float acx = cx - ax, acy = cy - ay, acz = cz - az;
        const float apx = point.x - ax, apy = point.y - ay, apz = point.z - az;

        const float d1 = abx * apx + aby * apy + abz * apz;
        const float d2 = acx * apx + acy * apy + acz * apz;

        float qx, qy, qz;

        if (d1 <= 0.0f && d2 <= 0.0f)
        {
            qx = ax; qy = ay; qz = az;
        }
        else
        {
            const float bpx = point.x - bx, bpy = point.y - by, bpz = point.z - bz;
            const float d3 = abx * bpx + aby * bpy + abz * bpz;
            const float d4 = acx * bpx + acy * bpy + acz * bpz;

            if (d3 >= 0.0f && d4 <= d3)
            {
                qx = bx; qy = by; qz = bz;
            }
            else
            {
                const float vc = d1 * d4 - d3 * d2;

                if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f)
                {
                    const float v = (d1 - d3) != 0.0f ? d1 / (d1 - d3) : 0.0f;
                    qx = ax + abx * v; qy = ay + aby * v; qz = az + abz * v;
                }
                else
                {
                    const float cpx = point.x - cx, cpy = point.y - cy, cpz = point.z - cz;
                    const float d5 = abx * cpx + aby * cpy + abz * cpz;
                    const float d6 = acx * cpx + acy * cpy + acz * cpz;

                    if (d6 >= 0.0f && d5 <= d6)
                    {
                        qx = cx; qy = cy; qz = cz;
                    }
                    else
                    {
                        const float vb = d5 * d2 - d1 * d6;

                        if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f)
                        {
                            const float w = (d2 - d6) != 0.0f ? d2 / (d2 - d6) : 0.0f;
                            qx = ax + acx * w; qy = ay + acy * w; qz = az + acz * w;
                        }
                        else
                        {
                            const float va = d3 * d6 - d5 * d4;

                            if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f)
                            {
                                const float denom = (d4 - d3) + (d5 - d6);
                                const float w = denom != 0.0f ? (d4 - d3) / denom : 0.0f;
                                qx = bx + (cx - bx) * w;
                                qy = by + (cy - by) * w;
                                qz = bz + (cz - bz) * w;
                            }
                            else
                            {
                                const float denom = va + vb + vc;
                                const float v = denom != 0.0f ? vb / denom : 0.0f;
                                const float w = denom != 0.0f ? vc / denom : 0.0f;
                                qx = ax + abx * v + acx * w;
                                qy = ay + aby * v + acy * w;
                                qz = az + abz * v + acz * w;
                            }
                        }
                    }
                }
            }
        }

        const float dx = point.x - qx;
        const float dy = point.y - qy;
        const float dz = point.z - qz;

        const float distSq = dx * dx + dy * dy + dz * dz;

        if (distSq >= triMargin * triMargin || distSq < 1e-12f) continue;

        if (t.ownCol != 0)
        {
            // 自己物件的网格：推出方向取 **prev 所在那一侧**。
            // 用"最近点 → 粒子"的方向时，粒子已经在壳里侧就会被推得更深
            // （这也是"穿过去之后再也出不来"的原因之一）。
            const float dNow  = t.nx * (point.x - t.ax) + t.ny * (point.y - t.ay) + t.nz * (point.z - t.az);
            const float dPrev = t.nx * (prev.x - t.ax) + t.ny * (prev.y - t.ay) + t.nz * (prev.z - t.az);

            const float side = (dPrev >= 0.0f) ? 1.0f : -1.0f;
            const float push = (side * triMargin) - dNow;

            if (std::fabs(push) > 5e-4f)
            {
                point.x += t.nx * push;
                point.y += t.ny * push;
                point.z += t.nz * push;

                g_lastHit = true;
                g_lastImpulse = { t.vx, t.vy, t.vz };
                g_hitKind = 2;
                g_hitModel = t.modelId;
                g_hitDepth = std::fabs(push);
            }

            continue;
        }

        const float dist = std::sqrt(distSq);
        const float inv = triMargin / dist;

        point.x = qx + dx * inv;
        point.y = qy + dy * inv;
        point.z = qz + dz * inv;
        g_lastHit = true;
        g_lastImpulse = { t.vx, t.vy, t.vz };
        g_hitKind = 2;
        g_hitModel = t.modelId;
        g_hitDepth = margin - dist;
    }

    // ------------------------------------------------------------
    // 人物骨胶囊（点 vs 胶囊）
    // ------------------------------------------------------------
    for (const auto& cp : g_capsules)
    {
        const float abx = cp.bx - cp.ax, aby = cp.by - cp.ay, abz = cp.bz - cp.az;
        const float abLenSq = abx * abx + aby * aby + abz * abz;

        const float apx = point.x - cp.ax;
        const float apy = point.y - cp.ay;
        const float apz = point.z - cp.az;

        float t = 0.0f;

        if (abLenSq > 1e-8f)
            t = std::clamp((apx * abx + apy * aby + apz * abz) / abLenSq, 0.0f, 1.0f);

        const float qx = cp.ax + abx * t;
        const float qy = cp.ay + aby * t;
        const float qz = cp.az + abz * t;

        const float dx = point.x - qx;
        const float dy = point.y - qy;
        const float dz = point.z - qz;

        const float r = cp.radius + margin;
        const float distSq = dx * dx + dy * dy + dz * dz;

        if (distSq >= r * r || distSq < 1e-9f) continue;

        const float dist = std::sqrt(distSq);
        const float inv = r / dist;

        point.x = qx + dx * inv;
        point.y = qy + dy * inv;
        point.z = qz + dz * inv;

        g_lastHit = true;
        g_lastImpulse = { cp.vx, cp.vy, cp.vz };
        g_hitKind = 5;
        g_hitModel = cp.modelId;
        g_hitDepth = r - dist;
    }

    for (const auto& o : g_obstacles)
    {
        if (o.kind == Obstacle::Kind::Sphere)
        {
            const float dx = point.x - o.cx;
            const float dy = point.y - o.cy;
            const float dz = point.z - o.cz;

            const float r = o.radius + margin + o.grow;
            const float distSq = dx * dx + dy * dy + dz * dz;

            if (distSq >= r * r || distSq < 1e-9f) continue;

            const float dist = std::sqrt(distSq);

            // "所属建筑自己的碰撞体"：深陷在球里就不管（布可能本来就长在里面）
            if (o.shallowOnly != 0 && dist < o.radius - kShallowDepth) continue;

            const float inv = r / dist;

            point.x = o.cx + dx * inv;
            point.y = o.cy + dy * inv;
            point.z = o.cz + dz * inv;

            g_lastHit = true;
            g_lastImpulse = { o.vx, o.vy, o.vz };
            g_hitKind = 3;
            g_hitModel = o.modelId;
            g_hitDepth = r - dist;
            continue;
        }

        // 盒子：在**实体局部空间**判定（建筑可能带旋转）
        // 2026-11-23 13:2x：盒子的有效边距 = margin + grow（载具的 col 盒
        // 常比可见车身小，grow 用来补那一段差距）
        const float bm = margin + o.grow;

        float lx = 0.0f, ly = 0.0f, lz = 0.0f;
        o.basis.WorldToLocal(point.x, point.y, point.z, lx, ly, lz);

        if (lx < o.minX - bm || lx > o.maxX + bm) continue;
        if (ly < o.minY - bm || ly > o.maxY + bm) continue;
        if (lz < o.minZ - bm || lz > o.maxZ + bm) continue;

        float plx = 0.0f, ply = 0.0f, plz = 0.0f;
        o.basis.WorldToLocal(prev.x, prev.y, prev.z, plx, ply, plz);

        // 用"上一帧位置"判断从哪一侧进入，避免把布料弹到另一侧（穿模的主要来源）
        int axis = -1;
        bool positive = false;
        float pen = 0.0f;

        if (plx > o.maxX + bm)      { axis = 0; positive = true; }
        else if (plx < o.minX - bm) { axis = 0; positive = false; }
        else if (ply > o.maxY + bm) { axis = 1; positive = true; }
        else if (ply < o.minY - bm) { axis = 1; positive = false; }
        else if (plz > o.maxZ + bm) { axis = 2; positive = true; }
        else if (plz < o.minZ - bm) { axis = 2; positive = false; }

        if (axis == 0)      pen = positive ? (o.maxX + bm - lx) : (lx - (o.minX - bm));
        else if (axis == 1) pen = positive ? (o.maxY + bm - ly) : (ly - (o.minY - bm));
        else if (axis == 2) pen = positive ? (o.maxZ + bm - lz) : (lz - (o.minZ - bm));

        if (axis < 0)
        {
            // 上一帧就已经在盒内：退回"最浅穿透"
            const float dMinX = lx - (o.minX - bm);
            const float dMaxX = (o.maxX + bm) - lx;
            const float dMinY = ly - (o.minY - bm);
            const float dMaxY = (o.maxY + bm) - ly;
            const float dMinZ = lz - (o.minZ - bm);
            const float dMaxZ = (o.maxZ + bm) - lz;

            float best = dMinX;
            axis = 0;
            positive = false;

            if (dMaxX < best) { best = dMaxX; axis = 0; positive = true; }
            if (dMinY < best) { best = dMinY; axis = 1; positive = false; }
            if (dMaxY < best) { best = dMaxY; axis = 1; positive = true; }
            if (dMinZ < best) { best = dMinZ; axis = 2; positive = false; }
            if (dMaxZ < best) { best = dMaxZ; axis = 2; positive = true; }

            pen = best;
        }

        // "所属建筑自己的碰撞体"：只修刚穿进去的浅穿透
        if (o.shallowOnly != 0 && pen > kShallowDepth) continue;

        g_lastHit = true;
        g_lastImpulse = { o.vx, o.vy, o.vz };
        g_hitKind = 4;
        g_hitModel = o.modelId;
        g_hitDepth = pen;

        switch (axis)
        {
        case 0: lx = positive ? (o.maxX + bm) : (o.minX - bm); break;
        case 1: ly = positive ? (o.maxY + bm) : (o.minY - bm); break;
        default: lz = positive ? (o.maxZ + bm) : (o.minZ - bm); break;
        }

        float wx = 0.0f, wy = 0.0f, wz = 0.0f;
        o.basis.LocalToWorld(lx, ly, lz, wx, wy, wz);

        point.x = wx;
        point.y = wy;
        point.z = wz;
    }
}

bool WorldCollision_LastHit()
{
    return g_lastHit;
}

void WorldCollision_LastImpulse(RwV3d& out)
{
    out = g_lastImpulse;
}

void WorldCollision_LastHitInfo(WorldCollisionHitInfo& out)
{
    out.kind = g_hitKind;
    out.modelId = g_hitModel;
    out.depth = g_hitDepth;
}

// ============================================================
// 线段 prev→point 与三角形求交（Möller–Trumbore，双面）
//
// 2026-11-23 08:2x：这是"薄壳挡不住快布"的正面解法。
// 只靠"点到面距离 < skin"时，粒子一旦一步跨过零厚度网格就再也不会被检测到；
// 这里检查这一帧的位移线段是否穿过三角形，命中就推回**入射那一侧**。
// entrySign 由 det 的符号推出，与三角形绕序无关（RW 的绕序不可靠）。
// ============================================================
bool SegmentHitsTriangle(const WorldTriangle& t,
                         float p0x, float p0y, float p0z,
                         float p1x, float p1y, float p1z,
                         float& outT, float& entrySign)
{
    const float e1x = t.bx - t.ax, e1y = t.by - t.ay, e1z = t.bz - t.az;
    const float e2x = t.cx - t.ax, e2y = t.cy - t.ay, e2z = t.cz - t.az;

    const float dx = p1x - p0x, dy = p1y - p0y, dz = p1z - p0z;

    const float hx = dy * e2z - dz * e2y;
    const float hy = dz * e2x - dx * e2z;
    const float hz = dx * e2y - dy * e2x;

    const float det = e1x * hx + e1y * hy + e1z * hz;

    if (std::fabs(det) < 1e-12f) return false;

    const float invDet = 1.0f / det;

    const float sx = p0x - t.ax, sy = p0y - t.ay, sz = p0z - t.az;

    const float u = (sx * hx + sy * hy + sz * hz) * invDet;
    if (u < 0.0f || u > 1.0f) return false;

    const float qx = sy * e1z - sz * e1y;
    const float qy = sz * e1x - sx * e1z;
    const float qz = sx * e1y - sy * e1x;

    const float v = (dx * qx + dy * qy + dz * qz) * invDet;
    if (v < 0.0f || u + v > 1.0f) return false;

    const float tHit = (e2x * qx + e2y * qy + e2z * qz) * invDet;
    if (tHit < 0.0f || tHit > 1.0f) return false;

    outT = tHit;
    entrySign = (det > 0.0f) ? 1.0f : -1.0f;
    return true;
}

void WorldCollision_SetMaxObstacles(int count)
{
    g_maxObstacles = std::clamp(count, 100, 2000);
}

void WorldCollision_SetBuildingTriangleBudget(int total, int perBuilding)
{
    g_buildingTriBudget = std::clamp(total, 0, 8000);
    g_buildingTrisPer   = std::clamp(perBuilding, 0, 2000);
}

void WorldCollision_SetBuildingBoxShallow(int enabled)
{
    g_buildingBoxShallow = (enabled != 0);
}

void WorldCollision_SetOwnColSkin(float skin)
{
    g_ownColSkin = std::clamp(skin, 0.01f, 0.15f);
}

void WorldCollision_SetOwnColSweep(int enabled)
{
    g_ownColSweep = (enabled != 0);
}

void WorldCollision_SetVehicleMesh(int enabled)
{
    g_vehicleMesh = (enabled != 0);
}

void WorldCollision_SetCarMargin(float margin)
{
    g_carMargin = std::clamp(margin, 0.0f, 0.15f);
}

void WorldCollision_SetGroundMargin(float margin)
{
    g_groundMargin = std::clamp(margin, 0.0f, 0.15f);
}

void WorldCollision_SetObjectMargin(float margin)
{
    g_objectMargin = std::clamp(margin, 0.0f, 0.15f);
}

void WorldCollision_SetMapObjectMargin(float margin)
{
    g_mapObjMargin = std::clamp(margin, 0.0f, 0.15f);
}

void WorldCollision_SetBuildingCollision(int enabled)
{
    g_buildingCol = (enabled != 0);
}

void WorldCollision_SetBuildingMargin(float margin)
{
    g_buildingMargin = std::clamp(margin, 0.0f, 0.15f);
}

int WorldCollision_BuildingTriangleBudget()
{
    return g_buildingTriBudget;
}

void WorldCollision_GetBuildingStats(WorldBuildingStats& out)
{
    out = g_bldStats;
}

int WorldCollision_ObstacleCount()
{
    return static_cast<int>(g_obstacles.size());
}

int WorldCollision_OwnerObstacleCount()
{
    return g_ownerObstacleCount;
}

void WorldCollision_BuildTriangleGrid()
{
    BuildTriangleGrid();
}

void WorldCollision_SetOwnTopPlate(float minX, float minY, float maxX, float maxY,
                                   float topZ, float band)
{
    SetOwnTopPlate(minX, minY, maxX, maxY, topZ, band);
}

void WorldCollision_ClearOwnTopPlate()
{
    g_ownTopPlate.active = false;
}

void WorldCollision_SetOwnTopPlateFromEntity(CEntity* entity, float band)
{
    if (!entity) return;

    CColModel* col = entity->GetColModel();
    if (!col) return;

    const Basis basis = GetEntityBasis(entity);

    const CBox& bb = col->m_boundBox;

    const float xs[2] = { bb.m_vecMin.x, bb.m_vecMax.x };
    const float ys[2] = { bb.m_vecMin.y, bb.m_vecMax.y };
    const float zs[2] = { bb.m_vecMin.z, bb.m_vecMax.z };

    float lo[3] = { 1e9f, 1e9f, 1e9f };
    float hi[3] = { -1e9f, -1e9f, -1e9f };

    for (int i = 0; i < 8; ++i)
    {
        const float lx = xs[i & 1];
        const float ly = ys[(i >> 1) & 1];
        const float lz = zs[(i >> 2) & 1];

        float wx = 0.0f, wy = 0.0f, wz = 0.0f;
        basis.LocalToWorld(lx, ly, lz, wx, wy, wz);

        const float w[3] = { wx, wy, wz };

        for (int k = 0; k < 3; ++k)
        {
            if (w[k] < lo[k]) lo[k] = w[k];
            if (w[k] > hi[k]) hi[k] = w[k];
        }
    }

    SetOwnTopPlate(lo[0], lo[1], hi[0], hi[1], hi[2], band);
}

// 诊断：自身三角面的包围盒 + 数量
void WorldCollision_OwnColTriBounds(RwV3d& outMin, RwV3d& outMax, int& outCount)
{
    outMin = { 1e9f, 1e9f, 1e9f };
    outMax = { -1e9f, -1e9f, -1e9f };
    outCount = 0;

    for (const auto& t : g_triangles)
    {
        if (t.ownCol == 0) continue;

        if (t.minX < outMin.x) outMin.x = t.minX;
        if (t.minY < outMin.y) outMin.y = t.minY;
        if (t.minZ < outMin.z) outMin.z = t.minZ;

        if (t.maxX > outMax.x) outMax.x = t.maxX;
        if (t.maxY > outMax.y) outMax.y = t.maxY;
        if (t.maxZ > outMax.z) outMax.z = t.maxZ;

        ++outCount;
    }
}

// 诊断：点到自身三角面集合的最近距离（点到三角形面积距离，不做 margin 判断）
float WorldCollision_DistanceToOwnColTris(const RwV3d& p)
{
    float best = -1.0f;

    for (const auto& t : g_triangles)
    {
        if (t.ownCol == 0) continue;

        const float ax = t.ax, ay = t.ay, az = t.az;
        const float bx = t.bx, by = t.by, bz = t.bz;
        const float cx = t.cx, cy = t.cy, cz = t.cz;

        const float abx = bx - ax, aby = by - ay, abz = bz - az;
        const float acx = cx - ax, acy = cy - ay, acz = cz - az;
        const float apx = p.x - ax, apy = p.y - ay, apz = p.z - az;

        const float d1 = abx * apx + aby * apy + abz * apz;
        const float d2 = acx * apx + acy * apy + acz * apz;

        float qx, qy, qz;

        if (d1 <= 0.0f && d2 <= 0.0f) { qx = ax; qy = ay; qz = az; }
        else
        {
            const float bpx = p.x - bx, bpy = p.y - by, bpz = p.z - bz;
            const float d3 = abx * bpx + aby * bpy + abz * bpz;
            const float d4 = acx * bpx + acy * bpy + acz * bpz;

            if (d3 >= 0.0f && d4 <= d3) { qx = bx; qy = by; qz = bz; }
            else
            {
                const float vc = d1 * d4 - d3 * d2;

                if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f)
                {
                    const float v = (d1 - d3) != 0.0f ? d1 / (d1 - d3) : 0.0f;
                    qx = ax + abx * v; qy = ay + aby * v; qz = az + abz * v;
                }
                else
                {
                    const float cpx = p.x - cx, cpy = p.y - cy, cpz = p.z - cz;
                    const float d5 = abx * cpx + aby * cpy + abz * cpz;
                    const float d6 = acx * cpx + acy * cpy + acz * cpz;

                    if (d6 >= 0.0f && d5 <= d6) { qx = cx; qy = cy; qz = cz; }
                    else
                    {
                        const float vb = d5 * d2 - d1 * d6;

                        if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f)
                        {
                            const float w = (d2 - d6) != 0.0f ? d2 / (d2 - d6) : 0.0f;
                            qx = ax + acx * w; qy = ay + acy * w; qz = az + acz * w;
                        }
                        else
                        {
                            const float va = d3 * d6 - d5 * d4;

                            if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f)
                            {
                                const float denom = (d4 - d3) + (d5 - d6);
                                const float w = denom != 0.0f ? (d4 - d3) / denom : 0.0f;
                                qx = bx + (cx - bx) * w;
                                qy = by + (cy - by) * w;
                                qz = bz + (cz - bz) * w;
                            }
                            else
                            {
                                const float denom = va + vb + vc;
                                const float v = denom != 0.0f ? vb / denom : 0.0f;
                                const float w = denom != 0.0f ? vc / denom : 0.0f;
                                qx = ax + abx * v + acx * w;
                                qy = ay + aby * v + acy * w;
                                qz = az + abz * v + acz * w;
                            }
                        }
                    }
                }
            }
        }

        const float dx = p.x - qx;
        const float dy = p.y - qy;
        const float dz = p.z - qz;

        const float d = std::sqrt(dx * dx + dy * dy + dz * dz);

        if (best < 0.0f || d < best) best = d;
    }

    return best;
}

int WorldCollision_TriangleCount()
{
    return static_cast<int>(g_triangles.size());
}

// ============================================================
// 把某个实体**自己**的 col model（球 + 盒）当成障碍加进来
//
// 用途：地图物件布料 vs 它所属的物件本身。
// 现状：WorldCollision_Collect 只扫 Ped / Vehicle / Object 三个池，
//       CBuilding（静态地图物件，例如 .ide objs + .ipl 摆的那个）根本不在里面，
//       所以布料以前"碰不到自己的碰撞体"，只有 FindGroundZFor3DCoord 那条
//       高度场通道能从上往下托一下（圆环/洞/侧面全失效）。
//
// 这里直接把 col->m_pColData 里的球和盒按**实体矩阵**变换成世界障碍：
//   * 球：把球心换到世界（球各向同性）；
//   * 盒：存局部范围 + 实体基，在局部空间判定（建筑常有旋转）。
// 两者都标 shallowOnly：只修"刚穿进去"的浅穿透（<=15cm），
// 深陷的粒子不动 —— 布料常常本来就挂在物件上/穿过它，硬推会把整片布掀飞。
//
// center/cullRadius：按布料质心裁剪，只收附近的形状（建筑 col 可能很大）。
// 返回实际加入的障碍数量（诊断用）。
// ============================================================
int WorldCollision_AddEntityOwnColModel(CEntity* entity,
                                        const RwV3d& center, float cullRadius,
                                        WorldColModelInfo* outInfo,
                                        float marginOverride)
{
    if (!entity) return 0;

    CColModel* col = entity->GetColModel();
    if (!col) return 0;

    CCollisionData* data = col->m_pColData;

    if (outInfo)
    {
        outInfo->spheres = data ? static_cast<int>(data->m_nNumSpheres) : 0;
        outInfo->boxes   = data ? static_cast<int>(data->m_nNumBoxes) : 0;
        outInfo->tris    = data ? static_cast<int>(data->m_nNumTriangles) : 0;
        outInfo->boundRadius = col->m_boundSphere.m_fRadius;
        outInfo->hasData = (data != nullptr) ? 1 : 0;
    }

    if (!data) return 0;

    const Basis basis = GetEntityBasis(entity);
    const int modelId = static_cast<int>(entity->m_nModelIndex);
    const int before = static_cast<int>(g_obstacles.size());

    const float cull = (cullRadius > 0.2f) ? cullRadius : 0.2f;

    // ---- 球 ----
    for (int i = 0; i < data->m_nNumSpheres; ++i)
    {
        const CSphere& s = data->m_pSpheres[i];

        if (s.m_fRadius <= 0.01f) continue;

        float wx = 0.0f, wy = 0.0f, wz = 0.0f;
        basis.LocalToWorld(s.m_vecCenter.x, s.m_vecCenter.y, s.m_vecCenter.z,
                           wx, wy, wz);

        const float dx = wx - center.x;
        const float dy = wy - center.y;
        const float dz = wz - center.z;

        const float reach = cull + s.m_fRadius;

        if (dx * dx + dy * dy + dz * dz > reach * reach) continue;

        AddSphereWorld(wx, wy, wz, s.m_fRadius, 0.0f, 0.0f, 0.0f, 1, modelId);
    }

    // ---- 盒 ----
    for (int i = 0; i < data->m_nNumBoxes; ++i)
    {
        const CBox& b = data->m_pBoxes[i];

        const float lminx = std::min(b.m_vecMin.x, b.m_vecMax.x);
        const float lminy = std::min(b.m_vecMin.y, b.m_vecMax.y);
        const float lminz = std::min(b.m_vecMin.z, b.m_vecMax.z);

        const float lmaxx = std::max(b.m_vecMin.x, b.m_vecMax.x);
        const float lmaxy = std::max(b.m_vecMin.y, b.m_vecMax.y);
        const float lmaxz = std::max(b.m_vecMin.z, b.m_vecMax.z);

        // 盒子中心的距离裁剪（用半对角线当半径）
        const float lcx = (lminx + lmaxx) * 0.5f;
        const float lcy = (lminy + lmaxy) * 0.5f;
        const float lcz = (lminz + lmaxz) * 0.5f;

        float wx = 0.0f, wy = 0.0f, wz = 0.0f;
        basis.LocalToWorld(lcx, lcy, lcz, wx, wy, wz);

        const float halfDiag = 0.5f * std::sqrt(
            (lmaxx - lminx) * (lmaxx - lminx) +
            (lmaxy - lminy) * (lmaxy - lminy) +
            (lmaxz - lminz) * (lmaxz - lminz));

        const float dx = wx - center.x;
        const float dy = wy - center.y;
        const float dz = wz - center.z;

        const float reach = cull + halfDiag;

        if (dx * dx + dy * dy + dz * dz > reach * reach) continue;

        AddBoxLocal(basis, lminx, lminy, lminz, lmaxx, lmaxy, lmaxz,
                    0.0f, 0.0f, 0.0f, 1, modelId);
    }

    // 自身碰撞体统一加余量（默认沿用 mapselfmargin；objselfcol 传自己的 objselfmargin）
    const float marginUse = (marginOverride >= 0.0f) ? marginOverride : g_mapObjMargin;

    if (marginUse > 0.0f && g_obstacles.size() > static_cast<size_t>(before))
    {
        for (size_t k = static_cast<size_t>(before); k < g_obstacles.size(); ++k)
            g_obstacles[k].grow = marginUse;
    }

    // ---- 网格碰撞体（三角面）----
    // test.col 这类"只有网格、没有球/盒"的碰撞体走这里。
    // 用 ownCol=1：判定时吃 kOwnColMargin(4cm) 的 skin，否则 1cm 会漏。
    // forcedScale = 1/128：COL 的压缩顶点是 int16，标准比例；
    // 不强制的话，包围盒不可信时会标定出离谱缩放 ⇒ 顶点被甩远 ⇒ 全被裁剪。
    const int trisBefore = static_cast<int>(g_triangles.size());

    const bool hasTriData = (data->m_nNumTriangles > 0);

    if (outInfo && hasTriData)
    {
        outInfo->hasVerts = (data->m_pVertices != nullptr) ? 1 : 0;
        outInfo->hasTris  = (data->m_pTriangles != nullptr) ? 1 : 0;
    }

    if (hasTriData && data->m_pTriangles && data->m_pVertices)
    {
        const float kColScale = 1.0f / 128.0f;

        // 放宽三角面预算：自己物件的 col 经常上千面，900 会把尾巴裁掉，
        // 表现是"布料从被裁掉的那一段直接穿过去"。
        const int savedCap = g_trianglesCap;

        g_trianglesCap = kMaxTrianglesWide;

        AddTriangles(entity, col, entity->GetPosition(), center, cull,
                     0.0f, 0.0f, 0.0f, &basis, 1, &kColScale, marginUse);

        g_trianglesCap = savedCap;
    }

    const int trisAdded = static_cast<int>(g_triangles.size()) - trisBefore;

    // 返回值把三角面也算进去（诊断里 selfCol 能反映"到底加到了东西没有"）
    return (static_cast<int>(g_obstacles.size()) - before) + trisAdded;
}

// ============================================================
// 回退：用实体**自己的渲染网格**当碰撞面
//
// 有些 .col（COL3 mesh 之类）在插件里读出来是 0 个三角面，但渲染网格一定在。
// 这里直接读 RpAtomic 的 geometry 三角形，按 atomic LTM 换到世界。
// dynamicVertexMap 用来**排除布料自己的三角形**（否则布和自己每帧互相顶）。
// ============================================================
namespace
{
    // 2026-11-24 14:4x：把"从几何体+世界矩阵加自身网格"抽出来，
    // 这样地图实例可以直接传**我们自己的克隆几何体**——地图模型的原始
    // 几何体是 rpGEOMETRYNATIVE（没有 CPU 顶点/三角形），直接读永远 0。
    int AddOwnMeshGeometry(RpGeometry* geometry, const RwMatrix* ltm,
                           const RwV3d& center, float cullRadius,
                           const int* dynMap, int numVerts, int modelId,
                           WorldOwnMeshStats* st, float extraSkin = 0.0f)
    {
        if (!geometry || !ltm) return 0;
        if (geometry->flags & rpGEOMETRYNATIVE) return 0;
        if (!geometry->triangles || geometry->numTriangles == 0) return 0;

        RwV3d* verts = RpMorphTargetGetVertices(&geometry->morphTarget[0]);
        if (!verts) return 0;

        const int geoVerts = static_cast<int>(geometry->numVertices);
        const int geoMode  = (dynMap != nullptr && numVerts == geoVerts) ? 1 : 0;

        if (st)
        {
            st->geoVerts  = geoVerts;
            st->maskVerts = numVerts;
            st->maskMode  = geoMode;
        }

        const int tris = static_cast<int>(geometry->numTriangles);
        const float cullSq = cullRadius * cullRadius;

        int added = 0;

        // ------------------------------------------------------------
        // 距离优先选面（2026-11-23 13:0x）
        //
        // 以前是"按网格顺序取，取满 g_trianglesCap 就停"：车网格 1000~3000 面，
        // 预算 600 时很可能取到车尾/车轮的面，而布贴在车头 —— 浪费预算、精度差。
        // 现在先把通过裁剪的候选面收集起来，按"到布料中心的距离"排序，
        // 只取最近的那些，所以同样 600 面能覆盖布真正接触的部位。
        // 用 nth_element（O(n)）而不是全排序，开销与原来同量级。
        // ------------------------------------------------------------
        struct MeshCand
        {
            float         d2;
            WorldTriangle tri;
        };

        static std::vector<MeshCand> s_cands;
        s_cands.clear();

        for (int t = 0; t < tris; ++t)
        {
            const RpTriangle& tri = geometry->triangles[t];

            const int vi[3] = { static_cast<int>(tri.vertIndex[0]),
                                static_cast<int>(tri.vertIndex[1]),
                                static_cast<int>(tri.vertIndex[2]) };

            if (st) ++st->examined;

            // 排除布料自己的三角形
            if (geoMode)
            {
                bool touchesCloth = false;

                for (int k = 0; k < 3; ++k)
                {
                    if (vi[k] >= 0 && vi[k] < geoVerts && dynMap[vi[k]] >= 0)
                    {
                        touchesCloth = true;
                        break;
                    }
                }

                if (touchesCloth)
                {
                    if (st) ++st->skippedCloth;
                    continue;
                }
            }

            float wx[3], wy[3], wz[3];
            bool  valid = true;

            for (int k = 0; k < 3; ++k)
            {
                if (vi[k] < 0 || vi[k] >= geoVerts) { valid = false; break; }

                const RwV3d& v = verts[vi[k]];

                wx[k] = ltm->right.x * v.x + ltm->up.x * v.y + ltm->at.x * v.z + ltm->pos.x;
                wy[k] = ltm->right.y * v.x + ltm->up.y * v.y + ltm->at.y * v.z + ltm->pos.y;
                wz[k] = ltm->right.z * v.x + ltm->up.z * v.y + ltm->at.z * v.z + ltm->pos.z;

                const float dx = wx[k] - center.x;
                const float dy = wy[k] - center.y;
                const float dz = wz[k] - center.z;

                if (dx * dx + dy * dy + dz * dz > cullSq) valid = false;
            }

            if (!valid)
            {
                if (st) ++st->culled;
                continue;
            }

            WorldTriangle wt;
            wt.ax = wx[0]; wt.ay = wy[0]; wt.az = wz[0];
            wt.bx = wx[1]; wt.by = wy[1]; wt.bz = wz[1];
            wt.cx = wx[2]; wt.cy = wy[2]; wt.cz = wz[2];

            wt.minX = std::min({ wx[0], wx[1], wx[2] });
            wt.minY = std::min({ wy[0], wy[1], wy[2] });
            wt.minZ = std::min({ wz[0], wz[1], wz[2] });

            wt.maxX = std::max({ wx[0], wx[1], wx[2] });
            wt.maxY = std::max({ wy[0], wy[1], wy[2] });
            wt.maxZ = std::max({ wz[0], wz[1], wz[2] });

            // 单位法线（同 col 路径；扫掠检测需要）
            {
                const float e1x = wt.bx - wt.ax, e1y = wt.by - wt.ay, e1z = wt.bz - wt.az;
                const float e2x = wt.cx - wt.ax, e2y = wt.cy - wt.ay, e2z = wt.cz - wt.az;

                float nx = e1y * e2z - e1z * e2y;
                float ny = e1z * e2x - e1x * e2z;
                float nz = e1x * e2y - e1y * e2x;

                const float nlen = std::sqrt(nx * nx + ny * ny + nz * nz);

                if (nlen > 1e-9f)
                {
                    nx /= nlen; ny /= nlen; nz /= nlen;
                }
                else
                {
                    nx = ny = nz = 0.0f;
                }

                wt.nx = nx; wt.ny = ny; wt.nz = nz;
            }

            wt.vx = 0.0f; wt.vy = 0.0f; wt.vz = 0.0f;
            wt.modelId = modelId;
            wt.ownCol = 1;
            wt.skin   = extraSkin;

            // 候选：记录"三角形中心到布料中心的距离平方"，最后按距离取最近的
            const float ccx = (wt.minX + wt.maxX) * 0.5f - center.x;
            const float ccy = (wt.minY + wt.maxY) * 0.5f - center.y;
            const float ccz = (wt.minZ + wt.maxZ) * 0.5f - center.z;

            MeshCand cand;
            cand.d2  = ccx * ccx + ccy * ccy + ccz * ccz;
            cand.tri = wt;

            s_cands.push_back(cand);
        }

        // 预算 = 全局上限 - 已有点数；按距离取最近的
        const int budget = static_cast<int>(g_trianglesCap)
                         - static_cast<int>(g_triangles.size());

        if (budget <= 0) return 0;

        const int total = static_cast<int>(s_cands.size());
        const int take  = (budget < total) ? budget : total;

        if (take < total)
        {
            std::nth_element(s_cands.begin(), s_cands.begin() + take, s_cands.end(),
                [](const MeshCand& a, const MeshCand& b) { return a.d2 < b.d2; });
        }

        for (int i = 0; i < take; ++i)
        {
            g_triangles.push_back(s_cands[i].tri);
            ++added;
        }

        return added;
    }

    int AddOwnMeshAtomic(RpAtomic* atomic, const RwV3d& center, float cullRadius,
                         const int* dynMap, int numVerts, int modelId,
                         WorldOwnMeshStats* st, float extraSkin = 0.0f)
    {
        if (!atomic) return 0;

        RpGeometry* geometry = RpAtomicGetGeometry(atomic);
        if (!geometry) return 0;

        RwFrame* frame = RpAtomicGetFrame(atomic);
        if (!frame) return 0;

        const RwMatrix* ltm = RwFrameGetLTM(frame);
        if (!ltm) return 0;

        return AddOwnMeshGeometry(geometry, ltm, center, cullRadius,
                                  dynMap, numVerts, modelId, st, extraSkin);
    }
}

int WorldCollision_AddEntityOwnMesh(CEntity* entity,
                                    const RwV3d& center, float cullRadius,
                                    const int* dynamicVertexMap, int numVerts,
                                    WorldOwnMeshStats* outStats, float extraSkin)
{
    if (!entity || !entity->m_pRwObject) return 0;

    const int modelId = static_cast<int>(entity->m_nModelIndex);
    int added = 0;

    RwObject* object = entity->m_pRwObject;

    if (RwObjectGetType(object) == rpCLUMP)
    {
        RpClump* clump = reinterpret_cast<RpClump*>(object);

        struct Ctx
        {
            int         added;
            RwV3d       center;
            float       cullRadius;
            const int*  dynMap;
            int         numVerts;
            int         modelId;
            WorldOwnMeshStats* st;
            float       extraSkin;
        };

        Ctx ctx{ 0, center, cullRadius, dynamicVertexMap, numVerts, modelId, outStats, extraSkin };

        RpClumpForAllAtomics(clump,
            [](RpAtomic* atomic, void* data) -> RpAtomic*
            {
                auto* c = static_cast<Ctx*>(data);
                if (atomic)
                    c->added += AddOwnMeshAtomic(atomic, c->center, c->cullRadius,
                                                 c->dynMap, c->numVerts, c->modelId, c->st,
                                                 c->extraSkin);
                return atomic;
            },
            &ctx);

        added = ctx.added;
    }
    else if (RwObjectGetType(object) == rpATOMIC)
    {
        added = AddOwnMeshAtomic(reinterpret_cast<RpAtomic*>(object),
                                 center, cullRadius, dynamicVertexMap, numVerts, modelId,
                                 outStats, extraSkin);
    }

    return added;
}

int WorldCollision_GroundLiftRejected()
{
    return g_groundLiftRejected;
}

void WorldCollision_ResetGroundLiftRejected()
{
    g_groundLiftRejected = 0;
}

void WorldCollision_Clear()
{
    g_obstacles.clear();
    g_triangles.clear();
    g_capsules.clear();
    g_ownTopPlate.active = false;
    g_triGridValid = false;
    g_triCells.clear();
    g_triWide.clear();
    g_triAllIdx.clear();
    g_haveGround = false;
}