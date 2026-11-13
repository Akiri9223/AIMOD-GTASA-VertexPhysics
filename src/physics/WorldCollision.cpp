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
#include "CVector.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <vector>

namespace
{
    struct Obstacle
    {
        enum class Kind { Box, Sphere };

        Kind  kind = Kind::Box;

        // Box
        float minX = 0.0f, minY = 0.0f, minZ = 0.0f;
        float maxX = 0.0f, maxY = 0.0f, maxZ = 0.0f;

        // Sphere
        float cx = 0.0f, cy = 0.0f, cz = 0.0f, radius = 0.0f;

        // 该障碍物本帧的速度（实体位置帧间差）
        float vx = 0.0f, vy = 0.0f, vz = 0.0f;
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

        float vx, vy, vz;
    };

    std::vector<WorldTriangle> g_triangles;

    bool  g_haveGround = false;
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

    constexpr int   kMaxObstacles = 400;
    constexpr int   kMaxTriangles = 900;
    constexpr float kIgnoreRadius = 0.15f;
    constexpr float kGroundMargin = 0.02f;

    void AddBox(const CVector& pos, const CBox& box, float vx, float vy, float vz)
    {
        if (static_cast<int>(g_obstacles.size()) >= kMaxObstacles) return;

        const float sx = std::fabs(box.m_vecMax.x - box.m_vecMin.x);
        const float sy = std::fabs(box.m_vecMax.y - box.m_vecMin.y);
        const float sz = std::fabs(box.m_vecMax.z - box.m_vecMin.z);

        if (sx <= 0.001f || sy <= 0.001f || sz <= 0.001f) return;

        Obstacle o;
        o.kind = Obstacle::Kind::Box;

        o.minX = std::min(box.m_vecMin.x, box.m_vecMax.x) + pos.x;
        o.minY = std::min(box.m_vecMin.y, box.m_vecMax.y) + pos.y;
        o.minZ = std::min(box.m_vecMin.z, box.m_vecMax.z) + pos.z;

        o.maxX = std::max(box.m_vecMin.x, box.m_vecMax.x) + pos.x;
        o.maxY = std::max(box.m_vecMin.y, box.m_vecMax.y) + pos.y;
        o.maxZ = std::max(box.m_vecMin.z, box.m_vecMax.z) + pos.z;

        o.vx = vx; o.vy = vy; o.vz = vz;

        g_obstacles.push_back(o);
    }

    void AddSphere(const CVector& pos, const CSphere& sphere, float vx, float vy, float vz)
    {
        if (static_cast<int>(g_obstacles.size()) >= kMaxObstacles) return;
        if (sphere.m_fRadius <= 0.001f) return;

        Obstacle o;
        o.kind = Obstacle::Kind::Sphere;

        o.cx = sphere.m_vecCenter.x + pos.x;
        o.cy = sphere.m_vecCenter.y + pos.y;
        o.cz = sphere.m_vecCenter.z + pos.z;
        o.radius = sphere.m_fRadius;

        o.vx = vx; o.vy = vy; o.vz = vz;

        g_obstacles.push_back(o);
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
                      float velX, float velY, float velZ)
    {
        (void)entity;

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

        const float baseX = col->m_boundBox.m_vecMin.x - rawMin[0] * scale + pos.x;
        const float baseY = col->m_boundBox.m_vecMin.y - rawMin[1] * scale + pos.y;
        const float baseZ = col->m_boundBox.m_vecMin.z - rawMin[2] * scale + pos.z;

        const float cullRadius = radius + 0.6f;

        for (int t = 0; t < triCount; ++t)
        {
            if (static_cast<int>(g_triangles.size()) >= kMaxTriangles) break;

            const CColTriangle& tri = data->m_pTriangles[t];
            const unsigned short idx[3] = { tri.m_nVertA, tri.m_nVertB, tri.m_nVertC };

            float vx[3], vy[3], vz[3];
            bool  valid = true;

            for (int k = 0; k < 3; ++k)
            {
                const CompressedVector& v = data->m_pVertices[idx[k]];

                vx[k] = baseX + static_cast<float>(v.x) * scale;
                vy[k] = baseY + static_cast<float>(v.y) * scale;
                vz[k] = baseZ + static_cast<float>(v.z) * scale;

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

            wt.vx = velX; wt.vy = velY; wt.vz = velZ;

            g_triangles.push_back(wt);
        }
    }

    void AddEntity(CEntity* entity, const RwV3d& center, float radius)
    {
        if (!entity || static_cast<int>(g_obstacles.size()) >= kMaxObstacles) return;

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

        if (distSq < kIgnoreRadius * kIgnoreRadius) return;   // 自己

        const float boundRadius = col->m_boundSphere.m_fRadius + radius;
        if (distSq > boundRadius * boundRadius) return;

        // 优先用游戏自带的精细碰撞体：球 + 盒
        CCollisionData* data = col->m_pColData;
        bool usedDetail = false;

        if (data)
        {
            for (int i = 0; i < data->m_nNumSpheres; ++i)
            {
                AddSphere(pos, data->m_pSpheres[i], velX, velY, velZ);
                usedDetail = true;
            }

            for (int i = 0; i < data->m_nNumBoxes; ++i)
            {
                AddBox(pos, data->m_pBoxes[i], velX, velY, velZ);
                usedDetail = true;
            }

            if (data->m_nNumTriangles > 0)
            {
                AddTriangles(entity, col, pos, center, radius, velX, velY, velZ);
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

void WorldCollision_Collect(const RwV3d& center, float radius)
{
    g_obstacles.clear();
    g_triangles.clear();
    g_groundCache.clear();

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
}

void WorldCollision_CollidePoint(RwV3d& point, const RwV3d& prev, float margin)
{
    g_lastHit = false;
    g_lastImpulse = { 0.0f, 0.0f, 0.0f };

    // 地面碰撞：只有接近地面时才逐点采样（长披风会覆盖较大面积）
    if (g_haveGround && point.z < g_groundZ + 1.2f)
    {
        const float gz = GroundZAt(point.x, point.y, point.z + 0.6f);

        if (point.z < gz + kGroundMargin)
        {
            point.z = gz + kGroundMargin;
            g_lastHit = true;
        }
    }

    // 逐三角面碰撞
    for (const auto& t : g_triangles)
    {
        if (point.x < t.minX - margin || point.x > t.maxX + margin) continue;
        if (point.y < t.minY - margin || point.y > t.maxY + margin) continue;
        if (point.z < t.minZ - margin || point.z > t.maxZ + margin) continue;

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

        if (distSq >= margin * margin || distSq < 1e-12f) continue;

        const float dist = std::sqrt(distSq);
        const float inv = margin / dist;

        point.x = qx + dx * inv;
        point.y = qy + dy * inv;
        point.z = qz + dz * inv;
        g_lastHit = true;
        g_lastImpulse = { t.vx, t.vy, t.vz };
    }

    for (const auto& o : g_obstacles)
    {
        if (o.kind == Obstacle::Kind::Sphere)
        {
            const float dx = point.x - o.cx;
            const float dy = point.y - o.cy;
            const float dz = point.z - o.cz;

            const float r = o.radius + margin;
            const float distSq = dx * dx + dy * dy + dz * dz;

            if (distSq >= r * r || distSq < 1e-9f) continue;

            const float dist = std::sqrt(distSq);
            const float inv = r / dist;

            point.x = o.cx + dx * inv;
            point.y = o.cy + dy * inv;
            point.z = o.cz + dz * inv;
            g_lastHit = true;
            g_lastImpulse = { o.vx, o.vy, o.vz };
            continue;
        }

        if (point.x < o.minX - margin || point.x > o.maxX + margin) continue;
        if (point.y < o.minY - margin || point.y > o.maxY + margin) continue;
        if (point.z < o.minZ - margin || point.z > o.maxZ + margin) continue;

        // 用"上一帧位置"判断从哪一侧进入，避免把布料弹到另一侧（穿模的主要来源）
        int axis = -1;
        bool positive = false;

        if (prev.x > o.maxX + margin)      { axis = 0; positive = true; }
        else if (prev.x < o.minX - margin) { axis = 0; positive = false; }
        else if (prev.y > o.maxY + margin) { axis = 1; positive = true; }
        else if (prev.y < o.minY - margin) { axis = 1; positive = false; }
        else if (prev.z > o.maxZ + margin) { axis = 2; positive = true; }
        else if (prev.z < o.minZ - margin) { axis = 2; positive = false; }

        if (axis < 0)
        {
            // 上一帧就已经在盒内：退回"最浅穿透"
            const float dMinX = point.x - (o.minX - margin);
            const float dMaxX = (o.maxX + margin) - point.x;
            const float dMinY = point.y - (o.minY - margin);
            const float dMaxY = (o.maxY + margin) - point.y;
            const float dMinZ = point.z - (o.minZ - margin);
            const float dMaxZ = (o.maxZ + margin) - point.z;

            float best = dMinX;
            axis = 0;
            positive = false;

            if (dMaxX < best) { best = dMaxX; axis = 0; positive = true; }
            if (dMinY < best) { best = dMinY; axis = 1; positive = false; }
            if (dMaxY < best) { best = dMaxY; axis = 1; positive = true; }
            if (dMinZ < best) { best = dMinZ; axis = 2; positive = false; }
            if (dMaxZ < best) { best = dMaxZ; axis = 2; positive = true; }
        }

        g_lastHit = true;
        g_lastImpulse = { o.vx, o.vy, o.vz };

        switch (axis)
        {
        case 0: point.x = positive ? (o.maxX + margin) : (o.minX - margin); break;
        case 1: point.y = positive ? (o.maxY + margin) : (o.minY - margin); break;
        default: point.z = positive ? (o.maxZ + margin) : (o.minZ - margin); break;
        }
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

void WorldCollision_Clear()
{
    g_obstacles.clear();
    g_haveGround = false;
}
