#pragma once

#include "RenderWare.h"

// ============================================================
// 跨模型碰撞
//
// 每帧收集物理角色附近的其他实体（人物 / 车辆 / 地图物件），
// 用它们的碰撞模型（CColModel 的包围盒 + 包围球）当作障碍，
// 把头发 / 布料的粒子推开。
//
// 说明：
//   - 只收集一定半径内的实体，避免全图遍历的开销；
//   - 包围盒按"位置对齐的 AABB"处理（忽略旋转），足够挡住穿透；
//   - 被碰撞的实体数量有上限，保证帧率。
// ============================================================

struct RwV3d;
class CEntity;

void WorldCollision_Collect(const RwV3d& center, float radius,
                            CEntity* owner = nullptr,
                            bool includeOwner = false);

// ============================================================
//  实体胶囊障碍（2026-11-23 09:2x：人物骨胶囊）
//
//  人物 col model 只有一个粗糙盒子，"人推车布"几乎没有反应。
//  把 ped rig 每帧算出来的 22 根骨胶囊注册成世界障碍，布料撞到的是真实人形。
// ============================================================
struct WorldCapsule
{
    float ax, ay, az;    // 世界端点 A
    float bx, by, bz;    // 世界端点 B
    float radius;
    float vx, vy, vz;    // 该实体本帧位移（米）
    int   modelId;
};

void WorldCollision_SetEntityCapsules(CEntity* entity,
                                      const WorldCapsule* capsules, int count);
void WorldCollision_ClearEntityCapsules(CEntity* entity);
int  WorldCollision_CapsuleCount();

// 诊断：某个点到"人物骨胶囊"集合的最近表面距离（米；没有胶囊返回 -1）
float WorldCollision_NearestCapsuleDistance(const RwV3d& p);

// prev：粒子上一帧的世界位置（用来判断"从哪一侧进入"，避免弹错方向）
void WorldCollision_CollidePoint(RwV3d& point, const RwV3d& prev, float margin);

void WorldCollision_Clear();

// 上一次 CollidePoint 是否发生了接触（用于施加摩擦）
bool WorldCollision_LastHit();

// 最近一次命中来自什么（诊断用）：kind 0=无 1=地面 2=三角面 3=球 4=盒
struct WorldCollisionHitInfo
{
    int   kind = 0;
    int   modelId = -1;
    float depth = 0.0f;
};

void WorldCollision_LastHitInfo(WorldCollisionHitInfo& out);

// 诊断：本次收集到的盒/球数量、三角面数量
int WorldCollision_ObstacleCount();
int WorldCollision_TriangleCount();

// 诊断：本次收集里"来自 owner 自己"的障碍数量（0 = 自身碰撞没生效）
int WorldCollision_OwnerObstacleCount();

// ============================================================
//  把某个实体**自己**的 col model（球 + 盒）加进障碍（2026-11-23 07:2x）
//
//  地图物件（CBuilding）不在 Ped/Vehicle/Object 池里，Collect 从来不会收它，
//  所以"物件布料 vs 它自己的碰撞体"一直是空的（只有 FindGroundZFor3DCoord
//  的高度场能从上往下托一下，圆环/洞/侧面全失效）。
//  这个函数把 col->m_pColData 的球/盒按实体矩阵变换后加进来，
//  并且只修"刚穿进去"的浅穿透（<=15cm），免得把挂在物件上的布掀飞。
//
//  center/cullRadius：按布料质心裁剪。返回加入的障碍数（诊断用）。
//  outInfo：可选，把 col model 里各类形状的原始数量带出来（诊断"为什么是 0"）。
struct WorldColModelInfo
{
    int   spheres = 0;
    int   boxes = 0;
    int   tris = 0;
    float boundRadius = 0.0f;
    int   hasData = 0;      // m_pColData 是否非空
    int   hasVerts = 0;     // m_pVertices 是否非空（0 会让三角面读取直接返回）
    int   hasTris = 0;      // m_pTriangles 是否非空
};

int WorldCollision_AddEntityOwnColModel(CEntity* entity,
                                        const RwV3d& center, float cullRadius,
                                        WorldColModelInfo* outInfo = nullptr,
                                        float marginOverride = -1.0f);

// ============================================================
//  回退：把该实体**自己可见网格**（RpAtomic 的渲染三角形）加进障碍
//
//  有些 .col（COL3 mesh 之类）读不到三角面，但渲染网格一定在。
//  dynamicVertexMap：该实体几何体的"逐顶点 -> 布料粒子"映射（>=0 = 布料顶点），
//  长度必须是几何体顶点数；含布料顶点的三角形会被跳过，
//  否则布料会和自己每帧互相顶（整片炸开）。
// ============================================================

// 可见网格回退的逐项统计（排查"为什么掩码没生效/为什么加不进来"）
struct WorldOwnMeshStats
{
    int examined = 0;      // 看过多少个三角形
    int skippedCloth = 0;  // 因为含布料顶点被跳过
    int culled = 0;        // 因为超出收集半径被裁掉
    int added = 0;
    int geoVerts = 0;      // 该 atomic 的顶点数
    int maskVerts = 0;     // 传进来的顶点掩码长度
    int maskMode = 0;      // 1 = 掩码生效
};

int WorldCollision_AddEntityOwnMesh(CEntity* entity,
                                    const RwV3d& center, float cullRadius,
                                    const int* dynamicVertexMap, int numVerts,
                                    WorldOwnMeshStats* outStats = nullptr,
                                    float extraSkin = 0.0f);


// 诊断：本次加进来的"自身三角面"的包围盒（判断它们落在哪）
void WorldCollision_OwnColTriBounds(RwV3d& outMin, RwV3d& outMax, int& outCount);

// 诊断：某个点到"自身三角面"集合的最近距离（米；没有三角面返回 -1）
float WorldCollision_DistanceToOwnColTris(const RwV3d& p);

// 建三角面空间索引：**所有**添加三角形之后、Step 之前调用一次。
// 不调用的话 CollidePoint 退回线性扫描（正确但慢）。
void WorldCollision_BuildTriangleGrid();

// ============================================================
//  "自己物件的实心顶面"（ini: mapselfbox）
//
//  网格碰撞挡不住"从洞里穿过去"（洞里没有三角形）。这里再补一个顶面：
//  布粒子落在 [minX,maxX]×[minY,maxY] 内、且低于 topZ（在 band 以内）时，
//  被抬到 topZ + margin 之上。只向上推，不动侧面/下方。
// ============================================================
void WorldCollision_SetOwnTopPlate(float minX, float minY, float maxX, float maxY,
                                   float topZ, float band);
void WorldCollision_ClearOwnTopPlate();

// "自己物件网格"的自适应 skin（米）：caller 传 clamp(0.45×布料平均边长, 4cm, 12cm)
void WorldCollision_SetOwnColSkin(float skin);

// 扫掠（穿越）检测开关：1 = 开（默认），0 = 只做近表面判定（旧行为）
void WorldCollision_SetOwnColSweep(int enabled);

// 载具渲染网格碰撞（ini: carmesh，默认 1）
// SA 载具 col 只有盒+球，没有三角面；开启后用渲染网格给"别的车的布料碰撞"提供精度
void WorldCollision_SetVehicleMesh(int enabled);

// 载具碰撞体的膨胀量（ini: carmargin，默认 0.02 = 2cm）
// 只作用于**载具**的盒/球：SA 车的 col 盒常比可见车身小 ⇒ 布料会穿进车身外板
void WorldCollision_SetCarMargin(float margin);

// 地面竖直查询的抬升余量（ini: groundmargin，默认 0.02）
void WorldCollision_SetGroundMargin(float margin);

// 普通物件（CObject）盒/球的膨胀量（ini: objmargin，默认 0.02）
void WorldCollision_SetObjectMargin(float margin);

// 地图物件"自己"的 col 盒/球/三角面余量（ini: mapselfmargin，默认 0.02）
void WorldCollision_SetMapObjectMargin(float margin);

// 静态建筑（CBuilding）碰撞（ini: buildingcol，默认 1）
// 按扇区枚举附近建筑，把它们的球/盒/三角面加进障碍（三角形有预算，避免卡顿）
void WorldCollision_SetBuildingCollision(int enabled);
void WorldCollision_SetBuildingMargin(float margin);
int  WorldCollision_BuildingTriangleBudget();

// 最近一次建筑收集的计数（诊断）
struct WorldBuildingStats
{
    int sectors = 0;
    int visited = 0;
    int noCol = 0;      // 没有 col / 没有 col data 的建筑
    int culled = 0;     // 距离裁剪掉的建筑
    int accepted = 0;   // 真正收下的建筑
    int obs = 0;        // 收下的球+盒数
    int tris = 0;       // 收下的三角形数
    int slabs = 0;      // 薄墙实心板（低模墙）加了多少个

    // 最近一栋"收下"的建筑，以及最近一栋"被距离裁掉"的建筑
    int   nearModel = -1;
    int   nearObs = 0;
    int   nearTris = 0;
    float nearDist = -1.0f;

    int   cullModel = -1;
    float cullDist = -1.0f;
};

void WorldCollision_GetBuildingStats(WorldBuildingStats& out);
// 按实体的 col 包围盒自动算出"顶面托盘"（世界 AABB 的顶面）
void WorldCollision_SetOwnTopPlateFromEntity(CEntity* entity, float band);

// 诊断：地面采样被"抬升量过大"拒绝的次数（上坡炸开排查）
int  WorldCollision_GroundLiftRejected();
void WorldCollision_ResetGroundLiftRejected();

// 接触时障碍物的速度（用于把车辆/人物的速度传给布料）
void WorldCollision_LastImpulse(RwV3d& out);
