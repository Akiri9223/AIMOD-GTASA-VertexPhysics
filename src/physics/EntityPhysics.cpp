#include "EntityPhysics.h"
#include "GeometryPhysics.h"
#include "WorldCollision.h"

#include "common.h"
#include "CPools.h"
#include "CPed.h"
#include "CVehicle.h"
#include "CObject.h"
#include "CModelInfo.h"
#include "CAtomicModelInfo.h"
#include "CBuilding.h"
#include "CSector.h"
#include "CPtrNodeSingleLink.h"
#include "CWorld.h"
#include "CStreaming.h"
#include "CRenderer.h"
#include "safetyhook.hpp"
#include "rpskin.h"
#include "rphanim.h"

#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <vector>
#include <windows.h>

namespace
{
    // ------------------------------------------------------------
    // 运行状态
    // ------------------------------------------------------------

    struct Entry
    {
        RpGeometry* clone = nullptr;
        GeometryPhysics rig;
    };

    struct Candidate
    {
        RpAtomic* atomic = nullptr;
        int       modelIndex = -1;
    };

    std::map<RpAtomic*, Entry>   g_entries;
    std::map<RpGeometry*, char>  g_markerCache;
    std::vector<Candidate>       g_candidates;

    std::vector<std::string>     g_whitelistNames;
    std::map<std::string, int>   g_whitelistIds;      // name -> model id（-1 表示还没解析出来）
    std::set<std::string>        g_reportedModels;

    bool g_configLoaded = false;
    bool g_autoMode = false;
    bool g_whitelistMissingLogged = false;

    int g_scanCountdown = 0;
    int g_frameCounter = 0;

    std::map<RpGeometry*, GeometryPhysics> g_mapRigs;
    std::set<int> g_whitelistDebugSet;

    // 这些定义在后面，这里先声明（避免改动函数顺序）
    extern float g_worldRadius;
    extern float g_interactRadius;
    extern int   g_maxInstances;
    extern float g_windStrength;
    extern bool  g_mapPhysics;
    extern int   g_mapConvertRaw;

    bool IsModelEnabled(int modelIndex);
    int  GetMarkerCountCached(RpGeometry* geometry);
    RpGeometry* CreatePrivateClone(RpGeometry* source);
    void ProbeWhitelistedModels();
    int  GetMarkerCountCached(RpGeometry* geometry);
    RpGeometry* CreatePrivateClone(RpGeometry* source);
    void ModLog(const std::string& text);
    double NowMs();
    struct PhysicsDat;
    const PhysicsDat* GetPhysicsDat(int modelId, int vertexCount, const RwV3d* verts);
    bool ApplyDatColors(RpGeometry* clone, int modelId, std::vector<RwRGBA>& savedVisual, bool& allocatedPrelit);
    void RestoreVisualColors(RpGeometry* clone, const std::vector<RwRGBA>& saved, bool allocatedPrelit);
    std::string GetExeDirectory();

    // ============================================================
    // 逐实例地图物件物理
    //
    // 每个建筑实例拥有自己的几何体克隆 + 原子，因此：
    //   - 每个实例独立模拟（不再同步摆动）
    //   - 走世界空间物理，和角色/车辆一样有碰撞
    // ============================================================
    struct MapInstance
    {
        RpGeometry* clone = nullptr;
        RpAtomic*   atomic = nullptr;
        RwFrame*    frame = nullptr;
        RpAtomic*   modelAtomic = nullptr;   // 模型共享原子
        RpAtomic*   drawAtomic  = nullptr;   // 该实例自己的渲染原子（m_pRwObject）
        RpGeometry* originalGeometry = nullptr;
        GeometryPhysics rig;
        int    modelId = -1;
        RwV3d  center{};
        int    age = 0;
    };

    std::map<CBuilding*, MapInstance> g_mapInstances;
    // ============================================================
    // 渲染钩子（vtable[18]）：逐实例换入自己的几何体
    // ============================================================
    using EntityRenderFn = void(__thiscall*)(CEntity*);

    EntityRenderFn g_originalEntityRender = nullptr;
    bool g_entityRenderHooked = false;
    int  g_renderHookCalls = 0;
    int  g_renderHookMatched = 0;

    void __fastcall HookEntityRender(CEntity* self, void* /*edx*/)
    {
        ++g_renderHookCalls;

        if (!g_originalEntityRender)
            return;

        if ((g_renderHookCalls % 1800) == 0)
        {
            std::ostringstream oss;
            oss << "[RenderHook] calls=" << g_renderHookCalls
                << " matched=" << g_renderHookMatched
                << " instances=" << g_mapInstances.size();
            ModLog(oss.str());
        }

        if (!self)
            return;

        auto it = g_mapInstances.find(reinterpret_cast<CBuilding*>(self));

        if (it == g_mapInstances.end() || !it->second.clone)
        {
            g_originalEntityRender(self);
            return;
        }

        ++g_renderHookMatched;

        MapInstance& inst = it->second;

        RpAtomic* drawAtomic = inst.modelAtomic;

        if (inst.drawAtomic)
        {
            if (reinterpret_cast<void*>(inst.drawAtomic) != self->m_pRwObject)
            {
                g_originalEntityRender(self);   // 实体被回收复用，跳过
                return;
            }

            drawAtomic = inst.drawAtomic;
        }
        else if (self->m_pRwObject && RwObjectGetType(self->m_pRwObject) == rpATOMIC)
        {
            drawAtomic = reinterpret_cast<RpAtomic*>(self->m_pRwObject);
            inst.drawAtomic = drawAtomic;
            inst.originalGeometry = RpAtomicGetGeometry(drawAtomic);
        }

        if (g_renderHookMatched == 1)
        {
            std::ostringstream oss;
            oss << "[RenderHook] first match entity=" << self
                << " rwObject=" << self->m_pRwObject
                << " drawAtomic=" << drawAtomic
                << " modelAtomic=" << inst.modelAtomic
                << " clone=" << inst.clone;
            ModLog(oss.str());
        }

        if (!drawAtomic)
        {
            g_originalEntityRender(self);
            return;
        }

        RpGeometry* original = RpAtomicGetGeometry(drawAtomic);

        if (original == inst.clone)
        {
            g_originalEntityRender(self);
            return;
        }

        RpAtomicSetGeometry(drawAtomic, inst.clone, 0);
        g_originalEntityRender(self);

        if (g_mapConvertRaw != 2)
            RpAtomicSetGeometry(drawAtomic, original, 0);
    }

    void InstallRenderHook(CBuilding* sample)
    {
        if (g_entityRenderHooked) return;
        g_entityRenderHooked = true;

        if (!sample)
        {
            ModLog("[RenderHook] no building sample, skip");
            return;
        }

        void** vtable = *reinterpret_cast<void***>(sample);

        if (!vtable)
        {
            ModLog("[RenderHook] empty vtable, skip");
            return;
        }

        DWORD oldProtect = 0;

        if (!VirtualProtect(&vtable[18], sizeof(void*), PAGE_READWRITE, &oldProtect))
        {
            ModLog("[RenderHook] VirtualProtect failed, skip");
            return;
        }

        g_originalEntityRender = reinterpret_cast<EntityRenderFn>(vtable[18]);
        vtable[18] = reinterpret_cast<void*>(&HookEntityRender);

        DWORD dummy = 0;
        VirtualProtect(&vtable[18], sizeof(void*), oldProtect, &dummy);

        std::ostringstream oss;
        oss << "[RenderHook] vtable=0x" << std::hex
            << reinterpret_cast<std::uintptr_t>(vtable)
            << " original=0x" << reinterpret_cast<std::uintptr_t>(g_originalEntityRender)
            << std::dec << " installed";
        ModLog(oss.str());
    }

    void DestroyMapInstance(MapInstance& inst)
    {
        if (inst.atomic)
        {
            RpAtomicDestroy(inst.atomic);
            inst.atomic = nullptr;
        }

        if (inst.frame)
        {
            RwFrameDestroy(inst.frame);
            inst.frame = nullptr;
        }

        if (inst.clone)
        {
            RpGeometryDestroy(inst.clone);
            inst.clone = nullptr;
        }
    }

    bool CreateMapInstance(CBuilding* building, int modelId, MapInstance& out,
                           const char** reason = nullptr)
    {
        auto fail = [&](const char* text) -> bool
        {
            if (reason) *reason = text;
            return false;
        };

        CBaseModelInfo* info = CModelInfo::GetModelInfo(modelId);
        if (!info || info->GetModelType() != MODEL_INFO_ATOMIC) return fail("not atomic model");

        RpAtomic* modelAtomic =
            reinterpret_cast<CAtomicModelInfo*>(info)->GetAtomicFromDistance(1.0f);
        if (!modelAtomic) return fail("no atomic yet");

        RpGeometry* source = RpAtomicGetGeometry(modelAtomic);
        if (!source) return fail("no geometry");
        // 有 .dat 时不再要求几何体带物理顶点色
        const bool hasDat = (GetPhysicsDat(modelId, static_cast<int>(source->numVertices),
                                           RpMorphTargetGetVertices(&source->morphTarget[0])) != nullptr);

        if (!hasDat && GetMarkerCountCached(source) <= 0) return fail("no physics colors");

        RpGeometry* clone = CreatePrivateClone(source);
        if (!clone) return fail("clone failed");

        RpAtomic* atomic = RpAtomicCreate();
        if (!atomic) { RpGeometryDestroy(clone); return fail("atomic create failed"); }

        RwFrame* frame = RwFrameCreate();
        if (!frame)
        {
            RpAtomicDestroy(atomic);
            RpGeometryDestroy(clone);
            return fail("frame create failed");
        }

        RpAtomicSetFrame(atomic, frame);

        if (!RpAtomicSetGeometry(atomic, clone, 0) || RpAtomicGetGeometry(atomic) != clone)
        {
            RwFrameDestroy(frame);
            RpAtomicDestroy(atomic);
            RpGeometryDestroy(clone);
            return false;
        }

        // 实例的世界矩阵（建筑是静态的，只需要设置一次）
        RwMatrix ltm;

        if (building->m_matrix)
        {
            ltm = *reinterpret_cast<RwMatrix*>(building->m_matrix);
        }
        else
        {
            const CVector& pos = building->GetPosition();

            std::memset(&ltm, 0, sizeof(ltm));

            ltm.right.x = 1.0f; ltm.right.y = 0.0f; ltm.right.z = 0.0f;
            ltm.up.x    = 0.0f; ltm.up.y    = 1.0f; ltm.up.z    = 0.0f;
            ltm.at.x    = 0.0f; ltm.at.y    = 0.0f; ltm.at.z    = 1.0f;

            ltm.pos.x = pos.x; ltm.pos.y = pos.y; ltm.pos.z = pos.z;
        }

        RwFrameTransform(frame, &ltm, rwCOMBINEREPLACE);
        RwFrameUpdateObjects(frame);

        std::vector<RwRGBA> savedVisual;
        bool datAllocatedPrelit = false;
        const bool usedDat = ApplyDatColors(clone, modelId, savedVisual, datAllocatedPrelit);

        const double tInit0 = NowMs();

        GeometryPhysics rig;
        std::string report;

        if (!InitGeometryPhysics(rig, clone, &ltm, report))
        {
            RwFrameDestroy(frame);
            RpAtomicDestroy(atomic);
            RpGeometryDestroy(clone);
            return false;
        }

        const double tInit1 = NowMs();

        if (usedDat)
            RestoreVisualColors(clone, savedVisual, datAllocatedPrelit);

        rig.iterations = 6;

        // 实例中心用粒子实际位置计算（建筑矩阵位置可能不可靠）
        float cx = 0.0f, cy = 0.0f, cz = 0.0f;
        int   cnt = 0;

        for (const auto& p : rig.particles)
        {
            cx += p.posWorld.x; cy += p.posWorld.y; cz += p.posWorld.z; ++cnt;
        }

        if (cnt > 0) { cx /= cnt; cy /= cnt; cz /= cnt; }
        else { cx = ltm.pos.x; cy = ltm.pos.y; cz = ltm.pos.z; }

        {
            std::ostringstream oss;
            oss << "[MapInst] init took=" << (tInit1 - tInit0) << "ms"
                << " center=(" << cx << "," << cy << "," << cz << ")";
            ModLog(oss.str());
        }

        out.clone = clone;
        out.atomic = atomic;
        out.frame = frame;
        out.rig = std::move(rig);
        out.modelId = modelId;
        out.center = { cx, cy, cz };
        out.age = 0;

        return true;
    }

    // ============================================================
    // 渲染钩子：CRenderer::RenderOneNonRoad
    //
    // 在引擎绘制某个建筑实例时，把模型原子的几何体临时换成该实例的
    // 克隆体，调用原函数（原生管线 / shader mod 正常处理），再换回来。
    // 这样引擎和画面类 mod 都感觉不到我们的存在。
    // ============================================================
    using RenderOneNonRoadFn = void(__cdecl*)(CEntity*);
    RenderOneNonRoadFn g_originalRenderOneNonRoad = nullptr;
    safetyhook::InlineHook g_renderHook;
    bool g_renderHookTried = false;

    std::uintptr_t ResolveThunkTarget(void* thunk)
    {
        const std::uint8_t* code = static_cast<const std::uint8_t*>(thunk);

        if (code[0] == 0xE9 || code[0] == 0xE8)   // jmp/call rel32
        {
            std::int32_t rel = 0;
            std::memcpy(&rel, code + 1, 4);
            return reinterpret_cast<std::uintptr_t>(code) + 5 + rel;
        }

        if (code[0] == 0xFF && code[1] == 0x25)   // jmp dword ptr [addr]
        {
            std::uint32_t abs = 0;
            std::memcpy(&abs, code + 2, 4);
            return *reinterpret_cast<std::uint32_t*>(abs);
        }

        if (code[0] == 0xB8 && code[5] == 0xFF && code[6] == 0xE0)  // mov eax, imm32; jmp eax
        {
            std::uint32_t abs = 0;
            std::memcpy(&abs, code + 1, 4);
            return abs;
        }

        return 0;
    }

    void HookRenderOneNonRoad(CEntity* entity);

    bool InOurModule(std::uintptr_t addr)
    {
        MEMORY_BASIC_INFORMATION mbi{};

        if (!VirtualQuery(reinterpret_cast<void*>(addr), &mbi, sizeof(mbi)))
            return false;

        static std::uintptr_t s_selfBase = 0;

        if (!s_selfBase)
        {
            HMODULE self = nullptr;

            GetModuleHandleExA(
                GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
                reinterpret_cast<LPCSTR>(&InOurModule), &self);

            s_selfBase = reinterpret_cast<std::uintptr_t>(self);
        }

        return reinterpret_cast<std::uintptr_t>(mbi.AllocationBase) == s_selfBase;
    }

    // plugin-sdk 的转发可能有多次跳转，一直跟到"不在本模块"的地址为止
    std::uintptr_t ResolveRealTarget(void* thunk)
    {
        std::uintptr_t cur = reinterpret_cast<std::uintptr_t>(thunk);

        for (int hop = 0; hop < 8; ++hop)
        {
            const std::uintptr_t next = ResolveThunkTarget(reinterpret_cast<void*>(cur));

            if (!next || next == cur) return cur;
            if (!InOurModule(next)) return next;

            cur = next;
        }

        return cur;
    }

    // ============================================================
    // 渲染钩子（vtable 方式）
    //
    // CEntity::Render() 只是转发到 vtable 第 18 个槽。
    // 我们把 CBuilding 的 vtable[18] 换成自己的函数：
    //   - 不改任何机器码，和 shader mod 的代码 hook 不冲突
    //   - 绘制前把模型原子的几何体换成该实例的克隆，画完换回
    // ============================================================
    // 分帧创建：克隆几何体 / 初始化物理 / 激活 分到不同帧，避免单帧卡顿
    struct PendingInstance
    {
        CBuilding*   building = nullptr;
        MapInstance  inst;
        int          stage = 0;     // 0 = 已克隆，1 = 已初始化，2 = 待激活
    };

    std::vector<PendingInstance> g_pendingInstances;
    std::map<CBuilding*, int> g_instanceRetry;

    static double NowMs()
    {
        static LARGE_INTEGER freq{};
        static bool init = false;

        if (!init) { QueryPerformanceFrequency(&freq); init = true; }

        LARGE_INTEGER now{};
        QueryPerformanceCounter(&now);

        return static_cast<double>(now.QuadPart) * 1000.0 / static_cast<double>(freq.QuadPart);
    }

    void UpdateMapInstances(const RwV3d& center, float deltaTime)
    {
        const int cap = std::max(1, g_maxInstances);
        const float releaseRadius = std::max(40.0f, g_interactRadius * 3.0f);

        const int sectorX = CWorld::GetSectorIndexX(center.x);
        const int sectorY = CWorld::GetSectorIndexY(center.y);

        // 安装渲染钩子（只需一次）
        if (!g_entityRenderHooked)
        {
            CBuilding* sample = nullptr;
            CSector* firstSector = CWorld::GetSector(sectorX, sectorY);

            if (firstSector && firstSector->m_buildingList.GetNode())
                sample = reinterpret_cast<CBuilding*>(firstSector->m_buildingList.GetNode()->pItem);

            if (sample)
                InstallRenderHook(sample);
        }

        if (!g_mapPhysics)
        {
            return;
        }

        // ------------------------------------------------------------
        // 1) 扫描：释放过期实例 + 收集候选
        // ------------------------------------------------------------
        static int s_createdThisCall = 0;
        s_createdThisCall = 0;

        std::vector<CBuilding*> candidates;

        for (int dx = -1; dx <= 1; ++dx)
        {
            for (int dy = -1; dy <= 1; ++dy)
            {
                CSector* sector = CWorld::GetSector(sectorX + dx, sectorY + dy);
                if (!sector) continue;

                for (CPtrNodeSingleLink* node = sector->m_buildingList.GetNode();
                     node != nullptr; node = node->pNext)
                {
                    CBuilding* building = reinterpret_cast<CBuilding*>(node->pItem);
                    if (!building) continue;
                    if (!IsModelEnabled(building->m_nModelIndex)) continue;

                    auto existing = g_mapInstances.find(building);

                    if (existing != g_mapInstances.end())
                    {
                        MapInstance& old = existing->second;

                        const float ex = old.center.x - center.x;
                        const float ey = old.center.y - center.y;
                        const float ez = old.center.z - center.z;

                        const bool tooFar =
                            (ex * ex + ey * ey + ez * ez) > (releaseRadius * releaseRadius);

                        const bool recycled = old.drawAtomic &&
                            (reinterpret_cast<void*>(old.drawAtomic) != building->m_pRwObject);

                        if (tooFar || recycled)
                        {
                            if (!recycled && old.drawAtomic && old.originalGeometry)
                                RpAtomicSetGeometry(old.drawAtomic, old.originalGeometry, 0);

                            DestroyMapInstance(old);

                            std::ostringstream oss;
                            oss << "[MapInst] released building=" << building
                                << " far=" << (tooFar ? 1 : 0)
                                << " recycled=" << (recycled ? 1 : 0);
                            ModLog(oss.str());

                            g_mapInstances.erase(existing);
                        }

                        continue;
                    }

                    // 是否已经在待创建队列里
                    bool pending = false;

                    for (const auto& p : g_pendingInstances)
                        if (p.building == building) { pending = true; break; }

                    if (!pending)
                    {
                        auto retry = g_instanceRetry.find(building);
                        if (retry == g_instanceRetry.end() || retry->second <= g_frameCounter)
                            candidates.push_back(building);
                    }
                }
            }
        }

        // ------------------------------------------------------------
        // 2) 分帧创建：每帧只推进一个阶段
        // ------------------------------------------------------------
        if (!candidates.empty() &&
            static_cast<int>(g_mapInstances.size() +
                             g_pendingInstances.size()) < cap)
        {
            PendingInstance pending;
            pending.building = candidates.front();

            const double t0 = NowMs();

            const char* failReason = nullptr;
            if (CreateMapInstance(pending.building, pending.building->m_nModelIndex, pending.inst, &failReason))
            {
                const double t1 = NowMs();

                std::ostringstream oss;
                oss << "[MapInst] clone done modelId=" << pending.building->m_nModelIndex
                    << " take=" << (t1 - t0) << "ms";
                ModLog(oss.str());

                g_pendingInstances.push_back(std::move(pending));
            }
            else
            {
                g_instanceRetry[pending.building] = g_frameCounter + 180;

                std::ostringstream oss;
                oss << "[MapInst] create failed modelId=" << pending.building->m_nModelIndex
                    << " reason=" << (failReason ? failReason : "unknown");
                ModLog(oss.str());
            }
        }

        // 待激活的实例：记录渲染原子并移入正式表（不重复做物理初始化）
        for (auto it = g_pendingInstances.begin(); it != g_pendingInstances.end();)
        {
            CBuilding* building = it->building;

            RpAtomic* draw = nullptr;

            if (building->m_pRwObject &&
                RwObjectGetType(building->m_pRwObject) == rpATOMIC)
            {
                draw = reinterpret_cast<RpAtomic*>(building->m_pRwObject);
            }

            it->inst.drawAtomic = draw;
            it->inst.originalGeometry = draw ? RpAtomicGetGeometry(draw) : nullptr;

            std::ostringstream oss;
            oss << "[MapInst] activated modelId=" << building->m_nModelIndex
                << " drawAtomic=" << draw
                << " cloth=" << it->inst.rig.clothCount
                << " particles=" << it->inst.rig.particles.size();
            ModLog(oss.str());

            g_mapInstances.emplace(building, std::move(it->inst));
            it = g_pendingInstances.erase(it);

            ++s_createdThisCall;
            break;   // 每帧只激活一个
        }

        // ------------------------------------------------------------
        // 3) 模拟 + 写回
        // ------------------------------------------------------------
        static int s_simFrame = 0;
        ++s_simFrame;

        // 碰撞每 3 帧一次，范围缩小到 1.5 m（大幅减少收集的三角形数量）
        const bool doCollision = ((s_simFrame % 3) == 0);

        for (auto& entry : g_mapInstances)
        {
            MapInstance& inst = entry.second;
            if (!inst.rig.initialized) continue;

            const double t0 = NowMs();

            if (doCollision)
                WorldCollision_Collect(inst.center, 1.5f);

            void (*savedCollide)(RwV3d&, const RwV3d&) = g_worldCollideFn;

            if (!doCollision)
                g_worldCollideFn = nullptr;

            StepGeometryPhysics(inst.rig, inst.atomic, deltaTime);
            ApplyGeometryPhysics(inst.rig, inst.atomic);

            g_worldCollideFn = savedCollide;

            if ((s_simFrame % 300) == 0)
            {
                const double t1 = NowMs();

                std::ostringstream oss;
                oss << "[MapInst] step took=" << (t1 - t0) << "ms"
                    << " instances=" << g_mapInstances.size()
                    << " collide=" << (doCollision ? 1 : 0);
                ModLog(oss.str());
            }
        }
    }


    void UpdateMapPhysics(const RwV3d& center, float deltaTime)
    {
        static int s_callCount = 0;
        ++s_callCount;

        // 入口即打印（前 3 次），用于确认函数是否真的被调用
        if (s_callCount <= 3)
        {
            std::ostringstream oss;
            oss << "[MapPhysics] call#" << s_callCount
                << " center=(" << center.x << "," << center.y << "," << center.z << ")"
                << " mapPhysics=" << (g_mapPhysics ? 1 : 0)
                << " wind=" << g_windStrength;
            ModLog(oss.str());
        }

        if (!g_mapPhysics) return;
        if (g_windStrength <= 0.0f) return;

        const float radiusSq = g_interactRadius * g_interactRadius;
        const int   cap = std::max(1, g_maxInstances);
        const float releaseRadius = std::max(40.0f, g_interactRadius * 3.0f);

        static int s_createdThisCall = 0;
        s_createdThisCall = 0;

        int scanned = 0;
        int inRange = 0;
        int atomicType = 0;
        int whitelisted = 0;
        int withMarkers = 0;

        std::set<int> seenIds;

        // 通过世界分区遍历：SA 的 ms_pBuildingPool 在部分版本里是空的，
        // 静态建筑真正的归属是 CWorld::ms_aSectors[..].m_buildingList
        const int sectorX = CWorld::GetSectorIndexX(center.x);
        const int sectorY = CWorld::GetSectorIndexY(center.y);

        if (s_callCount <= 3)
        {
            std::ostringstream oss;
            oss << "[MapPhysics] sectors=(" << sectorX << "," << sectorY << ")";
            ModLog(oss.str());
        }

        auto considerBuilding = [&](CBuilding* building)
        {
            ++scanned;

            if (!building) return;
            if (static_cast<int>(g_mapRigs.size()) >= cap) return;

            if (seenIds.size() < 32)
                seenIds.insert(building->m_nModelIndex);

            const CVector& pos = building->GetPosition();

            const float dx = pos.x - center.x;
            const float dy = pos.y - center.y;
            const float dz = pos.z - center.z;

            // 只统计"在半径内"的数量用于诊断；**不再用它过滤**，
            // 因为静态建筑的 GetPosition() 常常是 (0,0,0)，
            // 用它做距离判断会把所有建筑筛掉。
            if (dx * dx + dy * dy + dz * dz <= radiusSq)
                ++inRange;

            CBaseModelInfo* info = CModelInfo::GetModelInfo(building->m_nModelIndex);
            if (!info) return;
            if (info->GetModelType() != MODEL_INFO_ATOMIC) return;

            ++atomicType;

            if (!IsModelEnabled(building->m_nModelIndex)) return;

            ++whitelisted;

            if (g_whitelistDebugSet.size() < 12 &&
                g_whitelistDebugSet.insert(building->m_nModelIndex).second)
            {
                std::ostringstream oss;
                oss << "[MapPhysics] whitelisted model in sector: modelId="
                    << building->m_nModelIndex;
                ModLog(oss.str());
            }

            CAtomicModelInfo* atomicInfo = reinterpret_cast<CAtomicModelInfo*>(info);
            RpAtomic* modelAtomic = atomicInfo->GetAtomicFromDistance(1.0f);

            if (!modelAtomic)
            {
                static std::set<int> s_loggedNoAtomic;
                if (s_loggedNoAtomic.insert(building->m_nModelIndex).second)
                {
                    std::ostringstream oss;
                    oss << "[MapPhysics] STOP: no atomic yet (model not streamed?) modelId="
                        << building->m_nModelIndex;
                    ModLog(oss.str());
                }
                return;
            }

            RpGeometry* geometry = RpAtomicGetGeometry(modelAtomic);

            if (!geometry)
            {
                static std::set<int> s_loggedNoGeo;
                if (s_loggedNoGeo.insert(building->m_nModelIndex).second)
                {
                    std::ostringstream oss;
                    oss << "[MapPhysics] STOP: atomic has no geometry modelId="
                        << building->m_nModelIndex;
                    ModLog(oss.str());
                }
                return;
            }

            if (g_mapRigs.find(geometry) != g_mapRigs.end()) return;

            const int markerCount = GetMarkerCountCached(geometry);

            if (markerCount <= 0)
            {
                static std::set<RpGeometry*> s_loggedNoMarkers;
                if (s_loggedNoMarkers.insert(geometry).second)
                {
                    std::ostringstream oss;
                    oss << "[MapPhysics] STOP: no physics vertex colors modelId="
                        << building->m_nModelIndex
                        << " geometry=" << geometry
                        << " verts=" << geometry->numVertices
                        << " prelit=" << (geometry->preLitLum ? "YES" : "NO");
                    ModLog(oss.str());
                }
                return;
            }

            ++withMarkers;

            RwFrame* frame = RpAtomicGetFrame(modelAtomic);
            const RwMatrix* ltm = frame ? RwFrameGetLTM(frame) : nullptr;

            if (!ltm)
            {
                std::ostringstream oss;
                oss << "[MapPhysics] STOP: atomic frame/LTM is null modelId="
                    << building->m_nModelIndex;
                ModLog(oss.str());
                return;
            }

            // 关键：静态地图模型的几何体常常带 rpGEOMETRYNATIVE 标记，
            // 渲染直接用 D3D 顶点缓冲，改 CPU 侧顶点不会反映到画面。
            // 所以这里克隆一份"非 native"几何体挂给模型原子，
            // 之后所有写回都作用在这份克隆上（模型空间形变，所有实例共享）。
            RpGeometry* privateClone = CreatePrivateClone(geometry);

            if (!privateClone)
            {
                std::ostringstream oss;
                oss << "[MapPhysics] STOP: clone failed modelId="
                    << building->m_nModelIndex
                    << " flags=0x" << std::hex << geometry->flags << std::dec;
                ModLog(oss.str());
                return;
            }

            if (!RpAtomicSetGeometry(modelAtomic, privateClone, 0) ||
                RpAtomicGetGeometry(modelAtomic) != privateClone)
            {
                RpGeometryDestroy(privateClone);

                std::ostringstream oss;
                oss << "[MapPhysics] STOP: RpAtomicSetGeometry failed modelId="
                    << building->m_nModelIndex;
                ModLog(oss.str());
                return;
            }

            GeometryPhysics rig;
            rig.modelSpace = true;
            rig.windStrength = g_windStrength;

            std::string report;

            if (!InitGeometryPhysics(rig, privateClone, ltm, report))
            {
                RpAtomicSetGeometry(modelAtomic, geometry, 0);
                RpGeometryDestroy(privateClone);

                std::ostringstream oss;
                oss << "[MapPhysics] STOP: InitGeometryPhysics failed modelId="
                    << building->m_nModelIndex
                    << " geometry=" << privateClone
                    << " " << report;
                ModLog(oss.str());
                return;
            }

            AddDefaultColliders(rig, nullptr);

            {
                std::ostringstream oss;
                oss << "[MapPhysics] rig created modelId=" << building->m_nModelIndex
                    << " sourceGeometry=" << geometry
                    << " clone=" << privateClone
                    << " srcFlags=0x" << std::hex << geometry->flags << std::dec
                    << " hair=" << rig.hairCount
                    << " cloth=" << rig.clothCount
                    << " body=" << rig.bodyCount
                    << " particles=" << rig.particles.size()
                    << " " << report;
                ModLog(oss.str());
            }

            g_mapRigs.emplace(privateClone, std::move(rig));
        };

        for (int dx = -1; dx <= 1; ++dx)
        {
            for (int dy = -1; dy <= 1; ++dy)
            {
                CSector* sector = CWorld::GetSector(sectorX + dx, sectorY + dy);
                if (!sector) continue;

                for (CPtrNodeSingleLink* node = sector->m_buildingList.GetNode();
                     node != nullptr;
                     node = node->pNext)
                {
                    considerBuilding(reinterpret_cast<CBuilding*>(node->pItem));
                }
            }
        }

        // IPL 静态物件：m_pRwObject 为空（渲染直接用模型原子），
        // 所以实体路径会跳过它们；这里补一遍物体池
        {
            CPool<CObject, CCutsceneObject>* objectPool = CPools::ms_pObjectPool;

            if (objectPool)
            {
                for (int i = 0; i < objectPool->m_nSize; ++i)
                {
                    CObject* object = objectPool->GetAt(i);
                    if (!object) continue;
                    if (object->m_pRwObject != nullptr) continue;   // 有对象的走实体路径

                    CBuilding* asBuilding = reinterpret_cast<CBuilding*>(object);
                    considerBuilding(asBuilding);
                }
            }
        }

        // 诊断：每隔一段时间汇报扫描结果，便于定位"为什么没有物理"
        static int s_diagTimer = 0;

        if (++s_diagTimer >= 300)
        {
            s_diagTimer = 0;

            std::ostringstream oss;
            oss << "[MapPhysics] buildings=" << scanned
                << " inRange=" << inRange
                << " atomicType=" << atomicType
                << " whitelisted=" << whitelisted
                << " withMarkers=" << withMarkers
                << " rigs=" << g_mapRigs.size()
                << " radius=" << g_interactRadius
                << " ids=";

            int shownIds = 0;
            for (int id : seenIds)
            {
                if (shownIds++ >= 10) break;
                oss << id << ",";
            }

            ModLog(oss.str());
        }

        for (auto& entry : g_mapRigs)
        {
            GeometryPhysics& rig = entry.second;
            if (!rig.initialized) continue;

            StepGeometryPhysics(rig, nullptr, deltaTime);

            // 直接用几何体指针写入（模型空间模式不需要 atomic）
            if (RpGeometryLock(entry.first, rpGEOMETRYLOCKVERTICES))
            {
                RwV3d* verts = RpMorphTargetGetVertices(&entry.first->morphTarget[0]);
                const int numVerts = RpGeometryGetNumVertices(entry.first);

                if (verts)
                {
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
                }

                RpGeometryUnlock(entry.first);
            }
        }
    }

    // 供 GeometryPhysics 使用的固定边距版本
    void WorldCollideThunk(RwV3d& point, const RwV3d& prev)
    {
        WorldCollision_CollidePoint(point, prev, 0.01f);
    }

    constexpr int kMaxRigs = 128;

    // ------------------------------------------------------------
    // 日志
    // ------------------------------------------------------------

    // ============================================================
    // 外部物理数据：<exe>/vertexphysics/<模型名>.dat
    //
    // 格式（Blender 插件导出）：
    //   # index R G B x y z
    //   0 64 120 32 12.5 -3.2 1.8
    //
    // 用法：把 .dat 的颜色临时写进"私有克隆体"的 preLitLum，
    //       初始化物理之后再把视觉颜色还原，渲染完全不受影响。
    // ============================================================
    struct PhysicsDat
    {
        bool loaded = false;
        int  count = 0;
        std::vector<RwRGBA> colors;
        std::vector<RwV3d>  positions;   // 位置匹配用（DFF 会拆分顶点，索引对不上）
        bool   hasBbox = false;
        RwV3d  bboxMin{ 0.0f, 0.0f, 0.0f };
        RwV3d  bboxMax{ 0.0f, 0.0f, 0.0f };
    };

    std::map<std::uint64_t, PhysicsDat> g_physicsDatCache;   // (modelId,顶点数) -> data

    std::string GetDatDirectory()
    {
        std::string dir = GetExeDirectory();
        dir += "vertexphysics\\";
        return dir;
    }

    // 读取一个 .dat 文件（失败返回 false）
    bool LoadDatFile(const std::string& path, PhysicsDat& out)
    {
        std::ifstream file(path.c_str());
        if (!file.is_open()) return false;

        std::vector<std::pair<int, RwRGBA>> entries;
        std::map<int, RwV3d> positions;

        int maxIndex = -1;
        std::string line;

        while (std::getline(file, line))
        {
            const size_t first = line.find_first_not_of(" \t\r\n");
            if (first == std::string::npos) continue;

            // 包围盒指纹：# bbox: minx miny minz maxx maxy maxz
            if (line.compare(first, 7, "# bbox:") == 0)
            {
                std::istringstream bboxStream(line.substr(first + 7));

                float mnx = 0, mny = 0, mnz = 0, mxx = 0, mxy = 0, mxz = 0;

                if (bboxStream >> mnx >> mny >> mnz >> mxx >> mxy >> mxz)
                {
                    out.hasBbox = true;
                    out.bboxMin = { mnx, mny, mnz };
                    out.bboxMax = { mxx, mxy, mxz };
                }

                continue;
            }

            if (line[first] == '#') continue;

            std::istringstream iss(line);

            int index = 0, r = 0, g = 0, b = 0;
            float x = 0.0f, y = 0.0f, z = 0.0f;

            if (!(iss >> index >> r >> g >> b >> x >> y >> z)) continue;
            if (index < 0) continue;

            RwRGBA color;
            color.red   = static_cast<RwUInt8>(std::clamp(r, 0, 255));
            color.green = static_cast<RwUInt8>(std::clamp(g, 0, 255));
            color.blue  = static_cast<RwUInt8>(std::clamp(b, 0, 255));
            color.alpha = 255;

            entries.emplace_back(index, color);
            positions[index] = { x, y, z };

            if (index > maxIndex) maxIndex = index;
        }

        if (entries.empty() || maxIndex < 0) return false;

        out.loaded = true;
        out.count = maxIndex + 1;
        out.colors.assign(out.count, RwRGBA{ 255, 255, 255, 255 });
        out.positions.assign(out.count, RwV3d{ 0.0f, 0.0f, 0.0f });

        for (const auto& e : entries)
            out.colors[e.first] = e.second;

        for (const auto& p : positions)
        {
            if (p.first < out.count)
                out.positions[p.first] = p.second;
        }

        // 线性 -> sRGB 自动修正（旧版插件导出的 dat）
        {
            auto snapCount = [](const std::vector<RwRGBA>& colors) -> int
            {
                int n = 0;

                for (const auto& c : colors)
                {
                    if (c.red == 255 && c.green == 255 && c.blue == 255) continue;

                    const int r = c.red;

                    if (std::abs(r - 0) <= 6 || std::abs(r - 64) <= 6 ||
                        std::abs(r - 128) <= 6 || std::abs(r - 192) <= 6 ||
                        std::abs(r - 254) <= 6)
                        ++n;
                }

                return n;
            };

            const int before = snapCount(out.colors);

            std::vector<RwRGBA> converted = out.colors;

            for (auto& c : converted)
            {
                auto toSrgb = [](RwUInt8 v) -> RwUInt8
                {
                    const float lin = v / 255.0f;
                    const float s = (lin <= 0.0031308f)
                        ? (lin * 12.92f)
                        : (1.055f * std::pow(lin, 1.0f / 2.4f) - 0.055f);

                    return static_cast<RwUInt8>(std::clamp(s * 255.0f + 0.5f, 0.0f, 255.0f));
                };

                c.red   = toSrgb(c.red);
                c.green = toSrgb(c.green);
                c.blue  = toSrgb(c.blue);
            }

            const int after = snapCount(converted);

            if (after > before * 2 && after > 10)
                out.colors = converted;
        }

        return true;
    }

    // 按 (模型, 顶点数) 取 dat：
    //   1) 先找 <model>.dat（单部件模型）
    //   2) 再找 <model>\*.dat（多部件模型：车辆等），选顶点数匹配的那个
    const PhysicsDat* GetPhysicsDat(int modelId, int vertexCount, const RwV3d* verts)
    {
        const std::uint64_t key =
            (static_cast<std::uint64_t>(modelId) << 32) |
            static_cast<std::uint32_t>(vertexCount);

        auto cached = g_physicsDatCache.find(key);

        if (cached != g_physicsDatCache.end())
            return cached->second.loaded ? &cached->second : nullptr;

        PhysicsDat placeholder;
        g_physicsDatCache[key] = placeholder;

        std::string name;

        for (const auto& entry : g_whitelistIds)
        {
            if (entry.second == modelId) { name = entry.first; break; }
        }

        if (name.empty()) return nullptr;

        const std::string dir = GetDatDirectory();

        std::vector<std::string> candidates;

        const std::string single = dir + name + ".dat";

        // 也接受 dump 流程生成的名字：<模型名>_<顶点数>.dat（例如 wfyro_1094.dat）
        const std::string withCount =
            dir + name + "_" + std::to_string(vertexCount) + ".dat";

        if (GetFileAttributesA(withCount.c_str()) != INVALID_FILE_ATTRIBUTES)
            candidates.push_back(withCount);

        // 再兜底一层：<模型名>_*.dat
        {
            WIN32_FIND_DATAA fdWild{};
            HANDLE hWild = FindFirstFileA((dir + name + "_*.dat").c_str(), &fdWild);

            if (hWild != INVALID_HANDLE_VALUE)
            {
                do
                {
                    candidates.push_back(dir + fdWild.cFileName);
                } while (FindNextFileA(hWild, &fdWild));

                FindClose(hWild);
            }
        }

        if (GetFileAttributesA(single.c_str()) != INVALID_FILE_ATTRIBUTES)
            candidates.push_back(single);

        const std::string subDir = dir + name + "\\";
        WIN32_FIND_DATAA fd{};
        HANDLE hFind = FindFirstFileA((subDir + "*.dat").c_str(), &fd);

        if (hFind != INVALID_HANDLE_VALUE)
        {
            do
            {
                candidates.push_back(subDir + fd.cFileName);
            } while (FindNextFileA(hFind, &fd));

            FindClose(hFind);
        }

        if (candidates.empty())
        {
            ModLog("[Dat] not found: " + single);
            return nullptr;
        }

        // 几何体包围盒（用于在顶点数相同的多个部件之间区分）
        RwV3d geoMin = { 0.0f, 0.0f, 0.0f };
        RwV3d geoMax = { 0.0f, 0.0f, 0.0f };
        bool  hasGeoBbox = false;

        if (verts && vertexCount > 0)
        {
            hasGeoBbox = true;

            geoMin = geoMax = verts[0];

            for (int i = 1; i < vertexCount; ++i)
            {
                geoMin.x = std::min(geoMin.x, verts[i].x);
                geoMin.y = std::min(geoMin.y, verts[i].y);
                geoMin.z = std::min(geoMin.z, verts[i].z);
                geoMax.x = std::max(geoMax.x, verts[i].x);
                geoMax.y = std::max(geoMax.y, verts[i].y);
                geoMax.z = std::max(geoMax.z, verts[i].z);
            }
        }

        PhysicsDat bestDat;
        std::string bestPath;
        float bestScore = 1e30f;
        bool  bestFound = false;

        for (const auto& path : candidates)
        {
            PhysicsDat dat;

            if (!LoadDatFile(path, dat)) continue;

            if (vertexCount > 0 && dat.count != vertexCount) continue;

            float score = 0.0f;

            if (hasGeoBbox && dat.hasBbox)
            {
                // 包围盒差异：尺寸 + 中心
                const float gs[3] = { geoMax.x - geoMin.x, geoMax.y - geoMin.y, geoMax.z - geoMin.z };
                const float ds[3] = { dat.bboxMax.x - dat.bboxMin.x,
                                      dat.bboxMax.y - dat.bboxMin.y,
                                      dat.bboxMax.z - dat.bboxMin.z };

                const float gc[3] = { (geoMax.x + geoMin.x) * 0.5f,
                                      (geoMax.y + geoMin.y) * 0.5f,
                                      (geoMax.z + geoMin.z) * 0.5f };

                const float dc[3] = { (dat.bboxMax.x + dat.bboxMin.x) * 0.5f,
                                      (dat.bboxMax.y + dat.bboxMin.y) * 0.5f,
                                      (dat.bboxMax.z + dat.bboxMin.z) * 0.5f };

                for (int a = 0; a < 3; ++a)
                {
                    score += std::fabs(gs[a] - ds[a]);
                    score += std::fabs(gc[a] - dc[a]);
                }
            }

            if (!bestFound || score < bestScore)
            {
                bestFound = true;
                bestScore = score;
                bestDat = dat;
                bestPath = path;
            }
        }

        if (!bestFound)
        {
            // 没有顶点数完全一致的 dat：退而求其次，
            // 取"顶点数最接近"的那份，交给 ApplyDatColors 做坐标匹配
            // （车辆等模型：导出/导入时顶点会被拆分或焊接，数量常常不一致）
            int bestDelta = 0x7fffffff;

            for (const auto& path : candidates)
            {
                PhysicsDat dat;

                if (!LoadDatFile(path, dat)) continue;

                // 顶点数差太多就不是同一个部件了（例如拿 897 车身的 dat 去套 96 顶点的车灯），
                // 只接受两者比例 >= 45% 的候选
                const int lo = std::min(dat.count, vertexCount);
                const int hi = std::max(dat.count, vertexCount);

                if (hi <= 0) continue;
                if (static_cast<float>(lo) / static_cast<float>(hi) < 0.45f) continue;

                const int delta = std::abs(dat.count - vertexCount);

                if (delta < bestDelta)
                {
                    bestDelta = delta;
                    bestDat = dat;
                    bestPath = path;
                    bestFound = true;
                }
            }

            if (!bestFound)
            {
                std::ostringstream oss;
                oss << "[Dat] no usable dat modelId=" << modelId
                    << " verts=" << vertexCount << " (dir: " << subDir << ")";
                ModLog(oss.str());
                return nullptr;
            }

            std::ostringstream oss;
            oss << "[Dat] no exact count match modelId=" << modelId
                << " verts=" << vertexCount << " use " << bestPath
                << " (dat=" << bestDat.count << ") -> 走坐标匹配";
            ModLog(oss.str());

            g_physicsDatCache[key] = bestDat;
            return &g_physicsDatCache[key];
        }

        g_physicsDatCache[key] = bestDat;

        {
            std::ostringstream oss;
            oss << "[Dat] loaded " << bestPath << " vertices=" << bestDat.count;

            if (hasGeoBbox && bestDat.hasBbox)
                oss << " bboxScore=" << bestScore;

            ModLog(oss.str());
        }

        return &g_physicsDatCache[key];
    }


    // 把 .dat 颜色写入克隆；返回保存的视觉颜色（用于初始化后还原）
    // DFF 会在 UV 缝/硬边拆分顶点，导致 .dat 与几何体顶点数不同。
    // 这里按"位置最近邻 + 自动轴/缩放标定"把 .dat 颜色映射到几何体顶点。
    bool MatchDatByPosition(const PhysicsDat& dat, RpGeometry* clone,
                            std::vector<RwRGBA>& outColors)
    {
        RwV3d* verts = RpMorphTargetGetVertices(&clone->morphTarget[0]);

        if (!verts || dat.positions.empty()) return false;

        const int vertCount = static_cast<int>(clone->numVertices);
        const int datCount = static_cast<int>(dat.positions.size());

        if (datCount <= 0 || vertCount <= 0) return false;

        auto bounds = [](const RwV3d* pts, int n, RwV3d& mn, RwV3d& mx, RwV3d& c)
        {
            mn = { 1e9f, 1e9f, 1e9f };
            mx = { -1e9f, -1e9f, -1e9f };
            c = { 0.0f, 0.0f, 0.0f };

            for (int i = 0; i < n; ++i)
            {
                const RwV3d& p = pts[i];
                mn.x = std::min(mn.x, p.x); mn.y = std::min(mn.y, p.y); mn.z = std::min(mn.z, p.z);
                mx.x = std::max(mx.x, p.x); mx.y = std::max(mx.y, p.y); mx.z = std::max(mx.z, p.z);
                c.x += p.x; c.y += p.y; c.z += p.z;
            }

            c.x /= n; c.y /= n; c.z /= n;
        };

        RwV3d datMin, datMax, datCenter;
        RwV3d geoMin, geoMax, geoCenter;

        bounds(dat.positions.data(), datCount, datMin, datMax, datCenter);
        bounds(verts, vertCount, geoMin, geoMax, geoCenter);

        struct AxisMap { int ax[3]; float sign[3]; };

        static const AxisMap kMaps[] =
        {
            {{0,1,2},{ 1, 1, 1}}, {{0,2,1},{ 1, 1, 1}}, {{1,0,2},{ 1, 1, 1}},
            {{1,2,0},{ 1, 1, 1}}, {{2,0,1},{ 1, 1, 1}}, {{2,1,0},{ 1, 1, 1}},
            {{0,1,2},{ 1, 1,-1}}, {{0,1,2},{ 1,-1, 1}}, {{0,1,2},{ 1,-1,-1}},
            {{0,1,2},{-1, 1, 1}}, {{0,1,2},{-1, 1,-1}}, {{0,1,2},{-1,-1, 1}},
            {{0,1,2},{-1,-1,-1}}, {{0,2,1},{ 1, 1,-1}}, {{0,2,1},{ 1,-1, 1}},
            {{0,2,1},{-1,-1,-1}}, {{1,2,0},{ 1, 1,-1}}, {{1,2,0},{-1,-1,-1}},
            {{2,1,0},{ 1, 1,-1}}, {{2,1,0},{-1,-1,-1}},
        };

        float bestScore = 1e30f;
        int   bestMap = -1;
        float bestScale[3] = { 1.0f, 1.0f, 1.0f };
        RwV3d bestShift = { 0.0f, 0.0f, 0.0f };

        const int sample = std::min(datCount, 64);

        for (int m = 0; m < static_cast<int>(sizeof(kMaps) / sizeof(kMaps[0])); ++m)
        {
            const AxisMap& map = kMaps[m];

            float axisScale[3];

            for (int a = 0; a < 3; ++a)
            {
                const int src = map.ax[a];
                const float d = (src == 0 ? datMax.x - datMin.x : src == 1 ? datMax.y - datMin.y : datMax.z - datMin.z);
                const float g = (a == 0 ? geoMax.x - geoMin.x : a == 1 ? geoMax.y - geoMin.y : geoMax.z - geoMin.z);
                axisScale[a] = (d > 1e-6f) ? (g / d) : 1.0f;
            }

            auto mapPoint = [&](const RwV3d& p) -> RwV3d
            {
                const float src[3] = { p.x, p.y, p.z };

                const float c0 = (map.ax[0] == 0 ? datCenter.x : map.ax[0] == 1 ? datCenter.y : datCenter.z);
                const float c1 = (map.ax[1] == 0 ? datCenter.x : map.ax[1] == 1 ? datCenter.y : datCenter.z);
                const float c2 = (map.ax[2] == 0 ? datCenter.x : map.ax[2] == 1 ? datCenter.y : datCenter.z);

                RwV3d r;
                r.x = (src[map.ax[0]] - c0) * map.sign[0] * axisScale[0] + geoCenter.x;
                r.y = (src[map.ax[1]] - c1) * map.sign[1] * axisScale[1] + geoCenter.y;
                r.z = (src[map.ax[2]] - c2) * map.sign[2] * axisScale[2] + geoCenter.z;

                return r;
            };

            float score = 0.0f;

            for (int s = 0; s < sample; ++s)
            {
                const RwV3d mp = mapPoint(dat.positions[(s * datCount) / sample]);

                float bestDist = 1e30f;

                for (int v = 0; v < vertCount; ++v)
                {
                    const float dx = verts[v].x - mp.x;
                    const float dy = verts[v].y - mp.y;
                    const float dz = verts[v].z - mp.z;
                    const float d = dx * dx + dy * dy + dz * dz;

                    if (d < bestDist) bestDist = d;
                }

                score += bestDist;
            }

            if (score < bestScore)
            {
                bestScore = score;
                bestMap = m;

                for (int a = 0; a < 3; ++a) bestScale[a] = axisScale[a];

                bestShift = { 0.0f, 0.0f, 0.0f };
            }
        }

        if (bestMap < 0) return false;

        std::ostringstream sc0;
        sc0 << "[Dat] best mapping=" << bestMap << " hits=" << bestScore
            << "/" << sample << " scale=(" << bestScale[0] << ","
            << bestScale[1] << "," << bestScale[2] << ")";
        ModLog(sc0.str());

        const AxisMap& map = kMaps[bestMap];

        auto mapPointShifted = [&](const RwV3d& p, const RwV3d& shift) -> RwV3d
        {
            const float src[3] = { p.x, p.y, p.z };

            const float c0 = (map.ax[0] == 0 ? datCenter.x : map.ax[0] == 1 ? datCenter.y : datCenter.z);
            const float c1 = (map.ax[1] == 0 ? datCenter.x : map.ax[1] == 1 ? datCenter.y : datCenter.z);
            const float c2 = (map.ax[2] == 0 ? datCenter.x : map.ax[2] == 1 ? datCenter.y : datCenter.z);

            RwV3d r;
            r.x = (src[map.ax[0]] - c0) * map.sign[0] * bestScale[0] + geoCenter.x + shift.x;
            r.y = (src[map.ax[1]] - c1) * map.sign[1] * bestScale[1] + geoCenter.y + shift.y;
            r.z = (src[map.ax[2]] - c2) * map.sign[2] * bestScale[2] + geoCenter.z + shift.z;

            return r;
        };

        // ICP-lite：迭代修正整体平移
        for (int iter = 0; iter < 4; ++iter)
        {
            RwV3d sum = { 0.0f, 0.0f, 0.0f };
            int   cnt = 0;

            for (int i = 0; i < datCount; i += std::max(1, datCount / 128))
            {
                const RwV3d mp = mapPointShifted(dat.positions[i], bestShift);

                float bestDist = 1e30f;
                int   bestV = -1;

                for (int v = 0; v < vertCount; ++v)
                {
                    const float dx = verts[v].x - mp.x;
                    const float dy = verts[v].y - mp.y;
                    const float dz = verts[v].z - mp.z;
                    const float d = dx * dx + dy * dy + dz * dz;

                    if (d < bestDist) { bestDist = d; bestV = v; }
                }

                if (bestV >= 0)
                {
                    sum.x += verts[bestV].x - mp.x;
                    sum.y += verts[bestV].y - mp.y;
                    sum.z += verts[bestV].z - mp.z;
                    ++cnt;
                }
            }

            if (!cnt) break;

            bestShift.x += (sum.x / cnt) * 0.8f;
            bestShift.y += (sum.y / cnt) * 0.8f;
            bestShift.z += (sum.z / cnt) * 0.8f;
        }

        const float geoSize = std::max({ geoMax.x - geoMin.x, geoMax.y - geoMin.y, geoMax.z - geoMin.z });
        const float tol = std::max(0.02f, 0.03f * geoSize);

        outColors.assign(vertCount, RwRGBA{ 255, 255, 255, 255 });

        outColors.assign(vertCount, RwRGBA{ 255, 255, 255, 255 });

        // 正向匹配：每个几何顶点找最近的 .dat 顶点。
        // （DFF 在 UV 缝拆出的重复顶点会各自找到同一个 .dat 点，
        //   这样同位置的顶点拿到同一个类型，不会出现拉伸）
        int matched = 0;
        int markedMatched = 0;

        for (int v = 0; v < vertCount; ++v)
        {
            float bestDist = tol * tol;
            int   bestIdx = -1;

            for (int i = 0; i < datCount; ++i)
            {
                const RwV3d mp = mapPointShifted(dat.positions[i], bestShift);

                const float dx = verts[v].x - mp.x;
                const float dy = verts[v].y - mp.y;
                const float dz = verts[v].z - mp.z;
                const float d = dx * dx + dy * dy + dz * dz;

                if (d <= bestDist) { bestDist = d; bestIdx = i; }
            }

            if (bestIdx < 0) continue;

            ++matched;

            outColors[v] = dat.colors[bestIdx];

            if (!(dat.colors[bestIdx].red == 255 &&
                  dat.colors[bestIdx].green == 255 &&
                  dat.colors[bestIdx].blue == 255))
                ++markedMatched;
        }

        std::ostringstream oss;
        oss << "[Dat] match: matched=" << matched << "/" << vertCount
            << " marked=" << markedMatched
            << " map=" << bestMap << " tol=" << tol
            << " shift=(" << bestShift.x << "," << bestShift.y << "," << bestShift.z << ")";
        ModLog(oss.str());

        return markedMatched > 0;
    }

    bool ApplyDatColors(RpGeometry* clone, int modelId, std::vector<RwRGBA>& savedVisual,
                        bool& allocatedPrelit)
    {
        allocatedPrelit = false;

        if (!clone) return false;

        const int count = static_cast<int>(clone->numVertices);

        const RwV3d* datVerts = RpMorphTargetGetVertices(&clone->morphTarget[0]);

        const PhysicsDat* dat = GetPhysicsDat(modelId, count, datVerts);

        if (!dat) return false;

        // 几何体没有 prelit 数组时（DFF 里顶点色被清空/未导出），
        // 临时分配一份用来承载 .dat 数据，初始化完再释放。
        if (!clone->preLitLum)
        {
            clone->preLitLum = static_cast<RwRGBA*>(
                RwMalloc(static_cast<RwUInt32>(sizeof(RwRGBA) * count),
                         rwMEMHINTDUR_EVENT | rwMEMHINTFLAG_RESIZABLE));

            if (!clone->preLitLum) return false;

            for (int i = 0; i < count; ++i)
                clone->preLitLum[i] = RwRGBA{ 255, 255, 255, 255 };

            clone->flags |= rpGEOMETRYPRELIT;
            allocatedPrelit = true;

            ModLog("[Dat] geometry has no prelit; allocated temporary one");
        }

        std::vector<RwRGBA> mapped;

        if (dat->count != count)
        {
            {
                std::ostringstream oss;
                oss << "[Dat] vertex count mismatch modelId=" << modelId
                    << " dat=" << dat->count << " geometry=" << count
                    << "  -> 请用 Blender 插件的「按 DFF 顶点顺序导出」重新生成 dat"
                       "（顶点数必须与游戏几何体一致）";
                ModLog(oss.str());
            }

            if (!MatchDatByPosition(*dat, clone, mapped))
            {
                std::ostringstream oss;
                oss << "[Dat] position matching also failed modelId=" << modelId
                    << "  -> 物理未启用，请按上面的方法重新导出 dat";
                ModLog(oss.str());
                return false;
            }

            ModLog("[Dat] 使用坐标匹配（备选方案，可能不准）；"
                   "建议用「按 DFF 顶点顺序导出」重新生成 dat");
        }

        savedVisual.assign(clone->preLitLum, clone->preLitLum + count);

        for (int i = 0; i < count; ++i)
            clone->preLitLum[i] = mapped.empty() ? dat->colors[i] : mapped[i];

        return true;
    }

    void RestoreVisualColors(RpGeometry* clone, const std::vector<RwRGBA>& saved, bool allocatedPrelit)
    {
        if (!clone) return;

        if (allocatedPrelit)
        {
            if (clone->preLitLum)
            {
                RwFree(clone->preLitLum);
                clone->preLitLum = nullptr;
            }

            clone->flags &= ~rpGEOMETRYPRELIT;
            return;
        }

        if (!clone->preLitLum || saved.empty()) return;

        const int count = static_cast<int>(clone->numVertices);
        const int n = std::min(count, static_cast<int>(saved.size()));

        for (int i = 0; i < n; ++i)
            clone->preLitLum[i] = saved[i];
    }

    void ModLog(const std::string& text)
    {
        char exePath[MAX_PATH] = {};
        DWORD length = GetModuleFileNameA(nullptr, exePath, MAX_PATH);

        std::string path;

        if (length > 0 && length < MAX_PATH)
        {
            std::string tmp(exePath, length);
            size_t pos = tmp.find_last_of("\\/");
            if (pos != std::string::npos)
                path = tmp.substr(0, pos + 1);
        }

        path += "vertexphysics.log";

        // 日志精简：文件超过 4 MB 就截断重写
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
            f << text << std::endl;
    }

    std::string GetExeDirectory()
    {
        char exePath[MAX_PATH] = {};
        DWORD length = GetModuleFileNameA(nullptr, exePath, MAX_PATH);

        if (length == 0 || length >= MAX_PATH) return std::string();

        std::string result(exePath, length);
        size_t pos = result.find_last_of("\\/");

        if (pos == std::string::npos) return std::string();

        result.resize(pos + 1);
        return result;
    }

    // ------------------------------------------------------------
    // 配置：vertexphysics.ini（兼容旧的 hairphysics.ini）
    //
    //   每行一个模型名（不区分大小写）；* 表示所有模型
    //   也支持 key=value：
    //     worldRadius=2.5      跨模型碰撞收集半径（米）
    //     interactRadius=8     地图物件"逐实例物理"的交互半径（米）
    //     maxInstances=8       同时逐实例模拟的地图物件上限
    //     wind=1.0             模型空间摇摆的强度（0 = 关闭）
    //     mapPhysics=1         是否启用地图物件的模型空间物理
    // ------------------------------------------------------------

    float g_worldRadius = 2.5f;
    float g_interactRadius = 8.0f;
    int   g_maxInstances = 8;
    float g_windStrength = 1.0f;
bool  g_datOnly = true;      // true = 只读外部 .dat，忽略模型自带顶点色
    bool  g_mapPhysics = false;
    bool  g_mapConvert = false;   // 逐实例渲染开关
    bool  g_dumpGeometry = false; // 把白名单模型的几何体导出成 dump（供插件生成精确 dat）
    int   g_mapConvertRaw = 0;    // ini 里的原始数值（2 = 不换回，诊断用）

    std::string ToLower(std::string s)
    {
        std::transform(s.begin(), s.end(), s.begin(),
            [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
        return s;
    }

    // 解析 "角度,强度,湍流频率" 三个数
    bool ParseWindTriple(const std::string& value,
                        float& dirDeg, float& strength, float& turb)
    {
        float v[3] = { 0.0f, 0.0f, 0.0f };
        int n = 0;
        size_t pos = 0;

        while (n < 3 && pos <= value.size())
        {
            const size_t comma = value.find(',', pos);

            std::string part = value.substr(pos,
                (comma == std::string::npos) ? std::string::npos : comma - pos);

            // 去掉两端空白（此处不能用下面的 Trim，它定义在本函数之后）
            const size_t b = part.find_first_not_of(" \t\r\n");
            const size_t e = part.find_last_not_of(" \t\r\n");

            part = (b == std::string::npos) ? std::string() : part.substr(b, e - b + 1);

            if (!part.empty()) v[n++] = static_cast<float>(atof(part.c_str()));

            if (comma == std::string::npos) break;
            pos = comma + 1;
        }

        if (n == 0) return false;

        dirDeg   = v[0];
        strength = v[1];
        turb     = v[2];
        return true;
    }

    std::string Trim(const std::string& s)
    {
        const char* ws = " \t\r\n";
        const size_t begin = s.find_first_not_of(ws);
        if (begin == std::string::npos) return std::string();
        const size_t end = s.find_last_not_of(ws);
        return s.substr(begin, end - begin + 1);
    }

    void LoadConfig()
    {
        g_configLoaded = true;

        std::string iniPath = GetExeDirectory() + "vertexphysics.ini";
        std::ifstream ini(iniPath.c_str());

        if (!ini.is_open())
        {
            // 向后兼容：旧的 hairphysics.ini
            iniPath = GetExeDirectory() + "hairphysics.ini";
            ini.open(iniPath.c_str());
        }

        if (!ini.is_open())
        {
            ModLog("[Config] vertexphysics.ini not found -> report-only mode "
                   "(no physics will be applied)");
            return;
        }

        {
            std::ostringstream oss;
            oss << "[Config] loading " << iniPath;
            ModLog(oss.str());
        }

        std::string line;

        while (std::getline(ini, line))
        {
            line = Trim(line);
            if (line.empty() || line[0] == '#' || line[0] == ';') continue;

            const size_t eq = line.find('=');

            if (eq != std::string::npos)
            {
                const std::string key = ToLower(Trim(line.substr(0, eq)));
                const std::string value = Trim(line.substr(eq + 1));

                if (key == "worldradius")
                    g_worldRadius = std::clamp(static_cast<float>(atof(value.c_str())), 0.5f, 20.0f);
                else if (key == "interactradius")
                    g_interactRadius = std::clamp(static_cast<float>(atof(value.c_str())), 1.0f, 50.0f);
                else if (key == "maxinstances")
                    g_maxInstances = std::clamp(atoi(value.c_str()), 0, 64);
                else if (key == "wind")
                    g_windStrength = std::clamp(static_cast<float>(atof(value.c_str())), 0.0f, 5.0f);
                else if (key == "datonly")
                    g_datOnly = (atoi(value.c_str()) != 0);
                else if (key.rfind("wind", 0) == 0 && key.size() > 4)
                {
                    // wind0=角度,强度,湍流频率   或   winddefault=...
                    float dirDeg = 0.0f, strength = 0.0f, turb = 0.0f;

                    if (ParseWindTriple(value, dirDeg, strength, turb))
                    {
                        const std::string suffix = key.substr(4);

                        if (suffix == "default")
                            SetDefaultWeatherWind(dirDeg, strength, turb);
                        else
                            SetWeatherWind(atoi(suffix.c_str()), dirDeg, strength, turb);
                    }
                }
                else if (key == "dumpgeometry")
                    g_dumpGeometry = (atoi(value.c_str()) != 0);
                else if (key == "mapconvert")
                {
                    g_mapConvertRaw = atoi(value.c_str());
                    g_mapConvert = (g_mapConvertRaw != 0);
                }
                else if (key == "mapphysics")
                    g_mapPhysics = (atoi(value.c_str()) != 0);

                continue;
            }

            if (line == "*")
            {
                g_autoMode = true;
                continue;
            }

            g_whitelistNames.push_back(line);
            if (g_whitelistIds.find(line) == g_whitelistIds.end())
                g_whitelistIds[line] = -1;
        }

        std::ostringstream oss;
        oss << "[Config] autoMode=" << (g_autoMode ? "YES" : "NO")
            << " whitelist=" << g_whitelistNames.size()
            << " worldRadius=" << g_worldRadius
            << " interactRadius=" << g_interactRadius
            << " maxInstances=" << g_maxInstances
            << " wind=" << g_windStrength
            << " datOnly=" << (g_datOnly ? "ON" : "OFF")
            << " mapPhysics=" << (g_mapPhysics ? "ON" : "OFF")
            << " mapConvert=" << (g_mapConvert ? "ON" : "OFF")
            << " dumpGeometry=" << (g_dumpGeometry ? "ON" : "OFF");
        ModLog(oss.str());

        // 风力倍率（ini 的 wind）作用到天气风系统
        SetWindMultiplier(g_windStrength);
    }

    // ============================================================
    // 几何体 dump：把白名单模型的游戏几何体写到
    //   <exe>/vertexphysics/dump/<模型名>_<顶点数>.txt
    // 供 Blender 插件生成"索引精确"的 dat（顶点数一定一致）
    // ============================================================
    void DumpGeometry(RpGeometry* geometry, int modelId)
    {
        if (!g_dumpGeometry || !geometry) return;

        RwV3d* verts = RpMorphTargetGetVertices(&geometry->morphTarget[0]);
        if (!verts) return;

        const int count = static_cast<int>(geometry->numVertices);
        if (count <= 0) return;

        std::string name;

        for (const auto& entry : g_whitelistIds)
        {
            if (entry.second == modelId) { name = entry.first; break; }
        }

        if (name.empty()) return;

        const std::string dir = GetDatDirectory() + "dump\\";
        CreateDirectoryA(dir.c_str(), nullptr);

        char fileName[MAX_PATH] = {};
        _snprintf_s(fileName, sizeof(fileName), _TRUNCATE,
                    "%s_%d.txt", name.c_str(), count);

        const std::string path = dir + fileName;

        if (GetFileAttributesA(path.c_str()) != INVALID_FILE_ATTRIBUTES)
            return;   // 已存在

        std::ofstream out(path.c_str(), std::ios::out | std::ios::trunc);
        if (!out.is_open()) return;

        out << "# VertexPhysics geometry dump\n";
        out << "# model: " << name << "\n";
        out << "# verts: " << count << "\n";
        out << "# index x y z\n";

        for (int i = 0; i < count; ++i)
        {
            out << i << " " << verts[i].x << " " << verts[i].y << " " << verts[i].z << "\n";
        }

        std::ostringstream oss;
        oss << "[Dump] wrote " << path << " verts=" << count;
        ModLog(oss.str());
    }

    // 模型名 -> 模型 ID（模型要已经被游戏注册过）
    void ResolveWhitelist()
    {
        for (auto& entry : g_whitelistIds)
        {
            if (entry.second >= 0) continue;

            int id = -1;

            if (CModelInfo::GetModelInfo(entry.first.c_str(), &id) && id >= 0)
            {
                entry.second = id;

                std::ostringstream oss;
                oss << "[Config] resolved " << entry.first << " -> modelId=" << id;
                ModLog(oss.str());
            }
        }

        ProbeWhitelistedModels();
    }

    // 不依赖"玩家是否在旁边"的探测：直接检查白名单模型的模型数据。
    // 模型需要先被流式加载，所以每 300 帧重试，直到拿到 atomic 为止。
    void ProbeWhitelistedModels()
    {
        static std::set<int> s_done;
        static int s_timer = 0;
        static int s_attempt = 0;

        if (s_timer > 0 && ++s_timer < 300) return;
        s_timer = 1;

        for (const auto& entry : g_whitelistIds)
        {
            if (entry.second < 0) return;   // 还没全部解析完
        }

        ++s_attempt;

        for (const auto& entry : g_whitelistIds)
        {
            const int id = entry.second;

            if (s_done.find(id) != s_done.end()) continue;

            std::ostringstream oss;
            oss << "[MapPhysics][Probe] " << entry.first << " modelId=" << id;

            CBaseModelInfo* info = CModelInfo::GetModelInfo(id);

            if (!info)
            {
                if (s_attempt >= 4) { oss << " -> model info NULL"; ModLog(oss.str()); s_done.insert(id); }
                continue;
            }

            oss << " type=" << static_cast<int>(info->GetModelType());

            if (info->GetModelType() != MODEL_INFO_ATOMIC)
            {
                oss << " (not atomic; map physics needs MODEL_INFO_ATOMIC)";
                ModLog(oss.str());
                s_done.insert(id);
                continue;
            }

            CAtomicModelInfo* atomicInfo = reinterpret_cast<CAtomicModelInfo*>(info);
            RpAtomic* modelAtomic = atomicInfo->GetAtomicFromDistance(1.0f);

            if (!modelAtomic)
            {
                if (s_attempt >= 4)
                {
                    oss << " atomic=NULL (still not streamed after 4 tries)";
                    ModLog(oss.str());
                    s_done.insert(id);
                }
                continue;
            }

            RpGeometry* geometry = RpAtomicGetGeometry(modelAtomic);

            if (!geometry)
            {
                oss << " geometry=NULL";
                ModLog(oss.str());
                continue;
            }

            oss << " geometry=" << geometry
                << " verts=" << geometry->numVertices
                << " tris=" << geometry->numTriangles
                << " prelit=" << (geometry->preLitLum ? "YES" : "NO")
                << " markers=" << GetMarkerCountCached(geometry);

            ModLog(oss.str());
            s_done.insert(id);
        }
    }

    bool IsModelEnabled(int modelIndex)
    {
        if (g_autoMode) return true;

        for (const auto& entry : g_whitelistIds)
        {
            if (entry.second == modelIndex) return true;
        }

        return false;
    }

    // ------------------------------------------------------------
    // 私有几何体克隆
    // ------------------------------------------------------------

    bool CopySkinToClone(RpGeometry* source, RpGeometry* clone)
    {
        RpSkin* sourceSkin = RpSkinGeometryGetSkin(source);
        if (!sourceSkin) return true;

        if (RpSkinIsSplit(sourceSkin)) return true;

        const RwUInt32 boneCount = RpSkinGetNumBones(sourceSkin);
        const RwUInt32 vertexCount = static_cast<RwUInt32>(source->numVertices);

        const RwMatrixWeights* sourceWeights = RpSkinGetVertexBoneWeights(sourceSkin);
        const RwUInt32*        sourceIndices = RpSkinGetVertexBoneIndices(sourceSkin);
        const RwMatrix*        sourceMatrices = RpSkinGetSkinToBoneMatrices(sourceSkin);

        if (!sourceWeights || !sourceIndices || !sourceMatrices || boneCount == 0)
            return true;

        std::vector<RwMatrixWeights> weights(static_cast<size_t>(vertexCount));
        std::vector<RwUInt32> indices(static_cast<size_t>(vertexCount) * 4);
        std::vector<RwMatrix> matrices(static_cast<size_t>(boneCount));

        std::memcpy(weights.data(), sourceWeights,
            sizeof(RwMatrixWeights) * static_cast<size_t>(vertexCount));
        std::memcpy(indices.data(), sourceIndices,
            sizeof(RwUInt32) * static_cast<size_t>(vertexCount) * 4);
        std::memcpy(matrices.data(), sourceMatrices,
            sizeof(RwMatrix) * static_cast<size_t>(boneCount));

        RpSkin* skin = RpSkinCreate(vertexCount, boneCount,
            weights.data(), indices.data(), matrices.data());

        if (!skin) return false;

        if (!RpSkinGeometrySetSkin(clone, skin) ||
            RpSkinGeometryGetSkin(clone) != skin)
        {
            RpSkinDestroy(skin);
            return false;
        }

        return true;
    }

    RpGeometry* CreatePrivateClone(RpGeometry* source)
    {
        if (!source ||
            source->numVertices <= 0 ||
            source->numMorphTargets != 1 ||
            !source->morphTarget ||
            !source->morphTarget[0].verts)
            return nullptr;

        RwUInt32 flags = source->flags & ~0x03000000u;   // 去掉 native / nativeInstance

        RpGeometry* clone = RpGeometryCreate(source->numVertices, source->numTriangles, flags);
        if (!clone) return nullptr;

        if (!_rpMaterialListCopy(&clone->matList, &source->matList))
        {
            RpGeometryDestroy(clone);
            return nullptr;
        }

        if (!RpGeometryLock(clone, rpGEOMETRYLOCKALL))
        {
            RpGeometryDestroy(clone);
            return nullptr;
        }

        const size_t vertexCount = static_cast<size_t>(source->numVertices);
        const size_t triangleCount = static_cast<size_t>(source->numTriangles);

        if (triangleCount > 0 && source->triangles && clone->triangles)
            std::memcpy(clone->triangles, source->triangles, sizeof(RpTriangle) * triangleCount);

        std::memcpy(clone->morphTarget[0].verts, source->morphTarget[0].verts,
            sizeof(RwV3d) * vertexCount);

        if ((source->flags & rpGEOMETRYNORMALS) &&
            source->morphTarget[0].normals && clone->morphTarget[0].normals)
        {
            std::memcpy(clone->morphTarget[0].normals, source->morphTarget[0].normals,
                sizeof(RwV3d) * vertexCount);
        }

        if ((source->flags & rpGEOMETRYPRELIT) && source->preLitLum && clone->preLitLum)
            std::memcpy(clone->preLitLum, source->preLitLum, sizeof(RwRGBA) * vertexCount);

        for (RwInt32 i = 0; i < source->numTexCoordSets; ++i)
        {
            if (source->texCoords[i] && clone->texCoords[i])
                std::memcpy(clone->texCoords[i], source->texCoords[i],
                    sizeof(RwTexCoords) * vertexCount);
        }

        clone->morphTarget[0].boundingSphere = source->morphTarget[0].boundingSphere;

        RpGeometryUnlock(clone);

        if (!CopySkinToClone(source, clone))
        {
            RpGeometryDestroy(clone);
            return nullptr;
        }

        return clone;
    }

    // ------------------------------------------------------------
    // 顶点色标记检测（按几何体缓存）
    // ------------------------------------------------------------

    int CountPhysicsMarkers(RpGeometry* geometry)
    {
        if (!geometry || !geometry->preLitLum) return 0;

        int count = 0;

        for (RwInt32 i = 0; i < geometry->numVertices; ++i)
        {
            VertexPhysParams params;
            if (ClassifyVertexColor(geometry->preLitLum[i], params))
                ++count;
        }

        return count;
    }

    int GetMarkerCountCached(RpGeometry* geometry)
    {
        if (!geometry) return 0;

        auto it = g_markerCache.find(geometry);
        if (it != g_markerCache.end()) return it->second;

        const int count = CountPhysicsMarkers(geometry);
        g_markerCache[geometry] = static_cast<char>(std::min(count, 127));
        return count;
    }

    // ------------------------------------------------------------
    // 实体收集
    // ------------------------------------------------------------

    void CollectFromEntity(CEntity* entity, std::vector<Candidate>& out)
    {
        if (!entity || !entity->m_pRwObject) return;

        RwObject* object = entity->m_pRwObject;

        if (RwObjectGetType(object) == rpCLUMP)
        {
            RpClump* clump = reinterpret_cast<RpClump*>(object);

            RpClumpForAllAtomics(clump,
                [](RpAtomic* atomic, void* data) -> RpAtomic*
                {
                    auto* list = static_cast<std::vector<Candidate>*>(data);
                    if (atomic)
                    {
                        Candidate c;
                        c.atomic = atomic;
                        list->push_back(c);
                    }
                    return atomic;
                },
                &out);
        }
        else if (RwObjectGetType(object) == rpATOMIC)
        {
            Candidate c;
            c.atomic = reinterpret_cast<RpAtomic*>(object);
            out.push_back(c);
        }
    }

    template <typename PoolType>
    void CollectFromPool(PoolType* pool, std::vector<Candidate>& out)
    {
        if (!pool) return;

        for (int i = 0; i < pool->m_nSize; ++i)
        {
            auto* entity = pool->GetAt(i);
            if (!entity) continue;

            const size_t before = out.size();
            CollectFromEntity(entity, out);

            for (size_t k = before; k < out.size(); ++k)
                out[k].modelIndex = entity->m_nModelIndex;
        }
    }

    const RwMatrix* GetAnchorLTM(RpAtomic* atomic)
    {
        RpClump* clump = RpAtomicGetClumpMacro(atomic);

        if (clump)
        {
            RwFrame* frame = RpClumpGetFrame(clump);
            if (frame) return RwFrameGetLTM(frame);
        }

        RwFrame* frame = RpAtomicGetFrame(atomic);
        return frame ? RwFrameGetLTM(frame) : nullptr;
    }

} // namespace


void UpdateEntityPhysics(float deltaTime)
{
    if (!g_configLoaded)
        LoadConfig();

    // 风的时间推进（每帧一次，所有 rig 共用）
    AdvanceWindTime(deltaTime);

    ProbeWhitelistedModels();

    // 挂上跨模型碰撞回调（头发 / 布料 会调用它）
    g_worldCollideFn = &WorldCollideThunk;

    // 地图物件逐实例物理：用玩家位置驱动
    // 避免"玩家没有物理头发时地图物件也不动"的问题
    {
        CPlayerPed* player = FindPlayerPed(-1);

        if (player)
        {
            const CVector& playerPos = player->GetPosition();

            RwV3d center = { playerPos.x, playerPos.y, playerPos.z };

            if (g_mapConvert)
                UpdateMapInstances(center, deltaTime);
            else
                UpdateMapPhysics(center, deltaTime);
        }
    }

    ++g_frameCounter;

    if (--g_scanCountdown <= 0)
    {
        g_scanCountdown = 30;

        ResolveWhitelist();

        g_candidates.clear();
        CollectFromPool(CPools::ms_pPedPool, g_candidates);
        CollectFromPool(CPools::ms_pVehiclePool, g_candidates);
        CollectFromPool(CPools::ms_pObjectPool, g_candidates);

        for (auto it = g_entries.begin(); it != g_entries.end();)
        {
            const bool alive = std::any_of(g_candidates.begin(), g_candidates.end(),
                [&](const Candidate& c) { return c.atomic == it->first; });

            if (!alive) it = g_entries.erase(it);
            else        ++it;
        }

        if (g_frameCounter <= 120 || (g_frameCounter % 600) == 0)
        {
            std::ostringstream oss;
            oss << "[EntityScan] frame=" << g_frameCounter
                << " candidates=" << g_candidates.size()
                << " rigs=" << g_entries.size()
                << " autoMode=" << (g_autoMode ? "YES" : "NO");
            ModLog(oss.str());
        }
    }

    for (const Candidate& candidate : g_candidates)
    {
        RpAtomic* atomic = candidate.atomic;
        if (!atomic) continue;

        RpGeometry* current = RpAtomicGetGeometry(atomic);
        if (!current) continue;

        auto it = g_entries.find(atomic);

        if (it != g_entries.end())
        {
            if (it->second.clone == current)
            {
                GeometryPhysics& rig = it->second.rig;

                if (rig.initialized)
                {
                    // 收集附近其他实体的碰撞体（人物 / 车辆 / 地图物件）
                    const RwMatrix* ltm = GetAnchorLTM(atomic);
                    if (ltm)
                    {
                        WorldCollision_Collect(ltm->pos, g_worldRadius);

                        // 地图物件（模型空间风摆）以当前角色位置为中心扫描
                        if (g_mapConvert)
                            UpdateMapInstances(ltm->pos, deltaTime);
                        else
                            UpdateMapPhysics(ltm->pos, deltaTime);
                    }

                    StepGeometryPhysics(rig, atomic, deltaTime);
                    ApplyGeometryPhysics(rig, atomic);
                }
                continue;
            }

            // 几何体被游戏换回/换掉（车辆重流式加载、损伤模型切换等）：
            // 只要顶点数一致，就把我们的克隆重新挂上去，保留 rig 状态，
            // 而不是销毁重建（那样物理会被反复清零）。
            if (it->second.rig.initialized &&
                it->second.clone &&
                RpGeometryGetNumVertices(current) ==
                    RpGeometryGetNumVertices(it->second.clone))
            {
                RpAtomicSetGeometry(atomic, it->second.clone, 0);

                if (RpAtomicGetGeometry(atomic) == it->second.clone)
                {
                    static int s_reattachLogged = 0;

                    if (++s_reattachLogged <= 5)
                    {
                        std::ostringstream oss;
                        oss << "[Entity] reattach modelId=" << candidate.modelIndex
                            << " clone=" << it->second.clone;
                        ModLog(oss.str());
                    }

                    continue;
                }
            }

            {
                // 顶点数不一致才真正重建
                std::ostringstream oss;
                oss << "[Entity] relink modelId=" << candidate.modelIndex
                    << " oldClone=" << it->second.clone
                    << " current=" << current;
                ModLog(oss.str());
            }

            g_entries.erase(it);
        }

        // dump 与"是否有物理色/是否有 dat"无关：
        // 只要模型在白名单里、且 dumpGeometry=1，就把几何体导出一份，
        // 方便用户先拿到顶点顺序、再回 Blender 按该顺序导出 .dat。
        if (g_dumpGeometry && IsModelEnabled(candidate.modelIndex))
            DumpGeometry(current, candidate.modelIndex);

        const int markerCount = GetMarkerCountCached(current);
        const bool hasDat = (GetPhysicsDat(candidate.modelIndex, static_cast<int>(current->numVertices),
                                           RpMorphTargetGetVertices(&current->morphTarget[0])) != nullptr);

        // datOnly：只看外部 .dat，忽略模型自带顶点色
        if (g_datOnly)
        {
            if (!hasDat) continue;
        }
        else if (markerCount <= 0 && !hasDat)
        {
            continue;
        }

        // dat 存在但里面全是"无物理"的颜色（例如白色）：
        // 这种部件不需要物理，直接跳过，避免为每个部件建 rig、
        // 每帧做碰撞收集与克隆，导致车辆一出现就掉帧。
        if (hasDat)
        {
            const PhysicsDat* datInfo =
                GetPhysicsDat(candidate.modelIndex,
                              static_cast<int>(current->numVertices),
                              RpMorphTargetGetVertices(&current->morphTarget[0]));

            bool hasPhysicsColor = false;

            if (datInfo)
            {
                for (const auto& c : datInfo->colors)
                {
                    VertexPhysParams vp;
                    if (ClassifyVertexColor(c, vp) && vp.type != PhysType::Rigid)
                    {
                        hasPhysicsColor = true;
                        break;
                    }
                }
            }

            if (!hasPhysicsColor)
            {
                if (g_reportedModels.insert("nodata:" + std::to_string(candidate.modelIndex)).second)
                {
                    std::ostringstream oss;
                    oss << "[Entity] skip modelId=" << candidate.modelIndex
                        << " verts=" << current->numVertices
                        << " (dat has no physics colors)";
                    ModLog(oss.str());
                }

                continue;
            }
        }

        // 没有被白名单允许时只报告，不做任何改动
        if (!IsModelEnabled(candidate.modelIndex))
        {
            if (g_reportedModels.size() < 60 &&
                g_reportedModels.insert(std::to_string(candidate.modelIndex)).second)
            {
                std::ostringstream oss;
                oss << "[MarkerModel] modelId=" << candidate.modelIndex
                    << " markers=" << markerCount
                    << " verts=" << current->numVertices
                    << "  -> 把模型名加进 hairphysics.ini 才会启用";
                ModLog(oss.str());
            }
            continue;
        }

        if (static_cast<int>(g_entries.size()) >= kMaxRigs)
            continue;

        RpGeometry* clone = CreatePrivateClone(current);

        if (!clone)
        {
            std::ostringstream oss;
            oss << "[Entity] clone FAILED modelId=" << candidate.modelIndex
                << " verts=" << current->numVertices
                << " tris=" << current->numTriangles;
            ModLog(oss.str());
            continue;
        }

        if (!RpAtomicSetGeometry(atomic, clone, 0) ||
            RpAtomicGetGeometry(atomic) != clone)
        {
            RpGeometryDestroy(clone);
            continue;
        }

        const RwMatrix* anchorLTM = GetAnchorLTM(atomic);

        Entry entry;
        entry.clone = clone;

        std::string report;


        std::vector<RwRGBA> savedVisual;
        bool datAllocatedPrelit = false;
        const bool usedDat = ApplyDatColors(clone, candidate.modelIndex, savedVisual, datAllocatedPrelit);

        if (!InitGeometryPhysics(entry.rig, clone, anchorLTM, report))
        {
            RpAtomicSetGeometry(atomic, current, 0);
            RpGeometryDestroy(clone);

            std::ostringstream oss;
            oss << "[Entity] init FAILED modelId=" << candidate.modelIndex << " " << report;
            ModLog(oss.str());
            continue;
        }

        if (usedDat)
            RestoreVisualColors(clone, savedVisual, datAllocatedPrelit);

        AddDefaultColliders(entry.rig, nullptr);

        {
            std::ostringstream oss;
            oss << "[Entity] rig created modelId=" << candidate.modelIndex
                << " geometry=" << current
                << " clone=" << clone
                << " verts=" << clone->numVertices
                << " hair=" << entry.rig.hairCount
                << " cloth=" << entry.rig.clothCount
                << " body=" << entry.rig.bodyCount
                << " soft=" << entry.rig.softCount
                << " constraints=" << entry.rig.constraints.size()
                << " anchor=" << (entry.rig.hasSkinAnchor ? "skinned" : "rigid");
            ModLog(oss.str());
        }

        g_entries.emplace(atomic, std::move(entry));
    }
}


int GetEntityPhysicsRigCount()
{
    return static_cast<int>(g_entries.size());
}


void LogEntityPhysicsSummary()
{
    std::ostringstream oss;
    oss << "[EntitySummary] candidates=" << g_candidates.size()
        << " rigs=" << g_entries.size()
        << " knownGeometries=" << g_markerCache.size();
    ModLog(oss.str());
}
