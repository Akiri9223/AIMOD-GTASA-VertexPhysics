#include "EntityPhysics.h"
#include "GeometryPhysics.h"
#include "WorldCollision.h"

#include "common.h"
#include "CPools.h"
#include "CPed.h"
#include "CWeapon.h"
#include "CWeaponInfo.h"
#include "ePedBones.h"
#include "CVehicle.h"
#include "CObject.h"
#include "CModelInfo.h"
#include "CAtomicModelInfo.h"
#include "CBuilding.h"
#include "CSector.h"
#include "CPtrNodeSingleLink.h"
#include "CWorld.h"
#include "CText.h"
#include "CKeyGen.h"
#include "CPedClothesDesc.h"
#include "CStreaming.h"
#include "CRenderer.h"
#include "safetyhook.hpp"
#include "rpskin.h"
#include "rphanim.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iomanip>
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
        CEntity*  entity = nullptr;
    };

    std::map<RpAtomic*, Entry>   g_entries;
    std::map<RpGeometry*, char>  g_markerCache;    std::vector<Candidate>       g_candidates;

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
    extern float g_collideRadiusExtra;
    extern bool  g_mapPhysics;
    extern bool  g_mapCollision;
    extern bool  g_mapSelfCol;
    extern bool  g_mapSelfBox;
    extern bool  g_clothSweep;
    extern bool  g_selfCollide;
    extern bool  g_objSelfCol;
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

        // 布料迭代次数：6 次时整片旗面的重量会让顶边拉伸到 ~180%，
        // 10 次可压到 ~135%（离线网格仿真），代价约 +60% 步进时间。
        rig.iterations = 10;
        rig.modelId = modelId;

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

    // ============================================================
    //  [LoadGuard] 读档 / 玩家重建时的清理
    //
    //  2026-11-24 19:3x：物理更新挂在 drawingEvent 上，**加载画面也会触发**；
    //  读档时游戏会销毁并重建玩家 ped、重载模型、清空实体池，
    //  我们持有的 atomic / clone / CBuilding* 会变成悬空指针 ⇒ 崩溃/卡载入。
    //  这里在"玩家 ped 指针变化"时清空所有 rig，并跳过若干帧等世界稳定。
    //
    //  注意：这里**不销毁**我们的克隆几何体（atomic 可能仍被实体引用），
    //  只丢引用，避免二次释放；每次读档泄漏的量很小，安全优先。
    // ============================================================
    void ResetAllRigs(const char* reason)
    {
        const size_t entriesBefore = g_entries.size();
        const size_t instancesBefore = g_mapInstances.size();

        g_entries.clear();
        g_mapInstances.clear();
        g_pendingInstances.clear();
        g_candidates.clear();
        g_markerCache.clear();

        std::ostringstream oss;
        oss << "[LoadGuard] reset (" << reason << ")"
            << " entries=" << entriesBefore
            << " mapInst=" << instancesBefore;
        ModLog(oss.str());
    }

    static double NowMs()
    {
        static LARGE_INTEGER freq{};
        static bool init = false;

        if (!init) { QueryPerformanceFrequency(&freq); init = true; }

        LARGE_INTEGER now{};
        QueryPerformanceCounter(&now);

        return static_cast<double>(now.QuadPart) * 1000.0 / static_cast<double>(freq.QuadPart);
    }

    // perf 累加器（每 300 帧由 [Perf] 汇总后清零并换算成"每帧平均"）
    double g_perfCollectMs = 0.0;
    double g_perfStepMs = 0.0;
    double g_perfApplyMs = 0.0;
    double g_perfUpdateMs = 0.0;
    double g_perfMapMs = 0.0;
    double g_perfEntMs = 0.0;
    int    g_perfFrames = 0;

    double PerfMs()
    {
        static LARGE_INTEGER freq = {};
        if (freq.QuadPart == 0) QueryPerformanceFrequency(&freq);

        LARGE_INTEGER c = {};
        QueryPerformanceCounter(&c);

        return (freq.QuadPart > 0)
            ? 1000.0 * static_cast<double>(c.QuadPart) / static_cast<double>(freq.QuadPart)
            : 0.0;
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

        // 碰撞**每帧**做（原来每 3 帧一次）。
        //
        // 30fps 下 3 帧 = 100ms；旗子在风里一帧就摆几厘米，
        // 碰撞检查的间隙里整整一片布都能穿过去 ⇒ 地图布料的碰撞反馈明显偏弱。
        // 成本实测（[MapInst] step took，5 个实例含收集）：0.75~1.8ms，每帧做完全扛得住。
        const bool doCollision = g_mapCollision;

        for (auto& entry : g_mapInstances)
        {
            MapInstance& inst = entry.second;
            if (!inst.rig.initialized) continue;

            const double t0 = NowMs();

            // 收集半径：覆盖整片布（长旗的下半截离"实例平均位置"可能超过 1.5m）
            const float collideRadius =
                std::max(1.5f, inst.rig.boundingRadius + 0.5f) + g_collideRadiusExtra;

            int selfColAdded = 0;

            if (doCollision)
            {
                // owner 传建筑指针：让 AddEntity 的"0.15m 距离启发式"失效
                // （它只在 owner 未知时生效），否则走到旗子跟前的人物
                // 恰好会落进以布料质心为圆心、半径 15cm 的圈里被丢掉。
                const double perfCollect0 = PerfMs();
                WorldCollision_Collect(inst.center, collideRadius, entry.first, false);
                g_perfCollectMs += PerfMs() - perfCollect0;

                // ------------------------------------------------------------
                // 物件**自己的碰撞体**（2026-11-23 07:2x）
                //
                // CBuilding 不在 Ped/Vehicle/Object 三个池里，Collect 从来不收它，
                // 所以"布料 vs 它自己的碰撞体"一直是空的 —— 只有一个竖直高度查询
                // 偶尔能从上往下托一下（圆环的洞、侧面、旗杆全都失效）。
                // 这里把该建筑 col model 的**球 + 盒**按建筑矩阵加进障碍。
                // 只修 <=15cm 的浅穿透，挂在物件上的布不会被掀飞。
                // ------------------------------------------------------------
                if (g_mapSelfCol)
                {
                    WorldColModelInfo ci;

                    selfColAdded = WorldCollision_AddEntityOwnColModel(
                        entry.first, inst.center, collideRadius, &ci);

                    // col model 读不到东西（很多 .col 是 COL3 网格/特殊格式）时，
                    // 回退用物件**自己的可见网格**当碰撞面（排除布料自己的三角形）。
                    int meshAdded = 0;
                    WorldOwnMeshStats meshStats;

                    const int* dynMap = nullptr;
                    int dynCount = 0;

                    if (!inst.rig.vertexToParticle.empty() &&
                        inst.clone != nullptr &&
                        inst.rig.vertexToParticle.size() ==
                            static_cast<size_t>(inst.clone->numVertices))
                    {
                        dynMap = inst.rig.vertexToParticle.data();
                        dynCount = static_cast<int>(inst.rig.vertexToParticle.size());
                    }

                    if (selfColAdded == 0)
                    {
                        meshAdded = WorldCollision_AddEntityOwnMesh(
                            entry.first, inst.center, collideRadius, dynMap, dynCount,
                            &meshStats);
                    }

                    // 一次性诊断：为什么加了 0 个
                    static std::set<int> s_selfColLogged;

                    if (s_selfColLogged.find(entry.first->m_nModelIndex) == s_selfColLogged.end())
                    {
                        s_selfColLogged.insert(entry.first->m_nModelIndex);

                        std::ostringstream so;
                        so << std::fixed << std::setprecision(2);
                        so << "[SelfCol] modelId=" << entry.first->m_nModelIndex
                           << " hasData=" << ci.hasData
                           << " hasVerts=" << ci.hasVerts
                           << " hasTris=" << ci.hasTris
                           << " colSpheres=" << ci.spheres
                           << " colBoxes=" << ci.boxes
                           << " colTris=" << ci.tris
                           << " colBoundR=" << ci.boundRadius
                           << " addedCol=" << selfColAdded
                           << " meshExam=" << meshStats.examined
                           << " meshCloth=" << meshStats.skippedCloth
                           << " meshCull=" << meshStats.culled
                           << " meshAdded=" << meshAdded
                           << " mask=" << meshStats.maskMode
                           << "(" << meshStats.geoVerts << "/" << meshStats.maskVerts << ")";
                        ModLog(so.str());
                    }

                    selfColAdded += meshAdded;

                    // 自己物件的三角形是"收集之后"才加的，所以索引要在这里重建
                    WorldCollision_BuildTriangleGrid();

                    // 实心顶面：网格挡不住"从洞里穿过去"，托盘保证不会整片穿过
                    if (g_mapSelfBox)
                    {
                        WorldCollision_SetOwnTopPlateFromEntity(entry.first, 0.9f);
                    }
                }
            }

            void (*savedCollide)(RwV3d&, const RwV3d&) = g_worldCollideFn;

            if (!doCollision)
                g_worldCollideFn = nullptr;

            // 自己物件网格的自适应 skin（≈0.45×布料平均边长）+ 扫掠开关
            WorldCollision_SetOwnColSkin(
                std::clamp(0.45f * inst.rig.clothMeanEdge, 0.04f, 0.12f));
            WorldCollision_SetOwnColSweep(g_clothSweep ? 1 : 0);

            const double perfStep0 = PerfMs();
            StepGeometryPhysics(inst.rig, inst.atomic, deltaTime);
            g_perfStepMs += PerfMs() - perfStep0;

            const double perfApply0 = PerfMs();
            const bool applyOk = ApplyGeometryPhysics(inst.rig, inst.atomic);
            g_perfApplyMs += PerfMs() - perfApply0;
            // 诊断：逐实例看"这个实例这一帧到底撞到了什么"。
            //   hits=0 且 obs/tris=0 ⇒ 周围确实没有东西；
            //   obs>0 但 hits=0   ⇒ 接触没建立（半径/间隔）；
            //   hits>0 但看不出反应 ⇒ 响应太弱（margin/kick）。
            if (doCollision && (s_simFrame % 120) == 0)
            {
                std::ostringstream oss;
                oss << std::fixed << std::setprecision(2);
                oss << "[MapInst] collide modelId=" << entry.first->m_nModelIndex
                    << " center=(" << inst.center.x << "," << inst.center.y
                    << "," << inst.center.z << ")"
                    << " rad=" << collideRadius
                    << " hits=" << inst.rig.diagClothContacts
                    << " kick=" << inst.rig.diagClothKick << "m/s"
                    << " selfCol=" << selfColAdded
                    << " obs=" << WorldCollision_ObstacleCount()
                    << " tris=" << WorldCollision_TriangleCount();

                // 建筑收集的计数（诊断）：sectors/visited/noCol/culled/accepted/obs/tris
                {
                    WorldBuildingStats bld{};
                    WorldCollision_GetBuildingStats(bld);

                    oss << " bld=" << bld.sectors << "/" << bld.visited << "/"
                        << bld.noCol << "/" << bld.culled << "/" << bld.accepted
                        << "/" << bld.obs << "/" << bld.tris
                        << " slabs=" << bld.slabs
                        << " near=" << bld.nearModel << ":" << bld.nearObs
                        << "+" << bld.nearTris << "@" << bld.nearDist << "m"
                        << " cull=" << bld.cullModel << "@" << bld.cullDist << "m";
                }

                // 自身三角面到底在哪、离布多近（判断"位置对不对/有没有挨上"）
                {
                    RwV3d tmin{}, tmax{};
                    int   tcount = 0;

                    WorldCollision_OwnColTriBounds(tmin, tmax, tcount);

                    if (tcount > 0)
                    {
                        float minDist = -1.0f;

                        // 布料的实时包围盒 + "有多少布粒子落在环的体积带里"
                        //   slab > 0  ⇒ 布确实在环的空间范围内（那就是判定/响应问题）
                        //   slab = 0  ⇒ 布根本没经过环（可能从洞里/旁边过去了）
                        RwV3d cmin = { 1e9f, 1e9f, 1e9f };
                        RwV3d cmax = { -1e9f, -1e9f, -1e9f };
                        int   slab = 0;

                        const float slabZLo = tmin.z - 0.05f;
                        const float slabZHi = tmax.z + 0.05f;

                        for (const auto& p : inst.rig.particles)
                        {
                            if (static_cast<PhysType>(p.typeU32) != PhysType::Cloth) continue;

                            const float d = WorldCollision_DistanceToOwnColTris(p.posWorld);

                            if (d >= 0.0f && (minDist < 0.0f || d < minDist)) minDist = d;

                            if (p.posWorld.x < cmin.x) cmin.x = p.posWorld.x;
                            if (p.posWorld.y < cmin.y) cmin.y = p.posWorld.y;
                            if (p.posWorld.z < cmin.z) cmin.z = p.posWorld.z;

                            if (p.posWorld.x > cmax.x) cmax.x = p.posWorld.x;
                            if (p.posWorld.y > cmax.y) cmax.y = p.posWorld.y;
                            if (p.posWorld.z > cmax.z) cmax.z = p.posWorld.z;

                            if (p.posWorld.x > tmin.x && p.posWorld.x < tmax.x &&
                                p.posWorld.y > tmin.y && p.posWorld.y < tmax.y &&
                                p.posWorld.z > slabZLo && p.posWorld.z < slabZHi)
                                ++slab;
                        }

                        oss << " triBox=(" << tmin.x << "," << tmin.y << "," << tmin.z << ")-("
                            << tmax.x << "," << tmax.y << "," << tmax.z << ")"
                            << " clothBox=(" << cmin.x << "," << cmin.y << "," << cmin.z << ")-("
                            << cmax.x << "," << cmax.y << "," << cmax.z << ")"
                            << " slab=" << slab
                            << " triMinDist=" << (minDist * 1000.0f) << "mm";
                    }
                }

                ModLog(oss.str());
            }

            // 诊断：逐实例打印布料实际位移，确认"看不见动"到底是没步进、
            // 被挂点锁住，还是已经垂下但静止不动。
            if ((s_simFrame % 120) == 0)
            {
                int softCloth = 0;
                float maxCloth = 0.0f;
                float maxSoft = 0.0f;

                for (const auto& p : inst.rig.particles)
                {
                    if (static_cast<PhysType>(p.typeU32) != PhysType::Cloth) continue;

                    const float dx = p.posWorld.x - p.anchorWorld.x;
                    const float dy = p.posWorld.y - p.anchorWorld.y;
                    const float dz = p.posWorld.z - p.anchorWorld.z;
                    const float d = std::sqrt(dx * dx + dy * dy + dz * dz);

                    if (d > maxCloth) maxCloth = d;

                    if (p.softPins)
                    {
                        ++softCloth;
                        if (d > maxSoft) maxSoft = d;
                    }
                }

                std::ostringstream oss;
                oss << "[MapInst] sim modelId=" << entry.first->m_nModelIndex
                    << " cloth=" << inst.rig.clothCount
                    << " softCloth=" << softCloth
                    << " maxCloth=" << maxCloth << "m"
                    << " maxSoft=" << maxSoft << "m"
                    << " apply=" << (applyOk ? 1 : 0);

                // 直接对比：几何体里的顶点 vs 粒子的物理/静止坐标。
                //   v0 == restLocal 且 pos0 已经飞走 ⇒ 写回没生效（换入/写回通路问题）
                //   v0 跟着 pos0 走 ⇒ 写回正常，画面不动是渲染/换入问题
                if (inst.clone && inst.clone->morphTarget[0].verts &&
                    !inst.rig.particles.empty())
                {
                    const int vIdx = inst.rig.particles[0].geometryVertexIndex;
                    const RwV3d* mv = inst.clone->morphTarget[0].verts;

                    if (vIdx >= 0 && vIdx < static_cast<int>(inst.clone->numVertices))
                    {
                        oss << std::fixed << std::setprecision(3)
                            << " v0=(" << mv[vIdx].x << "," << mv[vIdx].y << "," << mv[vIdx].z << ")"
                            << " rest0=(" << inst.rig.particles[0].restLocal.x << ","
                            << inst.rig.particles[0].restLocal.y << ","
                            << inst.rig.particles[0].restLocal.z << ")"
                            << " pos0=(" << inst.rig.particles[0].posWorld.x << ","
                            << inst.rig.particles[0].posWorld.y << ","
                            << inst.rig.particles[0].posWorld.z << ")";
                    }
                }

                ModLog(oss.str());
            }

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

    // ============================================================
    // 给**所有**人物建骨胶囊（2026-11-23 09:3x）
    //
    // 原来胶囊只属于"白名单 ped rig"（`[Colliders] capsules=22` 绑在 rig 上），
    // 而玩家 CJ 没有 rig ⇒ 布料只看得见 CJ 那个粗糙 col 盒 ⇒ "布撞不到人"。
    // 这里直接读每个 ped 自己的 HAnim 层级，用同一张骨骼表建胶囊并注册成世界障碍。
    // 成本：只处理玩家 30m 内的人物，每人 ~16 根，每帧重算（跟随动画）。
    // ============================================================
    void RegisterAllPedCapsules()
    {
        if (!CPools::ms_pPedPool) return;

        CPlayerPed* player = FindPlayerPed(-1);

        const CVector ppos = player ? player->GetPosition()
                                    : CVector{ 0.0f, 0.0f, 0.0f };

        constexpr float kMaxDistSq = 30.0f * 30.0f;
        constexpr float kExtraMargin = 0.02f;

        static std::vector<WorldCapsule> buf;

        for (int i = 0; i < CPools::ms_pPedPool->m_nSize; ++i)
        {
            CPed* ped = CPools::ms_pPedPool->GetAt(i);
            if (!ped || !ped->m_pRwObject) continue;
            if (RwObjectGetType(ped->m_pRwObject) != rpCLUMP) continue;

            if (player)
            {
                const CVector d = ped->GetPosition() - ppos;
                if (d.x * d.x + d.y * d.y + d.z * d.z > kMaxDistSq) continue;
            }

            RpClump* clump = reinterpret_cast<RpClump*>(ped->m_pRwObject);

            struct HierCtx { RpHAnimHierarchy* h; };
            HierCtx hc{ nullptr };

            RpClumpForAllAtomics(clump,
                [](RpAtomic* atomic, void* data) -> RpAtomic*
                {
                    auto* c = static_cast<HierCtx*>(data);
                    if (!c->h && atomic)
                    {
                        RpHAnimHierarchy* h = RpSkinAtomicGetHAnimHierarchy(atomic);
                        if (h) c->h = h;
                    }
                    return atomic;
                },
                &hc);

            if (!hc.h || !hc.h->pNodeInfo)
            {
                WorldCollision_ClearEntityCapsules(ped);
                continue;
            }

            const RwMatrix* mats = RpHAnimHierarchyGetMatrixArray(hc.h);

            if (!mats)
            {
                WorldCollision_ClearEntityCapsules(ped);
                continue;
            }

            std::map<int, int> boneByNode;

            for (RwInt32 n = 0; n < hc.h->numNodes; ++n)
                boneByNode[hc.h->pNodeInfo[n].nodeID] = static_cast<int>(n);

            buf.clear();

            auto addCap = [&](int idA, int idB, float radius)
            {
                auto ia = boneByNode.find(idA);
                auto ib = boneByNode.find(idB);

                if (ia == boneByNode.end() || ib == boneByNode.end()) return;
                if (radius <= 0.0f) return;

                const RwMatrix& ma = mats[ia->second];
                const RwMatrix& mb = mats[ib->second];

                WorldCapsule w;
                w.ax = ma.pos.x; w.ay = ma.pos.y; w.az = ma.pos.z;
                w.bx = mb.pos.x; w.by = mb.pos.y; w.bz = mb.pos.z;
                w.radius = radius + kExtraMargin;
                w.vx = 0.0f; w.vy = 0.0f; w.vz = 0.0f;
                w.modelId = static_cast<int>(ped->m_nModelIndex);

                buf.push_back(w);
            };

            // 与 GeometryPhysics 的 [Colliders] 同一张表
            addCap(BONE_PELVIS1,    BONE_PELVIS1,    0.13f);
            addCap(BONE_PELVIS,     BONE_PELVIS,     0.13f);
            addCap(BONE_PELVIS,     BONE_LEFTHIP,    0.12f);
            addCap(BONE_PELVIS,     BONE_RIGHTHIP,   0.12f);
            addCap(BONE_LEFTHIP,    BONE_LEFTKNEE,   0.11f);
            addCap(BONE_RIGHTHIP,   BONE_RIGHTKNEE,  0.11f);
            addCap(BONE_LEFTKNEE,   BONE_LEFTANKLE,  0.09f);
            addCap(BONE_RIGHTKNEE,  BONE_RIGHTANKLE, 0.09f);

            addCap(BONE_LEFTHIP,    BONE_LEFTHIP,    0.115f);
            addCap(BONE_RIGHTHIP,   BONE_RIGHTHIP,   0.115f);
            addCap(BONE_LEFTKNEE,   BONE_LEFTKNEE,   0.09f);
            addCap(BONE_RIGHTKNEE,  BONE_RIGHTKNEE,  0.09f);

            addCap(BONE_UPPERTORSO, BONE_UPPERTORSO, 0.14f);
            addCap(BONE_PELVIS,     BONE_SPINE1,     0.13f);
            addCap(BONE_SPINE1,     BONE_UPPERTORSO, 0.13f);
            addCap(BONE_NECK,       BONE_HEAD,       0.10f);

            WorldCollision_SetEntityCapsules(ped,
                buf.empty() ? nullptr : buf.data(),
                static_cast<int>(buf.size()));
        }
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
    //     clothgravity=1.0     布料重力倍率
    //     clothfollow=0.0      有挂点的布料向静止姿势回拉的强度（0 = 自由垂坠）
    //     clothdamping=0.98    布料每帧阻尼（60fps 基准）
    //     clothinertia=0.6     布料惯性继承（跑动时跟着身体走，0 = 旧行为）
    //     bodycolliderscale=1  人体胶囊半径倍率
    //     bodycollidermargin=.01 人体胶囊半径额外加成（米）
    // ------------------------------------------------------------

    float g_worldRadius = 2.5f;
    float g_interactRadius = 8.0f;
    int   g_maxInstances = 8;
    float g_windStrength = 1.0f;
bool  g_datOnly = true;      // true = 只读外部 .dat，忽略模型自带顶点色
    bool  g_mapPhysics = false;
    bool  g_mapConvert = false;   // 逐实例渲染开关
    bool  g_mapCollision = false; // 地图实例布料是否与世界碰撞（默认关：避免被自家墙顶住 + 抖）

    // 地图实例布料是否把**自己所属物件**的 col model（球/盒）当障碍
    // （2026-11-23 07:2x：CBuilding 不在任何池里，以前布料碰不到自己的碰撞体）
    bool  g_mapSelfCol = true;

    // 另外给"自己物件"补一个**实心顶面**：网格/球盒挡不住"从洞里穿过去"，
    // 顶面托盘能保证布料不会整片穿过物件（只向上推，不掀飞、不抖）
    bool  g_mapSelfBox = true;

    // "自己物件网格"的扫掠（穿越）检测：1 = 开（薄壳能挡住快速布料）
    bool  g_clothSweep = true;

    // 载具渲染网格碰撞：1 = 开（别的车的布料碰撞能碰到车身曲面，而不是只有大盒子）
    bool  g_carMesh = true;

    // 载具碰撞体膨胀量（米）：补"col 盒比可见车身小"的差距
    float g_carMargin = 0.02f;

    // 地面 / 普通物件 / 地图物件自身碰撞体的余量（米）
    float g_groundMargin = 0.02f;
    float g_objectMargin = 0.02f;
    float g_mapObjMargin = 0.02f;

    // 静态建筑碰撞（按扇区枚举 CBuilding）+ 余量
    bool  g_buildingCol = true;
    float g_buildingMargin = 0.02f;

    // 世界碰撞的**额外收集半径**（米，ini: collideradius，默认 0）
    //   布料/载具/人物的收集球默认只覆盖"自己 + 0.5m"，
    //   如果想让更远的墙/车/人进碰撞列表，把这个值调大（例如 3）。
    float g_collideRadiusExtra = 0.0f;

    // 载具布料自碰撞：1 = 车布和**自己这台车**的 col model（球/盒/网格）碰撞
    bool  g_selfCollide = true;
    bool  g_dumpGeometry = false; // 把白名单模型的几何体导出成 dump（供插件生成精确 dat）

    // 记录 CJ 当前所有部件（原子/贴图名/顶点数）到 vertexphysics\cj_parts.txt
    // 换装导致部件变化时自动追加一条
    bool  g_cjPartsLog = true;

    // CObject 自身碰撞（ini: objselfcol，默认 1）
    // 只加 col 碰撞体（球/盒/col 三角面），不加渲染网格。
    bool  g_objSelfCol = true;

    // CObject 自身碰撞体的余量（ini: objselfmargin，默认 0.02 米）
    // 与 objmargin 的区别：
    //   objmargin      = 别人（人物布/车布/地图布）看这个物件时的膨胀
    //   objselfmargin  = 这个物件**自己的布料**看自己 col 时的膨胀
    float g_objSelfMargin = 0.02f;


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

        // 布料手感默认值（ini 没写时也用这套）
        float clothGravity = 1.0f;
        float clothFollow  = 0.0f;
        float clothDamping = 0.95f;
        float clothPin     = 0.15f;
        float clothWind    = 1.8f;
    bool  pedClothWindFlip = true;   // 人物（蒙皮）布料风向取反
        float clothInertia = 0.60f;
        // ini: characterclothselfmargin（旧名 clothmargin 仍兼容）
        // = 布料与自己身体骨胶囊的安全距离（只影响人物自身碰撞）
        float clothMargin  = 0.015f;

        // 人体胶囊默认值（自身碰撞）
        float bodyColliderScale  = 1.0f;
        float bodyColliderMargin = 0.01f;

        // 头发 A/B 诊断开关（默认全开 = 正常行为）
        int hairPins          = 1;
        int hairCapsuleMask   = 1;
        int hairRootExempt    = 1;
        int hairPieceFix      = 1;
        int hairRotTransport  = 1;
        int hairInertia       = 1;
        int hairWorldCollide  = 1;
        int hairDiag          = 1;

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
                    g_windStrength = std::clamp(static_cast<float>(atof(value.c_str())), 0.0f, 20.0f);
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
                else if (key == "cjparts")
                    g_cjPartsLog = (atoi(value.c_str()) != 0);
                else if (key == "mapconvert")
                {
                    g_mapConvertRaw = atoi(value.c_str());
                    g_mapConvert = (g_mapConvertRaw != 0);
                }
                else if (key == "mapphysics")
                    g_mapPhysics = (atoi(value.c_str()) != 0);
                else if (key == "mapcollision")
                    g_mapCollision = (atoi(value.c_str()) != 0);
                else if (key == "mapselfcol")
                    g_mapSelfCol = (atoi(value.c_str()) != 0);
                else if (key == "mapselfbox")
                    g_mapSelfBox = (atoi(value.c_str()) != 0);
                else if (key == "clothsweep")
                    g_clothSweep = (atoi(value.c_str()) != 0);
                else if (key == "carmesh")
                    g_carMesh = (atoi(value.c_str()) != 0);
                else if (key == "carmargin")
                    g_carMargin = std::clamp(static_cast<float>(atof(value.c_str())), 0.0f, 0.15f);
                else if (key == "groundmargin")
                    g_groundMargin = std::clamp(static_cast<float>(atof(value.c_str())), 0.0f, 0.15f);
                else if (key == "objmargin")
                    g_objectMargin = std::clamp(static_cast<float>(atof(value.c_str())), 0.0f, 0.15f);
                else if (key == "mapselfmargin" || key == "mapobjmargin")
                    g_mapObjMargin = std::clamp(static_cast<float>(atof(value.c_str())), 0.0f, 0.15f);
                else if (key == "buildingcol")
                    g_buildingCol = (atoi(value.c_str()) != 0);
                else if (key == "buildingmargin")
                    g_buildingMargin = std::clamp(static_cast<float>(atof(value.c_str())), 0.0f, 0.15f);
                else if (key == "collideradius")
                    g_collideRadiusExtra = std::clamp(static_cast<float>(atof(value.c_str())), 0.0f, 20.0f);
                else if (key == "selfcollide")
                    g_selfCollide = (atoi(value.c_str()) != 0);
                else if (key == "clothgravity")
                    clothGravity = std::clamp(static_cast<float>(atof(value.c_str())), 0.0f, 3.0f);
                else if (key == "clothfollow")
                    clothFollow = std::clamp(static_cast<float>(atof(value.c_str())), 0.0f, 1.0f);
                else if (key == "clothdamping")
                    clothDamping = std::clamp(static_cast<float>(atof(value.c_str())), 0.50f, 0.999f);
                else if (key == "clothpinstiffness")
                    clothPin = std::clamp(static_cast<float>(atof(value.c_str())), 0.02f, 1.0f);
                else if (key == "clothwind")
                    // 允许负值（2026-11-23 11:1x）：设 -1.8 就是"反向吹布料"，
                    // 用来确认"人物布料风向看着是反的"到底是不是风本身的问题。
                    clothWind = std::clamp(static_cast<float>(atof(value.c_str())), -10.0f, 10.0f);
                else if (key == "characterclothselfmargin" || key == "clothmargin")
                    clothMargin = std::clamp(static_cast<float>(atof(value.c_str())), 0.0f, 0.08f);
                else if (key == "clothinertia")
                    clothInertia = std::clamp(static_cast<float>(atof(value.c_str())), 0.0f, 0.95f);
                else if (key == "pedclothwindflip")
                    // 只翻"人物（蒙皮）布料"的风向；车辆/地图布料不受影响
                    pedClothWindFlip = (atoi(value.c_str()) != 0);
                else if (key == "bodycolliderscale")
                    bodyColliderScale = std::clamp(static_cast<float>(atof(value.c_str())), 0.5f, 2.0f);
                else if (key == "bodycollidermargin")
                    bodyColliderMargin = std::clamp(static_cast<float>(atof(value.c_str())), 0.0f, 0.08f);
                else if (key == "hairpins")
                    hairPins = (atoi(value.c_str()) != 0);
                else if (key == "haircapsulemask")
                    hairCapsuleMask = (atoi(value.c_str()) != 0);
                else if (key == "hairrootexempt")
                    hairRootExempt = (atoi(value.c_str()) != 0);
                else if (key == "hairpiecefix")
                    hairPieceFix = (atoi(value.c_str()) != 0);
                else if (key == "hairrotrans")
                    hairRotTransport = (atoi(value.c_str()) != 0);
                else if (key == "hairinertia")
                    hairInertia = (atoi(value.c_str()) != 0);
                else if (key == "hairworldcollide")
                    hairWorldCollide = (atoi(value.c_str()) != 0);
                else if (key == "hairride" || key == "hairdamping" || key == "hairpull")
                {
                    // 三个头发参数任意顺序出现，缓存后整组应用
                    static float hRide = 0.6f;
                    static float hDamp = 0.98f;
                    static float hPull = 1.0f;

                    if (key == "hairride")
                        hRide = static_cast<float>(atof(value.c_str()));
                    else if (key == "hairdamping")
                        hDamp = static_cast<float>(atof(value.c_str()));
                    else
                        hPull = static_cast<float>(atof(value.c_str()));

                    GeometryPhysics_SetHairRide(hRide, hDamp, hPull);
                }
                else if (key == "hairwind" || key == "hairfollow")
                {
                    static float hWind = 1.0f;
                    static float hFollow = 1.0f;

                    if (key == "hairwind")
                        hWind = static_cast<float>(atof(value.c_str()));
                    else
                        hFollow = static_cast<float>(atof(value.c_str()));

                    GeometryPhysics_SetHairWindFollow(hWind, hFollow);
                }
                else if (key == "hairdiag")
                    hairDiag = (atoi(value.c_str()) != 0);
                else if (key == "colordiag")
                    GeometryPhysics_SetColorDiag(atoi(value.c_str()) != 0);
                else if (key == "collsteps")
                    GeometryPhysics_SetCollisionSubsteps(atoi(value.c_str()));
                else if (key == "bodyk" || key == "bodyc" || key == "bodydrive" || key == "bodyamp")
                {
                    // 四个 Body 旋钮任意顺序出现，都先缓存，遇到任意一个就把整组应用
                    static float bk = 0.0f, bc = 0.0f, bdrive = 1.30f, bamp = 1.0f;

                    const float v = static_cast<float>(atof(value.c_str()));

                    if (key == "bodyk")      bk = v;
                    else if (key == "bodyc") bc = v;
                    else if (key == "bodydrive") bdrive = v;
                    else                     bamp = v;

                    GeometryPhysics_SetBodyTuning(bk, bc, bdrive, bamp);
                }
                else if (key == "bodygroup" || key == "bodygroupsplit" ||
                         key == "bodyrandom" || key == "bodysmooth")                {
                    // Body 分组参数（bodygroup 是字符串：piece/bone/cluster）
                    static int   gmode = 0;
                    static int   gsplit = 3;
                    static float grandom = 0.25f;
                    static int   gsmooth = 3;

                    if (key == "bodygroup")
                    {
                        if (value == "bone")         gmode = 1;
                        else if (value == "cluster") gmode = 2;
                        else                         gmode = 0;
                    }
                    else if (key == "bodygroupsplit") gsplit = atoi(value.c_str());
                    else if (key == "bodyrandom")     grandom = static_cast<float>(atof(value.c_str()));
                    else                              gsmooth = atoi(value.c_str());

                    GeometryPhysics_SetBodyGrouping(gmode, gsplit, grandom, gsmooth);
                }
                else if (key == "clothcapsulemask" || key == "clothcapsulemaskdepth")
                {
                    static int   cmEnabled = 1;
                    static float cmDepth = 0.008f;

                    if (key == "clothcapsulemask")
                        cmEnabled = (atoi(value.c_str()) != 0);
                    else
                        cmDepth = static_cast<float>(atof(value.c_str()));

                    GeometryPhysics_SetClothCapsuleMask(cmEnabled, cmDepth);
                }
                else if (key == "capskip")
                {
                    GeometryPhysics_SetCapsuleSkipMask(atoi(value.c_str()));
                }
                else if (key.compare(0, 14, "capsgroupscale") == 0)
                {
                    // capsgroupscalepelvis / hip / thigh / knee / calf / torso / neck / arms / feet
                    const std::string part = key.substr(14);

                    int group = 0;

                    if (part == "pelvis")      group = 1;
                    else if (part == "hip")    group = 2;
                    else if (part == "thigh")  group = 4;
                    else if (part == "knee")   group = 8;
                    else if (part == "calf")   group = 16;
                    else if (part == "torso")  group = 32;
                    else if (part == "neck")   group = 64;
                    else if (part == "arms")   group = 128;
                    else if (part == "feet")   group = 256;

                    if (group != 0)
                        GeometryPhysics_SetCapsuleGroupScale(
                            group, static_cast<float>(atof(value.c_str())));
                }
                else if (key == "capsulescale")                {
                    // 逗号分隔的逐胶囊倍率，顺序 = [Colliders] radii= 的顺序
                    std::vector<float> scales;
                    std::stringstream ss(value);
                    std::string item;

                    while (std::getline(ss, item, ','))
                    {
                        const size_t b = item.find_first_not_of(" \t");
                        if (b == std::string::npos) continue;

                        scales.push_back(static_cast<float>(atof(item.c_str() + b)));
                    }

                    GeometryPhysics_SetCapsuleIndexScale(scales);
                }
                else if (key == "objselfcol")
                {
                    g_objSelfCol = (atoi(value.c_str()) != 0);
                }
                else if (key == "objselfmargin")
                {
                    g_objSelfMargin = std::clamp(
                        static_cast<float>(atof(value.c_str())), 0.0f, 0.15f);
                }

                continue;
            }

            if (line == "*")
            {
                g_autoMode = true;
                continue;
            }

            g_whitelistNames.push_back(line);

            // 2026-11-24 05:4x：支持**数字 id** 白名单（例如只写一行 0 = 玩家 CJ）。
            // 原因：有些模型的"名字"解析不到（peds.ide 里模型 0 的名字是 null，
            // CModelInfo::GetModelInfo("null") 失败），但按 id 一定能启用。
            bool allDigits = !line.empty();

            for (char ch : line)
            {
                if (ch < '0' || ch > '9') { allDigits = false; break; }
            }

            if (g_whitelistIds.find(line) == g_whitelistIds.end())
                g_whitelistIds[line] = allDigits ? atoi(line.c_str()) : -1;
        }

        SetClothTuning(clothGravity, clothFollow, clothDamping, clothPin, clothWind,
                       clothInertia, clothMargin);
        WorldCollision_SetVehicleMesh(g_carMesh ? 1 : 0);
        WorldCollision_SetCarMargin(g_carMargin);
        WorldCollision_SetGroundMargin(g_groundMargin);
        WorldCollision_SetObjectMargin(g_objectMargin);
        WorldCollision_SetMapObjectMargin(g_mapObjMargin);
        WorldCollision_SetBuildingCollision(g_buildingCol ? 1 : 0);
        WorldCollision_SetBuildingMargin(g_buildingMargin);
        SetBodyColliderTuning(bodyColliderScale, bodyColliderMargin);
        SetPedClothWindFlip(pedClothWindFlip ? 1 : 0);
        SetHairDebugOptions(hairPins, hairCapsuleMask, hairRootExempt, hairPieceFix,
                           hairRotTransport, hairInertia, hairWorldCollide, hairDiag);

        std::ostringstream oss;
        oss << "[Config] autoMode=" << (g_autoMode ? "YES" : "NO")
            << " whitelist=" << g_whitelistNames.size()
            << " worldRadius=" << g_worldRadius
            << " interactRadius=" << g_interactRadius
            << " maxInstances=" << g_maxInstances
            << " wind=" << g_windStrength
            << " collRadiusExtra=" << g_collideRadiusExtra
            << " datOnly=" << (g_datOnly ? "ON" : "OFF")
            << " mapPhysics=" << (g_mapPhysics ? "ON" : "OFF")
            << " mapConvert=" << (g_mapConvert ? "ON" : "OFF")
            << " mapCollision=" << (g_mapCollision ? "ON" : "OFF")
            << " mapSelfCol=" << (g_mapSelfCol ? "ON" : "OFF")
            << " dumpGeometry=" << (g_dumpGeometry ? "ON" : "OFF")
            << " clothGravity=" << clothGravity
            << " clothFollow=" << clothFollow
            << " clothDamping=" << clothDamping
            << " clothPin=" << clothPin
            << " clothWind=" << clothWind
            << " clothInertia=" << clothInertia
            << " clothMargin=" << clothMargin
            << " bodyColliderScale=" << bodyColliderScale
            << " bodyColliderMargin=" << bodyColliderMargin
            << " hairPins=" << hairPins
            << " hairCapsuleMask=" << hairCapsuleMask
            << " hairRootExempt=" << hairRootExempt
            << " hairPieceFix=" << hairPieceFix
            << " hairRotTransport=" << hairRotTransport
            << " hairInertia=" << hairInertia
            << " hairWorldCollide=" << hairWorldCollide
            << " hairDiag=" << hairDiag;
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

        const size_t before = out.size();
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

        // 记住实体指针：骨骼的"世界矩阵数组"是渲染时才刷新的，
        // 自身碰撞要用本帧的骨架（见 SyncAnimPose）
        for (size_t k = before; k < out.size(); ++k)
            out[k].entity = entity;
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
        // 优先用原子自己的 frame：车辆/飞机是多 atomic clump，
        // 车门、机翼、布料各自带局部偏移。用 clump 根节点会让物理粒子
        // 和实际渲染的部件错位，碰撞也就检测不到布料周围的东西。
        RwFrame* atomicFrame = RpAtomicGetFrame(atomic);

        if (atomicFrame)
        {
            const RwMatrix* ltm = RwFrameGetLTM(atomicFrame);
            if (ltm) return ltm;
        }

        RpClump* clump = RpAtomicGetClumpMacro(atomic);

        if (clump)
        {
            RwFrame* frame = RpClumpGetFrame(clump);
            if (frame) return RwFrameGetLTM(frame);
        }

        return nullptr;
    }

    // ============================================================
    // 骨骼矩阵同步（自身碰撞的一个隐藏大坑）
    //
    // GTA 把"骨架世界矩阵数组"的刷新放在 CPed::Render / CVehicle::Render 里
    // （CEntity::UpdateRpHAnim，0x532B20）。而本插件在 drawingEvent.before
    // 里取 boneMatrices，拿到的是**上一帧渲染时**的骨架。
    // 站着不动没事；跑起来一帧腿就走 10~20cm，胶囊永远落后一帧，
    // 布料于是从"上一帧的腿"里穿出来。
    //
    // 这里在步进之前调一次游戏自己的 UpdateRpHAnim：节点变换（动作混合结果）
    // 在 ProcessControl 里已经是本帧的，重算出来的矩阵也就是本帧骨架。
    // 如果游戏其实已经刷新过，这次调用是幂等的（差值 0）。
    // ============================================================
    void SyncAnimPose(CEntity* entity, RpAtomic* atomic, int modelIndex, int boneCount)
    {
        if (!entity || !atomic) return;

        if (entity->m_nType != ENTITY_TYPE_PED && entity->m_nType != ENTITY_TYPE_VEHICLE)
            return;

        if (!entity->m_pRwObject || RwObjectGetType(entity->m_pRwObject) != rpCLUMP)
            return;

        RpHAnimHierarchy* hierarchy = RpSkinAtomicGetHAnimHierarchy(atomic);
        if (!hierarchy) return;

        RwMatrix* matrices = RpHAnimHierarchyGetMatrixArray(hierarchy);
        if (!matrices) return;

        const int count = std::clamp(boneCount, 1, 64);

        RwV3d before[64];
        for (int i = 0; i < count; ++i)
            before[i] = matrices[i].pos;

        entity->UpdateRpHAnim();

        matrices = RpHAnimHierarchyGetMatrixArray(hierarchy);
        if (!matrices) return;

        float worst = 0.0f;
        int worstBone = -1;

        for (int i = 0; i < count; ++i)
        {
            const RwV3d d =
            {
                matrices[i].pos.x - before[i].x,
                matrices[i].pos.y - before[i].y,
                matrices[i].pos.z - before[i].z
            };

            const float len = std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);

            if (len > worst) { worst = len; worstBone = i; }
        }

        // 诊断：跑动时这个差值就是"胶囊本该领先多少"
        static float s_maxDelta = 0.0f;
        static int   s_maxBone  = -1;
        static int   s_logged   = 0;

        if (worst > s_maxDelta)
        {
            s_maxDelta = worst;
            s_maxBone  = worstBone;
        }

        if ((++s_logged % 600) == 0)
        {
            std::ostringstream oss;
            oss << "[VER_82][AnimPose] modelId=" << modelIndex
                << " bones=" << count
                << " staleMax=" << (s_maxDelta * 1000.0f) << "mm"
                << " bone=" << s_maxBone;
            ModLog(oss.str());

            s_maxDelta = 0.0f;
            s_maxBone  = -1;
        }
    }

} // namespace


    // ============================================================
    //  [CjParts] 记录 CJ 当前使用的所有身体部件
    //
    //  2026-11-24 06:4x：玩家模型是运行时拼装的，形状/顶点数会随换装变化。
    //  这里把"当前每个 atomic 的顶点数、三角形数、贴图名"写进
    //      <exe>\vertexphysics\cj_parts.txt
    //  只有签名变化（换装/换发型）时才追加一条，不会刷屏。
    // ============================================================

    // 材质贴图名 -> 部位名（英文，避免源码编码导致乱码）
    const char* PartLabelCn(const std::string& tex)
    {
        std::string s;
        s.reserve(tex.size());

        for (char c : tex)
            s.push_back(static_cast<char>(::tolower(static_cast<unsigned char>(c))));

        if (s == "torso")  return "Torso";
        if (s == "head")   return "Head/Face";
        if (s == "legs")   return "Legs";
        if (s == "feet")   return "Feet/Shoes";
        if (s == "vest")   return "Vest";
        if (s == "shirta" || s == "shirtb") return "Shirt";
        if (s == "jeansa" || s == "jeansb") return "Jeans";
        if (s == "sneaker" || s == "shoe")  return "Shoes";
        if (s.empty())     return "?";
        return "Other";
    }

    // ============================================================
    //  [CjClothes] hook 换装重建，读取 catalog key
    //
    //  CClothesBuilder::CreateSkinnedClump @ 0x5A69D0
    //  玩家每次换装/理发都会调用它，参数带 CPedClothesDesc：
    //      m_anModelKeys[10]   —— 部件模型 key（索引 = eClothesModelPart）
    //      m_anTextureKeys[18] —— 纹理 key（索引 = eClothesTexturePart）
    //  key 就是 CKeyGen::GetUppercaseKey(名字)，所以能用 shopping.dat 的名字表反查。
    // ============================================================
    using CreateSkinnedClumpFn = RpClump* (__cdecl*)(RpClump*, RwTexDictionary*,
                                                    CPedClothesDesc&, const CPedClothesDesc*, bool);

    safetyhook::InlineHook g_clothesHook;
    bool g_clothesHookInstalled = false;
    std::string g_lastClothesSig;

    std::map<unsigned int, std::string> g_keyNames;

    // player.img 的 id 表（30000+/30152+）：名字(无扩展名,小写) -> id
    std::map<std::string, int> g_imgDffIds;
    std::map<std::string, int> g_imgTxdIds;
    bool g_imgIdTableTried = false;

    std::string LowerNoExt(const std::string& name)
    {
        std::string s;
        s.reserve(name.size());

        for (char c : name)
        {
            if (c == '.') break;   // 去掉 .dff/.txd
            s.push_back(static_cast<char>(::tolower(static_cast<unsigned char>(c))));
        }

        return s;
    }

    void LoadImgIdTableFromTxt()
    {
        std::ifstream f((GetDatDirectory() + "player_img_ids.txt").c_str());
        if (!f.is_open()) return;

        std::string line;

        while (std::getline(f, line))
        {
            const size_t first = line.find_first_not_of(" \t\r\n");
            if (first == std::string::npos || line[first] == '#') continue;

            std::istringstream iss(line);

            int id = 0;
            std::string name, type;

            if (!(iss >> id >> name >> type)) continue;

            const std::string key = LowerNoExt(name);

            if (type == "dff") g_imgDffIds[key] = id;
            else if (type == "txd") g_imgTxdIds[key] = id;
        }
    }

    // ============================================================
    //  解析 models\player.img（VER2）→ 生成 30000+/30152+ 的 ID 表
    //
    //  2026-11-24 12:4x：以前依赖 vertexphysics\player_img_ids.txt，
    //  但装了服装 mod 后 IMG 目录会变（ID 按目录顺序分配），表就过时了。
    //  现在启动时直接读 player.img，永不失效；读不到才退回读 txt。
    // ============================================================
    void LoadImgIdTable()
    {
        if (g_imgIdTableTried) return;
        g_imgIdTableTried = true;

        const std::string imgPath = GetExeDirectory() + "models\\player.img";

        std::ifstream f(imgPath.c_str(), std::ios::binary);

        if (f.is_open())
        {
            char magic[4] = {};
            f.read(magic, 4);

            if (f.good() && std::memcmp(magic, "VER2", 4) == 0)
            {
                unsigned int count = 0;
                f.read(reinterpret_cast<char*>(&count), 4);

                if (f.good() && count > 0 && count < 100000)
                {
                    int dffIdx = 0;
                    int txdIdx = 0;

                    for (unsigned int i = 0; i < count && f.good(); ++i)
                    {
                        unsigned int offset = 0;
                        unsigned short streamSize = 0;
                        unsigned short arcSize = 0;
                        char nameBuf[24] = {};

                        f.read(reinterpret_cast<char*>(&offset), 4);
                        f.read(reinterpret_cast<char*>(&streamSize), 2);
                        f.read(reinterpret_cast<char*>(&arcSize), 2);
                        f.read(nameBuf, 24);

                        size_t len = 0;
                        while (len < 24 && nameBuf[len] != 0) ++len;

                        const std::string name(nameBuf, len);
                        const std::string key = LowerNoExt(name);

                        if (key.empty()) continue;

                        if (name.size() >= 4 &&
                            (name.compare(name.size() - 4, 4, ".dff") == 0 ||
                             name.compare(name.size() - 4, 4, ".DFF") == 0))
                        {
                            g_imgDffIds[key] = 30000 + dffIdx;
                            ++dffIdx;
                        }
                        else if (name.size() >= 4 &&
                                 (name.compare(name.size() - 4, 4, ".txd") == 0 ||
                                  name.compare(name.size() - 4, 4, ".TXD") == 0))
                        {
                            g_imgTxdIds[key] = 30152 + txdIdx;
                            ++txdIdx;
                        }
                    }

                    std::ostringstream oss;
                    oss << "[ImgIds] player.img entries=" << count
                        << " dff=" << dffIdx << " txd=" << txdIdx;
                    ModLog(oss.str());

                    return;
                }
            }
        }

        // 回退：旧的文本表
        LoadImgIdTableFromTxt();

        ModLog("[ImgIds] player.img unreadable, fell back to player_img_ids.txt");
    }

    int FindImgId(const std::string& name, bool txd)
    {
        LoadImgIdTable();

        const std::string key = LowerNoExt(name);
        const auto& m = txd ? g_imgTxdIds : g_imgDffIds;
        const auto it = m.find(key);

        return (it == m.end()) ? -1 : it->second;
    }

    void LoadClothesKeyTable()
    {
        if (!g_keyNames.empty()) return;

        std::ifstream f((GetExeDirectory() + "data\\shopping.dat").c_str());
        if (!f.is_open()) return;

        std::string line;
        std::string section;

        while (std::getline(f, line))
        {
            const size_t first = line.find_first_not_of(" \t\r\n");

            if (first == std::string::npos || line[first] == '#') continue;

            std::istringstream iss(line);

            std::string tok0;
            if (!(iss >> tok0)) continue;

            if (tok0 == "section") { iss >> section; continue; }

            std::string tag, part;
            if (!(iss >> tag)) continue;
            if (!(iss >> part)) continue;

            g_keyNames[CKeyGen::GetUppercaseKey(tok0.c_str())] = tok0;
            g_keyNames[CKeyGen::GetUppercaseKey(part.c_str())] = part;
        }
    }

    const char* LookupKeyName(unsigned int key)
    {
        if (key == 0) return nullptr;

        const auto it = g_keyNames.find(key);
        return (it == g_keyNames.end()) ? nullptr : it->second.c_str();
    }

    RpClump* __cdecl HookCreateSkinnedClump(RpClump* clump, RwTexDictionary* dict,
                                            CPedClothesDesc& newclothes,
                                            const CPedClothesDesc* oldclothes, bool cutscene)
    {
        if (g_cjPartsLog)
        {
            LoadClothesKeyTable();

            // 前 5 次调用都记一条日志，确认 hook 真的被触发
            static int s_fired = 0;

            if (s_fired < 5)
            {
                ++s_fired;

                std::ostringstream fo;
                fo << "[CjClothes] hook fired #" << s_fired
                   << " cutscene=" << (cutscene ? 1 : 0)
                   << " m0=0x" << std::hex << newclothes.m_anModelKeys[0]
                   << " t0=0x" << newclothes.m_anTextureKeys[0]
                   << " t3=0x" << newclothes.m_anTextureKeys[3];
                ModLog(fo.str());
            }

            std::ostringstream sig;

            for (int i = 0; i < 10; ++i) sig << newclothes.m_anModelKeys[i] << ",";
            for (int i = 0; i < 18; ++i) sig << newclothes.m_anTextureKeys[i] << ",";

            const std::string s = sig.str();

            if (s != g_lastClothesSig)
            {
                g_lastClothesSig = s;

                SYSTEMTIME st{};
                GetLocalTime(&st);

                std::ofstream f((GetDatDirectory() + "cj_parts.txt").c_str(), std::ios::app);

                if (f.is_open())
                {
                    f << "---- clothes rebuild ----\n";
                    f << "time=" << st.wHour << ":" << st.wMinute << ":" << st.wSecond
                      << " cutscene=" << (cutscene ? 1 : 0) << "\n";

                    f << "  modelKeys:";

                    for (int i = 0; i < 10; ++i)
                    {
                        const unsigned int k = newclothes.m_anModelKeys[i];
                        if (!k) continue;

                        const char* n = LookupKeyName(k);

                        f << " [" << i << "]=" << (n ? n : "?");

                        if (n)
                        {
                            const int dffId = FindImgId(n, false);
                            if (dffId >= 0) f << "(dff=" << dffId << ")";
                        }

                        f << "(0x" << std::hex << k << std::dec << ")";
                    }

                    f << "\n  textureKeys:";

                    for (int i = 0; i < 18; ++i)
                    {
                        const unsigned int k = newclothes.m_anTextureKeys[i];
                        if (!k) continue;

                        const char* n = LookupKeyName(k);

                        f << " [" << i << "]=" << (n ? n : "?");

                        if (n)
                        {
                            const int txdId = FindImgId(n, true);
                            if (txdId >= 0) f << "(txd=" << txdId << ")";
                        }

                        f << "(0x" << std::hex << k << std::dec << ")";
                    }

                    f << "\n\n";
                }

                ModLog("[CjClothes] rebuild keys logged");
            }
        }

        return g_clothesHook.original<CreateSkinnedClumpFn>()(
            clump, dict, newclothes, oldclothes, cutscene);
    }

    void InstallClothesHook()
    {
        if (g_clothesHookInstalled) return;
        g_clothesHookInstalled = true;

        g_clothesHook = safetyhook::create_inline(
            reinterpret_cast<void*>(0x5A69D0),
            reinterpret_cast<void*>(&HookCreateSkinnedClump));

        ModLog(g_clothesHook ? "[CjClothes] hook OK (0x5A69D0)"
                             : "[CjClothes] hook FAILED (0x5A69D0)");
    }

    struct PlayerPart
    {
        int         verts = 0;
        int         tris = 0;
        std::string tex;

        // 每个材质（= 身体部件）的顶点数/三角形数/matOfVert 映射
        std::vector<int> matVertCount;
        std::vector<int> matTriCount;
        std::vector<int> vertMat;      // 顶点 -> 材质索引
        std::vector<std::string> matName;
    };

    RpAtomic* CollectPlayerPart(RpAtomic* atomic, void* data)
    {
        auto* out = static_cast<std::vector<PlayerPart>*>(data);

        if (!atomic) return atomic;

        RpGeometry* g = RpAtomicGetGeometry(atomic);
        if (!g) return atomic;

        PlayerPart part;
        part.verts = static_cast<int>(g->numVertices);
        part.tris  = static_cast<int>(g->numTriangles);

        const int mats = static_cast<int>(RpGeometryGetNumMaterials(g));

        part.matVertCount.assign(mats > 0 ? mats : 1, 0);
        part.matTriCount.assign(mats > 0 ? mats : 1, 0);
        part.vertMat.assign(static_cast<size_t>(g->numVertices), -1);
        part.matName.assign(mats > 0 ? mats : 1, std::string());

        for (int i = 0; i < mats; ++i)
        {
            RpMaterial* m = RpGeometryGetMaterial(g, i);
            if (!m) continue;

            RwTexture* tex = RpMaterialGetTexture(m);
            if (!tex) continue;

            const char* name = RwTextureGetName(tex);
            if (!name || !name[0]) continue;

            if (!part.tex.empty()) part.tex += "+";
            part.tex += name;

            part.matName[i] = name;
        }

        // 统计每个材质的三角形数，并把顶点归属到材质（同一顶点取第一次出现的材质）
        if (mats > 0 && g->triangles)
        {
            for (int t = 0; t < static_cast<int>(g->numTriangles); ++t)
            {
                const RpTriangle& tri = g->triangles[t];
                const int m = static_cast<int>(tri.matIndex);

                if (m < 0 || m >= mats) continue;

                ++part.matTriCount[m];

                for (int k = 0; k < 3; ++k)
                {
                    const int v = static_cast<int>(tri.vertIndex[k]);

                    if (v < 0 || v >= static_cast<int>(g->numVertices)) continue;

                    if (part.vertMat[v] < 0)
                    {
                        part.vertMat[v] = m;
                        ++part.matVertCount[m];
                    }
                }
            }
        }

        out->push_back(part);
        return atomic;
    }

    void DumpPlayerParts()
    {
        if (!g_cjPartsLog) return;

        CPlayerPed* player = FindPlayerPed(-1);
        if (!player || !player->m_pRwObject) return;
        if (RwObjectGetType(player->m_pRwObject) != rpCLUMP) return;

        static std::string s_prevSig;

        std::vector<PlayerPart> parts;

        RpClumpForAllAtomics(reinterpret_cast<RpClump*>(player->m_pRwObject),
                             CollectPlayerPart, &parts);

        if (parts.empty()) return;

        std::ostringstream sig;
        int totalVerts = 0;

        for (const auto& p : parts)
        {
            sig << p.verts << ":" << p.tex << ";";
            totalVerts += p.verts;
        }

        const std::string s = sig.str();
        if (s == s_prevSig) return;

        s_prevSig = s;

        SYSTEMTIME st{};
        GetLocalTime(&st);

        const std::string path = GetDatDirectory() + "cj_parts.txt";
        std::ofstream f(path.c_str(), std::ios::app);

        if (!f.is_open()) return;

        f << "==== " << st.wYear << "-" << st.wMonth << "-" << st.wDay << " "
          << st.wHour << ":" << st.wMinute << ":" << st.wSecond << " ====\n"
          << "atomics=" << parts.size() << " totalVerts=" << totalVerts << "\n";

        for (size_t i = 0; i < parts.size(); ++i)
        {
            f << "  #" << i
              << " verts=" << parts[i].verts
              << " tris=" << parts[i].tris
              << " tex=" << (parts[i].tex.empty() ? "(none)" : parts[i].tex)
              << "\n";

            for (size_t m = 0; m < parts[i].matVertCount.size(); ++m)
            {
                const std::string tex = (m < parts[i].matName.size())
                                          ? parts[i].matName[m] : std::string();

                f << "    mat[" << m << "]"
                  << " tex=" << (tex.empty() ? "(none)" : tex)
                  << " part=" << PartLabelCn(tex);

                // player.img 的 ID（由 player_img_ids.txt 反查）：模型 30000+ / 纹理 30152+
                if (!tex.empty())
                {
                    const int dffId = FindImgId(tex, false);
                    const int txdId = FindImgId(tex, true);

                    if (dffId >= 0) f << " dff=" << dffId;
                    if (txdId >= 0) f << " txd=" << txdId;
                }

                f << " verts=" << parts[i].matVertCount[m]
                  << " tris=" << (m < parts[i].matTriCount.size() ? parts[i].matTriCount[m] : 0)
                  << "\n";
            }
        }

        // 顶点 -> 材质 映射（供 .dat 生成器"按部件"选区）
        {
            std::ofstream vm((GetDatDirectory() + "cj_vertmat.txt").c_str());

            if (vm.is_open())
            {
                vm << "# vertIndex matIndex\n";

                for (size_t i = 0; i < parts.size(); ++i)
                {
                    for (size_t v = 0; v < parts[i].vertMat.size(); ++v)
                        vm << v << " " << parts[i].vertMat[v] << "\n";
                }
            }
        }

        f << "\n";

        ModLog("[CjParts] changed -> " + path);
    }

void UpdateEntityPhysics(float deltaTime)
{
    if (!g_configLoaded)
        LoadConfig();

    // ------------------------------------------------------------
    //  [LoadGuard] 读档 / 换人 / 重开时的保护
    //
    //  物理更新挂在 drawingEvent 上，加载画面也会触发；读档时玩家 ped 会被
    //  销毁重建、模型重载、实体池清空 —— 我们持有的 atomic/clone 指针会悬空。
    //  这里：玩家指针一变就清空所有 rig，并跳过 60 帧等世界稳定。
    // ------------------------------------------------------------
    static CPlayerPed* s_lastPlayer = nullptr;
    static int s_graceFrames = 0;

    CPlayerPed* playerNow = FindPlayerPed(-1);

    if (playerNow != s_lastPlayer)
    {
        ResetAllRigs(s_lastPlayer ? "player changed" : "player appeared");
        s_lastPlayer = playerNow;
        s_graceFrames = 60;
    }

    if (!playerNow || !playerNow->m_pRwObject)
        return;

    if (s_graceFrames > 0)
    {
        --s_graceFrames;
        return;
    }

    // 2026-11-24 13:0x：perf 计时（每 300 帧打一行 [Perf]），
    // 用来定位"卡"到底是地图实例、实体 rig 还是扫描造成的。
    const double perfStart = []()
    {
        static LARGE_INTEGER freq = {};
        if (freq.QuadPart == 0) QueryPerformanceFrequency(&freq);

        LARGE_INTEGER c = {};
        QueryPerformanceCounter(&c);

        return freq.QuadPart > 0
            ? 1000.0 * static_cast<double>(c.QuadPart) / static_cast<double>(freq.QuadPart)
            : 0.0;
    }();

    double perfAfterMap = perfStart;

    // 风的时间推进（每帧一次，所有 rig 共用）
    AdvanceWindTime(deltaTime);

    ProbeWhitelistedModels();

    // 挂上跨模型碰撞回调（头发 / 布料 会调用它）
    g_worldCollideFn = &WorldCollideThunk;

    // 所有人物的骨胶囊（不只白名单模型）——布料撞"真实人形"的前提
    RegisterAllPedCapsules();

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

    // 地图部分到此为止
    {
        static LARGE_INTEGER freq = {};
        if (freq.QuadPart == 0) QueryPerformanceFrequency(&freq);

        LARGE_INTEGER c = {};
        QueryPerformanceCounter(&c);

        if (freq.QuadPart > 0)
            perfAfterMap = 1000.0 * static_cast<double>(c.QuadPart) / static_cast<double>(freq.QuadPart);
    }

    ++g_frameCounter;

    if (--g_scanCountdown <= 0)
    {
        g_scanCountdown = 30;

        ResolveWhitelist();

        // 记录 CJ 当前部件（换装时自动追加到 cj_parts.txt）
        DumpPlayerParts();

        // 换装 hook：读取 catalog key（部件 ID + 物品哈希）
        if (g_cjPartsLog)
            InstallClothesHook();

        g_candidates.clear();
        CollectFromPool(CPools::ms_pPedPool, g_candidates);
        CollectFromPool(CPools::ms_pVehiclePool, g_candidates);
        CollectFromPool(CPools::ms_pObjectPool, g_candidates);

        // ------------------------------------------------------------
        // 手持武器（2026-11-23 17:0x）
        //
        // 手上武器**不是池里的实体**：它是 CPed::m_pWeaponObject（RwObject*，
        // 挂在手部骨骼上的 clump），所以三个池都扫不到它 ——
        // 这就是"ak47 加白名单/刷了顶点色仍没物理"的原因。
        // 这里遍历 ped 池，把它的武器 clump 原子也加进候选。
        // owner 传 ped：武器 rig 会把它自己的持有者当"自己"排除。
        // ------------------------------------------------------------
        if (CPools::ms_pPedPool)
        {
            struct WeaponCtx
            {
                std::vector<Candidate>* out;
                int      modelIndex;
                CEntity* owner;
            };

            for (int i = 0; i < CPools::ms_pPedPool->m_nSize; ++i)
            {
                CPed* ped = CPools::ms_pPedPool->GetAt(i);
                if (!ped || !ped->m_pWeaponObject) continue;

                if (RwObjectGetType(ped->m_pWeaponObject) != rpCLUMP) continue;

                const CWeapon& slot = ped->m_aWeapons[ped->m_nSelectedWepSlot];

                CWeaponInfo* wi = CWeaponInfo::GetWeaponInfo(slot.m_eWeaponType);
                if (!wi || wi->m_nModelId < 0) continue;

                RpClump* clump = reinterpret_cast<RpClump*>(ped->m_pWeaponObject);

                WeaponCtx wctx{ &g_candidates, wi->m_nModelId, ped };

                RpClumpForAllAtomics(clump,
                    [](RpAtomic* atomic, void* data) -> RpAtomic*
                    {
                        auto* c = static_cast<WeaponCtx*>(data);
                        if (!atomic) return atomic;

                        Candidate cand;
                        cand.atomic     = atomic;
                        cand.modelIndex = c->modelIndex;
                        cand.entity     = c->owner;

                        c->out->push_back(cand);
                        return atomic;
                    },
                    &wctx);
            }
        }

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
                    // 注意顺序：UpdateMapInstances 内部会对每个地图实例调用
                    // WorldCollision_Collect，清空并重写全局碰撞缓存。
                    // 如果先给当前实体 Collect，数据会被地图实例冲掉，
                    // 车辆/人物布料就会拿到"最后一个地图实例周围"的碰撞，
                    // 表现为车辆布料跟人物和地图都没有碰撞。
                    const RwMatrix* ltm = GetAnchorLTM(atomic);
                    if (ltm)
                    {
                        if (g_mapConvert)
                            UpdateMapInstances(ltm->pos, deltaTime);
                        else
                            UpdateMapPhysics(ltm->pos, deltaTime);

                        // 地图实例更新完之后，再为当前实体重新收集碰撞。
                        // 半径至少覆盖该原子自己的包围球：飞机/长车的布料
                        // 可能离 clump 根节点很远，固定 2.5m 会漏掉它周围的对象。
                        const float collectRadius =
                            std::max(g_worldRadius, rig.boundingRadius + 0.5f)
                            + g_collideRadiusExtra;

                        // 传 owner：Collision 里按实体指针判断"自己"。
                        // **载具包含自己**（车体就是 col model，不包含则车上的布直接穿车身），
                        // 人物排除自己（身体由骨胶囊代表）。
                        const bool ownerIsVehicle =
                            (candidate.entity != nullptr &&
                             candidate.entity->m_nType == ENTITY_TYPE_VEHICLE);

                        const bool includeSelf = ownerIsVehicle && g_selfCollide;

                        const double perfCollect0 = PerfMs();
                        WorldCollision_Collect(ltm->pos, collectRadius, candidate.entity,
                                               includeSelf);

                        // ------------------------------------------------------------
                        // CObject 自身碰撞（ini: objselfcol，默认 1）
                        //
                        // 只加"碰撞体"：col 球 + col 盒 + col 三角面（.col 里支持的），
                        // **不加渲染网格** ⇒ 不需要 vertexToParticle 掩码/法线翻转。
                        // 以前物件 owner 走 includeSelf=false，自己的 col 从来没进过
                        // 障碍表，挂在物件上的布会直接穿过它自己。
                        // ------------------------------------------------------------
                        if (g_objSelfCol && candidate.entity &&
                            candidate.entity->m_nType == ENTITY_TYPE_OBJECT)
                        {
                            WorldColModelInfo ci{};

                            const int ownAdded = WorldCollision_AddEntityOwnColModel(
                                candidate.entity, ltm->pos, collectRadius, &ci,
                                g_objSelfMargin);

                            static std::set<int> s_objSelfLogged;

                            if (ownAdded > 0 &&
                                s_objSelfLogged.find(candidate.modelIndex) == s_objSelfLogged.end())
                            {
                                s_objSelfLogged.insert(candidate.modelIndex);

                                std::ostringstream sc;
                                sc << "[ObjSelfCol] modelId=" << candidate.modelIndex
                                   << " added=" << ownAdded
                                   << " spheres=" << ci.spheres
                                   << " boxes=" << ci.boxes
                                   << " tris=" << ci.tris;
                                ModLog(sc.str());
                            }
                        }

                        // 三角面空间索引（收集完再建；Step 里只扫本格）
                        WorldCollision_BuildTriangleGrid();
                        g_perfCollectMs += PerfMs() - perfCollect0;
                    }

                    // 自身碰撞用的是骨骼胶囊：必须让骨架先刷到本帧，
                    // 否则跑动时胶囊会落后一帧（见 SyncAnimPose 注释）
                    if (!rig.colliders.empty() && candidate.entity)
                    {
                        SyncAnimPose(candidate.entity, atomic, candidate.modelIndex,
                            static_cast<int>(rig.skinBoneCount));
                    }

                    // 自己物件网格的自适应 skin + 扫掠开关（对所有 rig 设一次，
                    // 没布料时 clothMeanEdge=0 → clamp 到 4cm，无副作用）
                    //
                    // skin 上限 8cm：车布边长可能到 30cm+，按 0.45×边长 算会到 12cm，
                    // 那样布料会离车身太远（浮空）。8cm 是"不漏"和"贴合"的折中。
                    WorldCollision_SetOwnColSkin(
                        std::clamp(0.45f * rig.clothMeanEdge, 0.04f, 0.08f));
                    WorldCollision_SetOwnColSweep(g_clothSweep ? 1 : 0);

                    const double perfStep0 = PerfMs();
                    StepGeometryPhysics(rig, atomic, deltaTime);
                    g_perfStepMs += PerfMs() - perfStep0;

                    const double perfApply0 = PerfMs();
                    ApplyGeometryPhysics(rig, atomic);
                    g_perfApplyMs += PerfMs() - perfApply0;

                    // ------------------------------------------------------------
                    // [BodyDiag] Body（R=0xC0）分组诊断（每 300 帧，写主日志）
                    //
                    // 存在的意义：Body 抖动的位移是"每组一个刚性 offset"，
                    // 所以只要 groups 少、或者各组 offset 相同，左胸/右胸/腹
                    // 就会往同一个方向抖。这里直接把组数和各组的 offset 打出来。
                    // ------------------------------------------------------------
                    if (rig.bodyCount > 0 && (g_frameCounter % 300) == 0)
                    {
                        std::ostringstream bd;
                        bd << std::fixed << std::setprecision(4);
                        bd << "[BodyDiag] modelId=" << candidate.modelIndex
                           << " body=" << rig.bodyCount
                           << " groups=" << rig.bodyGroups.size();

                        int shown = 0;

                        for (const auto& kv : rig.bodyGroups)
                        {
                            if (shown++ >= 6) break;

                            bd << " g" << kv.first
                               << "(" << kv.second.offset.x
                               << "," << kv.second.offset.y
                               << "," << kv.second.offset.z << ")";
                        }

                        ModLog(bd.str());
                    }

                    // ------------------------------------------------------------
                    // 人物骨胶囊 -> 世界障碍（2026-11-23 09:2x）
                    // "人推车布不明显"的根因是人物 col model 只有一个粗盒；
                    // 把 ped rig 本帧的 22 根骨胶囊注册出去，别的 rig 的布料
                    // 撞到的就是**真实人形**。胶囊在 Step 之后才有本帧世界位置。
                    // ------------------------------------------------------------
                    if (!rig.colliders.empty() && candidate.entity)
                    {
                        static std::vector<WorldCapsule> s_capBuf;
                        s_capBuf.clear();

                        for (const auto& c : rig.colliders)
                        {
                            if (c.radius <= 0.0f) continue;

                            WorldCapsule w;
                            w.ax = c.aWorld.x; w.ay = c.aWorld.y; w.az = c.aWorld.z;
                            w.bx = c.bWorld.x; w.by = c.bWorld.y; w.bz = c.bWorld.z;
                            w.radius = c.radius;
                            w.vx = 0.0f; w.vy = 0.0f; w.vz = 0.0f;
                            w.modelId = candidate.modelIndex;

                            s_capBuf.push_back(w);
                        }

                        WorldCollision_SetEntityCapsules(
                            candidate.entity,
                            s_capBuf.empty() ? nullptr : s_capBuf.data(),
                            static_cast<int>(s_capBuf.size()));
                    }
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

        // ------------------------------------------------------------
        // 逐 atomic 清单（2026-11-24 05:5x）
        //
        // 玩家（模型 0）由多个 atomic 组成（头/躯干/腿/衣服），
        // 以前只按 modelId 打一条日志，看不出"哪个 atomic 有标记色"。
        // 这里按**几何体指针**去重，每个 atomic 各打一条：
        //   [AtomicScan] modelId=0 geom=0x.. verts=2081 markers=0
        // ------------------------------------------------------------
        if (IsModelEnabled(candidate.modelIndex) && markerCount <= 0)
        {
            const std::string key = "a:" + std::to_string(candidate.modelIndex) + ":"
                                  + std::to_string(reinterpret_cast<unsigned long long>(current));

            if (g_reportedModels.insert(key).second)
            {
                std::ostringstream oss;
                oss << "[AtomicScan] modelId=" << candidate.modelIndex
                    << " geom=" << static_cast<void*>(current)
                    << " verts=" << current->numVertices
                    << " tris=" << current->numTriangles
                    << " markers=" << markerCount;

                // 采样 4 个顶点色：确认拼装后的几何体里到底是什么颜色
                // （prelight 被全白覆盖 / 颜色丢失，都能一眼看出来）
                {
                    const RwRGBA* cols = current->preLitLum;

                    if (cols && current->numVertices > 0)
                    {
                        const int n = static_cast<int>(current->numVertices);

                        const int idx[4] = { 0, n / 4, n / 2, (3 * n) / 4 };

                        oss << " cols=";

                        for (int k = 0; k < 4; ++k)
                        {
                            const RwRGBA& c = cols[idx[k]];
                            oss << "(" << static_cast<int>(c.red) << ","
                                << static_cast<int>(c.green) << ","
                                << static_cast<int>(c.blue) << ","
                                << static_cast<int>(c.alpha) << ")";
                        }
                    }
                }

                ModLog(oss.str());
            }
        }
        const bool hasDat = (GetPhysicsDat(candidate.modelIndex, static_cast<int>(current->numVertices),
                                           RpMorphTargetGetVertices(&current->morphTarget[0])) != nullptr);

        // datOnly：只看外部 .dat，忽略模型自带顶点色
        if (g_datOnly)
        {
            if (!hasDat) continue;
        }
        else if (markerCount <= 0 && !hasDat)
        {
            // 2026-11-23 16:5x：白名单里、但读不到物理标记色 —— 以前这里静默 continue，
            // 完全看不出"模型没进候选"和"进了但颜色没读到"的区别。
            if (IsModelEnabled(candidate.modelIndex) &&
                g_reportedModels.insert("nomk:" + std::to_string(candidate.modelIndex)).second)
            {
                std::ostringstream oss;
                oss << "[Entity] nomarkers modelId=" << candidate.modelIndex
                    << " verts=" << current->numVertices
                    << " (白名单已启用，但 preLitLum 里没有物理标记色；"
                    << "可用 dat 或重新导出顶点色)";
                ModLog(oss.str());
            }

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
        entry.rig.modelId = candidate.modelIndex;

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

    // ---- [Perf] 每 300 帧汇总一次（全部换算成每帧平均）----
    ++g_perfFrames;
    g_perfUpdateMs += (PerfMs() - perfStart);
    g_perfMapMs    += (perfAfterMap - perfStart);
    g_perfEntMs    += (PerfMs() - perfAfterMap);

    if ((g_frameCounter % 300) == 0)
    {
        const double frames = (g_perfFrames > 0) ? static_cast<double>(g_perfFrames) : 1.0;

        size_t particles = 0;
        for (const auto& e : g_entries) particles += e.second.rig.particles.size();
        for (const auto& e : g_mapInstances) particles += e.second.rig.particles.size();

        std::ostringstream oss;
        oss << "[Perf] update=" << (g_perfUpdateMs / frames)
            << "ms map=" << (g_perfMapMs / frames)
            << "ms ent=" << (g_perfEntMs / frames)
            << "ms | collect=" << (g_perfCollectMs / frames)
            << " step=" << (g_perfStepMs / frames)
            << " apply=" << (g_perfApplyMs / frames)
            << " rest=" << ((g_perfEntMs - g_perfCollectMs - g_perfStepMs - g_perfApplyMs) / frames)
            << "ms/frame rigs=" << g_entries.size()
            << " inst=" << g_mapInstances.size()
            << " particles=" << particles;
        ModLog(oss.str());

        g_perfCollectMs = 0.0;
        g_perfStepMs = 0.0;
        g_perfApplyMs = 0.0;
        g_perfUpdateMs = 0.0;
        g_perfMapMs = 0.0;
        g_perfEntMs = 0.0;
        g_perfFrames = 0;
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


