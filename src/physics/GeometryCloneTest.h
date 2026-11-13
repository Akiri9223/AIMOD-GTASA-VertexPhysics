#pragma once

#include "RenderWare.h"
#include <string>

// true  = STEP I 持久化模式：clone 永久挂载，顶点写回由物理系统接管
// false = 原 120 帧测试模式：测试完恢复并销毁
constexpr bool kPersistentPhysics = true;

// 原有 STEP C-H 一次性克隆测试
bool RunGeometryStreamCloneTest(
    RpAtomic* targetAtomic,
    RpGeometry* source,
    std::string& report
);

// STEP I：创建并挂载持久化 Geometry clone
bool BeginGeometryVertexTest(
    RpAtomic* atomic,
    std::string& report
);

// STEP I：每帧更新
// - 持久化模式：只保活 + 打 source/clone 对比日志，不动顶点
// - 非持久化模式：改顶点，120 帧后恢复并销毁
bool UpdateGeometryVertexTest(
    RpAtomic* atomic,
    std::string& report
);

// STEP I：检查 Atomic 是否正在进行跨帧测试
bool IsGeometryVertexTestActive(
    RpAtomic* atomic
);