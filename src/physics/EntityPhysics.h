#pragma once

#include "RenderWare.h"

// ============================================================
// 通用实体物理驱动
//
// 每帧扫描 人物 / 车辆 / 物体 池，找出几何体里带物理顶点色
// (R = 0x40/0x80/0xC0/0xFF，或旧标记 0x03) 的原子，
// 给每个原子挂一份私有几何体克隆，然后跑物理并写回。
//
// 有蒙皮（人物）用骨骼矩阵锚定；没有蒙皮（车辆 / 物体）用
// 原子的世界矩阵锚定，走刚体换算。
// ============================================================

void UpdateEntityPhysics(float deltaTime);

// 统计信息（日志用）
int  GetEntityPhysicsRigCount();
void LogEntityPhysicsSummary();
