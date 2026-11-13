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

void WorldCollision_Collect(const RwV3d& center, float radius);

// prev：粒子上一帧的世界位置（用来判断"从哪一侧进入"，避免弹错方向）
void WorldCollision_CollidePoint(RwV3d& point, const RwV3d& prev, float margin);

void WorldCollision_Clear();

// 上一次 CollidePoint 是否发生了接触（用于施加摩擦）
bool WorldCollision_LastHit();

// 接触时障碍物的速度（用于把车辆/人物的速度传给布料）
void WorldCollision_LastImpulse(RwV3d& out);
