#pragma once

#include "RenderWare.h"

#include <string>

// 对第一个包含头发标记顶点的 Atomic 执行一次只读克隆测试。
// 不替换源 Atomic，不加入 Clump，不销毁克隆对象。
bool RunAtomicCloneReadOnlyTest(
    RpAtomic* source,
    std::string& report
);