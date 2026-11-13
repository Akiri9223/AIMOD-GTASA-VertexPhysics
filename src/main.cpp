#include "RenderWare.h"
#include "plugin.h"
#include "Events.h"
#include "common.h"

#include "physics/EntityPhysics.h"

#include <algorithm>
#include <fstream>
#include <string>
#include <windows.h>

// ============================================================
// 通用物理 Mod 入口
//
// 不再绑定任何模型名：每帧扫描 人物 / 车辆 / 物体 池，
// 只要几何体的顶点色带物理标记（R = 0x40/0x80/0xC0/0xFF，
// 或旧标记 0x03），就为它建立私有几何体克隆并驱动物理。
// ============================================================

namespace
{
    void WriteLog(const char* text)
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

        std::ofstream f(path.c_str(), std::ios::out | std::ios::app);
        if (f.is_open())
            f << text << std::endl;
    }
}


class GenericPhysicsMod {
public:
    GenericPhysicsMod() {
        WriteLog("--- HairPhysics (generic entities: peds / vehicles / objects) Init ---");

        plugin::Events::drawingEvent.before += []() {
            static LARGE_INTEGER s_freq = {};
            static LARGE_INTEGER s_last = {};

            if (s_freq.QuadPart == 0)
                QueryPerformanceFrequency(&s_freq);

            LARGE_INTEGER now = {};
            QueryPerformanceCounter(&now);

            float delta = 1.0f / 60.0f;

            if (s_last.QuadPart != 0 && s_freq.QuadPart > 0)
            {
                delta = static_cast<float>(
                    static_cast<double>(now.QuadPart - s_last.QuadPart) /
                    static_cast<double>(s_freq.QuadPart));
            }

            s_last = now;
            delta = std::clamp(delta, 1.0f / 240.0f, 1.0f / 20.0f);

            UpdateEntityPhysics(delta);
        };
    }
};

GenericPhysicsMod genericPhysicsMod;
