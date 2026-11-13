
cmake_minimum_required(VERSION 3.20)

cmake_policy(SET CMP0091 NEW)

project(HairPhysicsASI LANGUAGES CXX)

# MSVC 的 /showIncludes 提示在本机被探测成本地化文本，
# 但 cl 实际输出英文，导致 ninja 记录不到头文件依赖 ——
# 改头文件后不会重编所有 .obj，结构体布局不一致会直接崩栈。
# 这里统一强制英文前缀，保证增量编译正确。
if(MSVC)
    set(CMAKE_CL_SHOWINCLUDES_PREFIX "Note: including file: " CACHE STRING "" FORCE)
    set(CMAKE_CXX_CL_SHOWINCLUDES_PREFIX "Note: including file: " CACHE STRING "" FORCE)
endif()

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 设置一个默认路径，但允许用户在配置时修改
if(NOT DEFINED PLUGIN_SDK_DIR)
    set(PLUGIN_SDK_DIR "B:/plugin-sdk-master" CACHE PATH "Path to the GTA SA Plugin SDK")
endif()

add_library(HairPhysicsASI SHARED
    src/main.cpp
	src/physics/GeometryPhysics.cpp
	src/physics/EntityPhysics.cpp
	src/physics/WorldCollision.cpp
	src/physics/GeometryCloneTest.cpp
	src/physics/AtomicCloneTest.cpp
    "${PLUGIN_SDK_DIR}/safetyhook/safetyhook.cpp"
    "${PLUGIN_SDK_DIR}/safetyhook/Zydis.c"
)

# Plugin-SDK 头文件目录
target_include_directories(HairPhysicsASI PRIVATE
    "${PLUGIN_SDK_DIR}/plugin_sa"
    "${PLUGIN_SDK_DIR}/plugin_sa/game_sa"
    "${PLUGIN_SDK_DIR}/plugin_sa/game_sa/enums"
    "${PLUGIN_SDK_DIR}/plugin_sa/game_sa/rw"
    "${PLUGIN_SDK_DIR}/shared"
    "${PLUGIN_SDK_DIR}/shared/game"
	"${CMAKE_SOURCE_DIR}/src"
    "${PLUGIN_SDK_DIR}/safetyhook"
)

# GTA SA 1.0 US
target_compile_definitions(HairPhysicsASI PRIVATE
    GTASA
    PLUGIN_SGV_10US
    RW
)

# 明确链接现有的 Debug 库
target_link_libraries(HairPhysicsASI PRIVATE
    "${PLUGIN_SDK_DIR}/output/lib/Plugin_d.lib"
    "${PLUGIN_SDK_DIR}/shared/dxsdk/d3dx9.lib"
)

if(MSVC)
    target_compile_options(HairPhysicsASI PRIVATE /utf-8)

    # 关键：Debug 配置默认 /Od + /RTC1，物理循环会慢 5~20 倍
    # （实测 map 实例初始化 322 ms）。这里强制优化，但保留 Debug 运行库，
    # 以便继续链接 Plugin_d.lib。
    target_compile_options(HairPhysicsASI PRIVATE /O2 /Ob2 /Oi /Gy /GS-)

    # 确保 /O2 覆盖 Debug 默认的 /Od
    string(REGEX REPLACE "/Od" "" CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}")
    string(REGEX REPLACE "/RTC1" "" CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}")

    set_property(TARGET HairPhysicsASI PROPERTY
        MSVC_RUNTIME_LIBRARY "MultiThreadedDebug"
    )
endif()

set_target_properties(HairPhysicsASI PROPERTIES
    OUTPUT_NAME "VertexPhysics"
    PREFIX ""
    SUFFIX ".asi"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/output"
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/output"
)
