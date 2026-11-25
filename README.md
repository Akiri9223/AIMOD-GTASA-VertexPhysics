# AIMOD-GTASA-VertexPhysics
这是什么：GTA SA 1.0 US 的顶点色驱动布料/头发/身体/软体物理 ASI；

依赖：plugin-sdk（默认放在 B:\plugin-sdk-master，需自行修改 build_all.bat 里的 PLUGIN_SDK）、Visual Studio x86 工具链；

编译：build\build_all.bat → build\VertexPhysics.asi；

安装：ASI 放到游戏根目录 + vertexphysics.ini；

顶点色规则：R=0x40 头发 / R=0x80 布料 / R=0xC0 身体 / 未着色=刚性，G=软度，B=锚定强度（B=0 为硬锚定）；

# Cloth Physics
<img width="480" height="270" alt="2026-11-26 05-35-06" src="https://github.com/user-attachments/assets/b327a1ae-6a7e-49f1-9726-ea80ef300e8e" />
