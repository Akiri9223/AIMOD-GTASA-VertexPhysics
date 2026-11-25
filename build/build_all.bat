@echo off
rem ============================================================
rem  Location-independent build script (2026-11-25)
rem  Uses %~dp0 (this script's dir = build\) to derive:
rem      BUILD_DIR = <project>\build\
rem      ROOT      = <project>\
rem  All paths are quoted, so the project folder may contain spaces.
rem  NOTE: ASCII only -- cmd.exe would misread UTF-8 Chinese comments.
rem ============================================================
setlocal

set "BUILD_DIR=%~dp0"
for %%I in ("%BUILD_DIR%..") do set "ROOT=%%~fI"

set "PLUGIN_SDK=B:\plugin-sdk-master"
set "VCTOOLS=B:\Visual Studio\cache\VC\Tools\MSVC\14.51.36231\bin\Hostx86\x86"

call "B:\Visual Studio\cache\VC\Auxiliary\Build\vcvarsall.bat" x86 >nul
cd /d "%BUILD_DIR%"

set "CLFLAGS=/nologo /TP -DGTASA -DHairPhysicsASI_EXPORTS -DPLUGIN_SGV_10US -DRW "-I%PLUGIN_SDK%\plugin_sa" "-I%PLUGIN_SDK%\plugin_sa\game_sa" "-I%PLUGIN_SDK%\plugin_sa\game_sa\enums" "-I%PLUGIN_SDK%\plugin_sa\game_sa\rw" "-I%PLUGIN_SDK%\shared" "-I%PLUGIN_SDK%\shared\game" "-I%ROOT%\src" "-I%PLUGIN_SDK%\safetyhook" /DWIN32 /D_WINDOWS /EHsc /Z7 /Ob0 -std:c++latest -MTd /utf-8 /O2 /Ob2 /Oi /Gy /GS-"

set "CL_EXE=%VCTOOLS%\cl.exe"
set "LINK_EXE=%VCTOOLS%\link.exe"
set "OBJDIR=CMakeFiles\HairPhysicsASI.dir"

echo === compile GeometryPhysics === >> compile_all.log
"%CL_EXE%" %CLFLAGS% "/Fo%OBJDIR%\src\physics\GeometryPhysics.cpp.obj" "/Fd%OBJDIR%\\" -c "%ROOT%\src\physics\GeometryPhysics.cpp" >> compile_all.log 2>&1
if errorlevel 1 exit /b 2

echo === compile EntityPhysics === >> compile_all.log
"%CL_EXE%" %CLFLAGS% "/Fo%OBJDIR%\src\physics\EntityPhysics.cpp.obj" "/Fd%OBJDIR%\\" -c "%ROOT%\src\physics\EntityPhysics.cpp" >> compile_all.log 2>&1
if errorlevel 1 exit /b 2

echo === compile WorldCollision === >> compile_all.log
"%CL_EXE%" %CLFLAGS% "/Fo%OBJDIR%\src\physics\WorldCollision.cpp.obj" "/Fd%OBJDIR%\\" -c "%ROOT%\src\physics\WorldCollision.cpp" >> compile_all.log 2>&1
if errorlevel 1 exit /b 2

echo === compile GeometryCloneTest === >> compile_all.log
"%CL_EXE%" %CLFLAGS% "/Fo%OBJDIR%\src\physics\GeometryCloneTest.cpp.obj" "/Fd%OBJDIR%\\" -c "%ROOT%\src\physics\GeometryCloneTest.cpp" >> compile_all.log 2>&1
if errorlevel 1 exit /b 2

echo === link === >> compile_all.log
"%LINK_EXE%" /nologo "%OBJDIR%\src\main.cpp.obj" "%OBJDIR%\src\physics\GeometryPhysics.cpp.obj" "%OBJDIR%\src\physics\EntityPhysics.cpp.obj" "%OBJDIR%\src\physics\WorldCollision.cpp.obj" "%OBJDIR%\src\physics\GeometryCloneTest.cpp.obj" "%OBJDIR%\src\physics\AtomicCloneTest.cpp.obj" "%OBJDIR%\B_\plugin-sdk-master\safetyhook\safetyhook.cpp.obj" "/out:%BUILD_DIR%VertexPhysics.asi" "/implib:%BUILD_DIR%VertexPhysics.lib" /dll /version:0.0 /machine:X86 /INCREMENTAL:NO /DEBUG:NONE "%PLUGIN_SDK%\output\lib\Plugin_d.lib" "%PLUGIN_SDK%\shared\dxsdk\d3dx9.lib" kernel32.lib user32.lib gdi32.lib winspool.lib shell32.lib ole32.lib oleaut32.lib uuid.lib comdlg32.lib advapi32.lib >> compile_all.log 2>&1
if errorlevel 1 exit /b 3

exit /b 0
