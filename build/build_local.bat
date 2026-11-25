@echo off
rem 2026-11-25: build_all.bat is now location-independent (%~dp0 based).
rem build_local.bat used to duplicate the whole command line with hardcoded
rem B:\HairPhysicsASI paths; keep it as a thin wrapper so there is one source of truth.
call "%~dp0build_all.bat"
exit /b %ERRORLEVEL%
