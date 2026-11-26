@echo off
setlocal
cd /d "%~dp0"
if not exist build mkdir build
pushd build
cl /nologo /utf-8 /std:c11 /O2 /W3 /D_CRT_SECURE_NO_WARNINGS ..\c\robot.c ..\c\policy.c ..\c\devices.c ..\c\ipc.c /Ferobot.exe /link ws2_32.lib
if errorlevel 1 exit /b 1
cl /nologo /utf-8 /std:c11 /O2 /W3 /LD ..\c\ipc.c /Fepipeipc.dll
if errorlevel 1 exit /b 1
popd
