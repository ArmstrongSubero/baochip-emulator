@echo off
rem build.bat: build baoemu.exe and baoemu.dll on Windows
rem
rem Copyright (c) 2026 Armstrong Subero
rem SPDX-License-Identifier: Apache-2.0
rem
rem Uses gcc from PATH (w64devkit or MinGW) when present, otherwise the
rem clang that ships with "python -m pip install ziglang".

setlocal
set SRC=src\core.c src\cpu.c src\bus.c src\udma.c src\sce.c src\devices.c src\bio.c src\pke.c src\pwm.c src\mdma.c src\mbox.c src\usb.c
if not exist build mkdir build

where gcc >nul 2>nul
if %errorlevel%==0 (
    set CC=gcc
) else (
    set CC=python -m ziglang cc -target x86_64-windows-gnu
)

%CC% -O2 -std=c99 -Wall -o build\baoemu.exe %SRC% src\main.c
if errorlevel 1 goto fail
%CC% -O2 -std=c99 -Wall -shared -o build\baoemu.dll %SRC%
if errorlevel 1 goto fail
copy /y build\baoemu.dll python\ >nul
echo Built build\baoemu.exe and build\baoemu.dll
goto end

:fail
echo Build failed.
exit /b 1

:end
endlocal
