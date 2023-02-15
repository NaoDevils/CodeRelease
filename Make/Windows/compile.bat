@echo off

set argC=0
for %%x in (%*) do Set /A argC+=1

cd "%~dp0..\.."

IF %argC% EQU 0 (
    set preset=simulator-develop
) ELSE (
    set preset=%1
)

IF %argC% GTR 1 (
    set target=--target %2
) ELSE (
    set target=
)

IF %argC% GTR 2 (
    set /a nextversion=%3+1
)
IF %argC% GTR 2 (
    set version=-version [%3^^^,%nextversion%^^^)
) ELSE (
    set version=-latest
)

for /f "delims=" %%i in ('"%ProgramFiles(x86)%\\Microsoft Visual Studio\\Installer\\vswhere.exe" %version% -find **\vcvars64.bat -products *') do set ENVIRONMENT=%%i
call "%ENVIRONMENT%"

REM Do not configure CMake if CMakeCache is present.
REM We assume Ninja build system, which runs CMake automatically when necessary.

if not x%preset:simulator-multiconfig-=%==x%preset% (
    set configurepreset=simulator-multiconfig
) else (
    set configurepreset=%preset%
)

if not exist "Build\%configurepreset%\build.ninja" (
    cmake --preset %configurepreset%
)

cmake --build --preset %preset% %target%
