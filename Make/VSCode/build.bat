@ECHO OFF
REM Finds the path of the installed MSBuild.exe and the uses that to build a project from as solution.
SETLOCAL
REM Parameters:
REM solution (first parameter): The path to the .sln file to build
REM config (second parameter): The build config (Debug, Develop, Release)
REM target (third parameter, optional): The target to build.
SET solution=%1
SET config=/p:Configuration=%2
REM target parameter is optional, if missing the entire solution will be build
IF [%3]==[] (
  SET target=
) ELSE (
  SET target=/t:%3
)

ECHO Compiling %solution%:%target% with %config% configuration.

SET find_msbuild_command="%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe" -products * -version 16.0 -requires Microsoft.Component.MSBuild -find MSBuild\**\Bin\MSBuild.exe
for /f "usebackq tokens=*" %%i in (`%find_msbuild_command%`) do (
  REM vswhere finds the location of the msbuild.exe
  REM %%i in this loop holds the path to msbuild.exe
  SET command=("%%i" %target% %config% /m /v:m /nologo %solution%)
  ECHO Executing command: %command%
  %command%
  exit /b !errorlevel!
)

