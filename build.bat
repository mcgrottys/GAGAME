@echo off
rem Configure + build gagame with the newest installed VS x64 toolchain and Ninja.
rem Uses vswhere rather than a hardcoded VS path (vqview's build.bat pinned VS2022 Professional,
rem which does not exist on this machine; this one finds whatever is actually installed).
rem
rem TRAP (inherited): %~dp0 ends with a backslash, so "%~dp0" escapes its own closing quote.
setlocal
set "HERE=%~dp0"
if "%HERE:~-1%"=="\" set "HERE=%HERE:~0,-1%"

set "VSWHERE=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe"
if not exist "%VSWHERE%" (
  echo ERROR: vswhere.exe not found at "%VSWHERE%"
  exit /b 1
)
set "VSPATH="
for /f "usebackq tokens=*" %%i in (`"%VSWHERE%" -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath`) do set "VSPATH=%%i"
if not defined VSPATH (
  echo ERROR: no Visual Studio with the C++ x64 toolset was found
  exit /b 1
)

call "%VSPATH%\VC\Auxiliary\Build\vcvars64.bat" >nul
if errorlevel 1 (
  echo ERROR: vcvars64.bat failed
  exit /b 1
)
where cl.exe >nul 2>&1
if errorlevel 1 (
  echo ERROR: cl.exe is not on PATH after vcvars
  exit /b 1
)

rem cmake is not on the global PATH here; VS ships one.
where cmake.exe >nul 2>&1
if errorlevel 1 set "PATH=%VSPATH%\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin;%PATH%"
where cmake.exe >nul 2>&1
if errorlevel 1 (
  echo ERROR: cmake.exe not found ^(install the C++ CMake tools VS component^)
  exit /b 1
)

cmake -S "%HERE%" -B "%HERE%\build" -G Ninja -DCMAKE_BUILD_TYPE=RelWithDebInfo
if errorlevel 1 exit /b 1

cmake --build "%HERE%\build" --parallel
if errorlevel 1 exit /b 1

echo.
echo built: %HERE%\build\bin\gagame.exe
