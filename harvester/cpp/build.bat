@echo off
rem Builds planet_buildings.exe: libosmium + protozero + zlib from a conda-forge prefix (OSMIUM_PREFIX,
rem default D:\Tools\osmium -- `micromamba create -p D:\Tools\osmium -c conda-forge osmium-tool
rem libosmium protozero zlib`). Not part of the engine's build: the harvester's own tool.
setlocal
set "HERE=%~dp0"
if "%HERE:~-1%"=="\" set "HERE=%HERE:~0,-1%"
if not defined OSMIUM_PREFIX set "OSMIUM_PREFIX=D:\Tools\osmium"
set "INC=%OSMIUM_PREFIX%\Library\include"
set "LIB=%OSMIUM_PREFIX%\Library\lib"
if not exist "%INC%\osmium\io\pbf_input.hpp" (
  echo ERROR: no libosmium headers under "%INC%"
  exit /b 1
)
set "VSWHERE=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe"
for /f "usebackq tokens=*" %%i in (`"%VSWHERE%" -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath`) do set "VSPATH=%%i"
call "%VSPATH%\VC\Auxiliary\Build\vcvars64.bat" >nul || exit /b 1
if not exist "%HERE%\bin" mkdir "%HERE%\bin"
cl /nologo /O2 /std:c++20 /EHsc /MD /W3 /DNOMINMAX /D_CRT_SECURE_NO_WARNINGS /I"%INC%" ^
   "%HERE%\planet_buildings.cpp" /Fo"%HERE%\bin\\" /Fe"%HERE%\bin\planet_buildings.exe" ^
   /link /LIBPATH:"%LIB%" zlibstatic.lib ws2_32.lib || exit /b 1
echo built: %HERE%\bin\planet_buildings.exe
