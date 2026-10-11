@echo off
rem Builds one harvester: build.bat <name> [more names...]  (default: every .cpp here) -> bin\<name>.exe
rem libosmium + protozero + zlib from a conda-forge prefix (OSMIUM_PREFIX, default D:\Tools\osmium --
rem `micromamba create -p D:\Tools\osmium -c conda-forge osmium-tool libosmium protozero zlib`), needed
rem only by the harvesters that include osmium/; the others use nothing beyond the standard library.
rem Not part of the engine's build: the harvesters' own tool.
setlocal
set "HERE=%~dp0"
if "%HERE:~-1%"=="\" set "HERE=%HERE:~0,-1%"
if not defined OSMIUM_PREFIX set "OSMIUM_PREFIX=D:\Tools\osmium"
set "INC=%OSMIUM_PREFIX%\Library\include"
set "LIB=%OSMIUM_PREFIX%\Library\lib"
rem The compiler: the newest vcvars64.bat under Program Files, called BEFORE any delayed expansion is
rem enabled (VsDevCmd's own scripts break under it, failing at their vswhere call). vswhere is not
rem used here either: its path holds "(x86)", whose ")" ends a for /f set.
set "VCVARS="
for /d %%v in ("%ProgramFiles%\Microsoft Visual Studio\*") do for /d %%e in ("%%v\*") do if exist "%%e\VC\Auxiliary\Build\vcvars64.bat" set "VCVARS=%%e\VC\Auxiliary\Build\vcvars64.bat"
if not defined VCVARS (
  echo ERROR: no vcvars64.bat under "%ProgramFiles%\Microsoft Visual Studio"
  exit /b 1
)
rem (VsDevCmd prints a harmless "'vswhere.exe' is not recognized" under some shells: silenced.)
call "%VCVARS%" >nul 2>&1 || exit /b 1
where cl.exe >nul 2>&1 || (
  echo ERROR: cl.exe is not on PATH after "%VCVARS%"
  exit /b 1
)
if not exist "%HERE%\bin" mkdir "%HERE%\bin"
set "NAMES=%*"
if "%NAMES%"=="" for %%f in ("%HERE%\*.cpp") do call set "NAMES=%%NAMES%% %%~nf"
for %%n in (%NAMES%) do call :one %%n || exit /b 1
exit /b 0

:one
set "EXTRA="
findstr /m /c:"osmium/" "%HERE%\%1.cpp" >nul && set "EXTRA=/I"%INC%" /link /LIBPATH:"%LIB%" zlibstatic.lib ws2_32.lib"
if defined EXTRA if not exist "%INC%\osmium\io\pbf_input.hpp" (
  echo ERROR: %1 needs libosmium headers under "%INC%"
  exit /b 1
)
cl /nologo /O2 /std:c++20 /EHsc /MD /W3 /DNOMINMAX /D_CRT_SECURE_NO_WARNINGS "%HERE%\%1.cpp" /Fo"%HERE%\bin\\" /Fe"%HERE%\bin\%1.exe" %EXTRA% || exit /b 1
echo built: %HERE%\bin\%1.exe
exit /b 0
