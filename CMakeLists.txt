cmake_minimum_required(VERSION 3.20)
project(gagame LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

if (MSVC)
  add_compile_options(/W4 /permissive- /Zi /utf-8 /MP)
  add_link_options(/DEBUG)
  # D3D12 headers trip these and they are not actionable in our code
  add_compile_options(/wd4324 /wd4127)
endif()

add_executable(gagame
  src/main.cpp
  src/app/Options.cpp
  src/app/Scene.cpp
  src/app/Assembly.cpp
  src/app/FrameLoop.cpp
  src/app/Tools/PackTiles.cpp
  src/app/Tools/LoadField.cpp
  src/app/Tools/SelfTest.cpp
  src/app/Tools/WaterMap.cpp
  src/app/Tools/GisDump.cpp
  src/app/Tools/TreeAudit.cpp
  src/app/Tools/TreePrune.cpp
  src/app/Tools/FidelityMap.cpp
  src/app/Tools/OceanProbe.cpp
  src/app/Tools/SweUv.cpp
  src/app/Tools/Export.cpp
  src/app/Tools/WarmInlet.cpp
  src/app/Tools/DumpWaterState.cpp
  src/app/Tools/TwinSurface.cpp
  src/app/Tools/Trace.cpp
  src/app/Tools/SeaVerify.cpp
  src/app/Tools/WaterProbe.cpp
  src/app/Tools/HullProbe.cpp
  src/app/Tools/BedTrace.cpp
  src/core/BuildInfo.cpp
  src/core/GaAst.cpp
  src/core/Lattice.cpp
  src/core/SpaceTest.cpp
  src/hal/PixEvents.cpp
  src/core/GaTest.cpp
  src/hal/DxTest.cpp
  src/core/CrashTrace.cpp
  src/hal/Gpu.cpp
  src/hal/GpuProfiler.cpp
  src/hal/Readback.cpp
  src/hal/Shader.cpp
  src/core/Window.cpp
  src/core/Image.cpp
  src/core/ImageLoader.cpp
  src/hal/TileAtlas.cpp
  src/hal/Residency.cpp
  src/hal/ResidencyAudit.cpp
  src/hal/ResidencyOrder.cpp
  src/hal/ResidencyTest.cpp
  src/hal/Tenant.cpp
  src/hal/TenantTest.cpp
  src/core/TileProviders.cpp
  src/core/DayLedger.cpp
  src/core/DayLedgerTest.cpp
  src/core/ThreadTest.cpp
  src/core/ThreadManager.cpp
  src/compose/Compositor.cpp
  src/compose/SurfaceFrame.cpp
  src/compose/ComposeTest.cpp
  src/compose/TreePrune.cpp
  src/compose/TreePruneTest.cpp
  src/compose/TileTreeTest.cpp
  src/compose/Sources.cpp
  src/compose/RasterFileSource.cpp
  src/compose/RasterFileTest.cpp
  src/compose/GisMask.cpp
  src/compose/GisStencil.cpp
  src/compose/VectorPack.cpp
  src/compose/Exchange.cpp
  src/compose/WaterAtlas.cpp
  src/core/OceanFft.cpp
  src/render/Renderer.cpp
  src/render/Camera.cpp
  src/scene/FieldSet.cpp
  src/scene/Props.cpp
  src/scene/Node.cpp
  src/scene/Pose.cpp
  src/scene/View.cpp
  src/scene/SceneSchema.cpp
  src/scene/SceneBuilder.cpp
  src/scene/SceneReload.cpp
  src/scene/SchemaDoc.cpp
  src/scene/WaterComponent.cpp
  src/scene/Rail.cpp
  src/scene/Entity.cpp
  src/scene/Portal.cpp
  src/scene/Gateway.cpp
  src/scene/effects/SlicePlane.cpp
  src/scene/SceneTest.cpp
  src/scene/SkyLayer.cpp
  src/scene/TideLayer.cpp
  src/scene/SeaLayer.cpp
  src/scene/GulfLayer.cpp
  src/scene/GisLayer.cpp
  src/scene/HudLayer.cpp
  src/scene/MarkerLayer.cpp
  src/scene/VesselLayer.cpp
  src/sim/TideModel.cpp
  src/sim/SeaState.cpp
  src/sim/CurrentModel.cpp
  src/sim/BathyModel.cpp
  src/sim/SweSolver.cpp
  src/sim/SimClockTest.cpp
  src/sim/OceanCpu.cpp
  src/sim/WaterSurfaceTree.cpp
  src/sim/RigidBody.cpp
  src/sim/Vessel.cpp
  src/sim/VesselSpecs.cpp
  src/sim/VesselTest.cpp
  src/sim/WaveChartTest.cpp
  src/sim/WaveField.cpp
  src/sim/WeatherManager.cpp
  src/sim/GlobeModel.cpp
  src/scene/GlobeLayer.cpp
  src/scene/WaterBankLayer.cpp
)

target_include_directories(gagame PRIVATE src)
# Every Win32 call in this project is the ...W variant, so the resource macros (IDC_ARROW and
# friends) must resolve to their wide forms too.
target_compile_definitions(gagame PRIVATE UNICODE _UNICODE)

# ---- The build names its revision (core/BuildInfo.h). Resolved at configure time -- build.bat
# reconfigures on every build -- and pinned to ONE translation unit so a new commit recompiles
# BuildInfo.cpp alone. "-dirty" means src/, shaders/ or this file differ from HEAD (the data/
# junction and the regenerated docs/GA_AST.md are deliberately not consulted: they say nothing
# about the binary). Without git the string is "unknown" and the build proceeds.
find_package(Git QUIET)
set(GA_GIT_REV "unknown")
if(GIT_FOUND)
  execute_process(COMMAND ${GIT_EXECUTABLE} rev-parse --short HEAD
                  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
                  OUTPUT_VARIABLE GA_GIT_HEAD OUTPUT_STRIP_TRAILING_WHITESPACE
                  RESULT_VARIABLE GA_GIT_RC ERROR_QUIET)
  if(GA_GIT_RC EQUAL 0 AND NOT GA_GIT_HEAD STREQUAL "")
    set(GA_GIT_REV "${GA_GIT_HEAD}")
    execute_process(COMMAND ${GIT_EXECUTABLE} status --porcelain --untracked-files=no
                            -- src shaders CMakeLists.txt
                    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
                    OUTPUT_VARIABLE GA_GIT_STATUS OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
    if(NOT GA_GIT_STATUS STREQUAL "")
      set(GA_GIT_REV "${GA_GIT_REV}-dirty")
    endif()
  endif()
endif()
set_source_files_properties(src/core/BuildInfo.cpp PROPERTIES
  COMPILE_DEFINITIONS "GA_GIT_REV=\"${GA_GIT_REV}\"")
message(STATUS "build revision: ${GA_GIT_REV}")
target_link_libraries(gagame PRIVATE
  d3d12 dxgi dxguid windowscodecs
  dxcompiler      # DxcCreateInstance; the matching DLL is copied next to the exe below
)

# ---- DirectStorage: NVMe -> GPU without the CPU staging hop.
#
# The composed cache is already the right shape for it -- 64 KB tiles "laid out exactly for
# CopyTiles" -- so the read path can hand a queue a file, an offset and a destination and never
# touch the bytes. It ships as a NuGet redistributable rather than in the Windows SDK, so this
# looks for the package cache and degrades HONESTLY: if it is not there, GA_HAVE_DSTORAGE stays
# undefined and the loader keeps its ReadFile path, which is what every machine without the
# redist will do anyway.
set(DSTORAGE_ROOT "$ENV{USERPROFILE}/.nuget/packages/microsoft.direct3d.directstorage/1.2.2/native")
if(EXISTS "${DSTORAGE_ROOT}/include/dstorage.h")
  target_include_directories(gagame PRIVATE "${DSTORAGE_ROOT}/include")
  target_link_libraries(gagame PRIVATE "${DSTORAGE_ROOT}/lib/x64/dstorage.lib")
  target_compile_definitions(gagame PRIVATE GA_HAVE_DSTORAGE=1)
  # Both DLLs must sit beside the exe: dstorage.dll is the API, dstoragecore.dll the
  # implementation it loads, and missing the second fails at CreateStorageFactory, not at link.
  foreach(D dstorage.dll dstoragecore.dll)
    add_custom_command(TARGET gagame POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy_if_different
              "${DSTORAGE_ROOT}/bin/x64/${D}" "$<TARGET_FILE_DIR:gagame>/${D}")
  endforeach()
  message(STATUS "DirectStorage 1.2.2 found: NVMe -> GPU path enabled")
else()
  message(STATUS "DirectStorage not found; tile reads stay on the ReadFile path")
endif()

# Shaders compile at RUNTIME through dxcompiler.dll (hot reload on R). The DLL -- and dxil.dll,
# without which DXC emits unsigned DXIL that the runtime rejects outside developer mode -- must
# sit next to the exe.
find_file(DXCOMPILER_DLL dxcompiler.dll
  PATHS
    "$ENV{WindowsSdkDir}bin/$ENV{WindowsSDKVersion}x64"
    "C:/Program Files (x86)/Windows Kits/10/bin/10.0.26100.0/x64"
    "C:/Program Files (x86)/Windows Kits/10/bin/10.0.22621.0/x64"
  NO_DEFAULT_PATH)
find_file(DXIL_DLL dxil.dll
  PATHS
    "$ENV{WindowsSdkDir}bin/$ENV{WindowsSDKVersion}x64"
    "C:/Program Files (x86)/Windows Kits/10/bin/10.0.26100.0/x64"
    "C:/Program Files (x86)/Windows Kits/10/bin/10.0.22621.0/x64"
  NO_DEFAULT_PATH)

if (DXCOMPILER_DLL)
  add_custom_command(TARGET gagame POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different ${DXCOMPILER_DLL} $<TARGET_FILE_DIR:gagame>)
  message(STATUS "dxcompiler.dll: ${DXCOMPILER_DLL}")
else()
  message(WARNING "dxcompiler.dll not found - runtime shader compilation will fail")
endif()
if (DXIL_DLL)
  add_custom_command(TARGET gagame POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different ${DXIL_DLL} $<TARGET_FILE_DIR:gagame>)
endif()
