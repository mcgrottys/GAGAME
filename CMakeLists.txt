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
  src/core/Gpu.cpp
  src/core/Shader.cpp
  src/core/Window.cpp
  src/core/Image.cpp
  src/core/TileAtlas.cpp
  src/core/Residency.cpp
  src/core/TileProviders.cpp
  src/compose/Compositor.cpp
  src/compose/ComposeTest.cpp
  src/compose/Sources.cpp
  src/compose/GisStencil.cpp
  src/compose/VectorPack.cpp
  src/compose/Exchange.cpp
  src/compose/WaterAtlas.cpp
  src/core/OceanFft.cpp
  src/render/Renderer.cpp
  src/render/Camera.cpp
  src/scene/FieldSet.cpp
  src/scene/SkyLayer.cpp
  src/scene/TideLayer.cpp
  src/scene/SeaLayer.cpp
  src/scene/GulfLayer.cpp
  src/scene/TerrainLayer.cpp
  src/scene/GisLayer.cpp
  src/scene/MarkerLayer.cpp
  src/sim/TideModel.cpp
  src/sim/SeaState.cpp
  src/sim/CurrentModel.cpp
  src/sim/BathyModel.cpp
  src/sim/SweSolver.cpp
  src/sim/WeatherManager.cpp
  src/sim/GlobeModel.cpp
  src/scene/GlobeLayer.cpp
)

target_include_directories(gagame PRIVATE src)
# Every Win32 call in this project is the ...W variant, so the resource macros (IDC_ARROW and
# friends) must resolve to their wide forms too.
target_compile_definitions(gagame PRIVATE UNICODE _UNICODE)
target_link_libraries(gagame PRIVATE
  d3d12 dxgi dxguid windowscodecs
  dxcompiler      # DxcCreateInstance; the matching DLL is copied next to the exe below
)

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
