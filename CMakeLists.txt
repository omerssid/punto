cmake_minimum_required(VERSION 3.20)
project(pcd_viewer LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

option(PCV_NATIVE_ARCH "Build for the host CPU (-march=native)" ON)
option(PCV_WERROR      "Treat warnings as errors" OFF)

add_library(pcv_flags INTERFACE)
target_compile_options(pcv_flags INTERFACE
  -Wall -Wextra -Wpedantic
  $<$<BOOL:${PCV_WERROR}>:-Werror>
  $<$<CONFIG:Release>:-O3>
  # Loaders and stats run tight loops over tens of millions of points;
  # host-specific vectorisation is worth real time there.
  $<$<AND:$<BOOL:${PCV_NATIVE_ARCH}>,$<CONFIG:Release>>:-march=native>
)
target_compile_features(pcv_flags INTERFACE cxx_std_20)

# ---------------------------------------------------------------- core

add_library(pcv_core STATIC
  src/core/log.cpp
  src/core/parallel.cpp
  src/core/math.cpp
  src/io/field.cpp
  src/io/point_cloud.cpp
  src/io/mapped_file.cpp
  src/io/lzf.cpp
  src/io/cloud_builder.cpp
  src/io/loader.cpp
  src/io/pcd_loader.cpp
  src/io/ply_loader.cpp
  src/io/las_loader.cpp
  src/io/laz_decode.cpp
  src/scene/chunk_builder.cpp
  src/scene/radix_sort.cpp
  src/scene/ram_chunk_source.cpp
)
target_include_directories(pcv_core PUBLIC src)
target_link_libraries(pcv_core PUBLIC pcv_flags)

find_package(Threads REQUIRED)
target_link_libraries(pcv_core PUBLIC Threads::Threads)

# Optional: compressed LAS. Everything else builds without it.
find_path(LASZIP_INCLUDE_DIR laszip/laszip_api.h)
find_library(LASZIP_LIBRARY NAMES laszip laszip_api)
if(LASZIP_INCLUDE_DIR AND LASZIP_LIBRARY)
  message(STATUS "laszip found: LAZ reading enabled")
  target_include_directories(pcv_core PRIVATE ${LASZIP_INCLUDE_DIR})
  target_link_libraries(pcv_core PRIVATE ${LASZIP_LIBRARY})
  target_compile_definitions(pcv_core PRIVATE PCV_HAVE_LASZIP=1)
  set(PCV_LASZIP_FOUND TRUE)
else()
  message(STATUS "laszip not found: .laz files will be rejected "
                 "(install liblaszip-dev to enable)")
endif()

# ---------------------------------------------------------------- viewer

find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)
find_package(glfw3 QUIET)
if(NOT glfw3_FOUND)
  find_library(GLFW_LIBRARY NAMES glfw glfw3 REQUIRED)
  set(PCV_GLFW ${GLFW_LIBRARY})
else()
  set(PCV_GLFW glfw)
endif()

# Dear ImGui, vendored. Only the GLFW + OpenGL3 backend is compiled.
add_library(pcv_imgui STATIC
  third_party/imgui/imgui.cpp
  third_party/imgui/imgui_draw.cpp
  third_party/imgui/imgui_tables.cpp
  third_party/imgui/imgui_widgets.cpp
  third_party/imgui/backends/imgui_impl_glfw.cpp
  third_party/imgui/backends/imgui_impl_opengl3.cpp
)
target_include_directories(pcv_imgui PUBLIC
  third_party/imgui third_party/imgui/backends)
target_link_libraries(pcv_imgui PUBLIC ${PCV_GLFW} OpenGL::GL)
# Vendored third-party code; its warnings are not ours to fix.
target_compile_options(pcv_imgui PRIVATE -w)

# Shaders are embedded into the binary so the viewer is a single file.
include(cmake/embed_shaders.cmake)
set(PCV_EMBEDDED_SHADERS ${CMAKE_CURRENT_BINARY_DIR}/generated/embedded_shaders.cpp)
file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/generated)
pcv_embed_shaders(${PCV_EMBEDDED_SHADERS} ${CMAKE_CURRENT_SOURCE_DIR}/shaders)

add_library(pcv_gpu STATIC
  src/gpu/gl.cpp
  src/gpu/shader.cpp
  src/gpu/point_renderer.cpp
  src/gpu/overlay_renderer.cpp
  src/scene/camera.cpp
  ${PCV_EMBEDDED_SHADERS}
)
target_link_libraries(pcv_gpu PUBLIC pcv_core GLEW::GLEW OpenGL::GL ${PCV_GLFW})
# Lets ShaderLibrary pick up edits to shaders/ without a rebuild.
target_compile_definitions(pcv_gpu PRIVATE
  PCV_SOURCE_SHADER_DIR="${CMAKE_CURRENT_SOURCE_DIR}/shaders")

add_executable(pcd_viewer
  src/main.cpp
  src/app/viewer.cpp
  src/app/gpu_select.cpp
  src/ui/ui.cpp
)
target_include_directories(pcd_viewer PRIVATE third_party/stb)
target_link_libraries(pcd_viewer PRIVATE pcv_gpu pcv_imgui)

# ---------------------------------------------------------------- tools

add_executable(pcv_probe tools/pcv_probe.cpp)
target_link_libraries(pcv_probe PRIVATE pcv_core)

# Compresses a LAS fixture using laszip itself, so the LAZ tests run against
# data produced by the reference implementation.
if(PCV_LASZIP_FOUND)
  add_executable(pcv_make_laz tools/make_laz.cpp)
  target_include_directories(pcv_make_laz PRIVATE ${LASZIP_INCLUDE_DIR})
  target_link_libraries(pcv_make_laz PRIVATE ${LASZIP_LIBRARY} pcv_flags)
endif()

# ---------------------------------------------------------------- tests

enable_testing()
add_executable(pcv_tests tests/test_loaders.cpp)
target_link_libraries(pcv_tests PRIVATE pcv_core)
add_test(NAME loaders COMMAND pcv_tests)

add_executable(pcv_chunk_tests tests/test_chunks.cpp)
target_link_libraries(pcv_chunk_tests PRIVATE pcv_core)
add_test(NAME chunks COMMAND pcv_chunk_tests)
