cmake_minimum_required(VERSION 3.20)
project(punto VERSION 0.1.0 LANGUAGES CXX)

include(GNUInstallDirs)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

option(PUNTO_NATIVE_ARCH "Build for the host CPU (-march=native)" ON)
option(PUNTO_WERROR      "Treat warnings as errors" OFF)
# Link the C++ runtime statically. For binaries built on an older
# distribution and run on newer ones: the glibc requirement comes from the
# build host, but libstdc++ comes from the compiler, so a newer GCC on an
# older distribution otherwise produces something that runs nowhere.
option(PUNTO_STATIC_CXX_RUNTIME "Link libstdc++ and libgcc statically" OFF)
# Bake the source tree's shaders/ path into the binary so edits there are
# picked up live. Off for anything distributed: an installed binary should
# run its embedded shaders, not whatever happens to sit at the build
# machine's path on the user's.
option(PUNTO_DEV_SHADER_DIR "Live-reload shaders from the source tree" ON)

# std::format is used throughout the logging and UI layers -- 32 call sites
# -- and libstdc++ only shipped it in GCC 13. Fail here with a clear reason
# rather than 47 files deep on a missing <format>.
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 13)
  message(FATAL_ERROR
    "GCC ${CMAKE_CXX_COMPILER_VERSION} is too old: <format> arrived in GCC 13.\n"
    "  Ubuntu 22.04 ships GCC 11 by default. Install a newer one with:\n"
    "    sudo add-apt-repository ppa:ubuntu-toolchain-r/test\n"
    "    sudo apt install g++-13\n"
    "    cmake -S . -B build -DCMAKE_CXX_COMPILER=g++-13")
elseif(CMAKE_CXX_COMPILER_ID MATCHES "Clang" AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 17)
  message(WARNING
    "Clang ${CMAKE_CXX_COMPILER_VERSION} may lack <format>; it needs libc++ 17 "
    "or libstdc++ from GCC 13.")
endif()

add_library(punto-flags INTERFACE)
target_compile_options(punto-flags INTERFACE
  -Wall -Wextra -Wpedantic
  $<$<BOOL:${PUNTO_WERROR}>:-Werror>
  $<$<CONFIG:Release>:-O3>
  # Loaders and stats run tight loops over tens of millions of points;
  # host-specific vectorisation is worth real time there.
  $<$<AND:$<BOOL:${PUNTO_NATIVE_ARCH}>,$<CONFIG:Release>>:-march=native>
)
target_compile_features(punto-flags INTERFACE cxx_std_20)
target_compile_definitions(punto-flags INTERFACE PUNTO_VERSION="${PROJECT_VERSION}")
if(PUNTO_STATIC_CXX_RUNTIME)
  target_link_options(punto-flags INTERFACE -static-libstdc++ -static-libgcc)
endif()

# ---------------------------------------------------------------- core

add_library(punto-core STATIC
  src/core/log.cpp
  src/core/parallel.cpp
  src/core/math.cpp
  src/core/memory.cpp
  src/io/field.cpp
  src/io/point_cloud.cpp
  src/io/mapped_file.cpp
  src/io/lzf.cpp
  src/io/cloud_builder.cpp
  src/io/cloud_info.cpp
  src/io/loader.cpp
  src/io/pcd_loader.cpp
  src/io/ply_loader.cpp
  src/io/las_loader.cpp
  src/io/laz_decode.cpp
  src/scene/camera.cpp
  src/scene/chunk_builder.cpp
  src/scene/radix_sort.cpp
  src/scene/ram_chunk_source.cpp
)
target_include_directories(punto-core PUBLIC src)
target_link_libraries(punto-core PUBLIC punto-flags)

find_package(Threads REQUIRED)
target_link_libraries(punto-core PUBLIC Threads::Threads)

# Optional: compressed LAS. Everything else builds without it.
find_path(LASZIP_INCLUDE_DIR laszip/laszip_api.h)
find_library(LASZIP_LIBRARY NAMES laszip laszip_api)
if(LASZIP_INCLUDE_DIR AND LASZIP_LIBRARY)
  message(STATUS "laszip found: LAZ reading enabled")
  target_include_directories(punto-core PRIVATE ${LASZIP_INCLUDE_DIR})
  target_link_libraries(punto-core PRIVATE ${LASZIP_LIBRARY})
  target_compile_definitions(punto-core PRIVATE PUNTO_HAVE_LASZIP=1)
  set(PUNTO_LASZIP_FOUND TRUE)
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
  set(PUNTO_GLFW ${GLFW_LIBRARY})
else()
  set(PUNTO_GLFW glfw)
endif()

# Dear ImGui, vendored. Only the GLFW + OpenGL3 backend is compiled.
add_library(punto-imgui STATIC
  third_party/imgui/imgui.cpp
  third_party/imgui/imgui_draw.cpp
  third_party/imgui/imgui_tables.cpp
  third_party/imgui/imgui_widgets.cpp
  third_party/imgui/backends/imgui_impl_glfw.cpp
  third_party/imgui/backends/imgui_impl_opengl3.cpp
)
target_include_directories(punto-imgui PUBLIC
  third_party/imgui third_party/imgui/backends)
target_link_libraries(punto-imgui PUBLIC ${PUNTO_GLFW} OpenGL::GL)
# Vendored third-party code; its warnings are not ours to fix.
target_compile_options(punto-imgui PRIVATE -w)

# Shaders are embedded into the binary so the viewer is a single file.
include(cmake/embed_shaders.cmake)
set(PUNTO_EMBEDDED_SHADERS ${CMAKE_CURRENT_BINARY_DIR}/generated/embedded_shaders.cpp)
file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/generated)
punto_embed_shaders(${PUNTO_EMBEDDED_SHADERS} ${CMAKE_CURRENT_SOURCE_DIR}/shaders)

add_library(punto-gpu STATIC
  src/gpu/gl.cpp
  src/gpu/shader.cpp
  src/gpu/point_renderer.cpp
  src/gpu/overlay_renderer.cpp
  ${PUNTO_EMBEDDED_SHADERS}
)
target_link_libraries(punto-gpu PUBLIC punto-core GLEW::GLEW OpenGL::GL ${PUNTO_GLFW})
# Lets ShaderLibrary pick up edits to shaders/ without a rebuild.
if(PUNTO_DEV_SHADER_DIR)
  target_compile_definitions(punto-gpu PRIVATE
    PUNTO_SOURCE_SHADER_DIR="${CMAKE_CURRENT_SOURCE_DIR}/shaders")
endif()

# XRandR lets the viewer see how displays are actually driven, so it can
# avoid forcing render offload onto a reverse-PRIME setup where that makes
# things far worse. Optional: without it, auto simply always offloads.
find_path(XRANDR_INCLUDE_DIR X11/extensions/Xrandr.h)
find_library(XRANDR_LIBRARY NAMES Xrandr)
# Xrandr's own calls need Xlib for the display connection.
find_library(X11_LIBRARY NAMES X11)
if(NOT X11_LIBRARY)
  set(XRANDR_INCLUDE_DIR "")
endif()
if(XRANDR_INCLUDE_DIR AND XRANDR_LIBRARY)
  message(STATUS "Xrandr found: GPU/display topology detection enabled")
else()
  message(STATUS "Xrandr not found: --gpu auto will always prefer the discrete GPU")
endif()

add_executable(punto-viewer
  src/main.cpp
  src/app/viewer.cpp
  src/app/gpu_select.cpp
  src/app/stb_impl.cpp
  src/ui/ui.cpp
)
target_include_directories(punto-viewer PRIVATE third_party/stb)
target_link_libraries(punto-viewer PRIVATE punto-gpu punto-imgui)

# Vendored third-party code; its warnings are not ours to fix, and holding
# it to -Werror would make the build hostage to future compiler releases.
set_source_files_properties(src/app/stb_impl.cpp PROPERTIES COMPILE_OPTIONS "-w")
if(XRANDR_INCLUDE_DIR AND XRANDR_LIBRARY)
  target_include_directories(punto-viewer PRIVATE ${XRANDR_INCLUDE_DIR})
  target_link_libraries(punto-viewer PRIVATE ${XRANDR_LIBRARY} ${X11_LIBRARY})
  target_compile_definitions(punto-viewer PRIVATE PUNTO_HAVE_XRANDR=1)
endif()

# ---------------------------------------------------------------- tools

add_executable(punto-probe tools/punto_probe.cpp)
target_link_libraries(punto-probe PRIVATE punto-core)

# Compresses a LAS fixture using laszip itself, so the LAZ tests run against
# data produced by the reference implementation.
if(PUNTO_LASZIP_FOUND)
  add_executable(punto-make-laz tools/make_laz.cpp)
  target_include_directories(punto-make-laz PRIVATE ${LASZIP_INCLUDE_DIR})
  target_link_libraries(punto-make-laz PRIVATE ${LASZIP_LIBRARY} punto-flags)
endif()

# ---------------------------------------------------------------- install

install(TARGETS punto-viewer punto-probe RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
install(FILES packaging/punto.desktop
  DESTINATION ${CMAKE_INSTALL_DATADIR}/applications)
install(FILES packaging/punto.svg
  DESTINATION ${CMAKE_INSTALL_DATADIR}/icons/hicolor/scalable/apps)
# Bitmaps too, for docks and switchers that do not render SVG.
foreach(size 32 64 128 256 512)
  install(FILES packaging/icons/punto-${size}.png
    DESTINATION ${CMAKE_INSTALL_DATADIR}/icons/hicolor/${size}x${size}/apps
    RENAME punto.png)
endforeach()
install(FILES packaging/punto.xml
  DESTINATION ${CMAKE_INSTALL_DATADIR}/mime/packages)

# `cpack` in the build directory turns the install tree above into a .deb.
# No maintainer scripts: the MIME database, icon cache and desktop database
# all refresh through dpkg triggers owned by the packages that maintain
# them.
set(CPACK_GENERATOR DEB)
set(CPACK_PACKAGE_NAME punto)
set(CPACK_PACKAGE_VENDOR "Omer Sayed")
set(CPACK_PACKAGE_CONTACT "Omer Sayed <omarh9323@gmail.com>")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "GPU point cloud viewer for PCD, PLY, LAS and LAZ")
# One string, not several arguments: separate arguments would make a CMake
# list and leave a ';' at the start of every line of the control file.
set(CPACK_PACKAGE_DESCRIPTION "\
Punto opens point clouds of tens of millions of points and renders them
with level of detail and eye-dome lighting. It reads PCD (ASCII, binary
and compressed), PLY, LAS, LAZ and KITTI .bin files.

Includes punto-probe, a command-line tool that reports a file's fields,
bounds and the memory it would need to open.")
set(CPACK_DEBIAN_PACKAGE_SECTION graphics)
set(CPACK_DEBIAN_PACKAGE_HOMEPAGE "https://github.com/omerssid/punto")
set(CPACK_DEBIAN_FILE_NAME DEB-DEFAULT)
set(CPACK_STRIP_FILES ON)
# Otherwise directories take the build machine's umask.
set(CPACK_INSTALL_DEFAULT_DIRECTORY_PERMISSIONS
  OWNER_READ OWNER_WRITE OWNER_EXECUTE
  GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)
# Depends: is worked out by dpkg-shlibdeps from what the binaries actually
# link, so it names the right library packages for whichever release the
# .deb was built on and can never drift from the build.
set(CPACK_DEBIAN_PACKAGE_SHLIBDEPS ON)
include(CPack)

# ---------------------------------------------------------------- tests

enable_testing()
add_executable(punto-tests tests/test_loaders.cpp)
target_link_libraries(punto-tests PRIVATE punto-core)
add_test(NAME loaders COMMAND punto-tests)

add_executable(punto-camera-tests tests/test_camera.cpp)
target_link_libraries(punto-camera-tests PRIVATE punto-core)
add_test(NAME camera COMMAND punto-camera-tests)

add_executable(punto-chunk-tests tests/test_chunks.cpp)
target_link_libraries(punto-chunk-tests PRIVATE punto-core)
add_test(NAME chunks COMMAND punto-chunk-tests)
