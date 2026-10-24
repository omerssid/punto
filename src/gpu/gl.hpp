// OpenGL includes plus error and debug plumbing.
#pragma once

#include <GL/glew.h>
// GLFW must follow GLEW: it would otherwise pull in the system GL headers,
// which conflict with GLEW's function pointer declarations.
#include <GLFW/glfw3.h>

#include <string>

namespace punto {

// Installs the KHR_debug callback so driver errors arrive as log lines at
// the point they happen, rather than as a silently wrong frame.
void installGlDebugCallback();

// Reports any pending glGetError, tagged with `where`. Debug builds only;
// the debug callback covers release.
void checkGlError(const char* where);

#ifndef NDEBUG
#define PUNTO_GL_CHECK(where) ::punto::checkGlError(where)
#else
#define PUNTO_GL_CHECK(where) ((void)0)
#endif

struct GlDeviceInfo {
  std::string vendor;
  std::string renderer;
  std::string version;
  std::string glslVersion;
  int majorVersion = 0;
  int minorVersion = 0;
  int totalVramMB = 0;      // 0 when the driver does not report it
  bool isDiscrete = false;  // best-effort guess from the vendor string

  bool hasShaderDrawParameters = false;
  bool hasIndirectParameters = false;
  bool hasComputeShader = false;
  bool hasClipControl = false;
  bool hasDirectStateAccess = false;
};

GlDeviceInfo queryGlDevice();

// Current free VRAM in MB, or 0 if unavailable. NVIDIA and AMD expose this
// through different extensions.
int queryFreeVramMB();

}  // namespace punto
