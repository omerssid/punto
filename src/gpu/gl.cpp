#include "gpu/gl.hpp"

#include <cstdlib>
#include <cstring>

#include "core/log.hpp"

namespace pcv {
namespace {

// NVIDIA: GL_NVX_gpu_memory_info
constexpr GLenum kNvxTotalMemoryKB = 0x9048;
constexpr GLenum kNvxCurrentAvailableKB = 0x9049;
// AMD: GL_ATI_meminfo, returns a 4-element vector; element 0 is free KB.
constexpr GLenum kAtiVboFreeMemory = 0x87FB;

void APIENTRY debugCallback(GLenum source, GLenum type, GLuint id, GLenum severity,
                            GLsizei /*length*/, const GLchar* message, const void* /*user*/) {
  // Buffer-upload hints and "shader will be recompiled" notices are noise.
  if (severity == GL_DEBUG_SEVERITY_NOTIFICATION) return;
  // NVIDIA id 131185: "buffer will use VIDEO memory as the source" -- said
  // once per buffer, expected, and not useful.
  if (id == 131185 || id == 131218) return;

  const char* sourceStr = "other";
  switch (source) {
    case GL_DEBUG_SOURCE_API: sourceStr = "api"; break;
    case GL_DEBUG_SOURCE_SHADER_COMPILER: sourceStr = "shader"; break;
    case GL_DEBUG_SOURCE_WINDOW_SYSTEM: sourceStr = "wsi"; break;
    case GL_DEBUG_SOURCE_THIRD_PARTY: sourceStr = "3rd-party"; break;
    case GL_DEBUG_SOURCE_APPLICATION: sourceStr = "app"; break;
    default: break;
  }
  const char* typeStr = "other";
  switch (type) {
    case GL_DEBUG_TYPE_ERROR: typeStr = "error"; break;
    case GL_DEBUG_TYPE_DEPRECATED_BEHAVIOR: typeStr = "deprecated"; break;
    case GL_DEBUG_TYPE_UNDEFINED_BEHAVIOR: typeStr = "undefined"; break;
    case GL_DEBUG_TYPE_PORTABILITY: typeStr = "portability"; break;
    case GL_DEBUG_TYPE_PERFORMANCE: typeStr = "performance"; break;
    default: break;
  }

  if (severity == GL_DEBUG_SEVERITY_HIGH)
    logError("GL {}/{} [{}]: {}", sourceStr, typeStr, id, message);
  else
    logWarn("GL {}/{} [{}]: {}", sourceStr, typeStr, id, message);
}

bool hasExtension(const char* name) {
  GLint count = 0;
  glGetIntegerv(GL_NUM_EXTENSIONS, &count);
  for (GLint i = 0; i < count; ++i) {
    const auto* ext = reinterpret_cast<const char*>(glGetStringi(GL_EXTENSIONS, static_cast<GLuint>(i)));
    if (ext != nullptr && std::strcmp(ext, name) == 0) return true;
  }
  return false;
}

std::string glString(GLenum name) {
  const auto* s = reinterpret_cast<const char*>(glGetString(name));
  return s != nullptr ? std::string(s) : std::string();
}

}  // namespace

void installGlDebugCallback() {
  if (!GLEW_KHR_debug && !GLEW_ARB_debug_output) {
    logWarn("GL debug output unavailable; driver errors will be silent");
    return;
  }
  glEnable(GL_DEBUG_OUTPUT);
  glDebugMessageCallback(debugCallback, nullptr);
  glDebugMessageControl(GL_DONT_CARE, GL_DONT_CARE, GL_DONT_CARE, 0, nullptr, GL_TRUE);

  // Synchronous debug output makes the driver finish and report before
  // returning from every call that produces a message, which is what makes
  // a reported error point at the call that caused it. It also serialises
  // the pipeline, so it is confined to debug builds and to an explicit
  // opt-in -- having it on in a release build turns a slow frame into a
  // stalled one.
#ifndef NDEBUG
  constexpr bool kSynchronousByDefault = true;
#else
  constexpr bool kSynchronousByDefault = false;
#endif
  const char* env = std::getenv("PCV_GL_SYNC_DEBUG");
  const bool synchronous = env != nullptr ? (env[0] == '1') : kSynchronousByDefault;
  if (synchronous) {
    glEnable(GL_DEBUG_OUTPUT_SYNCHRONOUS);
    logDebug("synchronous GL debug output enabled (slow; unset PCV_GL_SYNC_DEBUG to disable)");
  } else {
    glDisable(GL_DEBUG_OUTPUT_SYNCHRONOUS);
  }
}

void checkGlError(const char* where) {
  for (GLenum err = glGetError(); err != GL_NO_ERROR; err = glGetError()) {
    const char* name = "unknown";
    switch (err) {
      case GL_INVALID_ENUM: name = "GL_INVALID_ENUM"; break;
      case GL_INVALID_VALUE: name = "GL_INVALID_VALUE"; break;
      case GL_INVALID_OPERATION: name = "GL_INVALID_OPERATION"; break;
      case GL_INVALID_FRAMEBUFFER_OPERATION: name = "GL_INVALID_FRAMEBUFFER_OPERATION"; break;
      case GL_OUT_OF_MEMORY: name = "GL_OUT_OF_MEMORY"; break;
      default: break;
    }
    logError("{}: {}", where, name);
  }
}

GlDeviceInfo queryGlDevice() {
  GlDeviceInfo info;
  info.vendor = glString(GL_VENDOR);
  info.renderer = glString(GL_RENDERER);
  info.version = glString(GL_VERSION);
  info.glslVersion = glString(GL_SHADING_LANGUAGE_VERSION);
  glGetIntegerv(GL_MAJOR_VERSION, &info.majorVersion);
  glGetIntegerv(GL_MINOR_VERSION, &info.minorVersion);

  info.hasShaderDrawParameters = hasExtension("GL_ARB_shader_draw_parameters");
  info.hasIndirectParameters = hasExtension("GL_ARB_indirect_parameters");
  info.hasComputeShader = hasExtension("GL_ARB_compute_shader") || info.majorVersion > 4 ||
                          (info.majorVersion == 4 && info.minorVersion >= 3);
  info.hasClipControl = hasExtension("GL_ARB_clip_control");
  info.hasDirectStateAccess = hasExtension("GL_ARB_direct_state_access");

  if (hasExtension("GL_NVX_gpu_memory_info")) {
    GLint kb = 0;
    glGetIntegerv(kNvxTotalMemoryKB, &kb);
    info.totalVramMB = kb / 1024;
  }
  // A discrete GPU is the thing we actually want; the vendor string is the
  // only portable signal, and it only has to be right enough to display.
  const std::string& r = info.renderer;
  info.isDiscrete = r.find("NVIDIA") != std::string::npos ||
                    r.find("Radeon") != std::string::npos ||
                    r.find("AMD") != std::string::npos;
  return info;
}

int queryFreeVramMB() {
  // Clear any pre-existing error so a failed query is distinguishable.
  while (glGetError() != GL_NO_ERROR) {
  }
  GLint kb = 0;
  glGetIntegerv(kNvxCurrentAvailableKB, &kb);
  if (glGetError() == GL_NO_ERROR && kb > 0) return kb / 1024;

  GLint ati[4] = {0, 0, 0, 0};
  glGetIntegerv(kAtiVboFreeMemory, ati);
  if (glGetError() == GL_NO_ERROR && ati[0] > 0) return ati[0] / 1024;
  return 0;
}

}  // namespace pcv
