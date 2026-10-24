#include "app/viewer.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <vector>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#include "core/log.hpp"
#include "gpu/gl.hpp"

namespace pcv {
namespace {

constexpr double kOrbitRadiansPerPixel = 0.006;

void glfwErrorCallback(int code, const char* description) {
  logError("GLFW error {}: {}", code, description);
}

}  // namespace

Viewer::Viewer() = default;
Viewer::~Viewer() { destroyWindow(); }

Viewer* Viewer::fromWindow(GLFWwindow* window) {
  return static_cast<Viewer*>(glfwGetWindowUserPointer(window));
}

bool Viewer::createWindow(const ViewerOptions& options) {
  glfwSetErrorCallback(glfwErrorCallback);
  if (glfwInit() != GLFW_TRUE) {
    logError("glfwInit failed");
    return false;
  }

  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 6);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_TRUE);
#ifndef NDEBUG
  glfwWindowHint(GLFW_OPENGL_DEBUG_CONTEXT, GLFW_TRUE);
#endif
  glfwWindowHint(GLFW_SAMPLES, 0);  // points are already round; MSAA buys nothing
  // Screenshot and benchmark runs need no visible window.
  const bool headless = !options.screenshotPath.empty();
  if (headless) glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);

  window_ = glfwCreateWindow(options.width, options.height, "pcd_viewer", nullptr, nullptr);
  if (window_ == nullptr) {
    logError("could not create an OpenGL 4.6 core context");
    glfwTerminate();
    return false;
  }

  glfwMakeContextCurrent(window_);
  glfwSwapInterval(options.vsync ? 1 : 0);
  glfwSetWindowUserPointer(window_, this);

  glewExperimental = GL_TRUE;
  const GLenum glewStatus = glewInit();
  if (glewStatus != GLEW_OK) {
    logError("glewInit failed: {}", reinterpret_cast<const char*>(glewGetErrorString(glewStatus)));
    return false;
  }
  // GLEW's own probing leaves a spurious GL_INVALID_ENUM on core profiles.
  while (glGetError() != GL_NO_ERROR) {
  }

  installGlDebugCallback();
  device_ = queryGlDevice();
  logInfo("GPU: {} | {} | GLSL {}", device_.renderer, device_.version, device_.glslVersion);
  if (device_.totalVramMB > 0) logInfo("VRAM: {} MB", device_.totalVramMB);
  if (!device_.isDiscrete)
    logWarn("rendering on an integrated GPU; pass --gpu nvidia to force the discrete one");
  if (!device_.hasShaderDrawParameters) {
    logError("GL_ARB_shader_draw_parameters is required but unavailable");
    return false;
  }
  if (!device_.hasComputeShader) {
    logError("compute shaders are required but unavailable");
    return false;
  }

  glfwSetMouseButtonCallback(window_, [](GLFWwindow* w, int b, int a, int m) {
    fromWindow(w)->onMouseButton(b, a, m);
  });
  glfwSetCursorPosCallback(window_, [](GLFWwindow* w, double x, double y) {
    fromWindow(w)->onCursorMove(x, y);
  });
  glfwSetScrollCallback(window_, [](GLFWwindow* w, double dx, double dy) {
    fromWindow(w)->onScroll(dx, dy);
  });
  glfwSetKeyCallback(window_, [](GLFWwindow* w, int k, int s, int a, int m) {
    fromWindow(w)->onKey(k, s, a, m);
  });
  glfwSetFramebufferSizeCallback(window_, [](GLFWwindow* w, int width, int height) {
    fromWindow(w)->onFramebufferSize(width, height);
  });

  glfwGetFramebufferSize(window_, &width_, &height_);
  return true;
}

void Viewer::destroyWindow() {
  if (window_ != nullptr) {
    glfwDestroyWindow(window_);
    window_ = nullptr;
    glfwTerminate();
  }
}

bool Viewer::openFile(const std::string& path) {
  OpenCloud opened;
  opened.path = path;
  opened.cloud = std::make_shared<PointCloud>();

  ViewerOptions defaults;
  opened.loadResult = loadPointCloud(path, defaults.loadOptions, *opened.cloud);
  if (!opened.loadResult.ok) {
    logError("{}: {}", path, opened.loadResult.error);
    return false;
  }

  opened.chunks = buildChunks(*opened.cloud, defaults.chunkOptions, &opened.chunkStats);
  opened.source = std::make_shared<RamChunkSource>(opened.cloud, opened.chunks);

  std::string error;
  if (!renderer_.setSource(opened.source, error)) {
    logError("upload failed: {}", error);
    return false;
  }

  open_ = std::move(opened);

  // Default the scalar channel to intensity if present, otherwise height.
  int preferred = -1;
  for (int i = 0; i < static_cast<int>(open_.cloud->fields.size()); ++i) {
    const FieldInfo& f = open_.cloud->fields[static_cast<std::size_t>(i)];
    if (f.role == FieldRole::Intensity) { preferred = i; break; }
    if (f.name == "z" && preferred < 0) preferred = i;
  }
  if (preferred >= 0) {
    const FieldInfo& f = open_.cloud->fields[static_cast<std::size_t>(preferred)];
    renderer_.setScalarField(preferred, f.stats.min, f.stats.max);
  }
  // Prefer true colour when the file has it; otherwise the scalar ramp.
  if (!colorModeExplicit_)
    settings_.colorMode = open_.cloud->hasColors() ? ColorMode::Rgb : ColorMode::Scalar;

  const float aspect = static_cast<float>(width_) / static_cast<float>(std::max(1, height_));
  camera_.frameBounds(open_.cloud->boundsLocal, aspect);

  if (startZoom_ != 1.0f && startZoom_ > 0.0f) {
    camera_.distance /= startZoom_;
    camera_.orthoHeight /= startZoom_;
  }

  logInfo("{}: {} points, {} chunks, {:.0f} ms load + {:.0f} ms chunk",
          path, open_.cloud->size(), open_.chunks.size(), open_.cloud->timing.totalMs,
          open_.chunkStats.totalMs);
  return true;
}

void Viewer::onFramebufferSize(int width, int height) {
  width_ = width;
  height_ = height;
}

void Viewer::onMouseButton(int button, int action, int /*mods*/) {
  const bool pressed = action == GLFW_PRESS;
  if (button == GLFW_MOUSE_BUTTON_LEFT) {
    const bool shift = glfwGetKey(window_, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                       glfwGetKey(window_, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS;
    // Shift-drag pans, matching the convention in most 3D tools.
    if (shift) dragPan_ = pressed;
    else dragOrbit_ = pressed;
  } else if (button == GLFW_MOUSE_BUTTON_MIDDLE || button == GLFW_MOUSE_BUTTON_RIGHT) {
    dragPan_ = pressed;
  }
}

void Viewer::onCursorMove(double x, double y) {
  const double dx = x - lastCursorX_;
  const double dy = y - lastCursorY_;
  lastCursorX_ = x;
  lastCursorY_ = y;
  cursorX_ = x;
  cursorY_ = y;

  if (dragOrbit_) {
    camera_.orbit(static_cast<float>(-dx * kOrbitRadiansPerPixel),
                  static_cast<float>(dy * kOrbitRadiansPerPixel));
  } else if (dragPan_) {
    camera_.pan(static_cast<float>(dx), static_cast<float>(dy), static_cast<float>(height_));
  }
}

void Viewer::onScroll(double /*dx*/, double dy) { camera_.dolly(static_cast<float>(dy)); }

void Viewer::onKey(int key, int /*scancode*/, int action, int /*mods*/) {
  if (key >= 0 && key < 512) {
    if (action == GLFW_PRESS) keyDown_[key] = true;
    if (action == GLFW_RELEASE) keyDown_[key] = false;
  }
  if (action != GLFW_PRESS) return;

  const float aspect = static_cast<float>(width_) / static_cast<float>(std::max(1, height_));
  switch (key) {
    case GLFW_KEY_ESCAPE: wantsClose_ = true; break;
    case GLFW_KEY_1: camera_.setMode(NavMode::Orbit, aspect); break;
    case GLFW_KEY_2: camera_.setMode(NavMode::Fly, aspect); break;
    case GLFW_KEY_3: camera_.setMode(NavMode::TopDown, aspect); break;
    case GLFW_KEY_F:
      if (open_.cloud != nullptr) camera_.frameBounds(open_.cloud->boundsLocal, aspect);
      break;
    case GLFW_KEY_L: settings_.lodEnabled = !settings_.lodEnabled; break;
    case GLFW_KEY_E: settings_.edlEnabled = !settings_.edlEnabled; break;
    case GLFW_KEY_C:
      settings_.colorMode = static_cast<ColorMode>((static_cast<int>(settings_.colorMode) + 1) % 4);
      break;
    case GLFW_KEY_R: {
      std::string error;
      if (renderer_.reloadShaders(error)) logInfo("shaders reloaded");
      else logError("shader reload failed: {}", error);
      break;
    }
    case GLFW_KEY_P: {
      // Pick the point under the cursor.
      std::uint32_t index = 0;
      float distance = 0.0f;
      const int mx = static_cast<int>(cursorX_);
      // GLFW's cursor origin is top-left; GL's viewport origin is bottom-left.
      const int my = height_ - static_cast<int>(cursorY_);
      if (renderer_.pick(camera_, width_, height_, mx, my, index, distance)) {
        hasSelection_ = true;
        selectedPoint_ = index;
        selectedDistance_ = distance;
        if (open_.cloud != nullptr && index < open_.cloud->size()) {
          const Vec3 world = open_.cloud->worldPosition(index);
          logInfo("picked row {} at ({:.4f} {:.4f} {:.4f}), {:.2f} m away",
                  open_.cloud->sourceRow(index), world.x, world.y, world.z, distance);
        }
      } else {
        hasSelection_ = false;
        logInfo("nothing under the cursor");
      }
      break;
    }
    default: break;
  }
}

void Viewer::processInput(float deltaSeconds) {
  if (camera_.mode != NavMode::Fly) return;

  Vec3f motion{0.0f, 0.0f, 0.0f};
  if (keyDown_[GLFW_KEY_W]) motion.z += 1.0f;
  if (keyDown_[GLFW_KEY_S]) motion.z -= 1.0f;
  if (keyDown_[GLFW_KEY_D]) motion.x += 1.0f;
  if (keyDown_[GLFW_KEY_A]) motion.x -= 1.0f;
  if (keyDown_[GLFW_KEY_E] || keyDown_[GLFW_KEY_SPACE]) motion.y += 1.0f;
  if (keyDown_[GLFW_KEY_Q] || keyDown_[GLFW_KEY_LEFT_CONTROL]) motion.y -= 1.0f;
  if (lengthSq(motion) == 0.0f) return;

  float speed = camera_.moveSpeed;
  if (keyDown_[GLFW_KEY_LEFT_SHIFT]) speed *= 6.0f;
  if (keyDown_[GLFW_KEY_LEFT_ALT]) speed *= 0.15f;
  camera_.moveLocal(normalize(motion) * (speed * deltaSeconds));
}

void Viewer::renderFrame() {
  renderer_.render(camera_, width_, height_, settings_);
}

bool Viewer::saveScreenshot(const std::string& path) {
  std::vector<unsigned char> pixels(static_cast<std::size_t>(width_) * height_ * 4);
  glPixelStorei(GL_PACK_ALIGNMENT, 1);
  glReadBuffer(GL_BACK);
  glReadPixels(0, 0, width_, height_, GL_RGBA, GL_UNSIGNED_BYTE, pixels.data());

  // GL's first row is the bottom of the image; PNG's is the top.
  stbi_flip_vertically_on_write(1);
  const int ok = stbi_write_png(path.c_str(), width_, height_, 4, pixels.data(), width_ * 4);
  if (ok == 0) {
    logError("could not write {}", path);
    return false;
  }
  logInfo("wrote {} ({}x{})", path, width_, height_);
  return true;
}

void Viewer::logFrameStats() {
  const RenderStats& s = renderer_.stats();
  const double drawnPercent =
      s.totalPoints > 0 ? 100.0 * static_cast<double>(s.drawnPoints) / static_cast<double>(s.totalPoints)
                        : 0.0;
  logInfo("{:.2f} ms/frame ({:.0f} fps) | {}/{} chunks | {} / {} points ({:.1f}%) | "
          "cull {:.2f} draw {:.2f} post {:.2f} ms",
          smoothedFrameMs_, smoothedFrameMs_ > 0.0 ? 1000.0 / smoothedFrameMs_ : 0.0,
          s.visibleChunks, s.totalChunks, s.drawnPoints, s.totalPoints, drawnPercent, s.cullMs,
          s.drawMs, s.postMs);
}

void Viewer::runBenchmark(int frames) {
  logInfo("benchmark: {} frames", frames);
  std::vector<double> times;
  times.reserve(static_cast<std::size_t>(frames));

  const float startYaw = camera_.yaw;
  for (int i = 0; i < frames; ++i) {
    // A full orbit, so the measurement covers every view of the cloud
    // rather than one lucky angle.
    camera_.yaw = startYaw + static_cast<float>(2.0 * kPi * i / frames);

    const double t0 = glfwGetTime();
    renderFrame();
    glFinish();  // the whole point is to measure GPU work, not queue depth
    const double ms = (glfwGetTime() - t0) * 1000.0;
    // Discard the first few: shader warm-up and first-touch VRAM paging.
    if (i >= 10) times.push_back(ms);
    glfwSwapBuffers(window_);
    glfwPollEvents();
  }

  if (times.empty()) return;
  std::sort(times.begin(), times.end());
  const double median = times[times.size() / 2];
  const double p95 = times[static_cast<std::size_t>(static_cast<double>(times.size()) * 0.95)];
  const RenderStats& s = renderer_.stats();
  logInfo("median {:.2f} ms ({:.0f} fps)  p95 {:.2f} ms  min {:.2f}  max {:.2f}", median,
          1000.0 / median, p95, times.front(), times.back());
  logInfo("last frame drew {} of {} points across {} of {} chunks", s.drawnPoints, s.totalPoints,
          s.visibleChunks, s.totalChunks);
}

int Viewer::run(const ViewerOptions& options) {
  applyGpuPreference(options.gpu);
  if (!createWindow(options)) return 1;

  std::string error;
  if (!renderer_.initialize(error)) {
    logError("renderer init failed: {}", error);
    return 1;
  }

  settings_ = options.render;
  camera_.mode = options.nav;
  startZoom_ = options.zoom;
  if (options.haveYaw) camera_.yaw = static_cast<float>(radians(options.yawDegrees));
  if (options.havePitch) camera_.pitch = static_cast<float>(radians(options.pitchDegrees));

  for (const std::string& path : options.files)
    if (!openFile(path)) return 1;

  if (!options.screenshotPath.empty()) {
    for (int i = 0; i < std::max(1, options.screenshotFrames); ++i) {
      renderFrame();
      glfwSwapBuffers(window_);
      glfwPollEvents();
    }
    glFinish();
    // The last swap moved the rendered image to the back buffer.
    renderFrame();
    glFinish();
    logFrameStats();
    return saveScreenshot(options.screenshotPath) ? 0 : 1;
  }

  if (options.benchmark) {
    runBenchmark(options.benchmarkFrames);
    return 0;
  }

  lastFrameTime_ = glfwGetTime();
  double statsTimer = 0.0;
  while (glfwWindowShouldClose(window_) == 0 && !wantsClose_) {
    const double now = glfwGetTime();
    const double delta = now - lastFrameTime_;
    lastFrameTime_ = now;
    frameMs_ = delta * 1000.0;
    // Exponential smoothing, so the readout is stable enough to read.
    smoothedFrameMs_ = smoothedFrameMs_ == 0.0 ? frameMs_
                                               : smoothedFrameMs_ * 0.9 + frameMs_ * 0.1;

    glfwPollEvents();
    processInput(static_cast<float>(delta));
    renderFrame();
    glfwSwapBuffers(window_);

    statsTimer += delta;
    if (statsTimer > 2.0) {
      statsTimer = 0.0;
      logFrameStats();
    }
  }
  return 0;
}

}  // namespace pcv
