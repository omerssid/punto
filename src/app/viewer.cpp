#include "app/viewer.hpp"

#include <format>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <vector>

// Implementation lives in stb_impl.cpp, which is compiled with warnings
// disabled; this is just the declarations.
#include "stb_image_write.h"

#include "core/log.hpp"
#include "core/memory.hpp"
#include "io/cloud_info.hpp"
#include "gpu/gl.hpp"
#include "gpu/overlay_renderer.hpp"
#include "gpu/shader.hpp"

namespace punto {
namespace {

constexpr double kOrbitRadiansPerPixel = 0.006;

void glfwErrorCallback(int code, const char* description) {
  logError("GLFW error {}: {}", code, description);
}

}  // namespace

Viewer::Viewer() = default;
Viewer::~Viewer() {
  ui_.shutdown();
  destroyWindow();
}

Viewer* Viewer::fromWindow(GLFWwindow* window) {
  return static_cast<Viewer*>(glfwGetWindowUserPointer(window));
}

bool Viewer::createWindow(const ViewerOptions& options, const PrimeTopology& topology) {
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
  // Desktops find a running window's icon by matching this against
  // punto.desktop. GLFW would otherwise derive it from the title.
  glfwWindowHintString(GLFW_X11_CLASS_NAME, "punto");
  glfwWindowHintString(GLFW_X11_INSTANCE_NAME, "punto");
#ifdef GLFW_WAYLAND_APP_ID
  glfwWindowHintString(GLFW_WAYLAND_APP_ID, "punto");
#endif

  window_ = glfwCreateWindow(options.width, options.height, "punto", nullptr, nullptr);
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
  // Only worth mentioning when the discrete GPU was actually available and
  // declined; saying it on a reverse-PRIME setup contradicts the message
  // explaining why offload was skipped.
  if (!device_.isDiscrete && !topology.reversePrimeActive && options.gpu != GpuPreference::Integrated)
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
  glfwSetDropCallback(window_, [](GLFWwindow* w, int count, const char** paths) {
    fromWindow(w)->onFileDrop(count, paths);
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
  // Pre-flight: read the header and predict the cost before allocating
  // anything. An 87M-point LAZ needs about 6 GB, and asking for that on a
  // machine that does not have it does not fail cleanly -- the desktop goes
  // to swap, the OOM killer picks whichever process it likes, and the first
  // thing the user sees is a frozen screen. Refusing up front is far kinder.
  const CloudInfo info = probeCloud(path, options_.loadOptions);
  if (info.ok) {
    const std::uint64_t needed = info.memory.peakHostPadded();
    const std::uint64_t available = availableMemoryBytes();
    logInfo("{}: {} points, needs about {:.1f} GB", path, info.pointCount,
            static_cast<double>(needed) / 1e9);

    // available == 0 means the figure could not be read, which is not the
    // same as there being none; in that case there is nothing to check.
    if (available > 0 && needed > available && !options_.forceLoad) {
      const std::string message = std::format(
          "{} needs about {:.1f} GB but only {:.1f} GB is available. "
          "Close something large, or pass --force to try anyway.",
          path, static_cast<double>(needed) / 1e9, static_cast<double>(available) / 1e9);
      logError("{}", message);
      ui_.setNotice(message, true);
      return false;
    }
  } else {
    // A probe failure is not itself fatal -- the loader reports errors far
    // better than the header parser can -- so fall through and let it try.
    logDebug("{}: could not pre-flight ({})", path, info.error);
  }

  OpenCloud opened;
  opened.path = path;
  opened.cloud = std::make_shared<PointCloud>();

  opened.loadResult = loadPointCloud(path, options_.loadOptions, *opened.cloud);
  if (!opened.loadResult.ok) {
    logError("{}: {}", path, opened.loadResult.error);
    ui_.setNotice(path + ": " + opened.loadResult.error, true);
    return false;
  }

  opened.chunks = buildChunks(*opened.cloud, options_.chunkOptions, &opened.chunkStats);
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
    if (!requestedField_.empty() && f.name == requestedField_) { preferred = i; break; }
    if (requestedField_.empty() && f.role == FieldRole::Intensity) { preferred = i; break; }
    if (requestedField_.empty() && f.name == "z" && preferred < 0) preferred = i;
  }
  if (!requestedField_.empty() && preferred < 0)
    logWarn("no field named '{}' in this cloud", requestedField_);
  if (preferred >= 0) {
    const FieldInfo& f = open_.cloud->fields[static_cast<std::size_t>(preferred)];
    renderer_.setScalarField(preferred, f.stats.min, f.stats.max);
    activeScalarField_ = preferred;
    activeScalarLo_ = f.stats.min;
    activeScalarHi_ = f.stats.max;
  } else {
    activeScalarField_ = -1;
  }
  // A new cloud invalidates any field the panel had selected.
  ui_.syncScalarField(activeScalarField_, activeScalarLo_, activeScalarHi_);
  // Prefer true colour when the file has it; otherwise the scalar ramp.
  if (!colorModeExplicit_)
    settings_.colorMode = open_.cloud->hasColors() ? ColorMode::Rgb : ColorMode::Scalar;

  const Ui::ViewportRect rect = viewport();
  camera_.frameBounds(open_.cloud->boundsLocal,
                      static_cast<float>(rect.width) /
                          static_cast<float>(std::max(1, rect.height)));
  // On the very first open the dockspace has not been laid out yet, so
  // viewport() still reports the whole window and the framing comes out
  // about a quarter too wide. Re-frame once the real rectangle is known.
  pendingFrameAll_ = true;

  if (cropFraction_ > 0.0f && open_.cloud->boundsLocal.valid()) {
    const Aabbf& b = open_.cloud->boundsLocal;
    const Vec3f centre = b.center();
    const Vec3f half = b.size() * (0.5f * cropFraction_);
    settings_.cropBox.lo = centre - half;
    settings_.cropBox.hi = centre + half;
    settings_.cropEnabled = true;
  }

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

void Viewer::onMouseButton(int button, int action, int mods) {
  // A press that began over a panel belongs to the panel. Releases are
  // always processed, so a drag that wanders over a panel still ends.
  if (action == GLFW_PRESS && ui_.wantsMouse()) return;
  const bool pressed = action == GLFW_PRESS;
  if (button == GLFW_MOUSE_BUTTON_LEFT) {
    // The modifier state GLFW reports with the event, rather than polling
    // the key: polling can disagree if shift is released between the click
    // and the callback running.
    const bool shift = (mods & GLFW_MOD_SHIFT) != 0;

    // Shift-click selects the point under the cursor.
    //
    // This used to be shift-drag to pan, but panning is already on both
    // middle and right drag, so the gesture was redundant -- and pointing
    // at the thing you want is a far more natural way to select it than
    // parking the cursor and reaching for a key.
    if (shift) {
      if (pressed) {
        // Read the live position rather than trusting the last move event,
        // which may not have fired if the click landed without motion.
        glfwGetCursorPos(window_, &cursorX_, &cursorY_);
        pickUnderCursor();
      }
      dragOrbit_ = false;
      dragPan_ = false;
      return;
    }

    if (pressed) {
      const double now = glfwGetTime();
      // GLFW reports presses, not double-clicks, so the interval is
      // measured here. 0.35 s is the usual system threshold.
      if (now - lastClickTime_ < 0.35) {
        glfwGetCursorPos(window_, &cursorX_, &cursorY_);
        pickUnderCursor();
        if (hasSelection_ && open_.cloud != nullptr && selectedPoint_ < open_.cloud->size()) {
          camera_.focusOn(open_.cloud->positions[selectedPoint_]);
          lastClickTime_ = 0.0;  // a triple click should not re-trigger
          return;
        }
      }
      lastClickTime_ = now;
    }
    dragOrbit_ = pressed;
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
    camera_.pan(static_cast<float>(dx), static_cast<float>(dy),
                static_cast<float>(viewport().height));
  }
}

void Viewer::onScroll(double /*dx*/, double dy) {
  if (ui_.wantsMouse()) return;
  camera_.dolly(static_cast<float>(dy));
}

void Viewer::onKey(int key, int /*scancode*/, int action, int /*mods*/) {
  if (key >= 0 && key < 512) {
    if (action == GLFW_PRESS) keyDown_[key] = true;
    if (action == GLFW_RELEASE) keyDown_[key] = false;
  }
  if (action != GLFW_PRESS) return;
  // Let text fields receive characters rather than triggering shortcuts.
  if (ui_.wantsKeyboard()) return;

  const Ui::ViewportRect rect = viewport();
  const float aspect =
      static_cast<float>(rect.width) / static_cast<float>(std::max(1, rect.height));
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
      std::string overlayError;
      const bool pointsOk = renderer_.reloadShaders(error);
      const bool overlayOk = overlay_.reloadShaders(overlayError);
      if (pointsOk && overlayOk) logInfo("shaders reloaded");
      else logError("shader reload failed: {}", pointsOk ? overlayError : error);
      break;
    }
    case GLFW_KEY_P: pickUnderCursor(); break;
    case GLFW_KEY_TAB: ui_.togglePanels(); break;
    case GLFW_KEY_G:
      if (hasSelection_ && open_.cloud != nullptr && selectedPoint_ < open_.cloud->size())
        camera_.focusOn(open_.cloud->positions[selectedPoint_]);
      break;
    default: break;
  }
}

void Viewer::processInput(float deltaSeconds) {
  if (camera_.mode != NavMode::Fly || ui_.wantsKeyboard()) return;

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

void Viewer::onFileDrop(int count, const char** paths) {
  // Open the first thing dropped; opening several at once would need
  // multi-cloud support, which the renderer does not have yet.
  if (count > 0 && paths != nullptr && paths[0] != nullptr) openFile(paths[0]);
}

PickedPoint Viewer::resolvePick(std::uint32_t index, float distance) const {
  PickedPoint picked;
  if (open_.cloud == nullptr || index >= open_.cloud->size()) return picked;

  picked.valid = true;
  picked.index = index;
  picked.sourceRow = open_.cloud->sourceRow(index);
  picked.world = open_.cloud->worldPosition(index);
  picked.cameraDistance = distance;

  // Chunks tile the point array in order, so the owning chunk is the last
  // one whose first point is at or before this index.
  const auto& chunks = open_.chunks;
  const auto it = std::upper_bound(chunks.begin(), chunks.end(), index,
                                   [](std::uint32_t value, const Chunk& chunk) {
                                     return value < chunk.firstPoint;
                                   });
  if (it != chunks.begin())
    picked.chunkIndex = static_cast<int>(std::distance(chunks.begin(), it) - 1);
  return picked;
}

void Viewer::pickUnderCursor() {
  std::uint32_t index = 0;
  float distance = 0.0f;
  const Ui::ViewportRect rect = viewport();
  // GLFW's cursor origin is top-left; GL's is bottom-left. The result is
  // then made relative to the viewport, since that is the rectangle the
  // scene was rendered into.
  const int mx = static_cast<int>(cursorX_) - rect.x;
  const int my = (height_ - static_cast<int>(cursorY_)) - rect.y;
  if (mx < 0 || my < 0 || mx >= rect.width || my >= rect.height) {
    hasSelection_ = false;
    ui_.clearSelection();
    return;
  }

  if (!renderer_.pick(camera_, rect.width, rect.height, mx, my, index, distance)) {
    hasSelection_ = false;
    ui_.clearSelection();
    return;
  }

  hasSelection_ = true;
  selectedPoint_ = index;
  selectedDistance_ = distance;
  const PickedPoint picked = resolvePick(index, distance);
  ui_.setSelection(picked);
  if (ui_.measureModeActive()) ui_.addMeasurementPoint(picked);

  logInfo("picked row {} at ({:.4f} {:.4f} {:.4f}), {:.2f} m away", picked.sourceRow,
          picked.world.x, picked.world.y, picked.world.z, distance);
}

void Viewer::applyUiActions(const UiActions& actions) {
  const Ui::ViewportRect rect = viewport();
  const float aspect =
      static_cast<float>(rect.width) / static_cast<float>(std::max(1, rect.height));

  if (!actions.openPath.empty()) openFile(actions.openPath);
  if (actions.frameAll && open_.cloud != nullptr)
    camera_.frameBounds(open_.cloud->boundsLocal, aspect);
  if (actions.focusSelection && hasSelection_ && open_.cloud != nullptr &&
      selectedPoint_ < open_.cloud->size())
    camera_.focusOn(open_.cloud->positions[selectedPoint_]);
  if (actions.scalarFieldChanged && actions.scalarField >= 0) {
    renderer_.setScalarField(actions.scalarField, actions.scalarLo, actions.scalarHi);
    activeScalarField_ = actions.scalarField;
    activeScalarLo_ = actions.scalarLo;
    activeScalarHi_ = actions.scalarHi;
  }
  if (actions.clearMeasurement) ui_.clearMeasurement();
  if (actions.reloadShaders) {
    std::string error;
    const bool pointsOk = renderer_.reloadShaders(error);
    std::string overlayError;
    const bool overlayOk = overlay_.reloadShaders(overlayError);
    if (pointsOk && overlayOk) logInfo("shaders reloaded");
    else logError("shader reload failed: {}", pointsOk ? overlayError : error);
  }
}

void Viewer::buildOverlay() {
  overlay_.begin();
  if (open_.cloud == nullptr) return;

  const float viewportHeight = static_cast<float>(std::max(1, viewport().height));
  const Vec3f eye = camera_.position();
  // Markers are sized in world units but chosen so they come out a fixed
  // number of pixels across, whatever the camera distance.
  auto markerRadius = [&](const Vec3f& at, float pixels) {
    return camera_.worldPerPixel(length(at - eye), viewportHeight) * pixels;
  };

  constexpr std::uint32_t kSelection = overlayColor(255, 214, 92);
  constexpr std::uint32_t kMeasure = overlayColor(120, 220, 255);
  constexpr std::uint32_t kCrop = overlayColor(130, 200, 150, 180);

  if (settings_.cropEnabled && settings_.cropBox.valid())
    overlay_.addBox(settings_.cropBox, kCrop);

  const std::vector<PickedPoint>& measured = ui_.measurementPoints();
  for (const PickedPoint& p : measured) {
    if (!p.valid || p.index >= open_.cloud->size()) continue;
    const Vec3f local = open_.cloud->positions[p.index];
    overlay_.addMarker(local, markerRadius(local, 7.0f), kMeasure, camera_);
  }
  for (std::size_t i = 1; i < measured.size(); ++i) {
    if (measured[i - 1].index >= open_.cloud->size() ||
        measured[i].index >= open_.cloud->size())
      continue;
    const Vec3f a = open_.cloud->positions[measured[i - 1].index];
    const Vec3f b = open_.cloud->positions[measured[i].index];
    // Dashed, so the line reads as a measurement rather than as geometry.
    const float dash = camera_.worldPerPixel(length((a + b) * 0.5f - eye), viewportHeight) * 8.0f;
    overlay_.addDashedLine(a, b, kMeasure, dash);
  }

  const PickedPoint& selection = ui_.selection();
  if (selection.valid && selection.index < open_.cloud->size()) {
    const Vec3f local = open_.cloud->positions[selection.index];
    overlay_.addMarker(local, markerRadius(local, 11.0f), kSelection, camera_);
  }
}

void Viewer::consumePendingFrame() {
  if (!pendingFrameAll_ || open_.cloud == nullptr) return;
  pendingFrameAll_ = false;
  const Ui::ViewportRect rect = viewport();
  camera_.frameBounds(open_.cloud->boundsLocal,
                      static_cast<float>(rect.width) /
                          static_cast<float>(std::max(1, rect.height)));
  if (startZoom_ != 1.0f && startZoom_ > 0.0f) {
    camera_.distance /= startZoom_;
    camera_.orthoHeight /= startZoom_;
  }
}

Ui::ViewportRect Viewer::viewport() const {
  return ui_.viewportRect(width_, height_);
}

void Viewer::renderFrame() {
  const Ui::ViewportRect rect = viewport();
  renderer_.render(camera_, rect.x, rect.y, rect.width, rect.height, settings_);
  buildOverlay();
  glViewport(rect.x, rect.y, rect.width, rect.height);
  overlay_.draw(camera_, rect.width, rect.height);
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

void Viewer::runResizeStress(int frames) {
  logInfo("resize stress: {} frames sweeping the viewport size", frames);
  const std::uint64_t before = renderer_.framebufferAllocations();

  const int baseWidth = width_;
  const int baseHeight = height_;
  int minWidth = baseWidth, maxWidth = 0;

  for (int i = 0; i < frames; ++i) {
    // A smooth sweep from a quarter size up to full and back, which is
    // what a maximise animation and a panel drag both look like.
    const double phase = static_cast<double>(i) / frames * 2.0 * kPi;
    const double t = 0.25 + 0.75 * (0.5 - 0.5 * std::cos(phase));
    width_ = std::max(64, static_cast<int>(baseWidth * t));
    height_ = std::max(64, static_cast<int>(baseHeight * t));
    minWidth = std::min(minWidth, width_);
    maxWidth = std::max(maxWidth, width_);

    renderFrame();
    glFinish();
    glfwSwapBuffers(window_);
    glfwPollEvents();
  }

  width_ = baseWidth;
  height_ = baseHeight;
  const std::uint64_t allocations = renderer_.framebufferAllocations() - before;
  logInfo("viewport swept {} .. {} px wide over {} frames", minWidth, maxWidth, frames);
  logInfo("offscreen targets reallocated {} times ({:.2f} per frame)", allocations,
          static_cast<double>(allocations) / std::max(1, frames));
}

int Viewer::run(const ViewerOptions& options) {
  options_ = options;
  const PrimeTopology topology = detectPrimeTopology();
  applyGpuPreference(options.gpu, topology);
  if (!createWindow(options, topology)) return 1;

  std::string error;
  if (!renderer_.initialize(error)) {
    logError("renderer init failed: {}", error);
    return 1;
  }
  if (!overlay_.initialize(error)) {
    logError("overlay init failed: {}", error);
    return 1;
  }
  // The benchmark measures the renderer, so it runs without the UI.
  // Screenshots keep it, which is how the panels get verified.
  const bool headless = options.benchmark || options.resizeStress;
  if (!headless && !ui_.initialize(window_, error)) {
    logError("UI init failed: {}", error);
    return 1;
  }

  settings_ = options.render;
  colorModeExplicit_ = options.colorModeExplicit;
  requestedField_ = options.fieldName;
  camera_.mode = options.nav;
  startZoom_ = options.zoom;
  cropFraction_ = options.cropFraction;
  if (options.haveStandardView) camera_.setStandardView(options.standardView);
  if (options.haveYaw) camera_.yaw = static_cast<float>(radians(options.yawDegrees));
  if (options.havePitch) camera_.pitch = static_cast<float>(radians(options.pitchDegrees));

  for (const std::string& path : options.files)
    if (!openFile(path)) return 1;

  if (!options.screenshotPath.empty()) {
    // Several frames so LOD, the statistics ring and ImGui's first-frame
    // layout all settle before the capture.
    const int frames = std::max(2, options.screenshotFrames);
    if (options.pickX >= 0 && options.pickY >= 0) {
      // Needs one render first so the cull pass has populated the draw
      // commands the pick pass reuses.
      renderFrame();
      cursorX_ = options.pickX;
      cursorY_ = options.pickY;
      pickUnderCursor();
    }
    if (!options.measureAt.empty()) {
      renderFrame();
      ui_.setMeasureMode(true);
      for (const auto& [mx, my] : options.measureAt) {
        cursorX_ = mx;
        cursorY_ = my;
        pickUnderCursor();
      }
    }
    for (int i = 0; i < frames; ++i) {
      glfwPollEvents();
      ui_.beginFrame();
      UiInput uiInput;
      uiInput.cloud = open_.cloud.get();
      uiInput.chunks = &open_.chunks;
      uiInput.chunkStats = &open_.chunkStats;
      uiInput.renderStats = &renderer_.stats();
      uiInput.device = &device_;
      uiInput.cloudPath = &open_.path;
      uiInput.frameMs = 0.7;
      uiInput.freeVramMB = queryFreeVramMB();
      uiInput.shaderLiveReload = ShaderLibrary::instance().liveReloadActive();
      uiInput.scalarField = activeScalarField_;
      uiInput.scalarLo = activeScalarLo_;
      uiInput.scalarHi = activeScalarHi_;
      ui_.draw(uiInput, settings_, camera_);
      renderFrame();
      ui_.drawViewportAnnotations(camera_, open_.cloud != nullptr ? open_.cloud->origin
                                                                 : Vec3{0, 0, 0},
                                  width_, height_);
      ui_.endFrame();
      consumePendingFrame();
      // Read from the back buffer on the final pass rather than swapping,
      // so the capture is the frame just drawn.
      if (i + 1 < frames) glfwSwapBuffers(window_);
    }
    glFinish();
    logFrameStats();
    return saveScreenshot(options.screenshotPath) ? 0 : 1;
  }

  if (options.resizeStress) {
    runResizeStress(options.resizeStressFrames);
    return 0;
  }

  if (options.benchmark) {
    runBenchmark(options.benchmarkFrames);
    return 0;
  }

  lastFrameTime_ = glfwGetTime();
  double statsTimer = 0.0;
  (void)statsTimer;
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

    ui_.beginFrame();
    UiInput uiInput;
    uiInput.cloud = open_.cloud.get();
    uiInput.chunks = &open_.chunks;
    uiInput.chunkStats = &open_.chunkStats;
    uiInput.renderStats = &renderer_.stats();
    uiInput.device = &device_;
    uiInput.cloudPath = &open_.path;
    uiInput.frameMs = smoothedFrameMs_;
    uiInput.freeVramMB = queryFreeVramMB();
    uiInput.shaderLiveReload = ShaderLibrary::instance().liveReloadActive();
    uiInput.scalarField = activeScalarField_;
    uiInput.scalarLo = activeScalarLo_;
    uiInput.scalarHi = activeScalarHi_;
    const UiActions actions = ui_.draw(uiInput, settings_, camera_);

    // The 3D pass clears and owns the default framebuffer, so it has to run
    // before ImGui's draw data is submitted on top of it.
    renderFrame();
    ui_.drawViewportAnnotations(camera_, open_.cloud != nullptr ? open_.cloud->origin
                                                               : Vec3{0, 0, 0},
                                width_, height_);
    ui_.endFrame();
    glfwSwapBuffers(window_);

    applyUiActions(actions);
    consumePendingFrame();

    statsTimer += delta;
    if (statsTimer > 2.0) {
      statsTimer = 0.0;
      logFrameStats();
    }
  }
  return 0;
}

}  // namespace punto
