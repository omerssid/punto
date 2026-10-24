// Window, input handling and the frame loop.
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "app/gpu_select.hpp"
#include "gpu/point_renderer.hpp"
#include "io/loader.hpp"
#include "scene/camera.hpp"
#include "scene/ram_chunk_source.hpp"
#include "ui/ui.hpp"

struct GLFWwindow;

namespace pcv {

struct ViewerOptions {
  std::vector<std::string> files;
  GpuPreference gpu = GpuPreference::Auto;
  int width = 1600;
  int height = 950;
  bool vsync = true;

  // Render one frame, write it to screenshotPath, and exit. Used for
  // verification without an interactive session.
  std::string screenshotPath;
  int screenshotFrames = 6;  // let LOD, the stats ring and ImGui settle

  // Run a fixed camera orbit and report frame times, then exit.
  bool benchmark = false;
  int benchmarkFrames = 300;

  // Initial view, mainly for scripted screenshots and benchmarks.
  NavMode nav = NavMode::Orbit;
  bool haveYaw = false;
  float yawDegrees = -45.0f;
  bool havePitch = false;
  float pitchDegrees = 40.0f;
  // Multiplier on the framed distance: 4 means four times closer.
  float zoom = 1.0f;

  RenderSettings render;
  bool colorModeExplicit = false;
  std::string fieldName;  // scalar field to colour by, by name

  // Pick at this screen position before capturing, so the inspector can be
  // exercised without an interactive session. Negative means no pick.
  int pickX = -1;
  int pickY = -1;

  LoadOptions loadOptions;
  ChunkBuildOptions chunkOptions;
};

// What the UI needs to know about the currently open cloud.
struct OpenCloud {
  std::shared_ptr<PointCloud> cloud;
  std::shared_ptr<RamChunkSource> source;
  std::vector<Chunk> chunks;
  ChunkBuildStats chunkStats;
  LoadResult loadResult;
  std::string path;
};

class Viewer {
 public:
  Viewer();
  ~Viewer();

  int run(const ViewerOptions& options);

 private:
  bool createWindow(const ViewerOptions& options);
  void destroyWindow();
  bool openFile(const std::string& path);

  void processInput(float deltaSeconds);
  void renderFrame();
  bool saveScreenshot(const std::string& path);
  void runBenchmark(int frames);
  void logFrameStats();

  // GLFW callbacks dispatch into these.
  void onMouseButton(int button, int action, int mods);
  void onCursorMove(double x, double y);
  void onScroll(double dx, double dy);
  void onKey(int key, int scancode, int action, int mods);
  void onFramebufferSize(int width, int height);
  void onFileDrop(int count, const char** paths);

  void applyUiActions(const UiActions& actions);
  void pickUnderCursor();
  PickedPoint resolvePick(std::uint32_t index, float distance) const;

  static Viewer* fromWindow(GLFWwindow* window);

  GLFWwindow* window_ = nullptr;
  int width_ = 0;
  int height_ = 0;

  PointRenderer renderer_;
  Ui ui_;
  Camera camera_;
  RenderSettings settings_;
  OpenCloud open_;
  GlDeviceInfo device_;

  // Input state.
  bool dragOrbit_ = false;
  bool dragPan_ = false;
  double lastCursorX_ = 0.0;
  double lastCursorY_ = 0.0;
  double cursorX_ = 0.0;
  double cursorY_ = 0.0;
  bool keyDown_[512] = {};

  // Frame pacing.
  double lastFrameTime_ = 0.0;
  double frameMs_ = 0.0;
  double smoothedFrameMs_ = 0.0;

  // Selection.
  bool hasSelection_ = false;
  std::uint32_t selectedPoint_ = 0;
  float selectedDistance_ = 0.0f;

  bool wantsClose_ = false;
  bool colorModeExplicit_ = false;
  float startZoom_ = 1.0f;
  std::string requestedField_;
  int activeScalarField_ = -1;
  double activeScalarLo_ = 0.0;
  double activeScalarHi_ = 1.0;
};

}  // namespace pcv
