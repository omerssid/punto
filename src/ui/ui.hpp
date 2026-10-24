// Dear ImGui interface: docked panels over a full-window viewport.
#pragma once

#include <cstdint>
#include <deque>
#include <string>
#include <vector>

#include "gpu/point_renderer.hpp"
#include "io/point_cloud.hpp"
#include "scene/camera.hpp"
#include "scene/chunk.hpp"

struct GLFWwindow;

namespace pcv {

// A point the user picked, resolved against the cloud.
struct PickedPoint {
  bool valid = false;
  std::uint32_t index = 0;   // into the cloud's arrays
  std::uint64_t sourceRow = 0;
  Vec3 world{0.0, 0.0, 0.0};
  float cameraDistance = 0.0f;
  int chunkIndex = -1;
};

// Things the UI asks the application to do. Returned once per frame and
// consumed by the caller, so the UI never touches the renderer directly.
struct UiActions {
  std::string openPath;
  bool frameAll = false;
  bool focusSelection = false;
  bool reloadShaders = false;
  bool clearMeasurement = false;

  // Set when the colour field or its range changed and the GPU scalar
  // buffer needs re-uploading.
  bool scalarFieldChanged = false;
  int scalarField = -1;
  double scalarLo = 0.0;
  double scalarHi = 1.0;
};

struct UiInput {
  const PointCloud* cloud = nullptr;
  std::vector<Chunk> const* chunks = nullptr;
  const ChunkBuildStats* chunkStats = nullptr;
  const RenderStats* renderStats = nullptr;
  const GlDeviceInfo* device = nullptr;
  const std::string* cloudPath = nullptr;
  double frameMs = 0.0;
  int freeVramMB = 0;
  bool shaderLiveReload = false;

  // The scalar channel the renderer currently has uploaded. The panel
  // mirrors this rather than keeping an independent idea of it, so a field
  // chosen on the command line or defaulted at load shows up correctly.
  int scalarField = -1;
  double scalarLo = 0.0;
  double scalarHi = 1.0;
};

class Ui {
 public:
  bool initialize(GLFWwindow* window, std::string& error);
  void shutdown();

  // Call once per frame, before rendering the viewport.
  void beginFrame();
  // Builds every panel. `settings` and `camera` are edited in place.
  UiActions draw(const UiInput& input, RenderSettings& settings, Camera& camera);
  void endFrame();

  // True when ImGui wants the event, so the viewer should not also orbit.
  bool wantsMouse() const;
  bool wantsKeyboard() const;

  // Adopts the renderer's current scalar channel. Call after the renderer's
  // field changes for a reason other than this panel.
  void syncScalarField(int field, double lo, double hi);

  void setSelection(const PickedPoint& point);
  void clearSelection();
  const PickedPoint& selection() const { return selection_; }

  // Measurement: successive picks extend the chain.
  void addMeasurementPoint(const PickedPoint& point);
  bool measureModeActive() const { return measureMode_; }

  bool panelsVisible() const { return panelsVisible_; }
  void togglePanels() { panelsVisible_ = !panelsVisible_; }

 private:
  void drawMenuBar(const UiInput& input, RenderSettings& settings, Camera& camera,
                   UiActions& actions);
  void drawStatisticsPanel(const UiInput& input);
  void drawDisplayPanel(const UiInput& input, RenderSettings& settings, UiActions& actions);
  void drawInspectorPanel(const UiInput& input, UiActions& actions);
  void drawPerformancePanel(const UiInput& input, RenderSettings& settings);
  void drawStatusBar(const UiInput& input, const Camera& camera);
  void drawFileBrowser(UiActions& actions);
  void drawHelpWindow();

  GLFWwindow* window_ = nullptr;
  bool initialized_ = false;

  bool panelsVisible_ = true;
  bool showStatistics_ = true;
  bool showDisplay_ = true;
  bool showInspector_ = true;
  bool showPerformance_ = true;
  bool showHelp_ = false;
  bool showFileBrowser_ = false;

  PickedPoint selection_;
  std::vector<PickedPoint> measurePoints_;
  bool measureMode_ = false;
  // Counts down over several frames: a focus request made on the frame the
  // dock layout is built gets overridden by DockBuilderFinish.
  int focusInspectorFrames_ = 0;

  // Colour-field editing state, mirrored so edits only trigger a re-upload
  // when the user actually lets go.
  int scalarField_ = -1;
  double scalarLo_ = 0.0;
  double scalarHi_ = 1.0;
  bool scalarRangeDirty_ = false;

  // Frame time history for the graph.
  std::deque<float> frameHistory_;
  static constexpr std::size_t kFrameHistory = 240;

  // File browser state.
  std::string browseDirectory_;
  std::string browseSelection_;

  std::string iniPath_;
  bool layoutBuilt_ = false;
  bool layoutRestored_ = false;
};

}  // namespace pcv
