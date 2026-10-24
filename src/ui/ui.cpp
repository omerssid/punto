#include "ui/ui.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <format>

#include "imgui.h"
#include "imgui_internal.h"  // DockBuilder, for the default layout
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#include "core/log.hpp"
#include "gpu/colormap.hpp"
#include "gpu/gl.hpp"

namespace punto {
namespace {

std::string withThousands(std::uint64_t v) {
  std::string s = std::to_string(v);
  for (int i = static_cast<int>(s.size()) - 3; i > 0; i -= 3)
    s.insert(static_cast<std::size_t>(i), ",");
  return s;
}

std::string humanBytes(std::uint64_t bytes) {
  static const char* units[] = {"B", "KB", "MB", "GB", "TB"};
  double v = static_cast<double>(bytes);
  int u = 0;
  while (v >= 1024.0 && u < 4) { v /= 1024.0; ++u; }
  return std::format("{:.1f} {}", v, units[u]);
}

// A label/value row, so every panel lines up the same way.
void labelValue(const char* label, const std::string& value) {
  ImGui::TableNextRow();
  ImGui::TableSetColumnIndex(0);
  ImGui::TextUnformatted(label);
  ImGui::TableSetColumnIndex(1);
  ImGui::TextUnformatted(value.c_str());
}

void labelValue(const char* label, const char* value) {
  labelValue(label, std::string(value));
}

bool beginKeyValueTable(const char* id) {
  return ImGui::BeginTable(id, 2,
                           ImGuiTableFlags_SizingStretchProp | ImGuiTableFlags_NoSavedSettings);
}

// Draws a histogram, optionally tinted by the active ramp and overlaid with
// the selected range. Returns true while the user is dragging an edge.
//
// Log-scaled vertically: point cloud attribute distributions are routinely
// dominated by one mode -- a road surface at a single intensity -- and a
// linear axis would flatten everything else to nothing.
bool histogramWidget(const char* id, const std::vector<std::uint32_t>& bins, double binMin,
                     double binMax, double& rangeLo, double& rangeHi, int ramp, bool tint,
                     float height) {
  if (bins.empty()) return false;

  ImDrawList* draw = ImGui::GetWindowDrawList();
  const ImVec2 origin = ImGui::GetCursorScreenPos();
  const float width = ImGui::GetContentRegionAvail().x;
  if (width <= 1.0f) return false;

  ImGui::InvisibleButton(id, ImVec2(width, height));
  const bool hovered = ImGui::IsItemHovered();
  const bool active = ImGui::IsItemActive();

  std::uint32_t peak = 0;
  for (std::uint32_t v : bins) peak = std::max(peak, v);
  if (peak == 0) peak = 1;
  const float logPeak = std::log1p(static_cast<float>(peak));

  draw->AddRectFilled(origin, ImVec2(origin.x + width, origin.y + height),
                      IM_COL32(18, 20, 24, 255), 3.0f);

  const double span = binMax - binMin;
  auto valueToX = [&](double v) {
    const double t = span > 0.0 ? (v - binMin) / span : 0.0;
    return origin.x + static_cast<float>(std::clamp(t, 0.0, 1.0)) * width;
  };

  const float barWidth = width / static_cast<float>(bins.size());
  for (std::size_t i = 0; i < bins.size(); ++i) {
    const float t = (static_cast<float>(i) + 0.5f) / static_cast<float>(bins.size());
    const float h = std::log1p(static_cast<float>(bins[i])) / logPeak * (height - 2.0f);
    const float x0 = origin.x + static_cast<float>(i) * barWidth;

    // Bars outside the selected range are dimmed, so the range reads at a
    // glance rather than needing the numbers.
    const double value = binMin + static_cast<double>(t) * span;
    const bool inRange = value >= rangeLo && value <= rangeHi;

    ImU32 color;
    if (tint) {
      const double norm = (rangeHi > rangeLo) ? (value - rangeLo) / (rangeHi - rangeLo) : 0.0;
      const Vec3f c = evaluateRamp(ramp, static_cast<float>(std::clamp(norm, 0.0, 1.0)));
      const float dim = inRange ? 1.0f : 0.25f;
      color = IM_COL32(static_cast<int>(c.x * 255 * dim), static_cast<int>(c.y * 255 * dim),
                       static_cast<int>(c.z * 255 * dim), 255);
    } else {
      color = inRange ? IM_COL32(120, 160, 220, 255) : IM_COL32(60, 68, 82, 255);
    }
    draw->AddRectFilled(ImVec2(x0, origin.y + height - h),
                        ImVec2(x0 + barWidth + 0.5f, origin.y + height), color);
  }

  // Range handles.
  const float loX = valueToX(rangeLo);
  const float hiX = valueToX(rangeHi);
  draw->AddLine(ImVec2(loX, origin.y), ImVec2(loX, origin.y + height), IM_COL32(255, 220, 120, 220), 1.5f);
  draw->AddLine(ImVec2(hiX, origin.y), ImVec2(hiX, origin.y + height), IM_COL32(255, 220, 120, 220), 1.5f);

  bool changed = false;
  if (active && span > 0.0) {
    const float mouseX = ImGui::GetIO().MousePos.x;
    const double value = binMin + static_cast<double>((mouseX - origin.x) / width) * span;
    // Drag whichever edge is nearer, so one widget serves both.
    if (std::abs(mouseX - loX) < std::abs(mouseX - hiX))
      rangeLo = std::clamp(value, binMin, rangeHi);
    else
      rangeHi = std::clamp(value, rangeLo, binMax);
    changed = true;
  }

  if (hovered && span > 0.0) {
    const float mouseX = ImGui::GetIO().MousePos.x;
    const double value = binMin + static_cast<double>((mouseX - origin.x) / width) * span;
    const std::size_t bin = std::min(
        bins.size() - 1, static_cast<std::size_t>(std::clamp((value - binMin) / span, 0.0, 0.9999) *
                                                  static_cast<double>(bins.size())));
    ImGui::SetTooltip("%.4g\n%s points", value, withThousands(bins[bin]).c_str());
  }
  return changed;
}

void rampPreview(int ramp, float height) {
  ImDrawList* draw = ImGui::GetWindowDrawList();
  const ImVec2 origin = ImGui::GetCursorScreenPos();
  const float width = ImGui::GetContentRegionAvail().x;
  ImGui::InvisibleButton("##ramp", ImVec2(width, height));

  constexpr int kSteps = 64;
  const float step = width / kSteps;
  for (int i = 0; i < kSteps; ++i) {
    const Vec3f c = evaluateRamp(ramp, (static_cast<float>(i) + 0.5f) / kSteps);
    draw->AddRectFilled(
        ImVec2(origin.x + static_cast<float>(i) * step, origin.y),
        ImVec2(origin.x + static_cast<float>(i + 1) * step + 0.5f, origin.y + height),
        IM_COL32(static_cast<int>(c.x * 255), static_cast<int>(c.y * 255),
                 static_cast<int>(c.z * 255), 255));
  }
}

// Emits a label on the left and stretches the following widget to fill the
// rest of the row. ImGui's default puts the label after the widget, which
// leaves a control panel reading right-to-left.
constexpr float kLabelWidth = 108.0f;

void helpMarker(const char* text);

void fieldLabel(const char* label, const char* tooltip = nullptr) {
  ImGui::AlignTextToFramePadding();
  ImGui::TextUnformatted(label);
  if (tooltip != nullptr) helpMarker(tooltip);
  ImGui::SameLine(kLabelWidth);
  ImGui::SetNextItemWidth(-1.0f);
}

void helpMarker(const char* text) {
  ImGui::SameLine();
  ImGui::TextDisabled("(?)");
  if (ImGui::BeginItemTooltip()) {
    ImGui::PushTextWrapPos(ImGui::GetFontSize() * 28.0f);
    ImGui::TextUnformatted(text);
    ImGui::PopTextWrapPos();
    ImGui::EndTooltip();
  }
}

void applyTheme() {
  ImGuiStyle& style = ImGui::GetStyle();
  ImGui::StyleColorsDark();
  style.WindowRounding = 4.0f;
  style.FrameRounding = 3.0f;
  style.GrabRounding = 3.0f;
  style.ScrollbarRounding = 3.0f;
  style.TabRounding = 3.0f;
  style.WindowBorderSize = 1.0f;
  style.FrameBorderSize = 0.0f;
  style.WindowPadding = ImVec2(10, 8);
  style.FramePadding = ImVec2(7, 4);
  style.ItemSpacing = ImVec2(8, 6);
  style.CellPadding = ImVec2(6, 3);

  ImVec4* c = style.Colors;
  c[ImGuiCol_WindowBg] = ImVec4(0.09f, 0.10f, 0.12f, 0.97f);
  c[ImGuiCol_ChildBg] = ImVec4(0.11f, 0.12f, 0.15f, 1.00f);
  c[ImGuiCol_PopupBg] = ImVec4(0.10f, 0.11f, 0.14f, 0.98f);
  c[ImGuiCol_Border] = ImVec4(0.22f, 0.24f, 0.29f, 0.70f);
  c[ImGuiCol_FrameBg] = ImVec4(0.16f, 0.18f, 0.22f, 1.00f);
  c[ImGuiCol_FrameBgHovered] = ImVec4(0.22f, 0.25f, 0.31f, 1.00f);
  c[ImGuiCol_FrameBgActive] = ImVec4(0.26f, 0.30f, 0.38f, 1.00f);
  c[ImGuiCol_TitleBg] = ImVec4(0.07f, 0.08f, 0.10f, 1.00f);
  c[ImGuiCol_TitleBgActive] = ImVec4(0.12f, 0.14f, 0.18f, 1.00f);
  c[ImGuiCol_Header] = ImVec4(0.20f, 0.24f, 0.32f, 1.00f);
  c[ImGuiCol_HeaderHovered] = ImVec4(0.26f, 0.32f, 0.42f, 1.00f);
  c[ImGuiCol_HeaderActive] = ImVec4(0.30f, 0.38f, 0.50f, 1.00f);
  c[ImGuiCol_Button] = ImVec4(0.20f, 0.24f, 0.31f, 1.00f);
  c[ImGuiCol_ButtonHovered] = ImVec4(0.27f, 0.33f, 0.43f, 1.00f);
  c[ImGuiCol_ButtonActive] = ImVec4(0.33f, 0.42f, 0.55f, 1.00f);
  c[ImGuiCol_SliderGrab] = ImVec4(0.42f, 0.56f, 0.78f, 1.00f);
  c[ImGuiCol_SliderGrabActive] = ImVec4(0.52f, 0.68f, 0.92f, 1.00f);
  c[ImGuiCol_CheckMark] = ImVec4(0.52f, 0.72f, 0.98f, 1.00f);
  c[ImGuiCol_Separator] = ImVec4(0.22f, 0.24f, 0.29f, 0.60f);
  c[ImGuiCol_TableHeaderBg] = ImVec4(0.14f, 0.16f, 0.20f, 1.00f);
  c[ImGuiCol_TableRowBgAlt] = ImVec4(1.00f, 1.00f, 1.00f, 0.025f);
  c[ImGuiCol_Tab] = ImVec4(0.13f, 0.15f, 0.19f, 1.00f);
  c[ImGuiCol_TabHovered] = ImVec4(0.26f, 0.32f, 0.42f, 1.00f);
  c[ImGuiCol_TabActive] = ImVec4(0.20f, 0.25f, 0.33f, 1.00f);
  c[ImGuiCol_DockingPreview] = ImVec4(0.35f, 0.50f, 0.75f, 0.60f);
}

}  // namespace

bool Ui::initialize(GLFWwindow* window, std::string& error) {
  window_ = window;
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();

  ImGuiIO& io = ImGui::GetIO();
  io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
  io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
  // Persist the layout next to the user's other config rather than in the
  // working directory.
  const char* home = std::getenv("HOME");
  if (home != nullptr) {
    const std::filesystem::path dir = std::filesystem::path(home) / ".config" / "punto";
    std::error_code ec;
    std::filesystem::create_directories(dir, ec);
    if (!ec) {
      iniPath_ = (dir / "layout.ini").string();
      io.IniFilename = iniPath_.c_str();
    }
  }

  applyTheme();

  if (!ImGui_ImplGlfw_InitForOpenGL(window, true)) {
    error = "ImGui_ImplGlfw_InitForOpenGL failed";
    return false;
  }
  if (!ImGui_ImplOpenGL3_Init("#version 460")) {
    error = "ImGui_ImplOpenGL3_Init failed";
    return false;
  }

  browseDirectory_ = std::filesystem::current_path().string();
  // A saved layout means the user has arranged things; do not overwrite it.
  layoutRestored_ = !iniPath_.empty() && std::filesystem::exists(iniPath_);
  initialized_ = true;
  return true;
}

void Ui::shutdown() {
  if (!initialized_) return;
  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImGui::DestroyContext();
  initialized_ = false;
}

void Ui::beginFrame() {
  if (!initialized_) return;
  ImGui_ImplOpenGL3_NewFrame();
  ImGui_ImplGlfw_NewFrame();
  ImGui::NewFrame();
}

void Ui::endFrame() {
  if (!initialized_) return;
  ImGui::Render();
  ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}

bool Ui::wantsMouse() const {
  return initialized_ && ImGui::GetIO().WantCaptureMouse;
}
bool Ui::wantsKeyboard() const {
  return initialized_ && ImGui::GetIO().WantCaptureKeyboard;
}

Ui::ViewportRect Ui::viewportRect(int framebufferWidth, int framebufferHeight) const {
  ViewportRect rect{0, 0, framebufferWidth, framebufferHeight};
  if (!initialized_ || !panelsVisible_) return rect;

  const ImGuiDockNode* central = ImGui::DockBuilderGetCentralNode(dockspaceId_);
  if (central == nullptr || central->Size.x < 1.0f || central->Size.y < 1.0f) return rect;

  // ImGui works in logical points with the origin at the top left; GL wants
  // framebuffer pixels with the origin at the bottom left.
  const ImGuiIO& io = ImGui::GetIO();
  const float scaleX = io.DisplaySize.x > 0.0f
                           ? static_cast<float>(framebufferWidth) / io.DisplaySize.x
                           : 1.0f;
  const float scaleY = io.DisplaySize.y > 0.0f
                           ? static_cast<float>(framebufferHeight) / io.DisplaySize.y
                           : 1.0f;

  const int x = static_cast<int>(central->Pos.x * scaleX);
  const int width = static_cast<int>(central->Size.x * scaleX);
  const int height = static_cast<int>(central->Size.y * scaleY);
  const int topY = static_cast<int>(central->Pos.y * scaleY);
  const int y = framebufferHeight - topY - height;

  rect.x = std::max(0, x);
  rect.y = std::max(0, y);
  rect.width = std::clamp(width, 1, framebufferWidth);
  rect.height = std::clamp(height, 1, framebufferHeight);
  return rect;
}

void Ui::syncScalarField(int field, double lo, double hi) {
  scalarField_ = field;
  scalarLo_ = lo;
  scalarHi_ = hi;
  scalarRangeDirty_ = false;
}

void Ui::setSelection(const PickedPoint& point) {
  selection_ = point;
  // Bring the Inspector forward: picking a point is a request to look at
  // it, and leaving it behind another tab hides the answer.
  if (point.valid) {
    focusInspectorFrames_ = 3;
    showInspector_ = true;
    panelsVisible_ = true;
  }
}
void Ui::clearSelection() { selection_ = PickedPoint{}; }

void Ui::addMeasurementPoint(const PickedPoint& point) {
  if (!point.valid) return;
  measurePoints_.push_back(point);
}

void Ui::drawViewportAnnotations(const Camera& camera, const Vec3& origin, int width,
                                 int height) {
  if (!initialized_ || measurePoints_.size() < 2) return;

  // Labels must land on the viewport the scene was rendered into, which is
  // the dockspace's central node rather than the whole window.
  const ViewportRect rect = viewportRect(width, height);
  const float aspect =
      static_cast<float>(rect.width) / static_cast<float>(std::max(1, rect.height));
  const Mat4 viewProj = camera.viewProjection(aspect);

  // ImGui's foreground list works in logical points from the top left.
  const ImGuiIO& io = ImGui::GetIO();
  const float scaleX = width > 0 ? io.DisplaySize.x / static_cast<float>(width) : 1.0f;
  const float scaleY = height > 0 ? io.DisplaySize.y / static_cast<float>(height) : 1.0f;
  const float originX = static_cast<float>(rect.x) * scaleX;
  const float originY = static_cast<float>(height - rect.y - rect.height) * scaleY;
  const float spanX = static_cast<float>(rect.width) * scaleX;
  const float spanY = static_cast<float>(rect.height) * scaleY;

  // World to screen. Returns false behind the camera, where the perspective
  // divide would mirror the point to the wrong side of the viewport.
  auto project = [&](const Vec3& world, ImVec2& out) {
    const Vec3 local = world - origin;
    const Vec4f clip = viewProj * Vec4f{static_cast<float>(local.x), static_cast<float>(local.y),
                                        static_cast<float>(local.z), 1.0f};
    if (clip.w <= 1e-6f) return false;
    out.x = originX + (clip.x / clip.w * 0.5f + 0.5f) * spanX;
    out.y = originY + (1.0f - (clip.y / clip.w * 0.5f + 0.5f)) * spanY;
    return true;
  };

  ImDrawList* draw = ImGui::GetForegroundDrawList();
  for (std::size_t i = 1; i < measurePoints_.size(); ++i) {
    const Vec3 a = measurePoints_[i - 1].world;
    const Vec3 b = measurePoints_[i].world;
    ImVec2 mid;
    if (!project((a + b) * 0.5, mid)) continue;

    char label[64];
    std::snprintf(label, sizeof(label), "%.3f m", length(b - a));
    const ImVec2 size = ImGui::CalcTextSize(label);
    const ImVec2 pad(6.0f, 3.0f);
    // A filled plate behind the text, or the label vanishes against a
    // bright patch of cloud.
    draw->AddRectFilled(ImVec2(mid.x - size.x * 0.5f - pad.x, mid.y - size.y * 0.5f - pad.y),
                        ImVec2(mid.x + size.x * 0.5f + pad.x, mid.y + size.y * 0.5f + pad.y),
                        IM_COL32(20, 22, 28, 220), 3.0f);
    draw->AddText(ImVec2(mid.x - size.x * 0.5f, mid.y - size.y * 0.5f),
                  IM_COL32(255, 228, 140, 255), label);
  }
}

// ------------------------------------------------------------------ draw

UiActions Ui::draw(const UiInput& input, RenderSettings& settings, Camera& camera) {
  UiActions actions;
  if (!initialized_) return actions;

  frameHistory_.push_back(static_cast<float>(input.frameMs));
  while (frameHistory_.size() > kFrameHistory) frameHistory_.pop_front();

  // A dockspace over the whole window, with no background, so the 3D
  // viewport shows through wherever no panel is docked.
  const ImGuiViewport* viewport = ImGui::GetMainViewport();
  // Leave room for the status bar, which is not part of the dockspace.
  const float statusHeight = ImGui::GetFrameHeight();
  ImGui::SetNextWindowPos(viewport->WorkPos);
  ImGui::SetNextWindowSize(ImVec2(viewport->WorkSize.x, viewport->WorkSize.y - statusHeight));
  ImGui::SetNextWindowViewport(viewport->ID);
  ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
  ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
  ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));

  const ImGuiWindowFlags hostFlags =
      ImGuiWindowFlags_NoDocking | ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoCollapse |
      ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoBringToFrontOnFocus |
      ImGuiWindowFlags_NoNavFocus | ImGuiWindowFlags_NoBackground | ImGuiWindowFlags_MenuBar;

  ImGui::Begin("##dockhost", nullptr, hostFlags);
  ImGui::PopStyleVar(3);

  drawMenuBar(input, settings, camera, actions);

  // PassthruCentralNode leaves the middle undrawn, which is what lets the
  // viewport render underneath instead of behind an opaque panel.
  const ImGuiID dockspaceId = ImGui::GetID("punto_dockspace");
  dockspaceId_ = dockspaceId;
  ImGui::DockSpace(dockspaceId, ImVec2(0, 0), ImGuiDockNodeFlags_PassthruCentralNode);

  // Build a sensible layout the first time, when no saved ini exists.
  // Without this ImGui floats every panel at the same spot, which is a
  // terrible first impression even though dragging them apart works.
  if (!layoutBuilt_) {
    layoutBuilt_ = true;
    if (ImGui::DockBuilderGetNode(dockspaceId) == nullptr || !layoutRestored_) {
      ImGui::DockBuilderRemoveNode(dockspaceId);
      // DockSpace is one of ImGui's private flags and PassthruCentralNode a
      // public one; C++20 deprecates OR-ing distinct enumeration types, so
      // combine them as the integer the API actually takes.
      ImGui::DockBuilderAddNode(
          dockspaceId, static_cast<ImGuiDockNodeFlags>(
                           static_cast<int>(ImGuiDockNodeFlags_DockSpace) |
                           static_cast<int>(ImGuiDockNodeFlags_PassthruCentralNode)));
      ImGui::DockBuilderSetNodeSize(dockspaceId, ImGui::GetMainViewport()->WorkSize);

      // One column on the right, split in two. Everything else stays
      // viewport, which is what people actually came to look at.
      ImGuiID centre = dockspaceId;
      const ImGuiID right = ImGui::DockBuilderSplitNode(centre, ImGuiDir_Right, 0.26f,
                                                        nullptr, &centre);
      // View gets its own slot at the top rather than joining the tab
      // group, so switching viewpoint never costs a tab change -- which is
      // the whole reason it exists instead of the menu.
      ImGuiID rightRest = right;
      const ImGuiID rightTop =
          ImGui::DockBuilderSplitNode(rightRest, ImGuiDir_Up, 0.29f, nullptr, &rightRest);
      ImGuiID rightMiddle = rightRest;
      const ImGuiID rightBottom =
          ImGui::DockBuilderSplitNode(rightMiddle, ImGuiDir_Down, 0.55f, nullptr, &rightMiddle);

      ImGui::DockBuilderDockWindow("View", rightTop);
      ImGui::DockBuilderDockWindow("Display", rightMiddle);
      // Tabbed together: you read one of these at a time.
      ImGui::DockBuilderDockWindow("Cloud statistics", rightBottom);
      ImGui::DockBuilderDockWindow("Inspector", rightBottom);
      ImGui::DockBuilderDockWindow("Performance", rightBottom);
      ImGui::DockBuilderFinish(dockspaceId);
    }
  }
  ImGui::End();

  if (panelsVisible_) {
    if (showView_) drawViewPanel(input, settings, camera, actions);
    if (showStatistics_) drawStatisticsPanel(input);
    if (showDisplay_) drawDisplayPanel(input, settings, actions);
    if (showInspector_) drawInspectorPanel(input, actions);
    if (showPerformance_) drawPerformancePanel(input, settings);
    if (showFileBrowser_) drawFileBrowser(actions);
    if (showHelp_) drawHelpWindow();
  }
  drawStatusBar(input, camera);

  return actions;
}

void Ui::drawMenuBar(const UiInput& input, RenderSettings& settings, Camera& camera,
                     UiActions& actions) {
  if (!ImGui::BeginMenuBar()) return;

  if (ImGui::BeginMenu("File")) {
    if (ImGui::MenuItem("Open...", "Ctrl+O")) showFileBrowser_ = true;
    ImGui::Separator();
    ImGui::TextDisabled("or drag a file onto the window");
    ImGui::Separator();
    if (ImGui::MenuItem("Quit", "Esc"))
      glfwSetWindowShouldClose(window_, 1);
    ImGui::EndMenu();
  }

  if (ImGui::BeginMenu("View")) {
    if (ImGui::MenuItem("Frame all", "F")) actions.frameAll = true;
    if (ImGui::MenuItem("Focus selection", "G", false, selection_.valid))
      actions.focusSelection = true;
    ImGui::Separator();
    const float aspect = ImGui::GetIO().DisplaySize.x /
                         std::max(1.0f, ImGui::GetIO().DisplaySize.y);
    if (ImGui::MenuItem("Orbit", "1", camera.mode == NavMode::Orbit))
      camera.setMode(NavMode::Orbit, aspect);
    if (ImGui::MenuItem("Fly", "2", camera.mode == NavMode::Fly))
      camera.setMode(NavMode::Fly, aspect);
    if (ImGui::MenuItem("Top-down", "3", camera.mode == NavMode::TopDown))
      camera.setMode(NavMode::TopDown, aspect);
    ImGui::EndMenu();
  }

  if (ImGui::BeginMenu("Panels")) {
    ImGui::MenuItem("View", nullptr, &showView_);
    ImGui::MenuItem("Cloud statistics", nullptr, &showStatistics_);
    ImGui::MenuItem("Display", nullptr, &showDisplay_);
    ImGui::MenuItem("Inspector", nullptr, &showInspector_);
    ImGui::MenuItem("Performance", nullptr, &showPerformance_);
    ImGui::Separator();
    if (ImGui::MenuItem("Hide all panels", "Tab")) panelsVisible_ = false;
    ImGui::EndMenu();
  }

  if (ImGui::BeginMenu("Render")) {
    ImGui::MenuItem("Eye-dome lighting", "E", &settings.edlEnabled);
    ImGui::MenuItem("Level of detail", "L", &settings.lodEnabled);
    ImGui::MenuItem("Round points", nullptr, &settings.roundPoints);
    ImGui::MenuItem("Shade splats", nullptr, &settings.shadeSplats);
    ImGui::MenuItem("Depth buffer view", nullptr, &settings.showDepthOnly);
    ImGui::Separator();
    if (ImGui::MenuItem("Reload shaders", "R", false, input.shaderLiveReload))
      actions.reloadShaders = true;
    ImGui::EndMenu();
  }

  if (ImGui::BeginMenu("Help")) {
    ImGui::MenuItem("Controls", nullptr, &showHelp_);
    ImGui::EndMenu();
  }

  // Right-aligned live readout, always visible regardless of panel state.
  if (input.renderStats != nullptr) {
    char buffer[160];
    std::snprintf(buffer, sizeof(buffer), "%.2f ms   %.0f fps   %s pts", input.frameMs,
                  input.frameMs > 0.0 ? 1000.0 / input.frameMs : 0.0,
                  withThousands(input.renderStats->drawnPoints).c_str());
    const float textWidth = ImGui::CalcTextSize(buffer).x;
    ImGui::SameLine(ImGui::GetWindowWidth() - textWidth - 16.0f);
    ImGui::TextDisabled("%s", buffer);
  }

  ImGui::EndMenuBar();
}

void Ui::drawViewPanel(const UiInput& input, RenderSettings& settings, Camera& camera,
                       UiActions& actions) {
  if (!ImGui::Begin("View", &showView_)) {
    ImGui::End();
    return;
  }

  const float full = ImGui::GetContentRegionAvail().x;
  const float spacing = ImGui::GetStyle().ItemSpacing.x;

  // ----------------------------------------------------- navigation mode
  //
  // A segmented row rather than a combo: the whole point of this panel is
  // that switching takes one click, and a combo takes two.
  const float third = (full - spacing * 2.0f) / 3.0f;
  struct ModeButton { NavMode mode; const char* label; const char* key; const char* tip; };
  static const ModeButton kModes[] = {
      {NavMode::Orbit, "Orbit", "1", "Turntable around a focus point. Best for inspecting "
                                     "one area."},
      {NavMode::Fly, "Fly", "2", "First person, WASD to move and drag to look. Best for "
                                 "travelling along a long map."},
      {NavMode::TopDown, "Top-down", "3", "Orthographic map view with pan and zoom. Best for "
                                          "judging coverage and spotting drift."},
  };
  for (int i = 0; i < 3; ++i) {
    if (i > 0) ImGui::SameLine();
    const bool active = camera.mode == kModes[i].mode;
    if (active) ImGui::PushStyleColor(ImGuiCol_Button, ImGui::GetStyle().Colors[ImGuiCol_ButtonActive]);
    if (ImGui::Button(kModes[i].label, ImVec2(third, 0))) {
      const float aspect = ImGui::GetIO().DisplaySize.x /
                           std::max(1.0f, ImGui::GetIO().DisplaySize.y);
      camera.setMode(kModes[i].mode, aspect);
    }
    if (active) ImGui::PopStyleColor();
    if (ImGui::IsItemHovered())
      ImGui::SetTooltip("%s  (%s)", kModes[i].tip, kModes[i].key);
  }

  // ------------------------------------------------------ standard views
  ImGui::SeparatorText("Look from");
  // Named by the side the camera sits on, because "front" and "left" mean
  // different things in different tools, while "+X" does not.
  static const StandardView kRow1[] = {StandardView::Top, StandardView::Bottom,
                                       StandardView::Isometric};
  static const StandardView kRow2[] = {StandardView::PlusX, StandardView::MinusX,
                                       StandardView::PlusY, StandardView::MinusY};

  for (int i = 0; i < 3; ++i) {
    if (i > 0) ImGui::SameLine();
    if (ImGui::Button(standardViewName(kRow1[i]), ImVec2(third, 0)))
      camera.setStandardView(kRow1[i]);
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("%s", standardViewTooltip(kRow1[i]));
  }

  const float quarter = (full - spacing * 3.0f) / 4.0f;
  for (int i = 0; i < 4; ++i) {
    if (i > 0) ImGui::SameLine();
    if (ImGui::Button(standardViewName(kRow2[i]), ImVec2(quarter, 0)))
      camera.setStandardView(kRow2[i]);
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("%s", standardViewTooltip(kRow2[i]));
  }

  // ------------------------------------------------------------- framing
  ImGui::SeparatorText("Framing");
  const float half = (full - spacing) / 2.0f;
  if (ImGui::Button("Frame all", ImVec2(half, 0))) actions.frameAll = true;
  if (ImGui::IsItemHovered()) ImGui::SetTooltip("Fit the whole cloud in view  (F)");
  ImGui::SameLine();
  ImGui::BeginDisabled(!selection_.valid);
  if (ImGui::Button("Focus selection", ImVec2(half, 0))) actions.focusSelection = true;
  ImGui::EndDisabled();
  if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
    ImGui::SetTooltip(selection_.valid ? "Centre on the selected point  (G)"
                                       : "Shift-click a point first");

  if (camera.mode != NavMode::TopDown) {
    float fov = static_cast<float>(degrees(camera.fovY));
    fieldLabel("FOV");
    if (ImGui::SliderFloat("##fov", &fov, 20.0f, 100.0f, "%.0f deg"))
      camera.fovY = static_cast<float>(radians(std::clamp(fov, 10.0f, 120.0f)));
  }
  if (camera.mode == NavMode::Fly) {
    fieldLabel("Speed");
    ImGui::SliderFloat("##flyspeed", &camera.moveSpeed,
                       std::max(0.01f, camera.sceneRadius * 0.005f), camera.sceneRadius * 2.0f,
                       "%.2f m/s", ImGuiSliderFlags_Logarithmic);
  }

  (void)settings;
  (void)input;
  ImGui::End();
}

void Ui::drawStatisticsPanel(const UiInput& input) {
  if (!ImGui::Begin("Cloud statistics", &showStatistics_)) {
    ImGui::End();
    return;
  }
  const PointCloud* cloud = input.cloud;
  if (cloud == nullptr || cloud->empty()) {
    ImGui::TextDisabled("No cloud loaded.");
    ImGui::Spacing();
    ImGui::TextWrapped("Open a file, or drag one onto the window.");
    ImGui::End();
    return;
  }

  if (ImGui::CollapsingHeader("Source", ImGuiTreeNodeFlags_DefaultOpen)) {
    if (beginKeyValueTable("source")) {
      labelValue("File", input.cloudPath != nullptr
                             ? std::filesystem::path(*input.cloudPath).filename().string()
                             : std::string("-"));
      labelValue("Format", cloud->formatName);
      labelValue("File size", humanBytes(cloud->sourceBytes));
      labelValue("Points", withThousands(cloud->size()));
      if (cloud->invalidPointsDropped > 0) {
        const double pct =
            100.0 * static_cast<double>(cloud->invalidPointsDropped) /
            static_cast<double>(cloud->invalidPointsDropped + cloud->size());
        labelValue("Invalid dropped",
                   std::format("{} ({:.2f}%)", withThousands(cloud->invalidPointsDropped), pct));
      }
      if (cloud->organized)
        labelValue("Organized", std::format("{} x {}", cloud->width, cloud->height));
      labelValue("Colours", cloud->hasColors() ? "yes" : "no");
      labelValue("Source rows", cloud->hasSourceIndex() ? "tracked" : "not tracked");
      labelValue("Host memory", humanBytes(cloud->memoryBytes()));
      ImGui::EndTable();
    }
    if (ImGui::IsItemHovered()) {}
    for (const std::string& w : std::vector<std::string>{})
      ImGui::TextColored(ImVec4(1, 0.8f, 0.3f, 1), "%s", w.c_str());
  }

  if (ImGui::CollapsingHeader("Bounds", ImGuiTreeNodeFlags_DefaultOpen)) {
    const Aabb& b = cloud->boundsWorld;
    if (b.valid()) {
      if (ImGui::BeginTable("bounds", 4,
                            ImGuiTableFlags_SizingStretchProp | ImGuiTableFlags_RowBg)) {
        ImGui::TableSetupColumn("");
        ImGui::TableSetupColumn("X");
        ImGui::TableSetupColumn("Y");
        ImGui::TableSetupColumn("Z");
        ImGui::TableHeadersRow();
        auto row = [](const char* name, const Vec3& v) {
          ImGui::TableNextRow();
          ImGui::TableSetColumnIndex(0);
          ImGui::TextUnformatted(name);
          for (int i = 0; i < 3; ++i) {
            ImGui::TableSetColumnIndex(i + 1);
            ImGui::Text("%.4f", v[i]);
          }
        };
        row("min", b.lo);
        row("max", b.hi);
        row("size", b.size());
        row("centre", b.center());
        ImGui::EndTable();
      }
      ImGui::Text("diagonal  %.4f m", b.diagonal());
      ImGui::TextDisabled("origin  %.1f  %.1f  %.1f", cloud->origin.x, cloud->origin.y,
                          cloud->origin.z);
      helpMarker(
          "Positions are stored as float32 relative to this origin. HD maps carry "
          "UTM-scale coordinates where float32 resolves to about half a metre; "
          "rebasing restores sub-millimetre precision.");
    }
  }

  if (ImGui::CollapsingHeader("Fields", ImGuiTreeNodeFlags_DefaultOpen)) {
    if (ImGui::BeginTable("fields", 6,
                          ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg |
                              ImGuiTableFlags_SizingStretchProp | ImGuiTableFlags_ScrollY,
                          ImVec2(0, 190))) {
      ImGui::TableSetupScrollFreeze(0, 1);
      ImGui::TableSetupColumn("name");
      ImGui::TableSetupColumn("type");
      ImGui::TableSetupColumn("role");
      ImGui::TableSetupColumn("min");
      ImGui::TableSetupColumn("max");
      ImGui::TableSetupColumn("mean");
      ImGui::TableHeadersRow();

      for (const FieldInfo& f : cloud->fields) {
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::TextUnformatted(f.name.c_str());
        ImGui::TableSetColumnIndex(1);
        ImGui::TextDisabled("%s", fieldTypeName(f.type));
        ImGui::TableSetColumnIndex(2);
        ImGui::TextDisabled("%s", fieldRoleName(f.role));
        if (f.stats.valid()) {
          ImGui::TableSetColumnIndex(3);
          ImGui::Text("%.4g", f.stats.min);
          ImGui::TableSetColumnIndex(4);
          ImGui::Text("%.4g", f.stats.max);
          ImGui::TableSetColumnIndex(5);
          ImGui::Text("%.4g", f.stats.mean);
        } else {
          ImGui::TableSetColumnIndex(3);
          ImGui::TextDisabled("%s", f.role == FieldRole::Color ? "packed" : "-");
        }
      }
      ImGui::EndTable();
    }

    // Per-field detail, including the distribution.
    for (const FieldInfo& f : cloud->fields) {
      if (!f.stats.valid()) continue;
      if (!ImGui::TreeNode(f.name.c_str())) continue;
      ImGui::Text("stddev %.6g    finite %s", f.stats.stddev,
                  withThousands(f.stats.finiteCount).c_str());
      if (f.stats.nonFiniteCount > 0)
        ImGui::TextColored(ImVec4(1, 0.8f, 0.3f, 1), "%s non-finite values",
                           withThousands(f.stats.nonFiniteCount).c_str());
      if (f.stats.discrete && !f.stats.distinctValues.empty()) {
        std::string list;
        for (std::size_t i = 0; i < f.stats.distinctValues.size() && i < 24; ++i) {
          if (i) list += "  ";
          list += std::format("{:g}", f.stats.distinctValues[i]);
        }
        if (f.stats.distinctValues.size() > 24) list += "  ...";
        ImGui::TextDisabled("%zu distinct", f.stats.distinctValues.size());
        ImGui::TextWrapped("%s", list.c_str());
      }
      double lo = f.stats.min;
      double hi = f.stats.max;
      histogramWidget((f.name + "##hist").c_str(), f.stats.histogram, f.stats.min, f.stats.max,
                      lo, hi, 0, false, 54.0f);
      ImGui::TreePop();
    }
  }

  if (cloud->colorStatsValid && ImGui::CollapsingHeader("Colour channels")) {
    static const char* names[] = {"red", "green", "blue", "alpha"};
    for (int ch = 0; ch < 3; ++ch) {
      const FieldStats& s = cloud->colorChannelStats[static_cast<std::size_t>(ch)];
      if (!s.valid()) continue;
      ImGui::Text("%-6s  min %3.0f  max %3.0f  mean %6.2f", names[ch], s.min, s.max, s.mean);
      double lo = 0.0, hi = 255.0;
      histogramWidget((std::string(names[ch]) + "##chist").c_str(), s.histogram, 0.0, 255.0, lo, hi,
                      0, false, 38.0f);
    }
    if (cloud->colorIsGreyscale)
      ImGui::TextColored(ImVec4(1, 0.8f, 0.3f, 1),
                         "R = G = B on %.1f%% of points: greyscale, probably "
                         "intensity stored as RGB",
                         cloud->colorGreyscaleFraction * 100.0);
  }

  if (ImGui::CollapsingHeader("Load and chunking")) {
    if (beginKeyValueTable("timing")) {
      const LoadTiming& t = cloud->timing;
      labelValue("Load total", std::format("{:.0f} ms  ({:.0f} MB/s)", t.totalMs,
                                           t.throughputMBs(cloud->sourceBytes)));
      labelValue("  decode", std::format("{:.1f} ms", t.decodeMs));
      labelValue("  statistics", std::format("{:.1f} ms", t.statsMs));
      if (input.chunkStats != nullptr) {
        const ChunkBuildStats& c = *input.chunkStats;
        labelValue("Chunking", std::format("{:.0f} ms", c.totalMs));
        labelValue("  partition", std::format("{:.1f} ms", c.partitionMs));
        labelValue("  LOD order", std::format("{:.1f} ms", c.lodShuffleMs));
        labelValue("  reorder", std::format("{:.1f} ms", c.reorderMs));
        labelValue("Chunks", withThousands(c.chunkCount));
        labelValue("Points/chunk",
                   std::format("{} .. {} (mean {:.0f})", withThousands(c.minChunkPoints),
                               withThousands(c.maxChunkPoints), c.meanChunkPoints));
      }
      ImGui::EndTable();
    }
  }

  ImGui::End();
}

void Ui::drawDisplayPanel(const UiInput& input, RenderSettings& settings, UiActions& actions) {
  if (!ImGui::Begin("Display", &showDisplay_)) {
    ImGui::End();
    return;
  }
  const PointCloud* cloud = input.cloud;

  // ------------------------------------------------------------- colour
  ImGui::SeparatorText("Colour");

  const bool hasColors = cloud != nullptr && cloud->hasColors();
  int mode = static_cast<int>(settings.colorMode);
  const char* modeNames[] = {"RGB", "Scalar field", "Flat", "Chunk (debug)"};
  fieldLabel("Source");
  if (ImGui::BeginCombo("##source", modeNames[mode])) {
    for (int i = 0; i < 4; ++i) {
      const bool disabled = (i == 0 && !hasColors);
      ImGui::BeginDisabled(disabled);
      if (ImGui::Selectable(modeNames[i], mode == i)) settings.colorMode = static_cast<ColorMode>(i);
      ImGui::EndDisabled();
      if (disabled && ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
        ImGui::SetTooltip("this cloud has no colour data");
    }
    ImGui::EndCombo();
  }

  if (settings.colorMode == ColorMode::Flat) {
    float c[3] = {settings.flatColor.x, settings.flatColor.y, settings.flatColor.z};
    fieldLabel("Colour");
    if (ImGui::ColorEdit3("##flatcolour", c))
      settings.flatColor = Vec3f{c[0], c[1], c[2]};
  }

  if (settings.colorMode == ColorMode::Scalar && cloud != nullptr) {
    const std::vector<int> colorable = cloud->colorableFieldIndices();

    // Adopt whatever the renderer already has loaded, rather than guessing.
    // Without this the panel would claim to be colouring by the first field
    // while the viewport showed a different one entirely.
    if (scalarField_ < 0 && input.scalarField >= 0) {
      scalarField_ = input.scalarField;
      scalarLo_ = input.scalarLo;
      scalarHi_ = input.scalarHi;
    }
    if (scalarField_ < 0 && !colorable.empty()) {
      scalarField_ = colorable.front();
      const FieldInfo& f = cloud->fields[static_cast<std::size_t>(scalarField_)];
      scalarLo_ = f.stats.min;
      scalarHi_ = f.stats.max;
    }

    const char* current = (scalarField_ >= 0 &&
                           scalarField_ < static_cast<int>(cloud->fields.size()))
                              ? cloud->fields[static_cast<std::size_t>(scalarField_)].name.c_str()
                              : "-";
    fieldLabel("Field");
    if (ImGui::BeginCombo("##field", current)) {
      for (int index : colorable) {
        const FieldInfo& f = cloud->fields[static_cast<std::size_t>(index)];
        if (ImGui::Selectable(f.name.c_str(), index == scalarField_)) {
          scalarField_ = index;
          // Reset the range to the new field's full extent; carrying the
          // previous one over would usually map everything to one colour.
          scalarLo_ = f.stats.min;
          scalarHi_ = f.stats.max;
          actions.scalarFieldChanged = true;
        }
        if (ImGui::IsItemHovered())
          ImGui::SetTooltip("%s  %s\nrange %.4g .. %.4g", fieldTypeName(f.type),
                            fieldRoleName(f.role), f.stats.min, f.stats.max);
      }
      ImGui::EndCombo();
    }

    int ramp = static_cast<int>(settings.ramp);
    const char* rampNames[] = {"Viridis", "Turbo", "Grayscale", "Cool-Warm", "Plasma", "Spectral"};
    fieldLabel("Ramp");
    if (ImGui::BeginCombo("##ramp_combo", rampNames[ramp])) {
      for (int i = 0; i < 6; ++i)
        if (ImGui::Selectable(rampNames[i], ramp == i)) settings.ramp = static_cast<ColorRamp>(i);
      ImGui::EndCombo();
    }
    rampPreview(static_cast<int>(settings.ramp), 14.0f);
    ImGui::Spacing();

    if (scalarField_ >= 0 && scalarField_ < static_cast<int>(cloud->fields.size())) {
      const FieldInfo& f = cloud->fields[static_cast<std::size_t>(scalarField_)];
      ImGui::TextDisabled("drag the edges of the histogram to set the range");
      if (histogramWidget("##scalarhist", f.stats.histogram, f.stats.min, f.stats.max, scalarLo_,
                          scalarHi_, static_cast<int>(settings.ramp), true, 70.0f))
        scalarRangeDirty_ = true;

      double lo = scalarLo_;
      double hi = scalarHi_;
      ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x * 0.48f);
      if (ImGui::InputDouble("##lo", &lo, 0.0, 0.0, "%.4g")) {
        scalarLo_ = std::min(lo, scalarHi_);
        scalarRangeDirty_ = true;
      }
      ImGui::SameLine();
      ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x);
      if (ImGui::InputDouble("##hi", &hi, 0.0, 0.0, "%.4g")) {
        scalarHi_ = std::max(hi, scalarLo_);
        scalarRangeDirty_ = true;
      }

      if (ImGui::Button("Full range")) {
        scalarLo_ = f.stats.min;
        scalarHi_ = f.stats.max;
        scalarRangeDirty_ = true;
      }
      ImGui::SameLine();
      if (ImGui::Button("2 sigma")) {
        // Clipping to two standard deviations is the usual fix for an
        // intensity channel whose range is set by a handful of outliers.
        scalarLo_ = std::max(f.stats.min, f.stats.mean - 2.0 * f.stats.stddev);
        scalarHi_ = std::min(f.stats.max, f.stats.mean + 2.0 * f.stats.stddev);
        scalarRangeDirty_ = true;
      }
      helpMarker(
          "Clamps the colour range to two standard deviations around the mean. "
          "A few extreme returns otherwise compress all the real variation into "
          "one or two colours.");

      fieldLabel("Gamma", "Biases the ramp towards the low or high end without changing the range.");
      ImGui::SliderFloat("##gamma", &settings.scalarGamma, 0.2f, 4.0f, "%.2f");
    }

    // Re-upload once the drag finishes rather than every frame: the scalar
    // buffer is tens of megabytes on a large cloud.
    if (scalarRangeDirty_ && !ImGui::IsAnyItemActive()) {
      scalarRangeDirty_ = false;
      actions.scalarFieldChanged = true;
    }
    if (actions.scalarFieldChanged) {
      actions.scalarField = scalarField_;
      actions.scalarLo = scalarLo_;
      actions.scalarHi = scalarHi_;
    }
  }

  // -------------------------------------------------------------- points
  ImGui::SeparatorText("Points");
  fieldLabel("Size", "Multiplies each chunk's measured point spacing. 1.0 draws splats "
                     "just large enough to close the gaps between neighbouring points.");
  ImGui::SliderFloat("##psize", &settings.pointScale, 0.1f, 8.0f, "%.2fx");
  fieldLabel("Max pixels");
  ImGui::SliderFloat("##pmax", &settings.maxPixels, 2.0f, 128.0f, "%.0f");
  ImGui::Checkbox("Round", &settings.roundPoints);
  ImGui::SameLine();
  ImGui::Checkbox("Shade", &settings.shadeSplats);
  helpMarker("Lights each splat as a small sphere, giving points volume at close range.");
  ImGui::SameLine();
  ImGui::Checkbox("Fixed size", &settings.fixedSize);
  helpMarker(
      "Ignores distance and draws every point the same number of pixels, the way "
      "pcl_viewer does. Useful for spotting sparse returns.");

  // --------------------------------------------------------------- EDL
  ImGui::SeparatorText("Eye-dome lighting");
  ImGui::Checkbox("Enabled##edl", &settings.edlEnabled);
  helpMarker(
      "Darkens pixels that sit behind their neighbours in depth, outlining every "
      "depth discontinuity. It is what makes an unlit cloud read as solid geometry "
      "rather than coloured confetti.");
  ImGui::BeginDisabled(!settings.edlEnabled);
  fieldLabel("Strength");
  ImGui::SliderFloat("##edlstr", &settings.edlStrength, 0.0f, 2.0f, "%.2f");
  fieldLabel("Radius");
  ImGui::SliderFloat("##edlrad", &settings.edlRadius, 0.5f, 5.0f, "%.2f px");
  ImGui::EndDisabled();

  // --------------------------------------------------------- background
  ImGui::SeparatorText("Scene");
  float bg[3] = {settings.background.x, settings.background.y, settings.background.z};
  fieldLabel("Background");
  if (ImGui::ColorEdit3("##bg", bg))
    settings.background = Vec3f{bg[0], bg[1], bg[2]};

  ImGui::Checkbox("Clip plane", &settings.clipEnabled);
  helpMarker("Hides everything below a horizontal plane. Useful for lifting a "
             "roof off, or dropping the ground away.");
  if (settings.clipEnabled && cloud != nullptr && cloud->boundsLocal.valid()) {
    const Aabbf& b = cloud->boundsLocal;
    // Expressed as a height, which is what you actually want on a Z-up map.
    float height = -settings.clipPlane.w;
    fieldLabel("Height");
    if (ImGui::SliderFloat("##cliph", &height, b.lo.z, b.hi.z, "%.2f m"))
      settings.clipPlane = Vec4f{0.0f, 0.0f, 1.0f, -height};
    ImGui::TextDisabled("world Z = %.2f", height + cloud->origin.z);
  }

  ImGui::Checkbox("Crop box", &settings.cropEnabled);
  helpMarker("Hides everything outside a box. Chunks wholly outside are rejected "
             "by the culling pass, so cropping makes rendering faster as well as "
             "clearer.");
  if (settings.cropEnabled && cloud != nullptr && cloud->boundsLocal.valid()) {
    const Aabbf& bounds = cloud->boundsLocal;
    // Start as the whole cloud, so enabling it changes nothing until moved.
    if (!settings.cropBox.valid()) settings.cropBox = bounds;

    static const char* axisNames[] = {"X", "Y", "Z"};
    for (int axis = 0; axis < 3; ++axis) {
      float lo = settings.cropBox.lo[axis];
      float hi = settings.cropBox.hi[axis];
      fieldLabel(axisNames[axis]);
      // One widget per axis with both ends, which is how a range reads.
      if (ImGui::DragFloatRange2(("##crop" + std::string(axisNames[axis])).c_str(), &lo, &hi,
                                 std::max(0.01f, bounds.size()[axis] * 0.002f), bounds.lo[axis],
                                 bounds.hi[axis], "%.2f", "%.2f",
                                 ImGuiSliderFlags_AlwaysClamp)) {
        settings.cropBox.lo[axis] = lo;
        settings.cropBox.hi[axis] = hi;
      }
    }

    if (ImGui::Button("Whole cloud")) settings.cropBox = bounds;
    ImGui::SameLine();
    ImGui::BeginDisabled(!selection_.valid || cloud == nullptr);
    if (ImGui::Button("Around selection")) {
      // A box of a fixed fraction of the scene around the picked point,
      // which is almost always what you want after picking something.
      const float reach = std::max(1.0f, maxComponent(bounds.size()) * 0.04f);
      const Vec3f centre{static_cast<float>(selection_.world.x - cloud->origin.x),
                         static_cast<float>(selection_.world.y - cloud->origin.y),
                         static_cast<float>(selection_.world.z - cloud->origin.z)};
      settings.cropBox.lo = centre - Vec3f(reach);
      settings.cropBox.hi = centre + Vec3f(reach);
    }
    ImGui::EndDisabled();

    const Vec3f size = settings.cropBox.size();
    ImGui::TextDisabled("%.1f x %.1f x %.1f m", size.x, size.y, size.z);
  }

  ImGui::End();
}

void Ui::drawInspectorPanel(const UiInput& input, UiActions& actions) {
  if (focusInspectorFrames_ > 0) {
    --focusInspectorFrames_;
    ImGui::SetNextWindowFocus();
  }
  if (!ImGui::Begin("Inspector", &showInspector_)) {
    ImGui::End();
    return;
  }
  const PointCloud* cloud = input.cloud;

  ImGui::Checkbox("Measure mode", &measureMode_);
  helpMarker("Each pick adds a point to a chain and reports the distances between them.");
  ImGui::SameLine();
  if (ImGui::Button("Clear")) {
    measurePoints_.clear();
    actions.clearMeasurement = true;
  }
  ImGui::SameLine();
  ImGui::TextDisabled("%zu picked", measurePoints_.size());
  ImGui::TextDisabled("shift-click a point to select it");
  ImGui::Separator();

  if (!selection_.valid) {
    ImGui::TextDisabled("Nothing selected.");
  } else if (cloud != nullptr && selection_.index < cloud->size()) {
    if (beginKeyValueTable("sel")) {
      labelValue("Source row", withThousands(selection_.sourceRow));
      helpMarker("The point's row in the original file. Chunking reorders points, "
                 "so this is what lets you cross-reference against the raw data.");
      labelValue("Array index", withThousands(selection_.index));
      if (selection_.chunkIndex >= 0)
        labelValue("Chunk", std::to_string(selection_.chunkIndex));
      labelValue("Distance", std::format("{:.3f} m from camera", selection_.cameraDistance));
      ImGui::EndTable();
    }

    ImGui::SeparatorText("Field values");
    if (ImGui::BeginTable("fieldvals", 2,
                          ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg |
                              ImGuiTableFlags_SizingStretchProp)) {
      ImGui::TableSetupColumn("field");
      ImGui::TableSetupColumn("value");
      ImGui::TableHeadersRow();

      for (const FieldInfo& f : cloud->fields) {
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::TextUnformatted(f.name.c_str());
        ImGui::TableSetColumnIndex(1);

        if (f.role == FieldRole::Position) {
          const int axis = f.name == "x" ? 0 : (f.name == "y" ? 1 : 2);
          // Full precision: this is the number you would compare against
          // another tool, so rounding it would defeat the purpose.
          ImGui::Text("%.6f", selection_.world[axis]);
        } else if (f.role == FieldRole::Color && cloud->hasColors()) {
          const std::uint32_t c = cloud->colors[selection_.index];
          const int r = static_cast<int>(c & 0xFFu);
          const int g = static_cast<int>((c >> 8) & 0xFFu);
          const int b = static_cast<int>((c >> 16) & 0xFFu);
          ImGui::ColorButton("##sw",
                             ImVec4(static_cast<float>(r) / 255.0f, static_cast<float>(g) / 255.0f,
                                    static_cast<float>(b) / 255.0f, 1.0f),
                             ImGuiColorEditFlags_NoTooltip, ImVec2(14, 14));
          ImGui::SameLine();
          ImGui::Text("#%02X%02X%02X   %d %d %d", r, g, b, r, g, b);
        } else if (f.columnIndex >= 0) {
          const ScalarColumn& col = cloud->columns[static_cast<std::size_t>(f.columnIndex)];
          const double v = col.at(selection_.index);
          // Integers print exactly; floats keep enough digits to be useful.
          if (fieldTypeIsFloat(f.type)) ImGui::Text("%.9g", v);
          else ImGui::Text("%lld", static_cast<long long>(v));
        } else {
          ImGui::TextDisabled("-");
        }
      }
      ImGui::EndTable();
    }

    if (ImGui::Button("Focus camera here")) actions.focusSelection = true;
  }

  if (measurePoints_.size() >= 2) {
    ImGui::SeparatorText("Measurements");
    double total = 0.0;
    for (std::size_t i = 1; i < measurePoints_.size(); ++i) {
      const Vec3 a = measurePoints_[i - 1].world;
      const Vec3 b = measurePoints_[i].world;
      const Vec3 d = b - a;
      const double dist = length(d);
      total += dist;
      ImGui::Text("%zu -> %zu   %.4f m", i - 1, i, dist);
      ImGui::TextDisabled("    dx %.4f   dy %.4f   dz %.4f", d.x, d.y, d.z);
      // Horizontal distance separately: on a Z-up map that is usually the
      // number you actually want.
      ImGui::TextDisabled("    horizontal %.4f   vertical %.4f",
                          std::sqrt(d.x * d.x + d.y * d.y), std::abs(d.z));
    }
    if (measurePoints_.size() > 2) ImGui::Text("total path  %.4f m", total);
  } else if (measureMode_) {
    ImGui::TextDisabled("%zu of 2 points picked", measurePoints_.size());
  }

  ImGui::End();
}

void Ui::drawPerformancePanel(const UiInput& input, RenderSettings& settings) {
  if (!ImGui::Begin("Performance", &showPerformance_)) {
    ImGui::End();
    return;
  }

  std::vector<float> history(frameHistory_.begin(), frameHistory_.end());
  float peak = 1.0f;
  for (float v : history) peak = std::max(peak, v);
  ImGui::PlotLines("##frames", history.data(), static_cast<int>(history.size()), 0, nullptr, 0.0f,
                   peak * 1.1f, ImVec2(-1, 60));
  ImGui::Text("%.2f ms   %.0f fps", input.frameMs,
              input.frameMs > 0.0 ? 1000.0 / input.frameMs : 0.0);

  const RenderStats* s = input.renderStats;
  if (s != nullptr) {
    ImGui::SeparatorText("GPU time");
    if (beginKeyValueTable("gputime")) {
      labelValue("Cull", std::format("{:.3f} ms", s->cullMs));
      labelValue("Draw", std::format("{:.3f} ms", s->drawMs));
      labelValue("Post (EDL)", std::format("{:.3f} ms", s->postMs));
      labelValue("Total", std::format("{:.3f} ms", s->cullMs + s->drawMs + s->postMs));
      ImGui::EndTable();
    }

    ImGui::SeparatorText("What was drawn");
    const double pointPct =
        s->totalPoints > 0
            ? 100.0 * static_cast<double>(s->drawnPoints) / static_cast<double>(s->totalPoints)
            : 0.0;
    const double chunkPct =
        s->totalChunks > 0
            ? 100.0 * static_cast<double>(s->visibleChunks) / static_cast<double>(s->totalChunks)
            : 0.0;

    ImGui::Text("points  %s of %s", withThousands(s->drawnPoints).c_str(),
                withThousands(s->totalPoints).c_str());
    ImGui::ProgressBar(static_cast<float>(pointPct / 100.0), ImVec2(-1, 0),
                       std::format("{:.1f}%", pointPct).c_str());
    ImGui::Text("chunks  %s of %s visible", withThousands(s->visibleChunks).c_str(),
                withThousands(s->totalChunks).c_str());
    ImGui::ProgressBar(static_cast<float>(chunkPct / 100.0), ImVec2(-1, 0),
                       std::format("{:.1f}%", chunkPct).c_str());

    ImGui::SeparatorText("Memory");
    if (beginKeyValueTable("mem")) {
      labelValue("GPU buffers", humanBytes(s->gpuBytes));
      if (input.freeVramMB > 0)
        labelValue("VRAM free", std::format("{} MB", input.freeVramMB));
      if (input.device != nullptr && input.device->totalVramMB > 0)
        labelValue("VRAM total", std::format("{} MB", input.device->totalVramMB));
      ImGui::EndTable();
    }
  }

  ImGui::SeparatorText("Level of detail");
  ImGui::Checkbox("Enabled##lod", &settings.lodEnabled);
  helpMarker(
      "Draws fewer points for chunks that are small on screen. Points were "
      "shuffled within each chunk at load, so any prefix of one is a uniform "
      "sample of the whole. Turn it off to see the full cloud and the cost.");
  ImGui::BeginDisabled(!settings.lodEnabled);
  fieldLabel("Density", "Target points per pixel. 1.0 is roughly one point per pixel.");
  ImGui::SliderFloat("##density", &settings.lodDensity, 0.1f, 8.0f, "%.2f pts/px");
  fieldLabel("Min/chunk");
  ImGui::SliderFloat("##minchunk", &settings.minChunkPoints, 0.0f, 512.0f, "%.0f");
  ImGui::EndDisabled();

  ImGui::End();
}

void Ui::drawStatusBar(const UiInput& input, const Camera& camera) {
  const ImGuiViewport* viewport = ImGui::GetMainViewport();
  const float height = ImGui::GetFrameHeight();
  ImGui::SetNextWindowPos(ImVec2(viewport->WorkPos.x, viewport->WorkPos.y + viewport->WorkSize.y - height));
  ImGui::SetNextWindowSize(ImVec2(viewport->WorkSize.x, height));
  ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
  ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);

  const ImGuiWindowFlags flags = ImGuiWindowFlags_NoDocking | ImGuiWindowFlags_NoTitleBar |
                                 ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
                                 ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoSavedSettings |
                                 ImGuiWindowFlags_NoNavFocus;
  if (ImGui::Begin("##statusbar", nullptr, flags)) {
    ImGui::Text("%s", navModeName(camera.mode));
    ImGui::SameLine();
    ImGui::TextDisabled("|");
    ImGui::SameLine();
    // Stated permanently, because an unexpected up axis is the first thing
    // that confuses anyone opening a cloud from another tool.
    ImGui::TextDisabled("Z up");
    ImGui::SameLine();
    ImGui::TextDisabled("|");
    ImGui::SameLine();

    const Vec3f eye = camera.position();
    const Vec3 origin = input.cloud != nullptr ? input.cloud->origin : Vec3{0, 0, 0};
    ImGui::Text("camera  %.1f  %.1f  %.1f", eye.x + origin.x, eye.y + origin.y, eye.z + origin.z);
    ImGui::SameLine();
    ImGui::TextDisabled("|");
    ImGui::SameLine();

    if (camera.mode == NavMode::TopDown)
      ImGui::Text("%.1f m across", camera.orthoHeight);
    else
      ImGui::Text("%.1f m out", camera.distance);

    if (input.device != nullptr) {
      const std::string gpu = input.device->renderer;
      const float w = ImGui::CalcTextSize(gpu.c_str()).x;
      ImGui::SameLine(ImGui::GetWindowWidth() - w - 14.0f);
      ImGui::TextDisabled("%s", gpu.c_str());
    }
  }
  ImGui::End();
  ImGui::PopStyleVar(2);
}

void Ui::drawFileBrowser(UiActions& actions) {
  ImGui::SetNextWindowSize(ImVec2(680, 460), ImGuiCond_FirstUseEver);
  if (!ImGui::Begin("Open point cloud", &showFileBrowser_)) {
    ImGui::End();
    return;
  }

  static char pathBuffer[1024] = {};
  if (pathBuffer[0] == '\0' && !browseDirectory_.empty())
    std::snprintf(pathBuffer, sizeof(pathBuffer), "%s", browseDirectory_.c_str());

  ImGui::SetNextItemWidth(-90.0f);
  if (ImGui::InputText("##path", pathBuffer, sizeof(pathBuffer),
                       ImGuiInputTextFlags_EnterReturnsTrue)) {
    std::error_code ec;
    if (std::filesystem::is_directory(pathBuffer, ec)) browseDirectory_ = pathBuffer;
    else if (std::filesystem::is_regular_file(pathBuffer, ec)) {
      actions.openPath = pathBuffer;
      showFileBrowser_ = false;
    }
  }
  ImGui::SameLine();
  if (ImGui::Button("Up")) {
    const std::filesystem::path parent = std::filesystem::path(browseDirectory_).parent_path();
    if (!parent.empty()) {
      browseDirectory_ = parent.string();
      std::snprintf(pathBuffer, sizeof(pathBuffer), "%s", browseDirectory_.c_str());
    }
  }

  ImGui::Separator();
  if (ImGui::BeginChild("##entries", ImVec2(0, -34))) {
    std::error_code ec;
    std::vector<std::filesystem::directory_entry> directories;
    std::vector<std::filesystem::directory_entry> files;
    for (const auto& entry : std::filesystem::directory_iterator(browseDirectory_, ec)) {
      const std::string name = entry.path().filename().string();
      if (!name.empty() && name[0] == '.') continue;  // skip hidden
      if (entry.is_directory(ec)) {
        directories.push_back(entry);
      } else {
        std::string ext = entry.path().extension().string();
        for (char& c : ext) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
        if (ext == ".pcd" || ext == ".ply" || ext == ".las" || ext == ".laz" || ext == ".bin")
          files.push_back(entry);
      }
    }
    auto byName = [](const auto& a, const auto& b) { return a.path().filename() < b.path().filename(); };
    std::sort(directories.begin(), directories.end(), byName);
    std::sort(files.begin(), files.end(), byName);

    for (const auto& dir : directories) {
      const std::string label = "[dir]  " + dir.path().filename().string();
      if (ImGui::Selectable(label.c_str())) {
        browseDirectory_ = dir.path().string();
        std::snprintf(pathBuffer, sizeof(pathBuffer), "%s", browseDirectory_.c_str());
      }
    }
    for (const auto& file : files) {
      std::error_code sizeEc;
      const auto size = std::filesystem::file_size(file.path(), sizeEc);
      const std::string label =
          file.path().filename().string() + "   " + (sizeEc ? "" : humanBytes(size));
      const bool selected = browseSelection_ == file.path().string();
      if (ImGui::Selectable(label.c_str(), selected,
                            ImGuiSelectableFlags_AllowDoubleClick)) {
        browseSelection_ = file.path().string();
        if (ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) {
          actions.openPath = browseSelection_;
          showFileBrowser_ = false;
        }
      }
    }
    if (directories.empty() && files.empty())
      ImGui::TextDisabled("no point clouds here");
  }
  ImGui::EndChild();

  ImGui::BeginDisabled(browseSelection_.empty());
  if (ImGui::Button("Open", ImVec2(110, 0))) {
    actions.openPath = browseSelection_;
    showFileBrowser_ = false;
  }
  ImGui::EndDisabled();
  ImGui::SameLine();
  if (ImGui::Button("Cancel", ImVec2(110, 0))) showFileBrowser_ = false;

  ImGui::End();
}

void Ui::drawHelpWindow() {
  ImGui::SetNextWindowSize(ImVec2(520, 440), ImGuiCond_FirstUseEver);
  if (!ImGui::Begin("Controls", &showHelp_)) {
    ImGui::End();
    return;
  }
  struct Row { const char* keys; const char* what; };
  static const Row navigation[] = {
      {"left drag", "orbit"},
      {"middle / right drag", "pan"},
      {"scroll", "zoom"},
      {"1 / 2 / 3", "orbit / fly / top-down"},
      {"W A S D", "fly forward, left, back, right"},
      {"Q E or ctrl/space", "fly down / up"},
      {"shift / alt while flying", "faster / slower"},
      {"F", "frame the whole cloud"},
      {"G", "focus on the selected point"},
      {"double click", "pick and focus in one gesture"},
  };
  static const Row display[] = {
      {"shift + click", "select the point under the cursor"},
      {"P", "select under the cursor, without the mouse"},
      {"C", "cycle colour source"},
      {"E", "toggle eye-dome lighting"},
      {"L", "toggle level of detail"},
      {"Tab", "show or hide all panels"},
      {"R", "reload shaders from disk"},
      {"Esc", "quit"},
  };
  auto table = [](const char* name, const Row* rows, std::size_t count) {
    ImGui::SeparatorText(name);
    if (ImGui::BeginTable(name, 2, ImGuiTableFlags_SizingStretchProp)) {
      for (std::size_t i = 0; i < count; ++i) {
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::TextDisabled("%s", rows[i].keys);
        ImGui::TableSetColumnIndex(1);
        ImGui::TextUnformatted(rows[i].what);
      }
      ImGui::EndTable();
    }
  };
  table("Navigation", navigation, std::size(navigation));
  table("Display", display, std::size(display));
  ImGui::End();
}

}  // namespace punto
