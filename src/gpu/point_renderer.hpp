// The GPU-driven point renderer.
//
// Per frame the CPU uploads a handful of uniforms and issues three calls:
// dispatch the cull compute shader, insert a barrier, then one
// MultiDrawArraysIndirectCount. Everything else -- which chunks are visible,
// how many points each contributes -- is decided on the GPU. That is why
// frame cost tracks screen area rather than cloud size.
#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "gpu/buffer.hpp"
#include "gpu/shader.hpp"
#include "scene/camera.hpp"
#include "scene/chunk.hpp"

namespace punto {

enum class ColorMode : int { Rgb = 0, Scalar = 1, Flat = 2, ChunkId = 3 };
enum class ColorRamp : int {
  Viridis = 0, Turbo = 1, Grayscale = 2, CoolWarm = 3, Plasma = 4, Spectral = 5
};

const char* colorModeName(ColorMode mode);
const char* colorRampName(ColorRamp ramp);

struct RenderSettings {
  ColorMode colorMode = ColorMode::Rgb;
  ColorRamp ramp = ColorRamp::Viridis;
  float scalarGamma = 1.0f;
  Vec3f flatColor{0.85f, 0.87f, 0.90f};
  Vec3f background{0.09f, 0.10f, 0.12f};

  // Splat radius multiplier applied to each chunk's mean point spacing.
  float pointScale = 1.0f;
  float minPixels = 1.0f;
  float maxPixels = 32.0f;
  bool roundPoints = true;
  bool shadeSplats = false;
  // Ignore distance and draw every point at pointScale pixels, the way
  // pcl_viewer does. Occasionally useful for counting sparse returns.
  bool fixedSize = false;

  bool lodEnabled = true;
  // Target points per pixel. 1.0 is roughly one point per pixel; higher
  // oversamples for a denser look at more cost.
  float lodDensity = 1.0f;
  float minChunkPoints = 32.0f;

  bool edlEnabled = true;
  float edlStrength = 0.6f;
  float edlRadius = 1.4f;
  bool showDepthOnly = false;

  bool clipEnabled = false;
  Vec4f clipPlane{0.0f, 0.0f, 1.0f, 0.0f};

  // Axis-aligned crop, in the cloud's local frame. Chunks wholly outside
  // are rejected by the cull pass, so cropping makes rendering faster as
  // well as clearer.
  bool cropEnabled = false;
  Aabbf cropBox;
};

struct RenderStats {
  std::uint32_t totalChunks = 0;
  std::uint32_t visibleChunks = 0;
  std::uint32_t culledChunks = 0;
  std::uint64_t drawnPoints = 0;
  std::uint64_t totalPoints = 0;

  double cullMs = 0.0;
  double drawMs = 0.0;
  double postMs = 0.0;

  std::uint64_t gpuBytes = 0;
};

class PointRenderer {
 public:
  PointRenderer();
  ~PointRenderer();
  PointRenderer(const PointRenderer&) = delete;
  PointRenderer& operator=(const PointRenderer&) = delete;

  bool initialize(std::string& error);

  // Uploads a cloud. Replaces whatever was resident. Returns false and sets
  // `error` if the data will not fit in available VRAM.
  bool setSource(std::shared_ptr<ChunkSource> source, std::string& error);
  void clearSource();
  bool hasSource() const { return source_ != nullptr; }
  const ChunkSource* source() const { return source_.get(); }

  // Re-uploads the scalar channel for a different field. `lo`/`hi` set the
  // range the ramp spans.
  void setScalarField(int fieldIndex, double lo, double hi);
  int scalarField() const { return scalarField_; }

  // Renders into a sub-rectangle of the default framebuffer, given in GL
  // coordinates with the origin bottom-left. The UI passes the dockspace's
  // central node, so the cloud is centred in the space actually left for
  // it rather than behind the docked panels.
  void render(const Camera& camera, int viewportX, int viewportY, int width, int height,
              const RenderSettings& settings);

  // Reads back the point under the cursor. `mouseX`/`mouseY` are relative
  // to the viewport rectangle, not the window. Returns false when the
  // cursor is over empty space; `pointIndex` indexes the cloud's arrays.
  bool pick(const Camera& camera, int width, int height, int mouseX, int mouseY,
            std::uint32_t& pointIndex, float& viewDistance);

  bool reloadShaders(std::string& error);

  const RenderStats& stats() const { return stats_; }
  // How many times the offscreen targets have been reallocated. Should
  // stay near-constant while a window is resized.
  std::uint64_t framebufferAllocations() const { return framebufferAllocations_; }
  bool supportsIndirectCount() const { return hasIndirectCount_; }

 private:
  bool ensureFramebuffer(int width, int height);
  void uploadChunkData();
  void runCull(const Camera& camera, int width, int height, const RenderSettings& settings);
  void bindPointBuffers() const;
  void readbackStats();

  std::shared_ptr<ChunkSource> source_;

  ShaderProgram cullProgram_;
  ShaderProgram pointProgram_;
  ShaderProgram pickProgram_;
  ShaderProgram edlProgram_;

  // Point data, GPU-resident.
  GlBuffer positions_;
  GlBuffer colors_;
  GlBuffer scalars_;
  GlBuffer chunkInfo_;
  GlBuffer lodScale_;
  GlBuffer drawCommands_;
  GlBuffer drawCount_;

  // Statistics readback ring.
  //
  // Reading drawCount_ directly each frame makes the driver migrate it out
  // of VRAM to host memory -- it warns about exactly this -- which then
  // slows every atomic the cull shader performs against it. Copying into a
  // persistently mapped buffer and reading it two frames later keeps the
  // counters device-local and never stalls on a fence.
  static constexpr int kStatsRing = 3;
  GlBuffer statsReadback_[kStatsRing];
  const std::uint32_t* statsMapped_[kStatsRing] = {nullptr, nullptr, nullptr};
  GLsync statsFence_[kStatsRing] = {nullptr, nullptr, nullptr};
  int statsSlot_ = 0;

  // Offscreen target: colour, linear eye depth for EDL, and depth.
  GLuint fbo_ = 0;
  GlTexture colorTex_;
  GlTexture viewDepthTex_;
  GlTexture depthTex_;
  // Allocated texture size, which is the viewport size rounded up to a
  // granularity. Reallocating on every pixel of change meant a maximise
  // animation -- or just dragging a panel edge -- destroyed and recreated
  // three large textures every frame. At 2560x1440 that is ~60 MB of
  // texture churn per frame, which is enough to wedge a driver that is
  // also copying each frame across PCIe.
  int fboWidth_ = 0;
  int fboHeight_ = 0;
  // The sub-rectangle of those textures actually rendered into.
  int viewportWidth_ = 0;
  int viewportHeight_ = 0;
  std::uint64_t framebufferAllocations_ = 0;

  GLuint pickFbo_ = 0;
  GlTexture pickTex_;
  GlTexture pickDepthTex_;
  int pickSize_ = 0;

  GLuint emptyVao_ = 0;

  // Timer queries, double-buffered so reading last frame's result never
  // stalls the pipeline waiting on this one.
  GLuint timerQueries_[2][3] = {{0, 0, 0}, {0, 0, 0}};
  int timerSlot_ = 0;
  bool timersPrimed_ = false;

  int scalarField_ = -1;
  double scalarLo_ = 0.0;
  double scalarHi_ = 1.0;

  // Mirrors of the last rendered frame's hiding state, so picking agrees
  // with what is on screen.
  bool lastClipEnabled_ = false;
  Vec4f lastClipPlane_{0.0f, 0.0f, 1.0f, 0.0f};
  bool lastCropEnabled_ = false;
  Aabbf lastCropBox_;

  bool hasIndirectCount_ = false;
  bool initialized_ = false;
  RenderStats stats_;
};

}  // namespace punto
