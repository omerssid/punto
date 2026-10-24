#include "gpu/point_renderer.hpp"

#include <algorithm>
#include <cstring>

#include "core/log.hpp"

namespace pcv {
namespace {

// Chunks per compute workgroup; must match local_size_x in cull.comp.
constexpr GLuint kCullGroupSize = 64;

// Picking renders a small square around the cursor rather than the whole
// view. Large enough to catch a point whose centre is a few pixels off.
constexpr int kPickRegion = 17;

std::uint64_t alignUp(std::uint64_t v, std::uint64_t a) { return (v + a - 1) / a * a; }

}  // namespace

const char* colorModeName(ColorMode mode) {
  switch (mode) {
    case ColorMode::Rgb:     return "RGB";
    case ColorMode::Scalar:  return "Scalar field";
    case ColorMode::Flat:    return "Flat";
    case ColorMode::ChunkId: return "Chunk (debug)";
  }
  return "?";
}

const char* colorRampName(ColorRamp ramp) {
  switch (ramp) {
    case ColorRamp::Viridis:   return "Viridis";
    case ColorRamp::Turbo:     return "Turbo";
    case ColorRamp::Grayscale: return "Grayscale";
    case ColorRamp::CoolWarm:  return "Cool-Warm";
    case ColorRamp::Plasma:    return "Plasma";
    case ColorRamp::Spectral:  return "Spectral";
  }
  return "?";
}

PointRenderer::PointRenderer() = default;

PointRenderer::~PointRenderer() {
  if (fbo_ != 0) glDeleteFramebuffers(1, &fbo_);
  if (pickFbo_ != 0) glDeleteFramebuffers(1, &pickFbo_);
  if (emptyVao_ != 0) glDeleteVertexArrays(1, &emptyVao_);
  for (auto& slot : timerQueries_)
    for (GLuint q : slot)
      if (q != 0) glDeleteQueries(1, &q);
  for (GLsync fence : statsFence_)
    if (fence != nullptr) glDeleteSync(fence);
}

bool PointRenderer::initialize(std::string& error) {
  if (!cullProgram_.buildCompute("cull.comp", {}, error)) return false;
  if (!pointProgram_.buildGraphics("points.vert", "points.frag", {}, error)) return false;
  if (!pickProgram_.buildGraphics("points.vert", "pick.frag", {}, error)) return false;
  if (!edlProgram_.buildGraphics("fullscreen.vert", "edl.frag", {}, error)) return false;

  // Core profile requires a bound VAO even when every attribute comes from
  // an SSBO, so one empty object is kept around for every draw.
  glCreateVertexArrays(1, &emptyVao_);

  for (auto& slot : timerQueries_)
    for (GLuint& q : slot) glGenQueries(1, &q);

  for (int i = 0; i < kStatsRing; ++i)
    statsMapped_[i] =
        static_cast<const std::uint32_t*>(statsReadback_[i].allocatePersistentRead(4 * sizeof(std::uint32_t)));

  hasIndirectCount_ = (GLEW_VERSION_4_6 != 0) || (GLEW_ARB_indirect_parameters != 0);
  if (!hasIndirectCount_)
    logInfo("no indirect-parameters support; falling back to zero-count draw commands");

  // Reversed-Z: depth runs 1 at the near plane to 0 at infinity, so the
  // buffer is cleared to 0 and the test keeps greater values. Paired with a
  // float depth buffer this removes z-fighting across the huge depth ranges
  // an HD map spans.
  glClipControl(GL_LOWER_LEFT, GL_ZERO_TO_ONE);
  glClearDepth(0.0);
  glDepthFunc(GL_GREATER);

  // Lets the vertex shader set gl_PointSize.
  glEnable(GL_PROGRAM_POINT_SIZE);

  initialized_ = true;
  return true;
}

bool PointRenderer::reloadShaders(std::string& error) {
  std::string firstError;
  bool ok = true;
  auto tryReload = [&](ShaderProgram& p, const char* label) {
    std::string e;
    if (!p.reload(e)) {
      ok = false;
      if (firstError.empty()) firstError = std::string(label) + ": " + e;
    }
  };
  tryReload(cullProgram_, "cull");
  tryReload(pointProgram_, "points");
  tryReload(pickProgram_, "pick");
  tryReload(edlProgram_, "edl");
  if (!ok) error = firstError;
  return ok;
}

void PointRenderer::clearSource() {
  source_.reset();
  positions_.destroy();
  colors_.destroy();
  scalars_.destroy();
  chunkInfo_.destroy();
  lodScale_.destroy();
  drawCommands_.destroy();
  drawCount_.destroy();
  stats_ = RenderStats{};
  scalarField_ = -1;
}

bool PointRenderer::setSource(std::shared_ptr<ChunkSource> source, std::string& error) {
  clearSource();
  if (source == nullptr || source->chunkCount() == 0) return true;

  source_ = std::move(source);
  const std::uint64_t pointCount = source_->totalPoints();
  const std::size_t chunkCount = source_->chunkCount();

  const std::uint64_t positionBytes = pointCount * sizeof(QuantizedPoint);
  // Two 16-bit scalars share a 32-bit word, so round the count up to even.
  const std::uint64_t scalarBytes = alignUp(pointCount, 2) * sizeof(std::uint16_t);
  const std::uint64_t colorBytes = pointCount * sizeof(std::uint32_t);
  const std::uint64_t chunkBytes = chunkCount * sizeof(GpuChunk);
  const std::uint64_t commandBytes = chunkCount * sizeof(DrawArraysIndirectCommand);
  const std::uint64_t lodBytes = chunkCount * sizeof(float);
  const std::uint64_t total =
      positionBytes + scalarBytes + colorBytes + chunkBytes + commandBytes + lodBytes;

  const int freeMB = queryFreeVramMB();
  const std::uint64_t neededMB = total / (1024 * 1024);
  if (freeMB > 0 && neededMB + 128 > static_cast<std::uint64_t>(freeMB)) {
    error = "cloud needs " + std::to_string(neededMB) + " MB of VRAM but only " +
            std::to_string(freeMB) + " MB is free";
    source_.reset();
    return false;
  }

  positions_.allocate(positionBytes, GL_DYNAMIC_STORAGE_BIT);
  scalars_.allocate(scalarBytes, GL_DYNAMIC_STORAGE_BIT);
  chunkInfo_.allocate(chunkBytes, GL_DYNAMIC_STORAGE_BIT);
  lodScale_.allocate(lodBytes);
  drawCommands_.allocate(commandBytes);
  // Four counters: draw count (also the indirect parameter), drawn points,
  // visible chunks, culled chunks.
  drawCount_.allocate(4 * sizeof(std::uint32_t), GL_DYNAMIC_STORAGE_BIT);

  // The colour buffer is allocated even for clouds with no colour, because
  // the shader binds it unconditionally and an unbound SSBO is undefined.
  colors_.allocate(std::max<std::uint64_t>(colorBytes, sizeof(std::uint32_t)),
                   GL_DYNAMIC_STORAGE_BIT);

  stats_.gpuBytes = total;
  stats_.totalChunks = static_cast<std::uint32_t>(chunkCount);
  stats_.totalPoints = pointCount;

  uploadChunkData();
  return true;
}

void PointRenderer::uploadChunkData() {
  const auto chunks = source_->chunks();

  std::vector<GpuChunk> gpuChunks(chunks.size());
  for (std::size_t i = 0; i < chunks.size(); ++i) {
    const Chunk& c = chunks[i];
    GpuChunk& g = gpuChunks[i];
    g.boundsMin[0] = c.bounds.lo.x;
    g.boundsMin[1] = c.bounds.lo.y;
    g.boundsMin[2] = c.bounds.lo.z;
    g.boundsMin[3] = c.spacing;  // the shader reads spacing from here
    g.boundsMax[0] = c.bounds.hi.x;
    g.boundsMax[1] = c.bounds.hi.y;
    g.boundsMax[2] = c.bounds.hi.z;
    g.boundsMax[3] = 0.0f;
    g.firstPoint = c.firstPoint;
    g.pointCount = c.pointCount;
    g.pad0 = 0;
    g.pad1 = 0;
  }
  chunkInfo_.upload(0, gpuChunks.size() * sizeof(GpuChunk), gpuChunks.data());

  // Stage per chunk rather than building one buffer the size of the whole
  // cloud: on a 32M-point cloud that would be another 250 MB of host memory
  // for no benefit.
  std::vector<QuantizedPoint> positionStaging;
  std::vector<std::uint32_t> colorStaging;
  const bool hasColors = source_->readChunkColors(0, {});

  for (std::size_t i = 0; i < chunks.size(); ++i) {
    const Chunk& c = chunks[i];
    positionStaging.resize(c.pointCount);
    source_->readChunkPositions(i, positionStaging);
    positions_.upload(static_cast<std::size_t>(c.firstPoint) * sizeof(QuantizedPoint),
                      c.pointCount * sizeof(QuantizedPoint), positionStaging.data());

    if (hasColors) {
      colorStaging.resize(c.pointCount);
      if (source_->readChunkColors(i, colorStaging))
        colors_.upload(static_cast<std::size_t>(c.firstPoint) * sizeof(std::uint32_t),
                       c.pointCount * sizeof(std::uint32_t), colorStaging.data());
    }
  }
  logInfo("uploaded {} points across {} chunks ({} MB)", source_->totalPoints(), chunks.size(),
          stats_.gpuBytes / (1024 * 1024));
}

void PointRenderer::setScalarField(int fieldIndex, double lo, double hi) {
  if (source_ == nullptr) return;
  scalarField_ = fieldIndex;
  scalarLo_ = lo;
  scalarHi_ = hi;
  if (fieldIndex < 0) return;

  const auto chunks = source_->chunks();
  std::vector<std::uint16_t> staging;
  for (std::size_t i = 0; i < chunks.size(); ++i) {
    const Chunk& c = chunks[i];
    staging.resize(c.pointCount);
    if (!source_->readChunkScalars(i, fieldIndex, lo, hi, staging)) {
      scalarField_ = -1;
      return;
    }
    scalars_.upload(static_cast<std::size_t>(c.firstPoint) * sizeof(std::uint16_t),
                    c.pointCount * sizeof(std::uint16_t), staging.data());
  }
}

bool PointRenderer::ensureFramebuffer(int width, int height) {
  if (fbo_ != 0 && width == fboWidth_ && height == fboHeight_) return true;

  if (fbo_ == 0) glCreateFramebuffers(1, &fbo_);

  colorTex_.create2D(width, height, GL_RGBA8);
  // Linear eye distance for EDL. R32F because the range spans centimetres
  // to kilometres and half-float would band badly at the far end.
  viewDepthTex_.create2D(width, height, GL_R32F);
  // Float depth is what makes reversed-Z worthwhile; a fixed-point buffer
  // would throw away the precision the projection just bought.
  depthTex_.create2D(width, height, GL_DEPTH_COMPONENT32F);

  glNamedFramebufferTexture(fbo_, GL_COLOR_ATTACHMENT0, colorTex_.id(), 0);
  glNamedFramebufferTexture(fbo_, GL_COLOR_ATTACHMENT1, viewDepthTex_.id(), 0);
  glNamedFramebufferTexture(fbo_, GL_DEPTH_ATTACHMENT, depthTex_.id(), 0);

  const GLenum buffers[2] = {GL_COLOR_ATTACHMENT0, GL_COLOR_ATTACHMENT1};
  glNamedFramebufferDrawBuffers(fbo_, 2, buffers);

  const GLenum status = glCheckNamedFramebufferStatus(fbo_, GL_FRAMEBUFFER);
  if (status != GL_FRAMEBUFFER_COMPLETE) {
    logError("framebuffer incomplete: 0x{:x}", static_cast<unsigned>(status));
    return false;
  }
  fboWidth_ = width;
  fboHeight_ = height;
  return true;
}

void PointRenderer::bindPointBuffers() const {
  chunkInfo_.bindBase(GL_SHADER_STORAGE_BUFFER, 0);
  drawCommands_.bindBase(GL_SHADER_STORAGE_BUFFER, 1);
  drawCount_.bindBase(GL_SHADER_STORAGE_BUFFER, 2);
  lodScale_.bindBase(GL_SHADER_STORAGE_BUFFER, 3);
  positions_.bindBase(GL_SHADER_STORAGE_BUFFER, 4);
  colors_.bindBase(GL_SHADER_STORAGE_BUFFER, 5);
  scalars_.bindBase(GL_SHADER_STORAGE_BUFFER, 6);
}

void PointRenderer::runCull(const Camera& camera, int width, int height,
                            const RenderSettings& settings) {
  const float aspect = static_cast<float>(width) / static_cast<float>(std::max(1, height));
  const Frustum frustum = camera.frustum(aspect);
  const Vec3f eye = camera.position();

  // Counters must start at zero: the cull shader appends with atomicAdd.
  drawCount_.clearToZero();
  // Stale commands past this frame's count would otherwise be re-executed
  // by the non-Count fallback path.
  if (!hasIndirectCount_) drawCommands_.clearToZero();
  glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT | GL_BUFFER_UPDATE_BARRIER_BIT);

  cullProgram_.use();
  const GLint planeLoc = glGetUniformLocation(cullProgram_.id(), "uFrustumPlanes");
  if (planeLoc >= 0) glUniform4fv(planeLoc, 6, &frustum.planes[0].x);

  cullProgram_.set("uCameraPos", eye);
  cullProgram_.set("uChunkCount", static_cast<unsigned>(source_->chunkCount()));
  cullProgram_.set("uProjScale", camera.projectionScale(static_cast<float>(height)));
  cullProgram_.set("uDensity", settings.lodDensity);
  cullProgram_.set("uMinChunkPoints", settings.minChunkPoints);
  cullProgram_.set("uLodEnabled", settings.lodEnabled ? 1 : 0);
  cullProgram_.set("uOrthographic", camera.mode == NavMode::TopDown ? 1 : 0);
  cullProgram_.set("uOrthoPixelsPerMetre", camera.orthoPixelsPerMetre(static_cast<float>(height)));
  cullProgram_.set("uClipEnabled", settings.clipEnabled ? 1 : 0);
  cullProgram_.set("uClipPlane", settings.clipPlane.x, settings.clipPlane.y, settings.clipPlane.z,
                   settings.clipPlane.w);
  const bool cropActive = settings.cropEnabled && settings.cropBox.valid();
  cullProgram_.set("uCropEnabled", cropActive ? 1 : 0);
  if (cropActive) {
    cullProgram_.set("uCropMin", settings.cropBox.lo);
    cullProgram_.set("uCropMax", settings.cropBox.hi);
  }

  const GLuint groups =
      (static_cast<GLuint>(source_->chunkCount()) + kCullGroupSize - 1) / kCullGroupSize;
  glDispatchCompute(groups, 1, 1);

  // The results are consumed three different ways, so all three barriers
  // are needed: as an indirect command source, as SSBO reads in the vertex
  // shader, and as a parameter buffer.
  glMemoryBarrier(GL_COMMAND_BARRIER_BIT | GL_SHADER_STORAGE_BARRIER_BIT |
                  GL_BUFFER_UPDATE_BARRIER_BIT);
}

void PointRenderer::render(const Camera& camera, int viewportX, int viewportY, int width,
                           int height, const RenderSettings& settings) {
  if (!initialized_ || width <= 0 || height <= 0) return;

  // Paint the whole window first. Only the viewport rectangle is rendered
  // into, and the rest would otherwise keep whatever the previous frame
  // left wherever a panel does not cover it.
  glBindFramebuffer(GL_FRAMEBUFFER, 0);
  glDisable(GL_SCISSOR_TEST);
  glClearColor(settings.background.x, settings.background.y, settings.background.z, 1.0f);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  if (source_ == nullptr || source_->chunkCount() == 0) return;
  if (!ensureFramebuffer(width, height)) return;

  const float aspect = static_cast<float>(width) / static_cast<float>(std::max(1, height));
  GLuint* queries = timerQueries_[timerSlot_];

  bindPointBuffers();

  glBeginQuery(GL_TIME_ELAPSED, queries[0]);
  runCull(camera, width, height, settings);
  glEndQuery(GL_TIME_ELAPSED);

  // ---------------------------------------------------------- point pass
  glBeginQuery(GL_TIME_ELAPSED, queries[1]);
  glBindFramebuffer(GL_FRAMEBUFFER, fbo_);
  // The offscreen target is exactly the viewport's size, so it starts at
  // its own origin; only the final composite is offset into the window.
  glViewport(0, 0, width, height);

  const float clearColor[4] = {settings.background.x, settings.background.y,
                               settings.background.z, 1.0f};
  glClearNamedFramebufferfv(fbo_, GL_COLOR, 0, clearColor);
  // Zero marks "no geometry" for the EDL pass.
  const float clearDepthTarget[4] = {0.0f, 0.0f, 0.0f, 0.0f};
  glClearNamedFramebufferfv(fbo_, GL_COLOR, 1, clearDepthTarget);
  const float zero = 0.0f;
  glClearNamedFramebufferfv(fbo_, GL_DEPTH, 0, &zero);

  glEnable(GL_DEPTH_TEST);
  glDisable(GL_BLEND);

  pointProgram_.use();
  pointProgram_.set("uViewProj", camera.viewProjection(aspect));
  pointProgram_.set("uCameraPos", camera.position());
  pointProgram_.set("uProjScale", camera.projectionScale(static_cast<float>(height)));
  pointProgram_.set("uPointScale", settings.pointScale);
  pointProgram_.set("uMinPixels", settings.minPixels);
  pointProgram_.set("uMaxPixels", settings.maxPixels);
  pointProgram_.set("uColorMode", static_cast<int>(settings.colorMode));
  pointProgram_.set("uRamp", static_cast<int>(settings.ramp));
  pointProgram_.set("uHasColors", source_->readChunkColors(0, {}) ? 1 : 0);
  pointProgram_.set("uFlatColor", settings.flatColor);
  pointProgram_.set("uOrthographic", camera.mode == NavMode::TopDown ? 1 : 0);
  pointProgram_.set("uOrthoPixelsPerMetre",
                    camera.orthoPixelsPerMetre(static_cast<float>(height)));
  pointProgram_.set("uFixedSize", settings.fixedSize ? 1 : 0);
  pointProgram_.set("uRoundPoints", settings.roundPoints ? 1 : 0);
  pointProgram_.set("uShadeSplats", settings.shadeSplats ? 1 : 0);
  pointProgram_.set("uLightDir", 0.4f, 0.3f, 0.86f);
  pointProgram_.set("uScalarGamma", settings.scalarGamma);
  pointProgram_.set("uClipEnabled", settings.clipEnabled ? 1 : 0);
  pointProgram_.set("uClipPlane", settings.clipPlane.x, settings.clipPlane.y, settings.clipPlane.z,
                    settings.clipPlane.w);
  const bool drawCrop = settings.cropEnabled && settings.cropBox.valid();
  pointProgram_.set("uCropEnabled", drawCrop ? 1 : 0);
  if (drawCrop) {
    pointProgram_.set("uCropMin", settings.cropBox.lo);
    pointProgram_.set("uCropMax", settings.cropBox.hi);
  }

  glBindVertexArray(emptyVao_);
  drawCommands_.bind(GL_DRAW_INDIRECT_BUFFER);

  const GLsizei maxDraws = static_cast<GLsizei>(source_->chunkCount());
  if (hasIndirectCount_) {
    glBindBuffer(GL_PARAMETER_BUFFER, drawCount_.id());
    // The count lives at offset 0 of drawCount_, written by the cull pass.
    glMultiDrawArraysIndirectCount(GL_POINTS, nullptr, 0, maxDraws,
                                   sizeof(DrawArraysIndirectCommand));
    glBindBuffer(GL_PARAMETER_BUFFER, 0);
  } else {
    // Commands beyond the live count were cleared to zero, so they draw
    // nothing.
    glMultiDrawArraysIndirect(GL_POINTS, nullptr, maxDraws,
                              sizeof(DrawArraysIndirectCommand));
  }
  glEndQuery(GL_TIME_ELAPSED);

  // ------------------------------------------------------------ EDL pass
  glBeginQuery(GL_TIME_ELAPSED, queries[2]);
  glBindFramebuffer(GL_FRAMEBUFFER, 0);
  glViewport(viewportX, viewportY, width, height);
  glDisable(GL_DEPTH_TEST);

  edlProgram_.use();
  colorTex_.bindUnit(0);
  viewDepthTex_.bindUnit(1);
  edlProgram_.set("uColor", 0);
  edlProgram_.set("uViewDepth", 1);
  edlProgram_.set("uTexelSize", 1.0f / static_cast<float>(width),
                  1.0f / static_cast<float>(height));
  edlProgram_.set("uStrength", settings.edlStrength);
  edlProgram_.set("uRadius", settings.edlRadius);
  edlProgram_.set("uBackground", settings.background);
  edlProgram_.set("uEnabled", settings.edlEnabled ? 1 : 0);
  edlProgram_.set("uShowDepthOnly", settings.showDepthOnly ? 1 : 0);

  glBindVertexArray(emptyVao_);
  glDrawArrays(GL_TRIANGLES, 0, 3);
  glEndQuery(GL_TIME_ELAPSED);

  // Remembered so a pick, which runs its own cull, hides what the last
  // visible frame hid.
  lastClipEnabled_ = settings.clipEnabled;
  lastClipPlane_ = settings.clipPlane;
  lastCropEnabled_ = settings.cropEnabled;
  lastCropBox_ = settings.cropBox;

  glBindVertexArray(0);
  readbackStats();
  timerSlot_ ^= 1;
  timersPrimed_ = true;
}

void PointRenderer::readbackStats() {
  // Queue this frame's counters for reading later.
  statsReadback_[statsSlot_].copyFrom(drawCount_, 4 * sizeof(std::uint32_t));
  if (statsFence_[statsSlot_] != nullptr) glDeleteSync(statsFence_[statsSlot_]);
  statsFence_[statsSlot_] = glFenceSync(GL_SYNC_GPU_COMMANDS_COMPLETE, 0);

  // Read the oldest slot, which by now is two frames behind and certain to
  // have landed. The fence is checked rather than waited on, so a slow
  // frame costs stale numbers instead of a pipeline stall.
  const int readSlot = (statsSlot_ + 1) % kStatsRing;
  statsSlot_ = readSlot;
  if (statsFence_[readSlot] != nullptr && statsMapped_[readSlot] != nullptr) {
    const GLenum status = glClientWaitSync(statsFence_[readSlot], 0, 0);
    if (status == GL_ALREADY_SIGNALED || status == GL_CONDITION_SATISFIED) {
      const std::uint32_t* counters = statsMapped_[readSlot];
      stats_.drawnPoints = counters[1];
      stats_.visibleChunks = counters[2];
      stats_.culledChunks = counters[3];
    }
  }

  // Read the *other* slot: this frame's queries have not resolved yet, and
  // waiting on them would serialise the CPU against the GPU.
  if (!timersPrimed_) return;
  const GLuint* previous = timerQueries_[timerSlot_ ^ 1];
  GLint available = 0;
  glGetQueryObjectiv(previous[2], GL_QUERY_RESULT_AVAILABLE, &available);
  if (available == 0) return;

  GLuint64 ns = 0;
  glGetQueryObjectui64v(previous[0], GL_QUERY_RESULT, &ns);
  stats_.cullMs = static_cast<double>(ns) * 1e-6;
  glGetQueryObjectui64v(previous[1], GL_QUERY_RESULT, &ns);
  stats_.drawMs = static_cast<double>(ns) * 1e-6;
  glGetQueryObjectui64v(previous[2], GL_QUERY_RESULT, &ns);
  stats_.postMs = static_cast<double>(ns) * 1e-6;
}

bool PointRenderer::pick(const Camera& camera, int width, int height, int mouseX, int mouseY,
                         std::uint32_t& pointIndex, float& viewDistance) {
  if (source_ == nullptr || !initialized_) return false;

  if (pickFbo_ == 0) {
    glCreateFramebuffers(1, &pickFbo_);
    pickSize_ = kPickRegion;
    // RG32UI: the point index plus its eye distance as raw float bits.
    pickTex_.create2D(pickSize_, pickSize_, GL_RG32UI);
    pickDepthTex_.create2D(pickSize_, pickSize_, GL_DEPTH_COMPONENT32F);
    glNamedFramebufferTexture(pickFbo_, GL_COLOR_ATTACHMENT0, pickTex_.id(), 0);
    glNamedFramebufferTexture(pickFbo_, GL_DEPTH_ATTACHMENT, pickDepthTex_.id(), 0);
    const GLenum buffers[1] = {GL_COLOR_ATTACHMENT0};
    glNamedFramebufferDrawBuffers(pickFbo_, 1, buffers);
    if (glCheckNamedFramebufferStatus(pickFbo_, GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
      logError("pick framebuffer incomplete");
      return false;
    }
  }

  const float aspect = static_cast<float>(width) / static_cast<float>(std::max(1, height));
  // Crop and clip carry over, so a hidden point cannot be picked.
  RenderSettings pickSettings;
  pickSettings.clipEnabled = lastClipEnabled_;
  pickSettings.clipPlane = lastClipPlane_;
  pickSettings.cropEnabled = lastCropEnabled_;
  pickSettings.cropBox = lastCropBox_;
  bindPointBuffers();
  runCull(camera, width, height, pickSettings);

  glBindFramebuffer(GL_FRAMEBUFFER, pickFbo_);
  // Render the full-size view but shifted so the cursor's neighbourhood
  // lands in this small target. A negative viewport origin is legal and is
  // what makes a cheap sub-region pick possible without a scissor pass.
  glViewport(-(mouseX - pickSize_ / 2), -(mouseY - pickSize_ / 2), width, height);

  const GLuint clearPick[4] = {0, 0, 0, 0};
  glClearNamedFramebufferuiv(pickFbo_, GL_COLOR, 0, clearPick);
  const float zero = 0.0f;
  glClearNamedFramebufferfv(pickFbo_, GL_DEPTH, 0, &zero);

  glEnable(GL_DEPTH_TEST);
  pickProgram_.use();
  pickProgram_.set("uViewProj", camera.viewProjection(aspect));
  pickProgram_.set("uCameraPos", camera.position());
  pickProgram_.set("uProjScale", camera.projectionScale(static_cast<float>(height)));
  pickProgram_.set("uPointScale", pickSettings.pointScale);
  pickProgram_.set("uMinPixels", pickSettings.minPixels);
  pickProgram_.set("uMaxPixels", pickSettings.maxPixels);
  pickProgram_.set("uColorMode", static_cast<int>(ColorMode::Flat));
  pickProgram_.set("uHasColors", 0);
  pickProgram_.set("uOrthographic", camera.mode == NavMode::TopDown ? 1 : 0);
  pickProgram_.set("uOrthoPixelsPerMetre",
                   camera.orthoPixelsPerMetre(static_cast<float>(height)));
  pickProgram_.set("uFixedSize", 0);
  pickProgram_.set("uRoundPoints", 1);
  pickProgram_.set("uClipEnabled", pickSettings.clipEnabled ? 1 : 0);
  pickProgram_.set("uClipPlane", pickSettings.clipPlane.x, pickSettings.clipPlane.y,
                   pickSettings.clipPlane.z, pickSettings.clipPlane.w);
  const bool pickCrop = pickSettings.cropEnabled && pickSettings.cropBox.valid();
  pickProgram_.set("uCropEnabled", pickCrop ? 1 : 0);
  if (pickCrop) {
    pickProgram_.set("uCropMin", pickSettings.cropBox.lo);
    pickProgram_.set("uCropMax", pickSettings.cropBox.hi);
  }
  pickProgram_.set("uScalarGamma", 1.0f);

  glBindVertexArray(emptyVao_);
  drawCommands_.bind(GL_DRAW_INDIRECT_BUFFER);
  const GLsizei maxDraws = static_cast<GLsizei>(source_->chunkCount());
  if (hasIndirectCount_) {
    glBindBuffer(GL_PARAMETER_BUFFER, drawCount_.id());
    glMultiDrawArraysIndirectCount(GL_POINTS, nullptr, 0, maxDraws,
                                   sizeof(DrawArraysIndirectCommand));
    glBindBuffer(GL_PARAMETER_BUFFER, 0);
  } else {
    glMultiDrawArraysIndirect(GL_POINTS, nullptr, maxDraws, sizeof(DrawArraysIndirectCommand));
  }

  std::vector<std::uint32_t> pixels(static_cast<std::size_t>(pickSize_ * pickSize_) * 2, 0u);
  glBindFramebuffer(GL_READ_FRAMEBUFFER, pickFbo_);
  glReadBuffer(GL_COLOR_ATTACHMENT0);
  glReadPixels(0, 0, pickSize_, pickSize_, GL_RG_INTEGER, GL_UNSIGNED_INT, pixels.data());
  glBindFramebuffer(GL_FRAMEBUFFER, 0);
  glBindVertexArray(0);

  // Search outwards from the centre so the nearest hit to the cursor wins
  // rather than whichever pixel happens to come first in memory.
  const int centre = pickSize_ / 2;
  int bestRadius = pickSize_ * pickSize_;
  bool found = false;
  for (int y = 0; y < pickSize_; ++y) {
    for (int x = 0; x < pickSize_; ++x) {
      const std::size_t i = (static_cast<std::size_t>(y) * pickSize_ + x) * 2;
      if (pixels[i] == 0) continue;
      const int dx = x - centre;
      const int dy = y - centre;
      const int r2 = dx * dx + dy * dy;
      if (r2 < bestRadius) {
        bestRadius = r2;
        pointIndex = pixels[i] - 1;  // the shader stored index + 1
        std::memcpy(&viewDistance, &pixels[i + 1], sizeof(float));
        found = true;
      }
    }
  }
  return found;
}

}  // namespace pcv
