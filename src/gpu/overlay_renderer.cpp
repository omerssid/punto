#include "gpu/overlay_renderer.hpp"

#include <cmath>

#include "core/log.hpp"

namespace punto {
namespace {
constexpr int kMarkerSegments = 32;
}  // namespace

OverlayRenderer::~OverlayRenderer() {
  if (vao_ != 0) glDeleteVertexArrays(1, &vao_);
}

bool OverlayRenderer::initialize(std::string& error) {
  if (!program_.buildGraphics("overlay.vert", "overlay.frag", {}, error)) return false;

  glCreateVertexArrays(1, &vao_);

  glEnableVertexArrayAttrib(vao_, 0);
  glVertexArrayAttribFormat(vao_, 0, 3, GL_FLOAT, GL_FALSE, offsetof(Vertex, start));
  glVertexArrayAttribBinding(vao_, 0, 0);

  glEnableVertexArrayAttrib(vao_, 1);
  glVertexArrayAttribFormat(vao_, 1, 3, GL_FLOAT, GL_FALSE, offsetof(Vertex, end));
  glVertexArrayAttribBinding(vao_, 1, 0);

  // endSelect and side arrive as signed bytes and are read as floats, so the
  // shader can mix() and multiply with them directly.
  glEnableVertexArrayAttrib(vao_, 2);
  glVertexArrayAttribFormat(vao_, 2, 2, GL_BYTE, GL_FALSE, offsetof(Vertex, endSelect));
  glVertexArrayAttribBinding(vao_, 2, 0);

  glEnableVertexArrayAttrib(vao_, 3);
  glVertexArrayAttribFormat(vao_, 3, 4, GL_UNSIGNED_BYTE, GL_TRUE, offsetof(Vertex, color));
  glVertexArrayAttribBinding(vao_, 3, 0);
  return true;
}

bool OverlayRenderer::reloadShaders(std::string& error) { return program_.reload(error); }

void OverlayRenderer::begin() { vertices_.clear(); }

void OverlayRenderer::addLine(const Vec3f& a, const Vec3f& b, std::uint32_t rgba) {
  // Two triangles per segment. Both endpoints are stored on every vertex
  // because the shader derives the offset direction from the segment.
  static constexpr std::int8_t kEnds[6] = {0, 0, 1, 1, 0, 1};
  static constexpr std::int8_t kSides[6] = {-1, 1, -1, -1, 1, 1};

  for (int corner = 0; corner < 6; ++corner) {
    Vertex v;
    v.start[0] = a.x; v.start[1] = a.y; v.start[2] = a.z;
    v.end[0] = b.x; v.end[1] = b.y; v.end[2] = b.z;
    v.endSelect = kEnds[corner];
    v.side = kSides[corner];
    v.pad[0] = 0;
    v.pad[1] = 0;
    v.color = rgba;
    vertices_.push_back(v);
  }
}

void OverlayRenderer::addDashedLine(const Vec3f& a, const Vec3f& b, std::uint32_t rgba,
                                    float dashLength) {
  const Vec3f delta = b - a;
  const float total = length(delta);
  if (total <= 0.0f || dashLength <= 0.0f) {
    addLine(a, b, rgba);
    return;
  }
  // Cap the dash count so a long line at a steep angle cannot generate
  // tens of thousands of quads.
  const float minDash = total / 512.0f;
  const float dash = std::max(dashLength, minDash);
  const Vec3f step = delta / total;
  for (float t = 0.0f; t < total; t += dash * 2.0f) {
    const float end = std::min(t + dash, total);
    addLine(a + step * t, a + step * end, rgba);
  }
}

void OverlayRenderer::addBox(const Aabbf& box, std::uint32_t rgba) {
  if (!box.valid()) return;
  const Vec3f lo = box.lo;
  const Vec3f hi = box.hi;
  const Vec3f corners[8] = {
      {lo.x, lo.y, lo.z}, {hi.x, lo.y, lo.z}, {hi.x, hi.y, lo.z}, {lo.x, hi.y, lo.z},
      {lo.x, lo.y, hi.z}, {hi.x, lo.y, hi.z}, {hi.x, hi.y, hi.z}, {lo.x, hi.y, hi.z}};
  static const int edges[12][2] = {{0, 1}, {1, 2}, {2, 3}, {3, 0},
                                   {4, 5}, {5, 6}, {6, 7}, {7, 4},
                                   {0, 4}, {1, 5}, {2, 6}, {3, 7}};
  for (const auto& edge : edges) addLine(corners[edge[0]], corners[edge[1]], rgba);
}

void OverlayRenderer::addMarker(const Vec3f& centre, float radius, std::uint32_t rgba,
                                const Camera& camera) {
  // Built in the camera's plane so the ring reads as a ring from any angle,
  // rather than foreshortening into a line.
  const Vec3f forward = camera.forward();
  Vec3f right = camera.right();
  if (lengthSq(right) < 1e-12f) right = Vec3f{1.0f, 0.0f, 0.0f};
  const Vec3f up = normalize(cross(right, forward));

  Vec3f previous{};
  for (int i = 0; i <= kMarkerSegments; ++i) {
    const float angle = static_cast<float>(i) / kMarkerSegments * 2.0f * static_cast<float>(kPi);
    const Vec3f point = centre + (right * std::cos(angle) + up * std::sin(angle)) * radius;
    if (i > 0) addLine(previous, point, rgba);
    previous = point;
  }

  // Crosshair ticks outside the ring, which is what makes the exact centre
  // readable when the ring sits against busy geometry.
  const float inner = radius * 1.35f;
  const float outer = radius * 2.1f;
  addLine(centre + right * inner, centre + right * outer, rgba);
  addLine(centre - right * inner, centre - right * outer, rgba);
  addLine(centre + up * inner, centre + up * outer, rgba);
  addLine(centre - up * inner, centre - up * outer, rgba);
}

void OverlayRenderer::draw(const Camera& camera, int width, int height, float lineWidthPixels) {
  if (vertices_.empty() || vao_ == 0) return;

  const std::size_t bytes = vertices_.size() * sizeof(Vertex);
  if (bytes > capacity_) {
    // Grow geometrically so a measurement chain does not reallocate on
    // every added point.
    capacity_ = std::max<std::size_t>(bytes * 2, 8192);
    vbo_.allocate(capacity_, GL_DYNAMIC_STORAGE_BIT);
    glVertexArrayVertexBuffer(vao_, 0, vbo_.id(), 0, sizeof(Vertex));
  }
  vbo_.upload(0, bytes, vertices_.data());

  const float aspect = static_cast<float>(width) / static_cast<float>(std::max(1, height));
  program_.use();
  program_.set("uViewProj", camera.viewProjection(aspect));
  program_.set("uViewport", static_cast<float>(width), static_cast<float>(height));
  program_.set("uHalfWidth", lineWidthPixels * 0.5f);
  program_.set("uAlpha", 1.0f);

  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glDisable(GL_DEPTH_TEST);
  // Quads are wound both ways depending on which side of the segment the
  // camera is on, so culling would drop half of them.
  glDisable(GL_CULL_FACE);

  glBindVertexArray(vao_);
  glDrawArrays(GL_TRIANGLES, 0, static_cast<GLsizei>(vertices_.size()));
  glBindVertexArray(0);

  glDisable(GL_BLEND);
}

}  // namespace punto
