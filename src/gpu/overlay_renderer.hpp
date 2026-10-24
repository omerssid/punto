// Line overlay drawn on top of the point cloud: selection markers,
// measurement chains and the crop box.
//
// Drawn after the EDL pass, straight to the default framebuffer, with depth
// testing off. That makes annotations visible through geometry, which is
// what you want from a measurement -- and it is also forced: the EDL pass
// replaces any pixel whose linear depth is zero with the background colour,
// so an overlay drawn before it would be erased wherever it crossed empty
// space.
//
// Segments are expanded into screen-space quads by the vertex shader. See
// shaders/overlay.vert for why glLineWidth is not an option here.
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "gpu/buffer.hpp"
#include "gpu/shader.hpp"
#include "scene/camera.hpp"

namespace pcv {

class OverlayRenderer {
 public:
  OverlayRenderer() = default;
  ~OverlayRenderer();
  OverlayRenderer(const OverlayRenderer&) = delete;
  OverlayRenderer& operator=(const OverlayRenderer&) = delete;

  bool initialize(std::string& error);
  bool reloadShaders(std::string& error);

  // Clears the geometry accumulated for this frame.
  void begin();

  void addLine(const Vec3f& a, const Vec3f& b, std::uint32_t rgba);
  void addBox(const Aabbf& box, std::uint32_t rgba);
  // A camera-facing ring plus crosshair ticks, sized in world units.
  void addMarker(const Vec3f& centre, float radius, std::uint32_t rgba, const Camera& camera);
  // Dashes along a line, so a measurement reads as a measurement.
  void addDashedLine(const Vec3f& a, const Vec3f& b, std::uint32_t rgba, float dashLength);

  bool empty() const { return vertices_.empty(); }
  void draw(const Camera& camera, int width, int height, float lineWidthPixels = 2.0f);

 private:
  // One corner of a segment's screen-space quad. Both endpoints travel with
  // every vertex, because the shader needs the segment's direction to know
  // which way to offset.
  struct Vertex {
    float start[3];
    float end[3];
    std::int8_t endSelect;  // 0 = start, 1 = end
    std::int8_t side;       // -1 or +1
    std::int8_t pad[2];
    std::uint32_t color;    // RGBA8
  };
  static_assert(sizeof(Vertex) == 32);

  ShaderProgram program_;
  GLuint vao_ = 0;
  GlBuffer vbo_;
  std::size_t capacity_ = 0;
  std::vector<Vertex> vertices_;
};

// Packs a colour for the overlay. Components are 0..255.
constexpr std::uint32_t overlayColor(int r, int g, int b, int a = 255) {
  return static_cast<std::uint32_t>(r) | (static_cast<std::uint32_t>(g) << 8) |
         (static_cast<std::uint32_t>(b) << 16) | (static_cast<std::uint32_t>(a) << 24);
}

}  // namespace pcv
