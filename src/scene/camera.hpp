// Navigation camera: orbit, first-person fly and top-down orthographic.
//
// Z-up throughout, per ROS REP-103: Z is height, XY is the ground plane.
// That matches the convention of every cloud this viewer is likely to open.
//
// Camera state is kept in the cloud's LOCAL frame -- positions relative to
// PointCloud::origin -- because that is the frame the GPU works in. World
// coordinates are reconstructed only for display.
#pragma once

#include "core/math.hpp"

namespace punto {

enum class NavMode {
  Orbit,    // turntable about a focus point
  Fly,      // first-person, WASD + mouse look
  TopDown,  // orthographic map view
};

const char* navModeName(NavMode mode);

// Standard viewpoints, named by the side the camera sits on rather than by
// "front" or "left", which mean different things in different tools. Z is
// up throughout, per REP-103.
enum class StandardView {
  Top,      // above, looking down
  Bottom,   // below, looking up
  PlusX,    // on the +X side, looking toward -X
  MinusX,
  PlusY,
  MinusY,
  Isometric,
};

const char* standardViewName(StandardView view);
const char* standardViewTooltip(StandardView view);

struct Frustum {
  // Inward-facing planes: xyz is the normal, w the distance. A point is
  // inside when dot(normal, p) + w >= 0 for all six.
  Vec4f planes[6];
};

class Camera {
 public:
  // ------------------------------------------------------------ framing

  // Frames `bounds` so the whole thing is visible, and sets the orbit focus
  // to its centre. Also derives sensible movement speeds and clip distances
  // from the scene size, which is what keeps navigation usable across
  // clouds spanning centimetres to kilometres.
  void frameBounds(const Aabbf& bounds, float aspect);

  // Re-centres the orbit focus without changing the viewing direction.
  void focusOn(const Vec3f& point, bool keepDistance = true);

  // ------------------------------------------------------------ matrices

  Mat4 view() const;
  Mat4 projection(float aspect) const;
  Mat4 viewProjection(float aspect) const;
  Frustum frustum(float aspect) const;

  Vec3f position() const;
  Vec3f forward() const;
  Vec3f right() const;
  Vec3f up() const;

  // Pixels per metre at `distance`, used for splat sizing and LOD.
  float projectionScale(float viewportHeight) const;
  // World size of one pixel at `distance`. Lets an overlay marker keep a
  // constant size on screen however far away the thing it marks is.
  float worldPerPixel(float distance, float viewportHeight) const;
  float orthoPixelsPerMetre(float viewportHeight) const;

  // ------------------------------------------------------------ controls

  void orbit(float deltaYawRadians, float deltaPitchRadians);
  void pan(float dxPixels, float dyPixels, float viewportHeight);
  void dolly(float scrollTicks);
  void moveLocal(const Vec3f& delta);   // fly mode, camera-relative
  void setMode(NavMode mode, float aspect);
  // Points the camera at the focus from a standard direction, leaving the
  // focus and distance alone so the framing is preserved.
  void setStandardView(StandardView view);

  // ------------------------------------------------------------- state

  NavMode mode = NavMode::Orbit;
  Vec3f target{0.0f, 0.0f, 0.0f};  // orbit focus, local frame
  Vec3f eye{0.0f, 0.0f, 0.0f};     // fly position, local frame
  float distance = 10.0f;          // orbit radius
  float yaw = radians(-45.0);      // about +Z
  float pitch = radians(40.0);     // above the XY plane
  float fovY = static_cast<float>(radians(55.0));
  float zNear = 0.05f;
  float orthoHeight = 100.0f;      // world metres visible vertically

  // Derived from the scene, so navigation feels the same at every scale.
  float sceneRadius = 10.0f;
  float moveSpeed = 5.0f;          // metres/second in fly mode

  // Clamp to just under vertical: at exactly 90 degrees the up vector and
  // the view direction are parallel and the view matrix degenerates.
  static constexpr float kMaxPitch = 1.5533f;  // 89 degrees

 private:
  Vec3f orbitOffset() const;
};

}  // namespace punto
