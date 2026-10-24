#include "scene/camera.hpp"

#include <algorithm>
#include <cmath>

namespace pcv {
namespace {

constexpr Vec3f kWorldUp{0.0f, 0.0f, 1.0f};

// Extracts the six frustum planes from a view-projection matrix using the
// Gribb-Hartmann method: each plane is a sum or difference of two rows.
// Derived from the matrix rather than from the camera parameters, so it
// stays correct for both the perspective and orthographic paths.
Frustum extractPlanes(const Mat4& m) {
  Frustum f;
  auto row = [&](int r) { return Vec4f{m[0][r], m[1][r], m[2][r], m[3][r]}; };
  const Vec4f r0 = row(0), r1 = row(1), r2 = row(2), r3 = row(3);

  auto combine = [](const Vec4f& a, const Vec4f& b, float sign) {
    Vec4f p{a.x + sign * b.x, a.y + sign * b.y, a.z + sign * b.z, a.w + sign * b.w};
    const float len = std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
    if (len > 0.0f) { p.x /= len; p.y /= len; p.z /= len; p.w /= len; }
    return p;
  };

  f.planes[0] = combine(r3, r0, +1.0f);  // left
  f.planes[1] = combine(r3, r0, -1.0f);  // right
  f.planes[2] = combine(r3, r1, +1.0f);  // bottom
  f.planes[3] = combine(r3, r1, -1.0f);  // top
  // Reversed-Z with a [0,1] depth range: near is z = w, far is z = 0, so
  // these two are not the usual w +/- z pair.
  f.planes[4] = combine(r3, r2, -1.0f);  // near
  f.planes[5] = r2;                      // far
  {
    Vec4f& p = f.planes[5];
    const float len = std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
    if (len > 0.0f) { p.x /= len; p.y /= len; p.z /= len; p.w /= len; }
  }
  return f;
}

}  // namespace

const char* navModeName(NavMode mode) {
  switch (mode) {
    case NavMode::Orbit:   return "Orbit";
    case NavMode::Fly:     return "Fly";
    case NavMode::TopDown: return "Top-down";
  }
  return "?";
}

Vec3f Camera::orbitOffset() const {
  const float cp = std::cos(pitch);
  return Vec3f{std::cos(yaw) * cp, std::sin(yaw) * cp, std::sin(pitch)} * distance;
}

Vec3f Camera::position() const {
  if (mode == NavMode::Fly) return eye;
  if (mode == NavMode::TopDown) return Vec3f{target.x, target.y, target.z + sceneRadius * 4.0f};
  return target + orbitOffset();
}

Vec3f Camera::forward() const {
  if (mode == NavMode::TopDown) return Vec3f{0.0f, 0.0f, -1.0f};
  if (mode == NavMode::Fly) {
    const float cp = std::cos(pitch);
    return normalize(Vec3f{std::cos(yaw) * cp, std::sin(yaw) * cp, std::sin(pitch)});
  }
  return normalize(target - position());
}

Vec3f Camera::up() const {
  // Looking straight down, the world up vector is parallel to the view
  // direction; +Y becomes screen-up, which is the usual map orientation.
  if (mode == NavMode::TopDown) return Vec3f{0.0f, 1.0f, 0.0f};
  return kWorldUp;
}

Vec3f Camera::right() const { return normalize(cross(forward(), up())); }

void Camera::frameBounds(const Aabbf& bounds, float aspect) {
  if (!bounds.valid()) return;
  target = bounds.center();
  eye = target;
  sceneRadius = std::max(0.001f, length(bounds.size()) * 0.5f);

  // Fit the actual box as seen from the current angle, not its bounding
  // sphere. Clouds are rarely cube-shaped -- an HD map is a flat slab, say
  // 400 x 300 x 17 m -- and framing a sphere around one leaves the map
  // occupying a fraction of the window.
  //
  // Solved exactly, per corner. With the camera at target - forward*d, a
  // corner at camera-space (x, y, z) sits at depth z + d, and stays inside
  // the frustum when |x| <= tanX*(z+d) and |y| <= tanY*(z+d). Rearranged,
  // each corner imposes a lower bound on d, and the answer is the largest.
  //
  // Bounding the box's extents separately -- the obvious approach -- is
  // noticeably too conservative, because it assumes one corner can be at
  // maximum lateral and maximum forward extent at once.
  const Vec3f f = forward();
  const Vec3f r = right();
  const Vec3f u = normalize(cross(r, f));

  const float tanY = std::tan(fovY * 0.5f);
  const float tanX = tanY * std::max(0.1f, aspect);

  float required = 0.0f;
  float maxHalfUp = 0.0f;
  float maxHalfRight = 0.0f;
  for (int corner = 0; corner < 8; ++corner) {
    const Vec3f p{(corner & 1) ? bounds.hi.x : bounds.lo.x,
                  (corner & 2) ? bounds.hi.y : bounds.lo.y,
                  (corner & 4) ? bounds.hi.z : bounds.lo.z};
    const Vec3f rel = p - target;
    const float x = dot(rel, r);
    const float y = dot(rel, u);
    const float z = dot(rel, f);
    required = std::max(required, std::abs(x) / tanX - z);
    required = std::max(required, std::abs(y) / tanY - z);
    maxHalfRight = std::max(maxHalfRight, std::abs(x));
    maxHalfUp = std::max(maxHalfUp, std::abs(y));
  }
  distance = std::max(required * 1.05f, 1e-3f);
  orthoHeight = std::max(maxHalfUp, maxHalfRight / std::max(0.1f, aspect)) * 2.1f;

  // Scale-relative defaults. A near plane proportional to the scene keeps
  // reversed-Z precision good without clipping anything the user can reach.
  zNear = std::max(0.01f, sceneRadius * 1e-4f);
  moveSpeed = sceneRadius * 0.5f;

  if (mode == NavMode::Fly) eye = target - forward() * distance;
}

void Camera::focusOn(const Vec3f& point, bool keepDistance) {
  if (mode == NavMode::Fly) {
    // Keep the viewing direction, move so the point sits at the old focus
    // distance ahead.
    eye = point - forward() * (keepDistance ? distance : sceneRadius * 0.25f);
    return;
  }
  target = point;
  if (!keepDistance) distance = sceneRadius * 0.25f;
}

Mat4 Camera::view() const {
  const Vec3f p = position();
  return lookAt(p, p + forward(), up());
}

Mat4 Camera::projection(float aspect) const {
  if (mode == NavMode::TopDown) {
    const float halfH = orthoHeight * 0.5f;
    const float halfW = halfH * aspect;
    const float depth = sceneRadius * 10.0f;
    return orthographic(-halfW, halfW, -halfH, halfH, 0.0f, depth);
  }
  return perspectiveReverseZ(fovY, aspect, zNear);
}

Mat4 Camera::viewProjection(float aspect) const { return projection(aspect) * view(); }

Frustum Camera::frustum(float aspect) const { return extractPlanes(viewProjection(aspect)); }

float Camera::projectionScale(float viewportHeight) const {
  return viewportHeight / (2.0f * std::tan(fovY * 0.5f));
}

float Camera::orthoPixelsPerMetre(float viewportHeight) const {
  return orthoHeight > 0.0f ? viewportHeight / orthoHeight : 1.0f;
}

void Camera::orbit(float deltaYaw, float deltaPitch) {
  if (mode == NavMode::TopDown) return;  // the map view stays axis-aligned
  yaw += deltaYaw;
  pitch = std::clamp(pitch + deltaPitch, -kMaxPitch, kMaxPitch);
  // Keep yaw bounded so it never loses float precision during long drags.
  const float twoPi = static_cast<float>(2.0 * kPi);
  if (yaw > twoPi) yaw -= twoPi;
  if (yaw < -twoPi) yaw += twoPi;
}

void Camera::pan(float dxPixels, float dyPixels, float viewportHeight) {
  // Convert pixel motion into world motion so the point under the cursor
  // tracks it, regardless of zoom level.
  float metresPerPixel;
  if (mode == NavMode::TopDown)
    metresPerPixel = orthoHeight / std::max(1.0f, viewportHeight);
  else
    metresPerPixel = 2.0f * distance * std::tan(fovY * 0.5f) / std::max(1.0f, viewportHeight);

  const Vec3f r = right();
  const Vec3f u = normalize(cross(r, forward()));
  const Vec3f delta = (-r * dxPixels + u * dyPixels) * metresPerPixel;

  target += delta;
  eye += delta;
}

void Camera::dolly(float scrollTicks) {
  // Geometric zoom: each tick is a fixed ratio, so zooming feels the same
  // whether you are 1 m or 1 km out.
  const float factor = std::pow(0.85f, scrollTicks);
  if (mode == NavMode::TopDown) {
    orthoHeight = std::clamp(orthoHeight * factor, sceneRadius * 1e-4f, sceneRadius * 20.0f);
    return;
  }
  if (mode == NavMode::Fly) {
    eye += forward() * (distance * (1.0f - factor));
    return;
  }
  distance = std::clamp(distance * factor, sceneRadius * 1e-4f, sceneRadius * 100.0f);
}

void Camera::moveLocal(const Vec3f& delta) {
  const Vec3f f = forward();
  const Vec3f r = right();
  // Vertical movement follows world up, not camera up: pressing "up" while
  // looking at the ground should rise, not fly backwards.
  const Vec3f motion = r * delta.x + kWorldUp * delta.y + f * delta.z;
  eye += motion;
  target += motion;
}

void Camera::setMode(NavMode newMode, float /*aspect*/) {
  if (newMode == mode) return;
  // Carry the viewpoint across so switching modes does not teleport.
  const Vec3f currentEye = position();
  const Vec3f currentForward = forward();

  if (newMode == NavMode::Fly) {
    eye = currentEye;
  } else if (mode == NavMode::Fly && newMode == NavMode::Orbit) {
    // Put the focus where the camera was already looking.
    target = currentEye + currentForward * distance;
  }
  mode = newMode;
}

}  // namespace pcv
