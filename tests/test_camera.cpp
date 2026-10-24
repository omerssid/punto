// Camera tests: standard views, framing and the reversed-Z projection.
//
// These are easy to get subtly wrong -- a sign error puts the camera on the
// opposite side, and a degenerate up vector produces a blank screen with no
// error anywhere -- and none of it is obvious from a screenshot.
#include <cmath>
#include <cstdio>
#include <string>

#include "scene/camera.hpp"

namespace {

int g_failures = 0;
int g_checks = 0;

void fail(const char* file, int line, const std::string& what) {
  ++g_failures;
  std::printf("  \x1b[31mFAIL\x1b[0m %s:%d  %s\n", file, line, what.c_str());
}

#define CHECK(cond)                                            \
  do {                                                         \
    ++g_checks;                                                \
    if (!(cond)) fail(__FILE__, __LINE__, "expected: " #cond); \
  } while (0)

#define CHECK_NEAR(a, b, eps)                                              \
  do {                                                                     \
    ++g_checks;                                                            \
    const double va_ = static_cast<double>(a);                             \
    const double vb_ = static_cast<double>(b);                             \
    if (!(std::fabs(va_ - vb_) <= (eps)))                                  \
      fail(__FILE__, __LINE__, std::string(#a " ~= " #b "  (") +           \
                                   std::to_string(va_) + " vs " +          \
                                   std::to_string(vb_) + ")");             \
  } while (0)

void beginTest(const char* name) { std::printf("\x1b[1m%s\x1b[0m\n", name); }

using namespace punto;

Camera framedCamera() {
  Camera camera;
  Aabbf bounds;
  bounds.expand(Vec3f{-10.0f, -20.0f, -2.0f});
  bounds.expand(Vec3f{30.0f, 15.0f, 8.0f});
  camera.frameBounds(bounds, 16.0f / 9.0f);
  return camera;
}

void testStandardViewDirections() {
  beginTest("standard views look from the side they name");
  Camera camera = framedCamera();

  struct Expect { StandardView view; Vec3f forward; const char* name; };
  // Each view is named for the side the camera sits on, so the viewing
  // direction is the negation of that axis.
  const Expect expected[] = {
      {StandardView::Top, {0.0f, 0.0f, -1.0f}, "Top"},
      {StandardView::Bottom, {0.0f, 0.0f, 1.0f}, "Bottom"},
      {StandardView::PlusX, {-1.0f, 0.0f, 0.0f}, "+X"},
      {StandardView::MinusX, {1.0f, 0.0f, 0.0f}, "-X"},
      {StandardView::PlusY, {0.0f, -1.0f, 0.0f}, "+Y"},
      {StandardView::MinusY, {0.0f, 1.0f, 0.0f}, "-Y"},
  };

  for (const Expect& e : expected) {
    camera.setStandardView(e.view);
    const Vec3f f = camera.forward();
    // Top and Bottom stop one degree short of vertical, so allow for it.
    const float tolerance = (e.view == StandardView::Top || e.view == StandardView::Bottom)
                                ? 0.02f
                                : 1e-5f;
    if (std::fabs(f.x - e.forward.x) > tolerance || std::fabs(f.y - e.forward.y) > tolerance ||
        std::fabs(f.z - e.forward.z) > tolerance)
      std::printf("    %s: forward (%.4f %.4f %.4f), expected (%.1f %.1f %.1f)\n", e.name, f.x,
                  f.y, f.z, e.forward.x, e.forward.y, e.forward.z);
    CHECK(std::fabs(f.x - e.forward.x) <= tolerance);
    CHECK(std::fabs(f.y - e.forward.y) <= tolerance);
    CHECK(std::fabs(f.z - e.forward.z) <= tolerance);

    // The camera must actually sit on the named side of the focus.
    const Vec3f offset = camera.position() - camera.target;
    CHECK_NEAR(dot(normalize(offset), e.forward), -1.0, 0.03);
  }
}

void testVerticalViewsDoNotDegenerate() {
  beginTest("vertical views stay numerically well conditioned");
  Camera camera = framedCamera();

  // Looking exactly along world up makes the view matrix singular and
  // renders nothing, with no error reported anywhere, so the vertical
  // views stop one degree short.
  //
  // Camera::up() returns the up HINT -- world Z -- which is deliberately
  // almost parallel to the view direction here; lookAt derives the real up
  // from it by cross product. So the property worth testing is that the
  // resulting basis is orthonormal, not that the hint is perpendicular.
  for (StandardView view : {StandardView::Top, StandardView::Bottom}) {
    camera.setStandardView(view);
    CHECK(std::fabs(camera.pitch) <= Camera::kMaxPitch + 1e-4f);

    const Vec3f f = camera.forward();
    const Vec3f r = camera.right();
    CHECK_NEAR(length(f), 1.0, 1e-4);
    CHECK_NEAR(length(r), 1.0, 1e-4);
    CHECK_NEAR(dot(f, r), 0.0, 1e-4);

    // The view matrix's upper-left 3x3 is the camera basis; it must be
    // orthonormal or the scene shears.
    const Mat4 m = camera.view();
    for (int c = 0; c < 4; ++c)
      for (int row = 0; row < 4; ++row) CHECK(std::isfinite(m[c][row]));

    const Vec3f bx{m[0][0], m[1][0], m[2][0]};
    const Vec3f by{m[0][1], m[1][1], m[2][1]};
    const Vec3f bz{m[0][2], m[1][2], m[2][2]};
    CHECK_NEAR(length(bx), 1.0, 1e-4);
    CHECK_NEAR(length(by), 1.0, 1e-4);
    CHECK_NEAR(length(bz), 1.0, 1e-4);
    CHECK_NEAR(dot(bx, by), 0.0, 1e-4);
    CHECK_NEAR(dot(by, bz), 0.0, 1e-4);
    CHECK_NEAR(dot(bx, bz), 0.0, 1e-4);
  }
}

void testIsometricForeshortensEqually() {
  beginTest("the isometric view foreshortens all three axes equally");
  Camera camera = framedCamera();
  camera.setStandardView(StandardView::Isometric);
  const Vec3f f = camera.forward();
  // True isometric: the view direction makes the same angle with each axis,
  // so the magnitudes of its components match.
  CHECK_NEAR(std::fabs(f.x), std::fabs(f.y), 1e-3);
  CHECK_NEAR(std::fabs(f.y), std::fabs(f.z), 1e-3);
}

void testFramingFitsTheBox() {
  beginTest("framing fits every corner of the bounding box on screen");
  Aabbf bounds;
  bounds.expand(Vec3f{-10.0f, -20.0f, -2.0f});
  bounds.expand(Vec3f{30.0f, 15.0f, 8.0f});

  for (float aspect : {16.0f / 9.0f, 1.0f, 0.6f}) {
    for (StandardView view : {StandardView::Isometric, StandardView::Top, StandardView::PlusX}) {
      Camera camera;
      camera.setStandardView(view);
      camera.frameBounds(bounds, aspect);

      const Mat4 viewProj = camera.viewProjection(aspect);
      int outside = 0;
      for (int corner = 0; corner < 8; ++corner) {
        const Vec3f p{(corner & 1) ? bounds.hi.x : bounds.lo.x,
                      (corner & 2) ? bounds.hi.y : bounds.lo.y,
                      (corner & 4) ? bounds.hi.z : bounds.lo.z};
        const Vec4f clip = viewProj * Vec4f{p.x, p.y, p.z, 1.0f};
        if (clip.w <= 0.0f) { ++outside; continue; }
        const float x = clip.x / clip.w;
        const float y = clip.y / clip.w;
        // A small tolerance for the margin framing deliberately leaves.
        if (x < -1.001f || x > 1.001f || y < -1.001f || y > 1.001f) ++outside;
      }
      if (outside != 0)
        std::printf("    aspect %.2f, view %s: %d corners off screen\n", aspect,
                    standardViewName(view), outside);
      CHECK(outside == 0);
    }
  }
}

void testReversedZDepthRange() {
  beginTest("reversed-Z maps the near plane to 1 and distance to 0");
  Camera camera = framedCamera();
  camera.setStandardView(StandardView::PlusX);
  const float aspect = 16.0f / 9.0f;
  const Mat4 proj = camera.projection(aspect);

  // A point at the near plane, in view space (looking down -Z).
  const Vec4f atNear = proj * Vec4f{0.0f, 0.0f, -camera.zNear, 1.0f};
  CHECK(atNear.w > 0.0f);
  CHECK_NEAR(atNear.z / atNear.w, 1.0, 1e-4);

  // Depth must fall monotonically with distance, which is what makes
  // GL_GREATER the right test.
  float previous = 2.0f;
  for (float distance : {1.0f, 10.0f, 100.0f, 1000.0f, 100000.0f}) {
    const Vec4f clip = proj * Vec4f{0.0f, 0.0f, -distance, 1.0f};
    const float depth = clip.z / clip.w;
    CHECK(depth < previous);
    CHECK(depth >= 0.0f);
    previous = depth;
  }
  CHECK(previous < 1e-4f);  // effectively zero at great distance
}

void testOrthographicDepthRange() {
  beginTest("the orthographic projection shares the reversed-Z convention");
  // A sign error here rendered zero chunks in top-down mode, because every
  // point fell outside the depth range the test expects.
  Camera camera = framedCamera();
  camera.mode = NavMode::TopDown;
  camera.setStandardView(StandardView::Top);
  const Mat4 proj = camera.projection(1.5f);

  const float far = camera.sceneRadius * 10.0f;
  const Vec4f atNear = proj * Vec4f{0.0f, 0.0f, 0.0f, 1.0f};
  const Vec4f atFar = proj * Vec4f{0.0f, 0.0f, -far, 1.0f};
  CHECK_NEAR(atNear.z / atNear.w, 1.0, 1e-4);
  CHECK_NEAR(atFar.z / atFar.w, 0.0, 1e-4);

  // And the points actually in the scene must land inside [0, 1].
  const Vec4f mid = proj * Vec4f{0.0f, 0.0f, -far * 0.5f, 1.0f};
  const float depth = mid.z / mid.w;
  CHECK(depth > 0.0f && depth < 1.0f);
}

void testModeSwitchKeepsTheViewpoint() {
  beginTest("switching navigation mode does not teleport the camera");
  Camera camera = framedCamera();
  camera.setStandardView(StandardView::Isometric);
  const Vec3f before = camera.position();

  camera.setMode(NavMode::Fly, 1.6f);
  const Vec3f after = camera.position();
  CHECK_NEAR(length(after - before), 0.0, 1e-3);
}

}  // namespace

int main() {
  testStandardViewDirections();
  testVerticalViewsDoNotDegenerate();
  testIsometricForeshortensEqually();
  testFramingFitsTheBox();
  testReversedZDepthRange();
  testOrthographicDepthRange();
  testModeSwitchKeepsTheViewpoint();

  std::printf("\n%d checks, \x1b[%sm%d failures\x1b[0m\n", g_checks, g_failures ? "31" : "32",
              g_failures);
  return g_failures == 0 ? 0 : 1;
}
