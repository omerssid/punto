#include "core/math.hpp"

namespace pcv {

Mat4 lookAt(Vec3f eye, Vec3f center, Vec3f up) {
  const Vec3f f = normalize(center - eye);   // forward
  const Vec3f s = normalize(cross(f, up));   // right
  const Vec3f u = cross(s, f);               // true up

  Mat4 m;
  m[0] = {s.x, u.x, -f.x, 0.0f};
  m[1] = {s.y, u.y, -f.y, 0.0f};
  m[2] = {s.z, u.z, -f.z, 0.0f};
  m[3] = {-dot(s, eye), -dot(u, eye), dot(f, eye), 1.0f};
  return m;
}

Mat4 perspectiveReverseZ(float fovYRadians, float aspect, float zNear) {
  // Infinite far plane with a reversed depth range: the near plane maps to
  // depth 1 and infinity to depth 0. Combined with a floating-point depth
  // buffer this spends float's dense exponent region on distant geometry,
  // which is where conventional depth runs out of precision. Requires
  // glClearDepth(0), glDepthFunc(GL_GREATER) and GL_ARB_clip_control.
  const float t = 1.0f / std::tan(fovYRadians * 0.5f);

  Mat4 m;
  m[0] = {t / aspect, 0.0f, 0.0f, 0.0f};
  m[1] = {0.0f, t, 0.0f, 0.0f};
  m[2] = {0.0f, 0.0f, 0.0f, -1.0f};
  m[3] = {0.0f, 0.0f, zNear, 0.0f};
  return m;
}

Mat4 orthographic(float l, float r, float b, float t, float n, float f) {
  // Reversed Z here too, so the orthographic map view shares the depth
  // setup with the perspective path: the near plane maps to 1 and the far
  // plane to 0, against a cleared depth of 0 and GL_GREATER.
  //
  // For a view-space z of -d (looking down -Z), the mapping that satisfies
  // d = n -> 1 and d = f -> 0 is clipZ = (f - d) / (f - n), i.e.
  // clipZ = z / (f - n) + f / (f - n).
  Mat4 m;
  m[0] = {2.0f / (r - l), 0.0f, 0.0f, 0.0f};
  m[1] = {0.0f, 2.0f / (t - b), 0.0f, 0.0f};
  m[2] = {0.0f, 0.0f, 1.0f / (f - n), 0.0f};
  m[3] = {-(r + l) / (r - l), -(t + b) / (t - b), f / (f - n), 1.0f};
  return m;
}

Mat4 transpose(const Mat4& mat) {
  Mat4 r;
  for (int c = 0; c < 4; ++c)
    for (int row = 0; row < 4; ++row) r[c][row] = mat[row][c];
  return r;
}

Mat4 inverse(const Mat4& mat) {
  // General 4x4 inverse via cofactors. Called once per frame at most, so
  // clarity beats cleverness.
  const float* m = mat.data();
  float inv[16];

  inv[0]  =  m[5]*m[10]*m[15] - m[5]*m[11]*m[14] - m[9]*m[6]*m[15] + m[9]*m[7]*m[14] + m[13]*m[6]*m[11] - m[13]*m[7]*m[10];
  inv[4]  = -m[4]*m[10]*m[15] + m[4]*m[11]*m[14] + m[8]*m[6]*m[15] - m[8]*m[7]*m[14] - m[12]*m[6]*m[11] + m[12]*m[7]*m[10];
  inv[8]  =  m[4]*m[9]*m[15]  - m[4]*m[11]*m[13] - m[8]*m[5]*m[15] + m[8]*m[7]*m[13] + m[12]*m[5]*m[11] - m[12]*m[7]*m[9];
  inv[12] = -m[4]*m[9]*m[14]  + m[4]*m[10]*m[13] + m[8]*m[5]*m[14] - m[8]*m[6]*m[13] - m[12]*m[5]*m[10] + m[12]*m[6]*m[9];
  inv[1]  = -m[1]*m[10]*m[15] + m[1]*m[11]*m[14] + m[9]*m[2]*m[15] - m[9]*m[3]*m[14] - m[13]*m[2]*m[11] + m[13]*m[3]*m[10];
  inv[5]  =  m[0]*m[10]*m[15] - m[0]*m[11]*m[14] - m[8]*m[2]*m[15] + m[8]*m[3]*m[14] + m[12]*m[2]*m[11] - m[12]*m[3]*m[10];
  inv[9]  = -m[0]*m[9]*m[15]  + m[0]*m[11]*m[13] + m[8]*m[1]*m[15] - m[8]*m[3]*m[13] - m[12]*m[1]*m[11] + m[12]*m[3]*m[9];
  inv[13] =  m[0]*m[9]*m[14]  - m[0]*m[10]*m[13] - m[8]*m[1]*m[14] + m[8]*m[2]*m[13] + m[12]*m[1]*m[10] - m[12]*m[2]*m[9];
  inv[2]  =  m[1]*m[6]*m[15]  - m[1]*m[7]*m[14]  - m[5]*m[2]*m[15] + m[5]*m[3]*m[14] + m[13]*m[2]*m[7]  - m[13]*m[3]*m[6];
  inv[6]  = -m[0]*m[6]*m[15]  + m[0]*m[7]*m[14]  + m[4]*m[2]*m[15] - m[4]*m[3]*m[14] - m[12]*m[2]*m[7]  + m[12]*m[3]*m[6];
  inv[10] =  m[0]*m[5]*m[15]  - m[0]*m[7]*m[13]  - m[4]*m[1]*m[15] + m[4]*m[3]*m[13] + m[12]*m[1]*m[7]  - m[12]*m[3]*m[5];
  inv[14] = -m[0]*m[5]*m[14]  + m[0]*m[6]*m[13]  + m[4]*m[1]*m[14] - m[4]*m[2]*m[13] - m[12]*m[1]*m[6]  + m[12]*m[2]*m[5];
  inv[3]  = -m[1]*m[6]*m[11]  + m[1]*m[7]*m[10]  + m[5]*m[2]*m[11] - m[5]*m[3]*m[10] - m[9]*m[2]*m[7]   + m[9]*m[3]*m[6];
  inv[7]  =  m[0]*m[6]*m[11]  - m[0]*m[7]*m[10]  - m[4]*m[2]*m[11] + m[4]*m[3]*m[10] + m[8]*m[2]*m[7]   - m[8]*m[3]*m[6];
  inv[11] = -m[0]*m[5]*m[11]  + m[0]*m[7]*m[9]   + m[4]*m[1]*m[11] - m[4]*m[3]*m[9]  - m[8]*m[1]*m[7]   + m[8]*m[3]*m[5];
  inv[15] =  m[0]*m[5]*m[10]  - m[0]*m[6]*m[9]   - m[4]*m[1]*m[10] + m[4]*m[2]*m[9]  + m[8]*m[1]*m[6]   - m[8]*m[2]*m[5];

  float det = m[0]*inv[0] + m[1]*inv[4] + m[2]*inv[8] + m[3]*inv[12];
  if (det == 0.0f) return Mat4::identity();
  det = 1.0f / det;

  Mat4 result;
  float* out = &result[0].x;
  for (int i = 0; i < 16; ++i) out[i] = inv[i] * det;
  return result;
}

}  // namespace pcv
