// Minimal linear algebra for the viewer.
//
// World-space quantities are double precision on purpose: SLAM and HD-map
// clouds routinely sit at UTM-like offsets (4e5, 5e6, ...) where float32 has
// ~0.5 m of resolution. Everything handed to the GPU is rebased onto a local
// origin first and only then narrowed to float.
#pragma once

#include <algorithm>
#include <cmath>
#include <limits>

namespace punto {

inline constexpr double kPi = 3.14159265358979323846;

inline constexpr double radians(double deg) { return deg * (kPi / 180.0); }
inline constexpr double degrees(double rad) { return rad * (180.0 / kPi); }

// ---------------------------------------------------------------- vectors

template <typename T>
struct Vec3T {
  T x{}, y{}, z{};

  constexpr Vec3T() = default;
  constexpr Vec3T(T x_, T y_, T z_) : x(x_), y(y_), z(z_) {}
  explicit constexpr Vec3T(T s) : x(s), y(s), z(s) {}

  template <typename U>
  explicit constexpr Vec3T(const Vec3T<U>& o)
      : x(static_cast<T>(o.x)), y(static_cast<T>(o.y)), z(static_cast<T>(o.z)) {}

  constexpr T operator[](int i) const { return (&x)[i]; }
  constexpr T& operator[](int i) { return (&x)[i]; }

  friend constexpr Vec3T operator+(Vec3T a, Vec3T b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
  friend constexpr Vec3T operator-(Vec3T a, Vec3T b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
  friend constexpr Vec3T operator*(Vec3T a, Vec3T b) { return {a.x * b.x, a.y * b.y, a.z * b.z}; }
  friend constexpr Vec3T operator/(Vec3T a, Vec3T b) { return {a.x / b.x, a.y / b.y, a.z / b.z}; }
  friend constexpr Vec3T operator*(Vec3T a, T s) { return {a.x * s, a.y * s, a.z * s}; }
  friend constexpr Vec3T operator*(T s, Vec3T a) { return a * s; }
  friend constexpr Vec3T operator/(Vec3T a, T s) { return {a.x / s, a.y / s, a.z / s}; }
  friend constexpr Vec3T operator-(Vec3T a) { return {-a.x, -a.y, -a.z}; }

  Vec3T& operator+=(Vec3T o) { x += o.x; y += o.y; z += o.z; return *this; }
  Vec3T& operator-=(Vec3T o) { x -= o.x; y -= o.y; z -= o.z; return *this; }
  Vec3T& operator*=(T s) { x *= s; y *= s; z *= s; return *this; }

  friend constexpr bool operator==(Vec3T a, Vec3T b) { return a.x == b.x && a.y == b.y && a.z == b.z; }
};

using Vec3 = Vec3T<double>;
using Vec3f = Vec3T<float>;

template <typename T>
constexpr T dot(Vec3T<T> a, Vec3T<T> b) { return a.x * b.x + a.y * b.y + a.z * b.z; }

template <typename T>
constexpr Vec3T<T> cross(Vec3T<T> a, Vec3T<T> b) {
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

template <typename T>
T length(Vec3T<T> v) { return std::sqrt(dot(v, v)); }

template <typename T>
constexpr T lengthSq(Vec3T<T> v) { return dot(v, v); }

template <typename T>
Vec3T<T> normalize(Vec3T<T> v) {
  const T len = length(v);
  return len > T(0) ? v / len : v;
}

template <typename T>
constexpr Vec3T<T> minv(Vec3T<T> a, Vec3T<T> b) {
  return {std::min(a.x, b.x), std::min(a.y, b.y), std::min(a.z, b.z)};
}

template <typename T>
constexpr Vec3T<T> maxv(Vec3T<T> a, Vec3T<T> b) {
  return {std::max(a.x, b.x), std::max(a.y, b.y), std::max(a.z, b.z)};
}

template <typename T>
constexpr T maxComponent(Vec3T<T> v) { return std::max(v.x, std::max(v.y, v.z)); }

struct Vec4f {
  float x{}, y{}, z{}, w{};
  constexpr Vec4f() = default;
  constexpr Vec4f(float x_, float y_, float z_, float w_) : x(x_), y(y_), z(z_), w(w_) {}
  constexpr float operator[](int i) const { return (&x)[i]; }
  constexpr float& operator[](int i) { return (&x)[i]; }
};

// ---------------------------------------------------------------- Mat4

// Column-major, matching GLSL's memory layout so these upload verbatim.
// m[c] is column c; element (row r, col c) is m[c][r].
struct Mat4 {
  Vec4f m[4];

  constexpr Mat4() : m{{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}} {}

  constexpr const Vec4f& operator[](int c) const { return m[c]; }
  constexpr Vec4f& operator[](int c) { return m[c]; }

  const float* data() const { return &m[0].x; }

  static constexpr Mat4 identity() { return Mat4{}; }

  friend Mat4 operator*(const Mat4& a, const Mat4& b) {
    Mat4 r;
    for (int c = 0; c < 4; ++c) {
      for (int row = 0; row < 4; ++row) {
        float sum = 0.0f;
        for (int k = 0; k < 4; ++k) sum += a.m[k][row] * b.m[c][k];
        r.m[c][row] = sum;
      }
    }
    return r;
  }

  friend Vec4f operator*(const Mat4& a, const Vec4f& v) {
    Vec4f r{0, 0, 0, 0};
    for (int row = 0; row < 4; ++row) {
      float sum = 0.0f;
      for (int k = 0; k < 4; ++k) sum += a.m[k][row] * v[k];
      r[row] = sum;
    }
    return r;
  }
};

Mat4 lookAt(Vec3f eye, Vec3f center, Vec3f up);
// Reversed-Z infinite perspective: near plane maps to 1, infinity to 0.
// Paired with GL_GREATER depth testing and a float depth buffer this removes
// essentially all z-fighting across the huge depth ranges HD maps span.
Mat4 perspectiveReverseZ(float fovYRadians, float aspect, float zNear);
Mat4 orthographic(float l, float r, float b, float t, float n, float f);
Mat4 inverse(const Mat4& mat);
Mat4 transpose(const Mat4& mat);

// ---------------------------------------------------------------- AABB

template <typename T>
struct AabbT {
  Vec3T<T> lo{std::numeric_limits<T>::max(), std::numeric_limits<T>::max(), std::numeric_limits<T>::max()};
  Vec3T<T> hi{std::numeric_limits<T>::lowest(), std::numeric_limits<T>::lowest(), std::numeric_limits<T>::lowest()};

  constexpr bool valid() const { return lo.x <= hi.x && lo.y <= hi.y && lo.z <= hi.z; }
  constexpr void expand(Vec3T<T> p) { lo = minv(lo, p); hi = maxv(hi, p); }
  constexpr void expand(const AabbT& o) { lo = minv(lo, o.lo); hi = maxv(hi, o.hi); }
  constexpr Vec3T<T> size() const { return valid() ? hi - lo : Vec3T<T>(T(0)); }
  constexpr Vec3T<T> center() const { return valid() ? (lo + hi) * T(0.5) : Vec3T<T>(T(0)); }
  T diagonal() const { return length(size()); }

  template <typename U>
  AabbT<U> cast() const {
    AabbT<U> r;
    r.lo = Vec3T<U>(lo);
    r.hi = Vec3T<U>(hi);
    return r;
  }
};

using Aabb = AabbT<double>;
using Aabbf = AabbT<float>;

}  // namespace punto
