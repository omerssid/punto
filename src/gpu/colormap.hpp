// CPU-side colour ramps, mirroring shaders/colormap.glsl.
//
// Duplicated deliberately: the UI needs to draw a preview strip and tint
// histogram bars, and reading them back from the GPU for that would be far
// more machinery than re-evaluating six polynomials. The coefficients are
// the same in both places and must stay in step.
#pragma once

#include <algorithm>
#include <array>

#include "core/math.hpp"

namespace pcv {

enum class ColorRamp : int;  // defined in point_renderer.hpp

namespace detail {

inline Vec3f polyRamp(float t, const std::array<Vec3f, 7>& c) {
  Vec3f v = c[6];
  for (int i = 5; i >= 0; --i) v = v * t + c[static_cast<std::size_t>(i)];
  return v;
}

inline Vec3f saturate(Vec3f v) {
  return Vec3f{std::clamp(v.x, 0.0f, 1.0f), std::clamp(v.y, 0.0f, 1.0f),
               std::clamp(v.z, 0.0f, 1.0f)};
}

}  // namespace detail

// `ramp` takes the same integer values as the GLSL RAMP_* constants.
inline Vec3f evaluateRamp(int ramp, float t) {
  t = std::clamp(t, 0.0f, 1.0f);
  switch (ramp) {
    case 1: {  // Turbo
      static const std::array<Vec3f, 7> c{
          Vec3f{0.1140890109f, 0.0623170017f, 0.2248337467f},
          Vec3f{6.7161151170f, 3.1827360244f, 7.5715099690f},
          Vec3f{-66.093991706f, -4.9279827040f, -10.089172730f},
          Vec3f{228.76660846f, 25.049498590f, -91.541147500f},
          Vec3f{-334.83643282f, -69.319149610f, 288.58858150f},
          Vec3f{218.76738840f, 67.522080500f, -305.20816330f},
          Vec3f{-52.889462800f, -21.540823620f, 110.51855750f}};
      return detail::saturate(detail::polyRamp(t, c));
    }
    case 2:  // Grayscale
      return Vec3f{t, t, t};
    case 3: {  // Cool-Warm
      const Vec3f cold{0.230f, 0.299f, 0.754f};
      const Vec3f mid{0.865f, 0.865f, 0.865f};
      const Vec3f warm{0.706f, 0.016f, 0.150f};
      return t < 0.5f ? cold + (mid - cold) * (t * 2.0f)
                      : mid + (warm - mid) * ((t - 0.5f) * 2.0f);
    }
    case 4: {  // Plasma
      static const std::array<Vec3f, 7> c{
          Vec3f{0.0587251965f, 0.0294136475f, 0.5307400996f},
          Vec3f{2.1765219460f, 0.2383834839f, 0.7538451595f},
          Vec3f{-2.6894992770f, -7.455851140f, 3.1105237260f},
          Vec3f{6.1308234530f, 42.345414110f, -28.518398110f},
          Vec3f{-11.107024930f, -82.667166700f, 60.139313580f},
          Vec3f{10.023306310f, 71.413806260f, -54.072138480f},
          Vec3f{-3.6589284510f, -22.930299000f, 18.190490200f}};
      return detail::saturate(detail::polyRamp(t, c));
    }
    case 5: {  // Spectral
      const Vec3f stops[5] = {{0.620f, 0.004f, 0.259f}, {0.957f, 0.427f, 0.263f},
                              {1.000f, 1.000f, 0.749f}, {0.400f, 0.761f, 0.647f},
                              {0.369f, 0.310f, 0.635f}};
      const float s = t * 4.0f;
      const int i = std::min(3, static_cast<int>(s));
      const float f = s - static_cast<float>(i);
      return stops[i] + (stops[i + 1] - stops[i]) * f;
    }
    default: {  // Viridis
      static const std::array<Vec3f, 7> c{
          Vec3f{0.2777273272f, 0.0054929071f, 0.3340998053f},
          Vec3f{0.1050930431f, 1.4040130306f, 1.3845230920f},
          Vec3f{-0.3308618287f, 0.2148529221f, -1.3704099570f},
          Vec3f{-4.6342214057f, -5.7991685017f, -5.6010601619f},
          Vec3f{6.2283282653f, 14.1799586770f, 56.6904639680f},
          Vec3f{4.7763380131f, -13.745449704f, -31.499378500f},
          Vec3f{-5.4354316899f, 4.6451234060f, 6.1308152837f}};
      return detail::saturate(detail::polyRamp(t, c));
    }
  }
}

}  // namespace pcv
