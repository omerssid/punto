// Scalar-to-colour ramps.
//
// Viridis and Turbo are polynomial fits rather than lookup textures: no
// texture binding, no sampler state, and the cost is a handful of FMAs.
// Both are perceptually uniform, which matters because the whole point of
// colouring by intensity or height is to read magnitude off the image.
#ifndef PCV_COLORMAP_GLSL
#define PCV_COLORMAP_GLSL

#define RAMP_VIRIDIS   0
#define RAMP_TURBO     1
#define RAMP_GRAYSCALE 2
#define RAMP_COOLWARM  3
#define RAMP_PLASMA    4
#define RAMP_SPECTRAL  5

// Fit by Matt Zucker; max error well under one 8-bit step.
vec3 rampViridis(float t) {
  const vec3 c0 = vec3( 0.2777273272, 0.0054929071, 0.3340998053);
  const vec3 c1 = vec3( 0.1050930431, 1.4040130306, 1.3845230920);
  const vec3 c2 = vec3(-0.3308618287, 0.2148529221,-1.3704099570);
  const vec3 c3 = vec3(-4.6342214057,-5.7991685017,-5.6010601619);
  const vec3 c4 = vec3( 6.2283282653,14.1799586770,56.6904639680);
  const vec3 c5 = vec3( 4.7763380131,-13.745449704,-31.499378500);
  const vec3 c6 = vec3(-5.4354316899, 4.6451234060, 6.1308152837);
  return c0 + t * (c1 + t * (c2 + t * (c3 + t * (c4 + t * (c5 + t * c6)))));
}

// Google's Turbo, polynomial fit. Higher contrast than Viridis and keeps
// detail at both ends, which suits LiDAR intensity.
vec3 rampTurbo(float t) {
  const vec3 c0 = vec3( 0.1140890109, 0.0623170017, 0.2248337467);
  const vec3 c1 = vec3( 6.7161151170, 3.1827360244, 7.5715099690);
  const vec3 c2 = vec3(-66.093991706,-4.9279827040,-10.089172730);
  const vec3 c3 = vec3( 228.76660846, 25.049498590,-91.541147500);
  const vec3 c4 = vec3(-334.83643282,-69.319149610, 288.58858150);
  const vec3 c5 = vec3( 218.76738840, 67.522080500,-305.20816330);
  const vec3 c6 = vec3(-52.889462800,-21.540823620, 110.51855750);
  return c0 + t * (c1 + t * (c2 + t * (c3 + t * (c4 + t * (c5 + t * c6)))));
}

vec3 rampPlasma(float t) {
  const vec3 c0 = vec3( 0.0587251965, 0.0294136475, 0.5307400996);
  const vec3 c1 = vec3( 2.1765219460, 0.2383834839, 0.7538451595);
  const vec3 c2 = vec3(-2.6894992770, -7.455851140, 3.1105237260);
  const vec3 c3 = vec3( 6.1308234530, 42.345414110,-28.518398110);
  const vec3 c4 = vec3(-11.107024930,-82.667166700, 60.139313580);
  const vec3 c5 = vec3( 10.023306310, 71.413806260,-54.072138480);
  const vec3 c6 = vec3(-3.6589284510,-22.930299000, 18.190490200);
  return c0 + t * (c1 + t * (c2 + t * (c3 + t * (c4 + t * (c5 + t * c6)))));
}

// Diverging: blue through near-white to red. For signed data -- elevation
// relative to a datum, registration residuals -- where the midpoint means
// something and a sequential ramp would hide it.
vec3 rampCoolWarm(float t) {
  const vec3 cold = vec3(0.230, 0.299, 0.754);
  const vec3 mid  = vec3(0.865, 0.865, 0.865);
  const vec3 warm = vec3(0.706, 0.016, 0.150);
  return t < 0.5 ? mix(cold, mid, t * 2.0) : mix(mid, warm, (t - 0.5) * 2.0);
}

vec3 rampSpectral(float t) {
  const vec3 a = vec3(0.620, 0.004, 0.259);
  const vec3 b = vec3(0.957, 0.427, 0.263);
  const vec3 c = vec3(1.000, 1.000, 0.749);
  const vec3 d = vec3(0.400, 0.761, 0.647);
  const vec3 e = vec3(0.369, 0.310, 0.635);
  float s = clamp(t, 0.0, 1.0) * 4.0;
  if (s < 1.0) return mix(a, b, s);
  if (s < 2.0) return mix(b, c, s - 1.0);
  if (s < 3.0) return mix(c, d, s - 2.0);
  return mix(d, e, s - 3.0);
}

vec3 applyRamp(int ramp, float t) {
  t = clamp(t, 0.0, 1.0);
  if (ramp == RAMP_TURBO)     return clamp(rampTurbo(t), 0.0, 1.0);
  if (ramp == RAMP_GRAYSCALE) return vec3(t);
  if (ramp == RAMP_COOLWARM)  return rampCoolWarm(t);
  if (ramp == RAMP_PLASMA)    return clamp(rampPlasma(t), 0.0, 1.0);
  if (ramp == RAMP_SPECTRAL)  return rampSpectral(t);
  return clamp(rampViridis(t), 0.0, 1.0);
}

// Distinct colours for debugging the chunk tree: adjacent chunk indices must
// look obviously different, so the hue is driven by a hash rather than by
// the index itself.
vec3 chunkDebugColor(uint index) {
  uint h = index * 2654435761u;
  return vec3(float((h >>  0) & 255u), float((h >>  8) & 255u), float((h >> 16) & 255u))
         * (1.0 / 255.0) * 0.7 + 0.3;
}

#endif  // PCV_COLORMAP_GLSL
