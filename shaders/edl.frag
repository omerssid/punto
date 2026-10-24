#version 460
// Eye-Dome Lighting.
//
// An unlit point cloud gives the eye almost nothing to read depth from: no
// shading, no occlusion, no silhouettes. EDL recovers all three from the
// depth buffer alone. For each pixel it compares its depth against its
// neighbours and darkens wherever the pixel sits further back, which draws
// a soft shadow along every depth discontinuity. The result reads as solid
// geometry rather than coloured confetti, and it costs one full-screen pass.
//
// Depths are compared in log space so the effect is scale-invariant: an edge
// one metre deep looks the same whether it is 5 m or 500 m from the camera,
// which matters when a single HD map spans both.
in vec2 vUv;
out vec4 outColor;

uniform sampler2D uColor;
uniform sampler2D uViewDepth;   // linear eye distance; 0 means background

uniform vec2 uTexelSize;
// The colour and depth targets are allocated larger than the viewport and
// reused across resizes, so only this fraction of them holds the frame.
uniform vec2 uUvScale;
uniform float uStrength;
uniform float uRadius;          // sampling radius in pixels
uniform vec3 uBackground;
uniform int uEnabled;
uniform int uShowDepthOnly;     // debug view

// Eight neighbours on a ring. Four is visibly directional on diagonal
// edges; sixteen is not noticeably better.
const vec2 kOffsets[8] = vec2[8](
  vec2( 1.0,  0.0), vec2( 0.7071,  0.7071),
  vec2( 0.0,  1.0), vec2(-0.7071,  0.7071),
  vec2(-1.0,  0.0), vec2(-0.7071, -0.7071),
  vec2( 0.0, -1.0), vec2( 0.7071, -0.7071)
);

void main() {
  vec2 uv = vUv * uUvScale;
  vec3 color = texture(uColor, uv).rgb;
  float centre = texture(uViewDepth, uv).r;

  // Nothing was drawn here.
  if (centre <= 0.0) {
    outColor = vec4(uBackground, 1.0);
    return;
  }

  if (uShowDepthOnly != 0) {
    float t = 1.0 - exp(-centre * 0.01);
    outColor = vec4(vec3(t), 1.0);
    return;
  }

  if (uEnabled == 0) {
    outColor = vec4(color, 1.0);
    return;
  }

  float logCentre = log2(centre);
  float response = 0.0;
  float contributors = 0.0;

  for (int i = 0; i < 8; ++i) {
    // Clamped to the live sub-rectangle: sampling past it would read
    // stale pixels left over from a previous, larger frame.
    vec2 sampleUv = clamp(uv + kOffsets[i] * uTexelSize * uRadius, vec2(0.0), uUvScale);
    float neighbour = texture(uViewDepth, sampleUv).r;
    // Background neighbours are skipped rather than treated as infinitely
    // far: counting them would ring a dark halo around every silhouette.
    if (neighbour <= 0.0) continue;
    // Positive only where this pixel is further away than its neighbour,
    // i.e. on the receding side of an edge.
    response += max(0.0, logCentre - log2(neighbour));
    contributors += 1.0;
  }

  if (contributors > 0.0) response /= contributors;

  float shade = exp(-response * uStrength * 300.0);
  outColor = vec4(color * shade, 1.0);
}
