#version 460
// Point fragment stage: round splats, optional spherical shading, and a
// linear depth target for the Eye-Dome Lighting pass.
#include "common.glsl"

in vec3 vColor;
in float vViewDepth;
flat in uint vPointId;

layout(location = 0) out vec4 outColor;
layout(location = 1) out float outViewDepth;

uniform int uRoundPoints;
uniform int uShadeSplats;   // fake a lit sphere for extra shape cue
uniform vec3 uLightDir;

void main() {
  vec3 color = vColor;

  if (uRoundPoints != 0) {
    // gl_PointCoord runs 0..1 across the splat; recentre to -1..1.
    vec2 d = gl_PointCoord * 2.0 - 1.0;
    float r2 = dot(d, d);
    // Discarding the corners is what turns a square into a disc. Square
    // points are what make dense clouds look like a tiled grid.
    if (r2 > 1.0) discard;

    if (uShadeSplats != 0) {
      // Treat the disc as a sphere facing the viewer and light it. Cheap,
      // and gives each point a little volume at close range.
      vec3 normal = vec3(d, sqrt(max(0.0, 1.0 - r2)));
      float lambert = max(dot(normal, normalize(uLightDir)), 0.0);
      color *= 0.55 + 0.45 * lambert;
    }
  }

  outColor = vec4(color, 1.0);
  // EDL needs linear eye distance, not the reversed-Z depth buffer value.
  // Writing it explicitly is cheaper and clearer than reconstructing it.
  outViewDepth = vViewDepth;
}
