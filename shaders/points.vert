#version 460
// Point vertex stage.
//
// Vertex data is fetched from SSBOs rather than through a VAO, because the
// colour source changes at runtime and SSBO indexing keeps that a uniform
// switch instead of a vertex-format rebuild.
#extension GL_ARB_shader_draw_parameters : require

#include "common.glsl"
#include "colormap.glsl"

layout(std430, binding = BIND_CHUNKS) readonly buffer ChunkBuffer { GpuChunk chunks[]; };
layout(std430, binding = BIND_LOD_SCALE) readonly buffer LodScaleBuffer { float lodScale[]; };
layout(std430, binding = BIND_POSITIONS) readonly buffer PositionBuffer { uvec2 positions[]; };
layout(std430, binding = BIND_COLORS) readonly buffer ColorBuffer { uint colors[]; };
layout(std430, binding = BIND_SCALARS) readonly buffer ScalarBuffer { uint scalars[]; };

uniform mat4 uViewProj;
uniform vec3 uCameraPos;
uniform float uProjScale;        // viewportHeight / (2 * tan(fovY / 2))
uniform float uPointScale;       // user size multiplier
uniform float uMinPixels;
uniform float uMaxPixels;
uniform int uColorMode;
uniform int uRamp;
uniform int uHasColors;
uniform vec3 uFlatColor;
uniform int uOrthographic;
uniform float uOrthoPixelsPerMetre;
uniform int uFixedSize;          // ignore distance; draw every point the same
uniform vec4 uClipPlane;
uniform int uClipEnabled;
uniform float uScalarGamma;

out vec3 vColor;
out float vViewDepth;   // linear distance, for EDL
flat out uint vPointId;

void main() {
  uint chunkIndex = uint(gl_BaseInstanceARB);
  GpuChunk chunk = chunks[chunkIndex];

  // The indirect command sets `first` to chunk.firstPoint, and for a
  // non-indexed draw gl_VertexID is first + i, so this is already the
  // point's global index into the cloud's arrays.
  uint pointId = uint(gl_VertexID);
  vPointId = pointId;

  vec3 local = dequantizePosition(positions[pointId], chunk);
  gl_Position = uViewProj * vec4(local, 1.0);

  // Per-point clipping. The cull pass already dropped chunks entirely on the
  // far side; this handles the chunks the plane cuts through.
  if (uClipEnabled != 0 && dot(uClipPlane.xyz, local) + uClipPlane.w < 0.0) {
    // Push behind the near plane so the point is discarded without a branch
    // in the fragment stage.
    gl_Position = vec4(0.0, 0.0, -1.0, 0.0);
    gl_PointSize = 0.0;
    vColor = vec3(0.0);
    vViewDepth = 0.0;
    return;
  }

  float distance = length(local - uCameraPos);
  vViewDepth = distance;

  // Splat radius in world units: the chunk's mean point spacing, widened by
  // however much LOD thinned this chunk, times the user's multiplier.
  float worldRadius = chunk.boundsMin.w * uPointScale * lodScale[chunkIndex];

  float pixels;
  if (uFixedSize != 0) {
    pixels = uPointScale;
  } else if (uOrthographic != 0) {
    pixels = worldRadius * uOrthoPixelsPerMetre;
  } else {
    pixels = worldRadius * uProjScale / max(distance, 1e-4);
  }
  gl_PointSize = clamp(pixels, uMinPixels, uMaxPixels);

  if (uColorMode == COLOR_MODE_RGB && uHasColors != 0) {
    vColor = unpackColor(colors[pointId]);
  } else if (uColorMode == COLOR_MODE_SCALAR) {
    float t = unpackScalar(scalars[pointId >> 1], pointId);
    // Gamma on the normalised value, so a ramp can be pushed towards either
    // end without reloading the field.
    if (uScalarGamma != 1.0) t = pow(t, uScalarGamma);
    vColor = applyRamp(uRamp, t);
  } else if (uColorMode == COLOR_MODE_CHUNK) {
    vColor = chunkDebugColor(chunkIndex);
  } else {
    vColor = uFlatColor;
  }
}
