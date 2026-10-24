// Shared declarations for the point pipeline.
//
// Mirrors pcv::GpuChunk in src/scene/chunk.hpp. The two layouts must agree
// exactly: vec4s first so nothing straddles a 16-byte std430 boundary.
#ifndef PCV_COMMON_GLSL
#define PCV_COMMON_GLSL

struct GpuChunk {
  vec4 boundsMin;  // xyz = minimum corner, w = mean point spacing in metres
  vec4 boundsMax;  // xyz = maximum corner, w unused
  uint firstPoint;
  uint pointCount;
  uint pad0;
  uint pad1;
};

// SSBO binding points, shared between the cull and draw passes.
#define BIND_CHUNKS     0
#define BIND_DRAW_CMDS  1
#define BIND_DRAW_COUNT 2
#define BIND_LOD_SCALE  3
#define BIND_POSITIONS  4
#define BIND_COLORS     5
#define BIND_SCALARS    6
#define BIND_PICK       7

// Colour sources, matching pcv::ColorMode.
#define COLOR_MODE_RGB      0
#define COLOR_MODE_SCALAR   1
#define COLOR_MODE_FLAT     2
#define COLOR_MODE_CHUNK    3

// Reconstructs a world position from its 16-bit-per-axis encoding.
//
// Each point stores a fraction of its own chunk's bounding box. Chunks span
// tens of metres, so a 1/65535 step is well under a millimetre -- far below
// the sensor noise in any real scan.
vec3 dequantizePosition(uvec2 encoded, GpuChunk chunk) {
  vec3 q = vec3(float(encoded.x & 0xFFFFu),
                float(encoded.x >> 16),
                float(encoded.y & 0xFFFFu)) * (1.0 / 65535.0);
  return mix(chunk.boundsMin.xyz, chunk.boundsMax.xyz, q);
}

// Unpacks one 16-bit scalar from the two-per-word scalar buffer.
float unpackScalar(uint word, uint index) {
  uint v = ((index & 1u) != 0u) ? (word >> 16) : (word & 0xFFFFu);
  return float(v) * (1.0 / 65535.0);
}

vec3 unpackColor(uint rgba) {
  return vec3(float(rgba & 0xFFu), float((rgba >> 8) & 0xFFu), float((rgba >> 16) & 0xFFu))
         * (1.0 / 255.0);
}

#endif  // PCV_COMMON_GLSL
