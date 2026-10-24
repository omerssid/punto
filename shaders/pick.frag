#version 460
// Picking pass: writes each fragment's global point index so the CPU can
// read back which point is under the cursor.
//
// Runs only on click, into a small scissored region, so the per-frame cost
// is zero. Index 0 is reserved for "nothing", hence the +1.
in vec3 vColor;
in float vViewDepth;
flat in uint vPointId;

layout(location = 0) out uvec2 outPick;

uniform int uRoundPoints;

void main() {
  if (uRoundPoints != 0) {
    vec2 d = gl_PointCoord * 2.0 - 1.0;
    // Match the visible splat exactly, or the cursor would pick points whose
    // corners were discarded in the colour pass.
    if (dot(d, d) > 1.0) discard;
  }
  // Depth is carried alongside so the reader can rank hits without a second
  // attachment: the depth test already keeps only the nearest, but the
  // value is useful for reporting distance.
  outPick = uvec2(vPointId + 1u, floatBitsToUint(vViewDepth));
}
