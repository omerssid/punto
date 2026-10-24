#version 460
// Line overlay: selection markers, measurement chains, crop box.
//
// Lines are expanded into screen-space quads here rather than drawn with
// GL_LINES and glLineWidth. In a forward-compatible core context wide lines
// are removed from the API, so any width above 1.0 raises GL_INVALID_VALUE
// -- and the driver still advertises a 1.0 to 10.0 range, so the only
// symptom is an error log and hairline-thin lines. Expanding manually also
// gives fractional widths and identical results on every driver.
layout(location = 0) in vec3 inStart;
layout(location = 1) in vec3 inEnd;
// x selects the endpoint (0 = start, 1 = end), y the side (-1 or +1).
layout(location = 2) in vec2 inCorner;
layout(location = 3) in vec4 inColor;

uniform mat4 uViewProj;
uniform vec2 uViewport;   // pixels
uniform float uHalfWidth; // pixels

out vec4 vColor;

void main() {
  vColor = inColor;

  vec4 clipStart = uViewProj * vec4(inStart, 1.0);
  vec4 clipEnd = uViewProj * vec4(inEnd, 1.0);

  // Clip against the near plane before the perspective divide. An endpoint
  // behind the camera has w <= 0 and would otherwise project to the wrong
  // side of the screen, drawing a line across the whole viewport.
  const float kMinW = 1e-5;
  if (clipStart.w <= kMinW && clipEnd.w <= kMinW) {
    gl_Position = vec4(0.0, 0.0, 0.0, 0.0);  // degenerate; nothing rasterises
    return;
  }
  if (clipStart.w <= kMinW) {
    float t = (kMinW - clipStart.w) / (clipEnd.w - clipStart.w);
    clipStart = mix(clipStart, clipEnd, t);
  } else if (clipEnd.w <= kMinW) {
    float t = (kMinW - clipEnd.w) / (clipStart.w - clipEnd.w);
    clipEnd = mix(clipEnd, clipStart, t);
  }

  vec2 screenStart = clipStart.xy / clipStart.w * uViewport * 0.5;
  vec2 screenEnd = clipEnd.xy / clipEnd.w * uViewport * 0.5;

  vec2 along = screenEnd - screenStart;
  float len = length(along);
  // A segment shorter than a pixel has no meaningful direction; pick one so
  // it still shows as a dot rather than vanishing.
  vec2 direction = len > 1e-6 ? along / len : vec2(1.0, 0.0);
  vec2 normal = vec2(-direction.y, direction.x);

  vec4 clip = mix(clipStart, clipEnd, inCorner.x);
  // Scale the offset by w so it survives the perspective divide as a
  // constant number of pixels.
  clip.xy += normal * inCorner.y * uHalfWidth / (uViewport * 0.5) * clip.w;
  gl_Position = clip;
}
