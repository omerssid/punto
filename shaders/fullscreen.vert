#version 460
// Full-screen triangle generated from gl_VertexID, so the post passes need
// no vertex buffer and no VAO contents. One oversized triangle rather than
// two triangles: no diagonal seam, and fewer quads shaded twice.
out vec2 vUv;

void main() {
  vec2 uv = vec2((gl_VertexID << 1) & 2, gl_VertexID & 2);
  vUv = uv;
  gl_Position = vec4(uv * 2.0 - 1.0, 0.0, 1.0);
}
